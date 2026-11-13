/*
 * Copyright (c) 2016 Grzegorz Hetman <ghetman@gmail.com>
 * Copyright (c) 2016 Alex Stewart <foogod@gmail.com>
 * Copyright (c) 2018 Ruslan V. Uss <unclerus@gmail.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of itscontributors
 *    may be used to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file ds18x20.c
 *
 * ESP-IDF driver for the DS18S20/DS18B20 one-wire temperature sensor ICs
 *
 * Ported from esp-open-rtos
 *
 * Copyright (c) 2016 Grzegorz Hetman <ghetman@gmail.com>\n
 * Copyright (c) 2016 Alex Stewart <foogod@gmail.com>\n
 * Copyright (c) 2018 Ruslan V. Uss <unclerus@gmail.com>
 *
 * BSD Licensed as described in the file LICENSE
 */

#include <math.h>
#include <string.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_idf_lib_helpers.h>
#include "ds18x20.h"

#if CONFIG_DS18X20_ONEWIRE_BACKEND_ESPRESSIF
#include <onewire_bus.h>
#include <onewire_device.h>
#include <onewire_crc.h>
#endif

#define ds18x20_WRITE_SCRATCHPAD 0x4E
#define ds18x20_READ_SCRATCHPAD  0xBE
#define ds18x20_COPY_SCRATCHPAD  0x48
#define ds18x20_READ_EEPROM      0xB8
#define ds18x20_READ_PWRSUPPLY   0xB4
#define ds18x20_SEARCHROM        0xF0
#define ds18x20_SKIP_ROM         0xCC
#define ds18x20_READROM          0x33
#define ds18x20_MATCHROM         0x55
#define ds18x20_ALARMSEARCH      0xEC
#define ds18x20_CONVERT_T        0x44

#define SLEEP_MS(x) vTaskDelay(((x) + portTICK_PERIOD_MS - 1) / portTICK_PERIOD_MS)
#define CHECK(x) do { esp_err_t __; if ((__ = x) != ESP_OK) return __; } while (0)
#define CHECK_ARG(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)

static const char *TAG = "ds18x20";

/*
 * 1-Wire bus abstraction. The bit-banging driver of esp-idf-lib takes a GPIO
 * pin in every call and keeps the bus state in the GPIO itself, while the
 * driver of Espressif is handle-based and owns a pair of RMT channels. The
 * wrappers below hide the difference: the rest of the driver works with the
 * pin only.
 */

#if CONFIG_DS18X20_ONEWIRE_BACKEND_ESPRESSIF

/** The largest single receive transaction is 9 bytes: 8 data bytes plus CRC */
#define ONEWIRE_RMT_MAX_RX_BYTES 10

typedef struct
{
    gpio_num_t pin;
    onewire_bus_handle_t handle;
} wire_bus_t;

typedef struct
{
    onewire_device_iter_handle_t iter;
    onewire_device_t device;
} wire_search_t;

static wire_bus_t buses[CONFIG_DS18X20_MAX_BUSES];

static esp_err_t wire_bus_get(gpio_num_t pin, onewire_bus_handle_t *handle)
{
    for (size_t i = 0; i < CONFIG_DS18X20_MAX_BUSES; i++)
        if (buses[i].handle && buses[i].pin == pin)
        {
            *handle = buses[i].handle;
            return ESP_OK;
        }

    onewire_bus_config_t bus_config =
    {
        .bus_gpio_num = pin,
    };
    onewire_bus_rmt_config_t rmt_config =
    {
        .max_rx_bytes = ONEWIRE_RMT_MAX_RX_BYTES,
    };

    onewire_bus_handle_t new_handle;
    esp_err_t err = onewire_new_bus_rmt(&bus_config, &rmt_config, &new_handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Could not create the 1-Wire bus on GPIO %d: %s", pin, esp_err_to_name(err));
        return err;
    }

    for (size_t i = 0; i < CONFIG_DS18X20_MAX_BUSES; i++)
        if (!buses[i].handle)
        {
            buses[i].pin = pin;
            buses[i].handle = new_handle;
            *handle = new_handle;
            return ESP_OK;
        }

    onewire_bus_del(new_handle);
    ESP_LOGE(TAG, "No free slot for the 1-Wire bus on GPIO %d, increase CONFIG_DS18X20_MAX_BUSES", pin);

    return ESP_ERR_NO_MEM;
}

esp_err_t ds18x20_bus_init(gpio_num_t pin)
{
    CHECK_ARG(GPIO_IS_VALID_OUTPUT_GPIO(pin));

    onewire_bus_handle_t handle;
    return wire_bus_get(pin, &handle);
}

esp_err_t ds18x20_bus_free(gpio_num_t pin)
{
    CHECK_ARG(GPIO_IS_VALID_OUTPUT_GPIO(pin));

    for (size_t i = 0; i < CONFIG_DS18X20_MAX_BUSES; i++)
        if (buses[i].handle && buses[i].pin == pin)
        {
            esp_err_t err = onewire_bus_del(buses[i].handle);
            buses[i].handle = NULL;
            buses[i].pin = GPIO_NUM_NC;
            return err;
        }

    return ESP_ERR_NOT_FOUND;
}

static bool wire_reset(gpio_num_t pin)
{
    onewire_bus_handle_t handle;
    if (wire_bus_get(pin, &handle) != ESP_OK)
        return false;

    return onewire_bus_reset(handle) == ESP_OK;
}

static bool wire_write(gpio_num_t pin, uint8_t value)
{
    onewire_bus_handle_t handle;
    if (wire_bus_get(pin, &handle) != ESP_OK)
        return false;

    return onewire_bus_write_bytes(handle, &value, 1) == ESP_OK;
}

static bool wire_write_bytes(gpio_num_t pin, const uint8_t *buffer, size_t size)
{
    onewire_bus_handle_t handle;
    if (wire_bus_get(pin, &handle) != ESP_OK)
        return false;

    return onewire_bus_write_bytes(handle, buffer, (uint8_t)size) == ESP_OK;
}

static bool wire_read_bytes(gpio_num_t pin, uint8_t *buffer, size_t size)
{
    onewire_bus_handle_t handle;
    if (wire_bus_get(pin, &handle) != ESP_OK)
        return false;

    return onewire_bus_read_bytes(handle, buffer, size) == ESP_OK;
}

static bool wire_convert(gpio_num_t pin, uint8_t command, uint32_t power_ms)
{
    if (!wire_write(pin, command))
        return false;

    // The RMT backend has no strong pull-up, so it cannot feed a parasitic
    // device during the conversion. The sensors with a power pin work as usual.
    if (power_ms)
    {
        static bool warned = false;
        if (!warned)
        {
            ESP_LOGW(TAG, "The RMT backend cannot hold the bus with a strong pull-up: "
                          "parasitically powered sensors will not convert");
            warned = true;
        }
        SLEEP_MS(power_ms);
    }

    return true;
}


static esp_err_t wire_search_start(wire_search_t *search, gpio_num_t pin)
{
    onewire_bus_handle_t handle;
    CHECK(wire_bus_get(pin, &handle));

    search->iter = NULL;
    esp_err_t err = onewire_new_device_iter(handle, &search->iter);
    if (err != ESP_OK)
        ESP_LOGE(TAG, "Could not create the device iterator: %s", esp_err_to_name(err));

    return err;
}

static ds18x20_addr_t wire_search_next(wire_search_t *search)
{
    for (;;)
    {
        esp_err_t err = onewire_device_iter_get_next(search->iter, &search->device);
        if (err == ESP_OK)
            return (ds18x20_addr_t)search->device.address;
        if (err != ESP_ERR_INVALID_CRC)
            return DS18X20_ANY;

        ESP_LOGW(TAG, "Skipping a device with an invalid ROM CRC");
    }
}

static void wire_search_stop(wire_search_t *search)
{
    if (search->iter)
        onewire_del_device_iter(search->iter);
}

static uint8_t wire_crc8(const uint8_t *buffer, size_t size)
{
    return onewire_crc8(0, (uint8_t *)buffer, size);
}

#else /* CONFIG_DS18X20_ONEWIRE_BACKEND_BITBANG */

typedef struct
{
    gpio_num_t pin;
    onewire_search_t search;
} wire_search_t;

static portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
#define PORT_ENTER_CRITICAL portENTER_CRITICAL(&mux)
#define PORT_EXIT_CRITICAL portEXIT_CRITICAL(&mux)

esp_err_t ds18x20_bus_init(gpio_num_t pin)
{
    CHECK_ARG(GPIO_IS_VALID_OUTPUT_GPIO(pin));

    return ESP_OK;
}

esp_err_t ds18x20_bus_free(gpio_num_t pin)
{
    CHECK_ARG(GPIO_IS_VALID_OUTPUT_GPIO(pin));

    return ESP_OK;
}

static bool wire_reset(gpio_num_t pin)
{
    return onewire_reset(pin);
}

static bool wire_write(gpio_num_t pin, uint8_t value)
{
    return onewire_write(pin, value);
}

static bool wire_write_bytes(gpio_num_t pin, const uint8_t *buffer, size_t size)
{
    return onewire_write_bytes(pin, buffer, size);
}

static bool wire_read_bytes(gpio_num_t pin, uint8_t *buffer, size_t size)
{
    return onewire_read_bytes(pin, buffer, size);
}

static bool wire_convert(gpio_num_t pin, uint8_t command, uint32_t power_ms)
{
    PORT_ENTER_CRITICAL;
    bool ok = onewire_write(pin, command);
    // For parasitic devices, power must be applied within 10us after issuing
    // the convert command.
    onewire_power(pin);
    PORT_EXIT_CRITICAL;

    if (power_ms)
    {
        SLEEP_MS(power_ms);
        onewire_depower(pin);
    }

    return ok;
}


static esp_err_t wire_search_start(wire_search_t *search, gpio_num_t pin)
{
    search->pin = pin;
    onewire_search_start(&search->search);

    return ESP_OK;
}

static ds18x20_addr_t wire_search_next(wire_search_t *search)
{
    return onewire_search_next(&search->search, search->pin);
}

static void wire_search_stop(wire_search_t *search)
{
    (void)search;
}

static uint8_t wire_crc8(const uint8_t *buffer, size_t size)
{
    return onewire_crc8(buffer, (uint8_t)size);
}

#endif /* CONFIG_DS18X20_ONEWIRE_BACKEND_ESPRESSIF */

static bool wire_skip_rom(gpio_num_t pin)
{
    return wire_write(pin, ds18x20_SKIP_ROM);
}

static bool wire_select(gpio_num_t pin, ds18x20_addr_t addr)
{
    uint8_t buffer[9] = { ds18x20_MATCHROM };
    for (size_t i = 0; i < 8; i++)
        buffer[i + 1] = (uint8_t)(addr >> (8 * i));

    return wire_write_bytes(pin, buffer, sizeof(buffer));
}

esp_err_t ds18x20_measure(gpio_num_t pin, ds18x20_addr_t addr, bool wait)
{
    if (!wire_reset(pin))
        return ESP_ERR_INVALID_RESPONSE;

    if (addr == DS18X20_ANY)
        wire_skip_rom(pin);
    else
        wire_select(pin, addr);

    if (!wire_convert(pin, ds18x20_CONVERT_T, wait ? 750 : 0))
        return ESP_ERR_INVALID_RESPONSE;

    return ESP_OK;
}

esp_err_t ds18x20_read_scratchpad(gpio_num_t pin, ds18x20_addr_t addr, uint8_t *buffer)
{
    CHECK_ARG(buffer);

    if (!wire_reset(pin))
        return ESP_ERR_INVALID_RESPONSE;

    if (addr == DS18X20_ANY)
        wire_skip_rom(pin);
    else
        wire_select(pin, addr);
    wire_write(pin, ds18x20_READ_SCRATCHPAD);

    uint8_t scratchpad[9];
    if (!wire_read_bytes(pin, scratchpad, sizeof(scratchpad)))
        return ESP_ERR_INVALID_RESPONSE;

    uint8_t expected_crc = wire_crc8(scratchpad, 8);
    if (scratchpad[8] != expected_crc)
    {
        ESP_LOGE(TAG, "CRC check failed reading scratchpad: %02x %02x %02x %02x %02x %02x %02x %02x : %02x (expected %02x)",
                 scratchpad[0], scratchpad[1], scratchpad[2], scratchpad[3], scratchpad[4], scratchpad[5],
                 scratchpad[6], scratchpad[7], scratchpad[8], expected_crc);
        return ESP_ERR_INVALID_CRC;
    }

    memcpy(buffer, scratchpad, 8);

    return ESP_OK;
}

esp_err_t ds18x20_write_scratchpad(gpio_num_t pin, ds18x20_addr_t addr, uint8_t *buffer)
{
    CHECK_ARG(buffer);

    if (!wire_reset(pin))
        return ESP_ERR_INVALID_RESPONSE;

    if (addr == DS18X20_ANY)
        wire_skip_rom(pin);
    else
        wire_select(pin, addr);
    wire_write(pin, ds18x20_WRITE_SCRATCHPAD);

    for (int i = 0; i < 3; i++)
        wire_write(pin, buffer[i]);

    return ESP_OK;
}

esp_err_t ds18x20_copy_scratchpad(gpio_num_t pin, ds18x20_addr_t addr)
{
    if (!wire_reset(pin))
        return ESP_ERR_INVALID_RESPONSE;

    if (addr == DS18X20_ANY)
        wire_skip_rom(pin);
    else
        wire_select(pin, addr);

    if (!wire_convert(pin, ds18x20_COPY_SCRATCHPAD, 10))
        return ESP_ERR_INVALID_RESPONSE;

    return ESP_OK;
}

esp_err_t ds18s20_read_temperature(gpio_num_t pin, ds18x20_addr_t addr, float *temperature)
{
    CHECK_ARG(temperature);

    uint8_t scratchpad[8];
    CHECK(ds18x20_read_scratchpad(pin, addr, scratchpad));

    // The power-on value of the temperature register is 0x00AA, and the
    // reserved byte 6 contains 0x0C until the first conversion completes.
    if (scratchpad[6] == 0x0c && scratchpad[0] == 0xaa && scratchpad[1] == 0x00)
    {
        ESP_LOGE(TAG, "The sensor did not convert: the temperature register holds the power-on value. "
                      "A parasitically powered sensor needs a strong pull-up during the conversion, "
                      "which the RMT backend does not provide");
        return ESP_ERR_INVALID_RESPONSE;
    }

    int16_t temp = (((scratchpad[1] << 8) | (scratchpad[0] & 0xfe)) << 3) | ((0x10 - scratchpad[6]) & 0x0f);
    *temperature = (float)temp * 0.0625f - 0.250f;

    return ESP_OK;
}

static uint16_t resolution_mask(uint8_t config)
{
    switch ((config >> 5) & 0x03)
    {
        case 0:
            return 0xfff8; // 9 bit
        case 1:
            return 0xfffc; // 10 bit
        case 2:
            return 0xfffe; // 11 bit
        default:
            return 0xffff; // 12 bit
    }
}

esp_err_t ds18b20_read_temperature(gpio_num_t pin, ds18x20_addr_t addr, float *temperature)
{
    CHECK_ARG(temperature);

    uint8_t scratchpad[8];
    CHECK(ds18x20_read_scratchpad(pin, addr, scratchpad));

    uint16_t raw = scratchpad[1] << 8 | scratchpad[0];

    // 85°C with a reserved byte 6 of 0x0C is the power-on value of the
    // register: the conversion did not happen, see
    // https://github.com/cpetrich/counterfeit_DS18B20#solution-to-the-85-c-problem
    if (scratchpad[6] == 0x0c && (raw & 0xfff8) == 0x0550)
    {
        ESP_LOGE(TAG, "The sensor did not convert: the temperature register holds the power-on value. "
                      "A parasitically powered sensor needs a strong pull-up during the conversion, "
                      "which the RMT backend does not provide");
        return ESP_ERR_INVALID_RESPONSE;
    }

    uint16_t temp = raw & resolution_mask(scratchpad[4]);
    int sign = 1;
    if (temp > 2047)
    {
        temp = ~temp + 1;
        sign = -1;
    }
    *temperature = (float)temp * (float)sign * 0.0625f;

    return ESP_OK;
}

esp_err_t max31850_read_temperature(gpio_num_t pin, ds18x20_addr_t addr, float *temperature)
{
    CHECK_ARG(temperature);

    uint8_t scratchpad[8];
    CHECK(ds18x20_read_scratchpad(pin, addr, scratchpad));

    int16_t temp = scratchpad[1] << 8 | (scratchpad[0] & 0xfc);
    *temperature = (float)temp * 0.0625f;

    return ESP_OK;
}

esp_err_t ds18x20_read_temperature(gpio_num_t pin, ds18x20_addr_t addr, float *temperature)
{
    uint8_t family = (uint8_t)addr;
    switch (family)
    {
        case DS18X20_FAMILY_DS18S20:
            return ds18s20_read_temperature(pin, addr, temperature);
        case DS18X20_FAMILY_DS1822:
        case DS18X20_FAMILY_DS18B20:
        case DS18X20_FAMILY_DS28EA00:
            // DS1822, DS18B20 and DS28EA00 share the same 16-bit register
            // with a 0.0625°C LSB and a configurable resolution. The DS1825
            // also belongs to this group, but it shares the family code with
            // the MAX31850, see below.
            return ds18b20_read_temperature(pin, addr, temperature);
        case DS18X20_FAMILY_MAX31850:
            return max31850_read_temperature(pin, addr, temperature);
        default:
            ESP_LOGE(TAG, "Unknown sensor family %02x. Please use an explicit read/measure function", family);
    }
    return ESP_ERR_NOT_SUPPORTED;
}

static const uint8_t resolution_config[] =
{
    [DS18X20_RESOLUTION_9_BIT - DS18X20_RESOLUTION_9_BIT] = 0x1f,
    [DS18X20_RESOLUTION_10_BIT - DS18X20_RESOLUTION_9_BIT] = 0x3f,
    [DS18X20_RESOLUTION_11_BIT - DS18X20_RESOLUTION_9_BIT] = 0x5f,
    [DS18X20_RESOLUTION_12_BIT - DS18X20_RESOLUTION_9_BIT] = 0x7f,
};

esp_err_t ds18x20_get_resolution(gpio_num_t pin, ds18x20_addr_t addr, ds18x20_resolution_t *resolution)
{
    CHECK_ARG(resolution);

    uint8_t family = (uint8_t)addr;
    if (family == DS18X20_FAMILY_DS18S20 || family == DS18X20_FAMILY_MAX31850)
    {
        ESP_LOGE(TAG, "The sensor %02x has a fixed resolution", family);
        return ESP_ERR_NOT_SUPPORTED;
    }

    uint8_t scratchpad[8];
    CHECK(ds18x20_read_scratchpad(pin, addr, scratchpad));

    *resolution = (ds18x20_resolution_t)(DS18X20_RESOLUTION_9_BIT + ((scratchpad[4] >> 5) & 0x03));

    return ESP_OK;
}

esp_err_t ds18x20_set_resolution(gpio_num_t pin, ds18x20_addr_t addr, ds18x20_resolution_t resolution)
{
    CHECK_ARG(resolution >= DS18X20_RESOLUTION_9_BIT && resolution <= DS18X20_RESOLUTION_12_BIT);

    uint8_t family = (uint8_t)addr;
    if (family == DS18X20_FAMILY_DS18S20 || family == DS18X20_FAMILY_MAX31850)
    {
        ESP_LOGE(TAG, "The sensor %02x has a fixed resolution", family);
        return ESP_ERR_NOT_SUPPORTED;
    }

    uint8_t scratchpad[8];
    CHECK(ds18x20_read_scratchpad(pin, addr, scratchpad));

    // The low five bits of the configuration register are reserved and read
    // as ones, so only the resolution bits are replaced.
    scratchpad[4] = resolution_config[resolution - DS18X20_RESOLUTION_9_BIT];

    CHECK(ds18x20_write_scratchpad(pin, addr, &scratchpad[2]));
    CHECK(ds18x20_copy_scratchpad(pin, addr));

    return ESP_OK;
}

esp_err_t ds18b20_measure_and_read(gpio_num_t pin, ds18x20_addr_t addr, float *temperature)
{
    CHECK_ARG(temperature);

    CHECK(ds18x20_measure(pin, addr, true));
    return ds18b20_read_temperature(pin, addr, temperature);
}

esp_err_t ds18s20_measure_and_read(gpio_num_t pin, ds18x20_addr_t addr, float *temperature)
{
    CHECK_ARG(temperature);

    CHECK(ds18x20_measure(pin, addr, true));
    return ds18s20_read_temperature(pin, addr, temperature);
}

esp_err_t max31850_measure_and_read(gpio_num_t pin, ds18x20_addr_t addr, float *temperature)
{
    CHECK_ARG(temperature);

    CHECK(ds18x20_measure(pin, addr, true));
    return max31850_read_temperature(pin, addr, temperature);
}

esp_err_t ds18x20_measure_and_read(gpio_num_t pin, ds18x20_addr_t addr, float *temperature)
{
    CHECK_ARG(temperature);

    CHECK(ds18x20_measure(pin, addr, true));
    return ds18x20_read_temperature(pin, addr, temperature);
}

esp_err_t ds18x20_measure_and_read_multi(gpio_num_t pin, ds18x20_addr_t *addr_list, size_t addr_count, float *result_list)
{
    CHECK_ARG(result_list && addr_count);

    CHECK(ds18x20_measure(pin, DS18X20_ANY, true));

    return ds18x20_read_temp_multi(pin, addr_list, addr_count, result_list);
}

esp_err_t ds18x20_scan_devices(gpio_num_t pin, ds18x20_addr_t *addr_list, size_t addr_count, size_t *found)
{
    CHECK_ARG(addr_list && addr_count);

    *found = 0;

    wire_search_t search;
    CHECK(wire_search_start(&search, pin));

    ds18x20_addr_t addr;
    while ((addr = wire_search_next(&search)) != DS18X20_ANY)
    {
        uint8_t family_id = (uint8_t)addr;
        if (family_id == DS18X20_FAMILY_DS18S20
                || family_id == DS18X20_FAMILY_DS1822
                || family_id == DS18X20_FAMILY_DS18B20
                || family_id == DS18X20_FAMILY_MAX31850
                || family_id == DS18X20_FAMILY_DS28EA00)
        {
            if (*found < addr_count)
                addr_list[*found] = addr;
            *found += 1;
        }
    }

    wire_search_stop(&search);

    return ESP_OK;
}

esp_err_t ds18x20_read_temp_multi(gpio_num_t pin, ds18x20_addr_t *addr_list, size_t addr_count, float *result_list)
{
    CHECK_ARG(result_list);

    esp_err_t res = ESP_OK;
    for (size_t i = 0; i < addr_count; i++)
    {
        esp_err_t tmp = ds18x20_read_temperature(pin, addr_list[i], &result_list[i]);
        if (tmp != ESP_OK)
            res = tmp;
    }
    return res;
}
