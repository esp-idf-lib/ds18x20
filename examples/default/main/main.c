#include <inttypes.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <ds18x20.h>
#include <esp_log.h>
#include <esp_err.h>

static const gpio_num_t SENSOR_GPIO = CONFIG_EXAMPLE_ONEWIRE_GPIO;
static const size_t MAX_SENSORS = 8;

static const char *TAG = "ds18x20_test";

static const char *sensor_type(uint8_t family_id)
{
    switch (family_id)
    {
        case DS18X20_FAMILY_DS18S20:
            return "DS18S20";
        case DS18X20_FAMILY_DS1822:
            return "DS1822";
        case DS18X20_FAMILY_DS18B20:
            return "DS18B20";
        case DS18X20_FAMILY_MAX31850:
            return "DS1825/MAX31850";
        case DS18X20_FAMILY_DS28EA00:
            return "DS28EA00";
    }
    return "Unknown";
}

// Returns true if the sensor failed before, for example because it is
// parasitically powered and the selected backend cannot hold the bus.
static bool is_rejected(ds18x20_addr_t addr, const ds18x20_addr_t *rejected, size_t rejected_count)
{
    for (size_t i = 0; i < rejected_count; i++)
        if (rejected[i] == addr)
            return true;

    return false;
}

// Scans the bus and returns the first sensor the driver can read.
static bool find_sensor(ds18x20_addr_t *addr, const ds18x20_addr_t *rejected, size_t rejected_count)
{
    ds18x20_addr_t addrs[MAX_SENSORS];
    size_t count = 0;

    esp_err_t res = ds18x20_scan_devices(SENSOR_GPIO, addrs, MAX_SENSORS, &count);
    if (res != ESP_OK)
    {
        ESP_LOGE(TAG, "Could not scan the bus: %s", esp_err_to_name(res));
        return false;
    }

    if (!count)
    {
        ESP_LOGW(TAG, "No sensors detected");
        return false;
    }

    ESP_LOGI(TAG, "%u sensors detected", (unsigned)count);

    for (size_t i = 0; i < count; i++)
    {
        if (is_rejected(addrs[i], rejected, rejected_count))
            continue;

        *addr = addrs[i];
        ESP_LOGI(TAG, "Using sensor %08" PRIx32 "%08" PRIx32 " (%s)",
                 (uint32_t)(*addr >> 32), (uint32_t)*addr,
                 sensor_type((uint8_t)*addr));
        return true;
    }

    return false;
}

void test(void *pvParameter)
{
    ds18x20_addr_t addr = 0;
    ds18x20_addr_t rejected[MAX_SENSORS];
    size_t rejected_count = 0;
    bool have_sensor = false;
    float temperature;
    esp_err_t res;

    // Make sure that the internal pull-up resistor is enabled on the GPIO pin
    // so that one can connect up a sensor without needing an external pull-up.
    // (Note: The internal (~47k) pull-ups of the ESP do appear to work, at
    // least for simple setups (one or two sensors connected with short leads),
    // but do not technically meet the pull-up requirements from the ds18x20
    // datasheet and may not always be reliable. For a real application, a proper
    // 4.7k external pull-up resistor is recommended instead!)
    gpio_set_pull_mode(SENSOR_GPIO, GPIO_PULLUP_ONLY);

    // Creates the bus when the RMT backend is selected and only validates
    // the pin when the bit-banging backend is used.
    res = ds18x20_bus_init(SENSOR_GPIO);
    if (res != ESP_OK)
    {
        ESP_LOGE(TAG, "Could not initialize the 1-Wire bus: %s", esp_err_to_name(res));
        vTaskDelete(NULL);
    }

    while (1)
    {
        if (!have_sensor)
        {
            have_sensor = find_sensor(&addr, rejected, rejected_count);
            if (!have_sensor)
            {
                vTaskDelay(pdMS_TO_TICKS(1000));
                continue;
            }
        }

        res = ds18x20_measure_and_read(SENSOR_GPIO, addr, &temperature);
        if (res == ESP_OK)
        {
            ESP_LOGI(TAG, "Sensor %08" PRIx32 "%08" PRIx32 ": %.4f°C",
                     (uint32_t)(addr >> 32), (uint32_t)addr, temperature);
        }
        else
        {
            // The sensor was removed or cannot be read with the selected
            // backend: forget it and scan the bus again.
            ESP_LOGE(TAG, "Could not read from sensor %08" PRIx32 "%08" PRIx32 ": %s",
                     (uint32_t)(addr >> 32), (uint32_t)addr, esp_err_to_name(res));
            if (rejected_count < MAX_SENSORS)
                rejected[rejected_count++] = addr;
            have_sensor = false;
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void app_main()
{
    xTaskCreate(test, TAG, configMINIMAL_STACK_SIZE * 4, NULL, 5, NULL);
}
