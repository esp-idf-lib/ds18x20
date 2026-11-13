list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR}/eil-cmake-utils/cmake)
include(eil_check_idf_features)
eil_check_idf_has_esp_drivers()

# Both 1-Wire components define onewire_crc8() with different prototypes, so
# the linker takes the first definition it sees: the selected backend has to
# come first, otherwise the calls go to the wrong function.
if(CONFIG_DS18X20_ONEWIRE_BACKEND_ESPRESSIF)
    set(ds18x20_backends onewire_bus onewire)
else()
    set(ds18x20_backends onewire onewire_bus)
endif()

idf_component_register(
    SRCS ds18x20.c
    INCLUDE_DIRS .
    REQUIRES ${ds18x20_backends} freertos log esp_idf_lib_helpers
)

include(eil_ci)
