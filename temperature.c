#include <stdint.h>
#include <stddef.h>
#include "temperature.h"
#include "sl_sensor_rht.h"
#include "app_log.h"

/**************************************************************************/
/* Read and Format Temperature                                            */
/**************************************************************************/
sl_status_t read_and_format_temperature(uint8_t *temperature_data, size_t *temperature_len) {
    int32_t raw_temp = 0; // Raw temperature in milli-degrees Celsius
    sl_status_t status;

    // Read raw temperature from the sensor
    status = sl_sensor_rht_get(NULL, &raw_temp); // NULL for ignoring humidity
    if (status != SL_STATUS_OK) {
        app_log_error("Failed to read RHT sensor: 0x%lX\n", status);
        return status;
    }

    // Convert raw temperature to BLE format (int16_t in 0.01°C resolution)
    int16_t ble_temp_value = (int16_t)(raw_temp / 10); // Convert to deci-degrees Celsius
    temperature_data[0] = ble_temp_value & 0xFF;       // Low byte
    temperature_data[1] = (ble_temp_value >> 8) & 0xFF; // High byte

    *temperature_len = 2; // Length of BLE data

    app_log_info("Raw temperature value: %ld\n", raw_temp);
    app_log_info("Formatted BLE temperature: 0x%02X%02X\n", temperature_data[1], temperature_data[0]);

    return SL_STATUS_OK;
}
