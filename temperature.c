#include <stdint.h>
#include <stddef.h>
#include "temperature.h"
#include "sl_sensor_rht.h"
#include "app_log.h"

sl_status_t read_and_format_temperature(uint8_t *ble_temperature, size_t *length) {
    int32_t raw_temp = 0; // Température brute
    sl_status_t status;

    // Lecture de la température brute depuis le capteur
    status = sl_sensor_rht_get(NULL, &raw_temp); // NULL pour ignorer l'humidité
    if (status != SL_STATUS_OK) {
        app_log_error("Erreur de lecture du capteur RHT: %lu\n", status);
        return status;
    }

    // Conversion de la température brute (en milli-degrés Celsius) en float
    float temp_celsius = (float) raw_temp / 1000;

    // Conversion en format BLE (int16_t, 0.01°C)
    int16_t ble_temp_value = (int16_t)(temp_celsius * 100);
    ble_temperature[0] = ble_temp_value & 0xFF;       // Octet faible
    ble_temperature[1] = (ble_temp_value >> 8) & 0xFF; // Octet fort

    *length = 2; // Longueur des données BLE


    app_log_info("Valeur brute de temperature : %ld\n", raw_temp);




    return SL_STATUS_OK;
}
