#ifndef TEMPERATURE_H
#define TEMPERATURE_H

#include <stdint.h>
#include <sl_status.h>
#include <stddef.h>


// Prototype de la fonction pour lire et formater la température
sl_status_t read_and_format_temperature(uint8_t *ble_temperature, size_t *length);

#endif // TEMPERATURE_H
