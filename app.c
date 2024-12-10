#include "em_common.h"
#include "app_assert.h"
#include "sl_bluetooth.h"
#include "app.h"
#include "app_log.h"
#include "sl_status.h"
#include "sl_sensor_rht.h"
#include "sl_sensor_light.h"
#include "sl_simple_led_instances.h"
#include "sl_sleeptimer.h"
#include "gatt_db.h"
#include "temperature.h" // Include temperature header

#define TEMPERATURE_TIMER_SIGNAL (1 << 0)
#define HUMIDITY_TIMER_SIGNAL (1 << 1)
#define IRRADIANCE_TIMER_SIGNAL (1 << 2)

#define DEFAULT_MEASUREMENT_INTERVAL 1

static uint8_t advertising_set_handle = 0xff;
static uint16_t measurement_interval = DEFAULT_MEASUREMENT_INTERVAL;
static sl_sleeptimer_timer_handle_t sensing_timer;
static uint8_t active_connection = 0;
static bool notifications_enabled = false;

/**************************************************************************/
/* Timer Callback                                                         */
/**************************************************************************/
void sensing_timer_callback(sl_sleeptimer_timer_handle_t *handle, void *data) {
    (void)handle;
    (void)data;

    if (notifications_enabled) {
        sl_bt_external_signal(TEMPERATURE_TIMER_SIGNAL | HUMIDITY_TIMER_SIGNAL | IRRADIANCE_TIMER_SIGNAL);
    }
}

/**************************************************************************/
/* Humidity Sensor Read and Format                                        */
/**************************************************************************/
sl_status_t read_and_format_humidity(uint8_t *humidity_data, size_t *humidity_len) {
    uint32_t humidity = 0;
    sl_status_t status = sl_sensor_rht_get(&humidity, NULL);
    if (status == SL_STATUS_OK) {
        int16_t formatted_humidity = (int16_t)(humidity / 100); // Convert to percentage
        memcpy(humidity_data, &formatted_humidity, sizeof(int16_t));
        *humidity_len = sizeof(int16_t);
    }
    return status;
}

/**************************************************************************/
/* Irradiance Sensor Read and Format                                      */
/**************************************************************************/
sl_status_t read_and_format_irradiance(uint8_t *irradiance_data, size_t *irradiance_len) {
    float lux = 0, uvi = 0;
    sl_status_t status = sl_sensor_light_get(&lux, &uvi);
    if (status == SL_STATUS_OK) {
        int16_t formatted_irradiance = (int16_t)lux;
        memcpy(irradiance_data, &formatted_irradiance, sizeof(int16_t));
        *irradiance_len = sizeof(int16_t);
    }
    return status;
}

/**************************************************************************/
/* Application Initialization                                             */
/**************************************************************************/
void app_init(void) {
    app_log_info("%s\n", __FUNCTION__);
    sl_sensor_rht_init();
    sl_sensor_light_init();
    sl_simple_led_init_instances();
    app_log_info("Sensors and LEDs initialized.\n");
}

/**************************************************************************/
/* Start Sensing Timer                                                    */
/**************************************************************************/
void start_sensing_timer(void) {
    sl_status_t sc = sl_sleeptimer_start_periodic_timer_ms(
        &sensing_timer,
        measurement_interval * 1000,
        sensing_timer_callback,
        NULL,
        0,
        0
    );
    if (sc == SL_STATUS_OK)
        app_log_info("Sensing Timer started with interval: %d seconds.\n", measurement_interval);
    else
        app_log_error("Failed to start timer: 0x%lX\n", sc);
}

/**************************************************************************/
/* Stop Sensing Timer                                                     */
/**************************************************************************/
void stop_sensing_timer(void) {
    sl_status_t sc = sl_sleeptimer_stop_timer(&sensing_timer);
    if (sc == SL_STATUS_OK)
        app_log_info("Sensing Timer stopped.\n");
    else
        app_log_error("Failed to stop timer: 0x%lX\n", sc);
}

/**************************************************************************/
/* Bluetooth Event Handler                                                */
/**************************************************************************/
void sl_bt_on_event(sl_bt_msg_t *evt) {
    switch (SL_BT_MSG_ID(evt->header)) {
    case sl_bt_evt_system_boot_id:
        sl_bt_advertiser_create_set(&advertising_set_handle);
        sl_bt_legacy_advertiser_start(advertising_set_handle, sl_bt_legacy_advertiser_connectable);
        break;

    case sl_bt_evt_connection_opened_id:
        active_connection = evt->data.evt_connection_opened.connection;
        break;

    case sl_bt_evt_gatt_server_user_read_request_id:
        if (evt->data.evt_gatt_server_user_read_request.characteristic == gattdb_measurement_interval) {
            sl_bt_gatt_server_send_user_read_response(
                active_connection,
                gattdb_measurement_interval,
                0,
                sizeof(measurement_interval),
                (const uint8_t *)&measurement_interval,
                NULL
            );
        }
        break;

    case sl_bt_evt_gatt_server_user_write_request_id:
        if (evt->data.evt_gatt_server_user_write_request.characteristic == gattdb_measurement_interval) {
            uint16_t new_interval = *(uint16_t *)evt->data.evt_gatt_server_user_write_request.value.data;
            if (new_interval > 0) {
                measurement_interval = new_interval;
                stop_sensing_timer();
                start_sensing_timer();
                app_log_info("Measurement Interval updated to: %d seconds.\n", measurement_interval);
            }
            sl_bt_gatt_server_send_user_write_response(active_connection, gattdb_measurement_interval, 0);
        }
        break;

    case sl_bt_evt_system_external_signal_id:
        if (evt->data.evt_system_external_signal.extsignals & TEMPERATURE_TIMER_SIGNAL) {
            uint8_t temperature_data[2];
            size_t temperature_len;
            if (read_and_format_temperature(temperature_data, &temperature_len) == SL_STATUS_OK) {
                sl_bt_gatt_server_send_notification(active_connection, gattdb_temperature, temperature_len, temperature_data);
                int16_t temperature_value = *((int16_t *)temperature_data);
                app_log_info("Temperature notification sent: %d deci-degree Celsius\n", temperature_value);
            }
        }

        if (evt->data.evt_system_external_signal.extsignals & HUMIDITY_TIMER_SIGNAL) {
            uint8_t humidity_data[2];
            size_t humidity_len;
            if (read_and_format_humidity(humidity_data, &humidity_len) == SL_STATUS_OK) {
                sl_bt_gatt_server_send_notification(active_connection, gattdb_humidity_0, humidity_len, humidity_data);
                int16_t humidity_value = *((int16_t *)humidity_data);
                app_log_info("Humidity notification sent: %d %%\n", humidity_value);
            }
        }

        if (evt->data.evt_system_external_signal.extsignals & IRRADIANCE_TIMER_SIGNAL) {
            uint8_t irradiance_data[2];
            size_t irradiance_len;
            if (read_and_format_irradiance(irradiance_data, &irradiance_len) == SL_STATUS_OK) {
                sl_bt_gatt_server_send_notification(active_connection, gattdb_irradiance_0, irradiance_len, irradiance_data);
                int16_t irradiance_value = *((int16_t *)irradiance_data);
                app_log_info("Irradiance notification sent: %d lux\n", irradiance_value);
            }
        }
        break;

    case sl_bt_evt_connection_closed_id:
        stop_sensing_timer();
        break;

    default:
        break;
    }
}

/**************************************************************************/
/* Placeholder for app_process_action                                     */
/**************************************************************************/
void app_process_action(void) {
    // Placeholder function to resolve undefined reference error
}
