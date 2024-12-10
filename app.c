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

#define TEMPERATURE_TIMER_SIGNAL (1 << 0)
#define HUMIDITY_TIMER_SIGNAL (1 << 1)
#define IRRADIANCE_TIMER_SIGNAL (1 << 2)

#define DEFAULT_MEASUREMENT_INTERVAL 1

static uint8_t advertising_set_handle = 0xff;
static uint16_t measurement_interval = DEFAULT_MEASUREMENT_INTERVAL;
static sl_sleeptimer_timer_handle_t sensing_timer;
static uint8_t active_connection = 0;
static bool notifications_enabled = false;

void sensing_timer_callback(sl_sleeptimer_timer_handle_t *handle, void *data) {
    (void)handle;
    (void)data;

    if (notifications_enabled) {
        sl_bt_external_signal(TEMPERATURE_TIMER_SIGNAL);
        sl_bt_external_signal(HUMIDITY_TIMER_SIGNAL);
        sl_bt_external_signal(IRRADIANCE_TIMER_SIGNAL);
    }
}

sl_status_t read_and_format_temperature(uint8_t *temperature_data, size_t *temperature_len) {
    float temperature = 0.0f;
    sl_status_t status = sl_sensor_rht_get(&temperature, NULL);
    if (status == SL_STATUS_OK) {
        int16_t formatted_temp = (int16_t)(temperature * 100);
        memcpy(temperature_data, &formatted_temp, sizeof(int16_t));
        *temperature_len = sizeof(int16_t);
    }
    return status;
}

sl_status_t read_and_format_humidity(uint8_t *humidity_data, size_t *humidity_len) {
    float humidity = 0.0f;
    sl_status_t status = sl_sensor_rht_get(NULL, &humidity);
    if (status == SL_STATUS_OK) {
        int16_t formatted_humidity = (int16_t)(humidity * 100);
        memcpy(humidity_data, &formatted_humidity, sizeof(int16_t));
        *humidity_len = sizeof(int16_t);
    }
    return status;
}

void app_init(void) {
    app_log_info("%s\n", __FUNCTION__);
    sl_sensor_rht_init();
    sl_sensor_light_init();
    sl_simple_led_init_instances();
    app_log_info("Sensors and LEDs initialized.\n");
}

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

void stop_sensing_timer(void) {
    sl_status_t sc = sl_sleeptimer_stop_timer(&sensing_timer);
    if (sc == SL_STATUS_OK)
        app_log_info("Sensing Timer stopped.\n");
    else
        app_log_error("Failed to stop timer: 0x%lX\n", sc);
}

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
            if (read_and_format_temperature(temperature_data, &temperature_len) == SL_STATUS_OK)
                sl_bt_gatt_server_send_notification(active_connection, gattdb_temperature, temperature_len, temperature_data);
        }

        if (evt->data.evt_system_external_signal.extsignals & HUMIDITY_TIMER_SIGNAL) {
            uint8_t humidity_data[2];
            size_t humidity_len;
            if (read_and_format_humidity(humidity_data, &humidity_len) == SL_STATUS_OK)
                sl_bt_gatt_server_send_notification(active_connection, gattdb_humidity_0, humidity_len, humidity_data);
        }
        break;

    case sl_bt_evt_connection_closed_id:
        stop_sensing_timer();
        break;

    default:
        break;
    }
}
