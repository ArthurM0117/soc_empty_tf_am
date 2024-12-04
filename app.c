#include "em_common.h"
#include "app_assert.h"
#include "sl_bluetooth.h"
#include "app.h"
#include "app_log.h"
#include <stdint.h>
#include "sl_status.h"
#include "sl_sensor_rht.h"
#include "temperature.h"
#include "gatt_db.h"
#include "sl_sleeptimer.h"
#include "sl_simple_led_instances.h"

#define TEMPERATURE_TIMER_SIGNAL (1 << 0)

// Default measurement interval in seconds
#define DEFAULT_MEASUREMENT_INTERVAL 1

static uint8_t advertising_set_handle = 0xff;
static sl_sleeptimer_timer_handle_t temperature_timer;
static bool notifications_enabled = false;
static uint8_t active_connection = 0;
static uint16_t measurement_interval = DEFAULT_MEASUREMENT_INTERVAL;

/**************************************************************************//**
 * Application Init.
 *****************************************************************************/
void app_init(void)
{
    app_log_info("%s\n", __FUNCTION__);

    // Initialize all Simple LED instances
    sl_simple_led_init_instances();
    app_log_info("LED instances initialized.\n");
}

/**************************************************************************//**
 * Application Process Action.
 *****************************************************************************/
void app_process_action(void)
{
    // Empty function, no actions required
}

/**************************************************************************//**
 * Timer Callback for Temperature Notifications.
 *****************************************************************************/
void temperature_timer_callback(sl_sleeptimer_timer_handle_t *handle, void *data)
{
    (void)handle;
    (void)data;
    sl_bt_external_signal(TEMPERATURE_TIMER_SIGNAL);
}

/**************************************************************************//**
 * Start Temperature Notifications.
 *****************************************************************************/
void start_temperature_notifications(void)
{
    sl_status_t sc;
    notifications_enabled = true;

    sc = sl_sleeptimer_start_periodic_timer_ms(
        &temperature_timer,
        measurement_interval * 1000, // Timer interval in milliseconds
        temperature_timer_callback,
        NULL,
        0,
        0);

    if (sc != SL_STATUS_OK) {
        app_log_error("Failed to start temperature timer: 0x%lX\n", sc);
    } else {
        app_log_info("Temperature notifications started with interval: %d seconds.\n", measurement_interval);
    }
}

/**************************************************************************//**
 * Stop Temperature Notifications.
 *****************************************************************************/
void stop_temperature_notifications(void)
{
    sl_status_t sc;
    notifications_enabled = false;

    sc = sl_sleeptimer_stop_timer(&temperature_timer);
    if (sc != SL_STATUS_OK) {
        app_log_error("Failed to stop temperature timer: 0x%lX\n", sc);
    } else {
        app_log_info("Temperature notifications stopped.\n");
    }
}

/**************************************************************************//**
 * Bluetooth Stack Event Handler.
 *****************************************************************************/
void sl_bt_on_event(sl_bt_msg_t *evt)
{
    sl_status_t sc;

    switch (SL_BT_MSG_ID(evt->header)) {
    case sl_bt_evt_system_boot_id:
        sc = sl_bt_advertiser_create_set(&advertising_set_handle);
        app_assert_status(sc);

        sc = sl_bt_legacy_advertiser_generate_data(advertising_set_handle,
                                                   sl_bt_advertiser_general_discoverable);
        app_assert_status(sc);

        sc = sl_bt_advertiser_set_timing(
            advertising_set_handle,
            160,
            160,
            0,
            0);
        app_assert_status(sc);

        sc = sl_bt_legacy_advertiser_start(advertising_set_handle,
                                           sl_bt_legacy_advertiser_connectable);
        app_assert_status(sc);
        break;

    case sl_bt_evt_connection_opened_id:
        app_log_info("%s: connection_opened!\n", __FUNCTION__);

        sc = sl_sensor_rht_init();
        app_assert_status(sc);
        app_log_info("RHT Sensor initialized with status: %lu\n", sc);

        active_connection = evt->data.evt_connection_opened.connection;
        break;

    case sl_bt_evt_gatt_server_user_read_request_id: {
        uint16_t characteristic = evt->data.evt_gatt_server_user_read_request.characteristic;

        if (characteristic == gattdb_measurement_interval) {
            app_log_info("Read request for Measurement Interval characteristic.\n");
            sc = sl_bt_gatt_server_send_user_read_response(
                evt->data.evt_gatt_server_user_read_request.connection,
                characteristic,
                0,
                sizeof(measurement_interval),
                (const uint8_t *)&measurement_interval,
                NULL);

            if (sc == SL_STATUS_OK) {
                app_log_info("Measurement Interval sent successfully: %d seconds.\n", measurement_interval);
            } else {
                app_log_error("Failed to send Measurement Interval: 0x%lX\n", sc);
            }
        }
        break;
    }

    case sl_bt_evt_gatt_server_user_write_request_id: {
        uint16_t characteristic = evt->data.evt_gatt_server_user_write_request.characteristic;

        if (characteristic == gattdb_measurement_interval) {
            uint8_t *written_data = evt->data.evt_gatt_server_user_write_request.value.data;

            if (evt->data.evt_gatt_server_user_write_request.value.len == sizeof(measurement_interval)) {
                measurement_interval = *(uint16_t *)written_data;
                app_log_info("Measurement Interval updated to: %d seconds.\n", measurement_interval);

                if (notifications_enabled) {
                    stop_temperature_notifications();
                    start_temperature_notifications();
                }
            } else {
                app_log_error("Invalid Measurement Interval write request.\n");
            }

            sl_bt_gatt_server_send_user_write_response(
                evt->data.evt_gatt_server_user_write_request.connection,
                characteristic,
                0);
        }
        break;
    }

    case sl_bt_evt_gatt_server_characteristic_status_id: {
        uint16_t characteristic = evt->data.evt_gatt_server_characteristic_status.characteristic;
        uint8_t status_flags = evt->data.evt_gatt_server_characteristic_status.status_flags;
        uint16_t client_config_flags = evt->data.evt_gatt_server_characteristic_status.client_config_flags;

        if (characteristic == gattdb_temperature) {
            if (status_flags & sl_bt_gatt_server_client_config) {
                if (client_config_flags & sl_bt_gatt_notification) {
                    start_temperature_notifications();
                } else {
                    stop_temperature_notifications();
                }
            }
        }
        break;
    }

    case sl_bt_evt_system_external_signal_id:
        if (evt->data.evt_system_external_signal.extsignals & TEMPERATURE_TIMER_SIGNAL) {
            if (notifications_enabled) {
                uint8_t ble_temperature[2];
                size_t temp_length;

                sc = read_and_format_temperature(ble_temperature, &temp_length);
                if (sc == SL_STATUS_OK) {
                    sc = sl_bt_gatt_server_send_notification(
                        active_connection,
                        gattdb_temperature,
                        temp_length,
                        ble_temperature);
                }
            }
        }
        break;

    case sl_bt_evt_connection_closed_id:
        sl_sensor_rht_deinit();
        stop_temperature_notifications();
        break;

    default:
        break;
    }
}
