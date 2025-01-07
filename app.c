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

static uint8_t advertising_set_handle = 0xff;
static bool notifications_enabled = false;
static sl_sleeptimer_timer_handle_t sensing_timer;

/**************************************************************************/
/* Timer Callback                                                         */
/**************************************************************************/
void sensing_timer_callback(sl_sleeptimer_timer_handle_t *handle, void *data) {
    (void)handle;
    (void)data;
    if (notifications_enabled) {
        sl_bt_external_signal(TEMPERATURE_TIMER_SIGNAL);
    }
}

/**************************************************************************/
/* Application Initialization                                             */
/**************************************************************************/
void app_init(void) {
    app_log_info("%s\n", __FUNCTION__);
    sl_sensor_rht_init();
    sl_simple_led_init_instances();
    app_log_info("Sensors and LEDs initialized.\n");
}

/**************************************************************************/
/* Start Sensing Timer                                                    */
/**************************************************************************/
void start_sensing_timer(void) {
    sl_status_t sc = sl_sleeptimer_start_periodic_timer_ms(
        &sensing_timer,
        1000, // 1 second interval
        sensing_timer_callback,
        NULL,
        0,
        0
    );
    if (sc == SL_STATUS_OK) {
        app_log_info("Sensing Timer started.\n");
    } else {
        app_log_error("Failed to start sensing timer: 0x%lX\n", sc);
    }
}

/**************************************************************************/
/* Stop Sensing Timer                                                     */
/**************************************************************************/
void stop_sensing_timer(void) {
    sl_status_t sc = sl_sleeptimer_stop_timer(&sensing_timer);
    if (sc == SL_STATUS_OK) {
        app_log_info("Sensing Timer stopped.\n");
    } else {
        app_log_error("Failed to stop sensing timer: 0x%lX\n", sc);
    }
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

    case sl_bt_evt_gatt_server_characteristic_status_id:
        if (evt->data.evt_gatt_server_characteristic_status.characteristic == gattdb_temperature) {
            app_log_info(
                "Characteristic status changed: Characteristic=%d, StatusFlags=0x%X, ClientConfigFlags=0x%X\n",
                evt->data.evt_gatt_server_characteristic_status.characteristic,
                evt->data.evt_gatt_server_characteristic_status.status_flags,
                evt->data.evt_gatt_server_characteristic_status.client_config_flags
            );

            if (evt->data.evt_gatt_server_characteristic_status.status_flags & sl_bt_gatt_server_client_config) {
                if (evt->data.evt_gatt_server_characteristic_status.client_config_flags == gatt_notification) {
                    notifications_enabled = true;
                    start_sensing_timer();
                    app_log_info("Notifications enabled for Temperature characteristic.\n");
                } else if (evt->data.evt_gatt_server_characteristic_status.client_config_flags == 0) {
                    notifications_enabled = false;
                    stop_sensing_timer();
                    app_log_info("Notifications disabled for Temperature characteristic.\n");
                }
            }
        }
        break;

    case sl_bt_evt_system_external_signal_id:
        if (evt->data.evt_system_external_signal.extsignals & TEMPERATURE_TIMER_SIGNAL) {
            uint8_t temperature_data[2];
            int32_t temperature;
            size_t temperature_len = 2;

            if (sl_sensor_rht_get(NULL, &temperature) == SL_STATUS_OK) {
                int16_t formatted_temperature = (int16_t)(temperature / 10);
                memcpy(temperature_data, &formatted_temperature, sizeof(formatted_temperature));

                sl_bt_gatt_server_send_notification(
                    evt->data.evt_gatt_server_user_read_request.connection,
                    gattdb_temperature,
                    temperature_len,
                    temperature_data
                );
                app_log_info("Temperature notification sent: %d deci-Celsius.\n", formatted_temperature);
            }
        }
        break;

    default:
        break;
    }
}
