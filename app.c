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
#include "sl_sleeptimer.h" // Include the sleep timer

// Define the signal for the temperature timer
#define TEMPERATURE_TIMER_SIGNAL (1 << 0)

// The advertising set handle allocated from Bluetooth stack.
static uint8_t advertising_set_handle = 0xff;
static sl_sleeptimer_timer_handle_t temperature_timer; // Timer handle
static bool notifications_enabled = false;            // Notification status
static uint8_t active_connection = 0;                 // Store active connection handle

/**************************************************************************//**
 * Application Init.
 *****************************************************************************/
SL_WEAK void app_init(void)
{
  app_log_info("%s\n", __FUNCTION__);
}

/**************************************************************************//**
 * Application Process Action.
 *****************************************************************************/
SL_WEAK void app_process_action(void)
{
  // This function is called infinitely. Add additional application code here.
}

/**************************************************************************//**
 * Temperature Timer Callback.
 *****************************************************************************/
void temperature_timer_callback(sl_sleeptimer_timer_handle_t *handle, void *data)
{
  (void)handle;
  (void)data;

  // Signal to process the timer event in the main loop
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
      1000, // Timer interval in milliseconds (1 second)
      temperature_timer_callback,
      NULL,
      0,
      0
  );

  if (sc != SL_STATUS_OK) {
    app_log_error("Failed to start temperature timer: 0x%lX\n", sc);
  } else {
    app_log_info("Temperature notifications started.\n");
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
 * Bluetooth stack event handler.
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

      // Store the active connection handle
      active_connection = evt->data.evt_connection_opened.connection;
      break;

    case sl_bt_evt_gatt_server_user_read_request_id: {
        uint8_t ble_temperature[2];
        size_t temp_length;

        app_log_info("Read request for characteristic: %d\n",
                     evt->data.evt_gatt_server_user_read_request.characteristic);

        if (evt->data.evt_gatt_server_user_read_request.characteristic == gattdb_temperature) {
            sc = read_and_format_temperature(ble_temperature, &temp_length);
            if (sc == SL_STATUS_OK) {
                sc = sl_bt_gatt_server_send_user_read_response(
                    evt->data.evt_gatt_server_user_read_request.connection,
                    evt->data.evt_gatt_server_user_read_request.characteristic,
                    0,
                    temp_length,
                    ble_temperature,
                    NULL);
                if (sc == SL_STATUS_OK) {
                    app_log_info("Temperature sent successfully.\n");
                }
            }
        }
        break;
    }

    case sl_bt_evt_gatt_server_characteristic_status_id: {
        uint16_t characteristic = evt->data.evt_gatt_server_characteristic_status.characteristic;
        uint8_t status_flags = evt->data.evt_gatt_server_characteristic_status.status_flags;
        uint16_t client_config_flags = evt->data.evt_gatt_server_characteristic_status.client_config_flags;

        // Log the event details
        app_log_info("Characteristic status changed: Characteristic=%d, StatusFlags=0x%X, ClientConfigFlags=0x%X\n",
                     characteristic, status_flags, client_config_flags);

        if (characteristic == gattdb_temperature) {
            app_log_info("Status change relates to the Temperature characteristic.\n");

            if (status_flags & sl_bt_gatt_server_client_config) {
                if (client_config_flags & sl_bt_gatt_notification) {
                    app_log_info("Notifications enabled for Temperature characteristic.\n");
                    start_temperature_notifications();
                } else {
                    app_log_info("Notifications disabled for Temperature characteristic.\n");
                    stop_temperature_notifications();
                }
            }
        }
        break;
    }

    case sl_bt_evt_system_external_signal_id: {
        // Check if the signal matches the temperature timer signal
        if (evt->data.evt_system_external_signal.extsignals & TEMPERATURE_TIMER_SIGNAL) {
            if (notifications_enabled) {
                uint8_t ble_temperature[2];
                size_t temp_length;

                // Read and format the temperature
                sc = read_and_format_temperature(ble_temperature, &temp_length);
                if (sc == SL_STATUS_OK) {
                    // Send the notification
                    sc = sl_bt_gatt_server_send_notification(
                        active_connection,
                        gattdb_temperature,
                        temp_length,
                        ble_temperature
                    );

                    if (sc == SL_STATUS_OK) {
                        app_log_info("Temperature notification sent successfully.\n");
                    }
                }
            }
        }
        break;
    }

    case sl_bt_evt_connection_closed_id:
      app_log_info("%s: connection_closed!\n", __FUNCTION__);

      sl_sensor_rht_deinit();
      app_log_info("RHT Sensor deinitialized\n");

      sc = sl_bt_legacy_advertiser_generate_data(advertising_set_handle,
                                                 sl_bt_advertiser_general_discoverable);
      app_assert_status(sc);

      sc = sl_bt_legacy_advertiser_start(advertising_set_handle,
                                         sl_bt_legacy_advertiser_connectable);
      app_assert_status(sc);

      stop_temperature_notifications();
      break;

    default:
      break;
  }
}
