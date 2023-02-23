#include <stdbool.h>


#include "sl_bt_api.h"
#include "src/ble.h"



#define INCLUDE_LOG_DEBUG 1
#include "src/log.h"
#include "gatt_db.h"


#define MINIMUM_ADVERTISING_INTERVAL 400
#define MAXIMUM_ADVERTISING_INTERVAL 400

#define CONNECTION_INTERVAL_MINIMUM 60
#define CONNECTION_INTERVAL_MAXIMUM 60
#define SLAVE_LATENCY 4
#define SUPERVISION_TIMEOUT 75

// BLE private data
ble_data_struct_t ble_data; // this is the declaration


// function that returns a pointer to the
// BLE private data
ble_data_struct_t* getBleDataPtr()
{

    return (&ble_data);


} // getBleDataPtr()


void handle_ble_event(sl_bt_msg_t *evt)
{
  sl_status_t sc; // status code
  switch (SL_BT_MSG_ID(evt->header))
  {
    // ******************************************************
    // Events common to both Servers and Clients
    // ******************************************************
    // --------------------------------------------------------
    // This event indicates the device has started and the radio is ready.
    // Do not call any stack API commands before receiving this boot event!
    // Including starting BT stack soft timers!
    // --------------------------------------------------------


    case sl_bt_evt_system_boot_id:
      {
        sc = sl_bt_system_get_identity_address(&ble_data.myAddress, &ble_data.myAddressType);
        if (sc != SL_STATUS_OK)
          {
            LOG_ERROR("sl_bt_system_get_identity_address() returned != 0 status=0x%04x", (unsigned int) sc);
          }

        // Create an advertising set.
        sc = sl_bt_advertiser_create_set(&ble_data.advertisingSetHandle);

        // Set advertising interval to 250ms.
        sc = sl_bt_advertiser_set_timing(ble_data.advertisingSetHandle,MINIMUM_ADVERTISING_INTERVAL,MAXIMUM_ADVERTISING_INTERVAL,0,0);

        // Start general advertising and enable connections.
        sc = sl_bt_advertiser_start(ble_data.advertisingSetHandle, sl_bt_advertiser_general_discoverable,
          sl_bt_advertiser_connectable_scannable);

      }// handle boot event

    break;


    case sl_bt_evt_connection_opened_id:
      {
        sl_bt_advertiser_stop(ble_data.advertisingSetHandle);
        sc = sl_bt_connection_set_parameters(ble_data.advertisingSetHandle,CONNECTION_INTERVAL_MINIMUM,CONNECTION_INTERVAL_MAXIMUM,
                                        SLAVE_LATENCY, SUPERVISION_TIMEOUT,0,0xffff);





        break;
      }// handle open event


    case sl_bt_evt_connection_closed_id:
      {
        // Start general advertising and enable connections.
        sc = sl_bt_advertiser_start(ble_data.advertisingSetHandle, sl_bt_advertiser_general_discoverable,
                                    sl_bt_advertiser_connectable_scannable);



        break;

      }// handle close event







    case sl_bt_evt_connection_parameters_id:
      {
        struct sl_bt_evt_connection_parameters_s param_info;
        param_info.connection=(evt->data.evt_connection_parameters.connection);
        param_info.interval=(evt->data.evt_connection_parameters.interval);
        param_info.latency=(evt->data.evt_connection_parameters.latency);
        param_info.timeout=(evt->data.evt_connection_parameters.timeout);
        param_info.security_mode=(evt->data.evt_connection_parameters.security_mode);



        break;
      }

//    case sl_bt_evt_system_external_signal_id:
//      {
//        sl_bt_gatt_server_write_attribute_value(gattdb_temperature_measurement,0,1,10);
//
//
//
//        break;
//      }



    // more case statements to handle other BT events

  } // end - switch

} // handle_ble_event()
