#include <stdbool.h>
#include "src/lcd.h"

#include "sl_bt_api.h"
#include "src/ble_device_type.h"
#include "src/ble.h"
#include "scheduler.h"


#define INCLUDE_LOG_DEBUG 1
#include "src/log.h"
#include "gatt_db.h"
#include "src/gpio.h"


#define MINIMUM_ADVERTISING_INTERVAL 400
#define MAXIMUM_ADVERTISING_INTERVAL 400

#define CONNECTION_INTERVAL_MINIMUM 60
#define CONNECTION_INTERVAL_MAXIMUM 60
#define SLAVE_LATENCY 4
#define SUPERVISION_TIMEOUT 150


#define SOFT_TIMER_DURATION 32768
#define PB0PRESS 8
#define PB0RELEASE 16
#define PB1PRESS 32


uint32_t gattdbData;
uint32_t actual_temp_local;
uint8_t state[2];
int event;
bool read_in_flight=0;
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
  ble_data.myAddressType=1;
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


#if DEVICE_IS_BLE_SERVER
    case sl_bt_evt_system_boot_id:
      {

        state[0]=0x00;
        sl_bt_sm_delete_bondings();
        ble_data.indication_in_flight=0;
        ble_data.connection_open=0;
        ble_data.ok_to_send_htm_indications=0;
        ble_data.ok_to_send_button_indications=0;
        ble_data.isBonded=0;
        gpioLed0SetOff();
        gpioLed1SetOff();

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
        if (sc != SL_STATUS_OK)
          {
            LOG_ERROR("sl_bt_advertiser_start() returned != 0 status=0x%04x", (unsigned int) sc);
          }

        displayInit();                                //init the LCD display
        displayPrintf(DISPLAY_ROW_NAME, "Attendance");
        displayPrintf(DISPLAY_ROW_BTADDR, "Monitoring");
        displayPrintf(DISPLAY_ROW_ASSIGNMENT, "Server");
        displayPrintf(DISPLAY_ROW_4, "Advertising",0);//print current state

        sl_bt_sm_configure(0x0F, sm_io_capability_displayyesno);
        sl_status_t          timer_response;
        timer_response = sl_bt_system_set_soft_timer(SOFT_TIMER_DURATION,1,0);// set to 1s, handler 1 and recurring mode
        if (timer_response != SL_STATUS_OK) {
            LOG_ERROR("Timer soft error");
         }




      }// handle boot event

    break;






    case sl_bt_evt_connection_opened_id:
      {

        ble_data.connectionHandle=evt->data.evt_connection_opened.connection;

        sc = sl_bt_connection_set_parameters(ble_data.connectionHandle,CONNECTION_INTERVAL_MINIMUM,CONNECTION_INTERVAL_MAXIMUM,
                                        SLAVE_LATENCY, SUPERVISION_TIMEOUT,0,0xffff);
        if (sc != SL_STATUS_OK)
        {
          LOG_ERROR("sl_bt_connection_set_parameters() returned != 0 status=0x%04x", (unsigned int) sc);
        }
        sc= sl_bt_advertiser_stop(ble_data.advertisingSetHandle);
        if (sc != SL_STATUS_OK)
        {
          LOG_ERROR("sl_bt_advertiser_stop() returned != 0 status=0x%04x", (unsigned int) sc);
        }
        ble_data.connection_open=1;
        ble_data.indication_in_flight=0;

        displayPrintf(DISPLAY_ROW_4, "Connected",0);//print current state
        //sl_bt_sm_configure(0b00001111, sm_io_capability_displayyesno);

        break;
      }// handle open event

    case sl_bt_evt_sm_confirm_bonding_id:
      ble_data.connectionHandle=evt->data.evt_sm_confirm_bonding.connection;
        sl_bt_sm_bonding_confirm(ble_data.connectionHandle,true);
        displayPrintf(DISPLAY_ROW_5, " ",0);//print current state
        displayPrintf(DISPLAY_ROW_6, " ",0);//print current state
        displayPrintf(DISPLAY_ROW_9, " ",0);//print current state
        break;



    case sl_bt_evt_sm_confirm_passkey_id:
        {
          ble_data.connectionHandle=evt->data.evt_sm_confirm_passkey.connection;
        uint32_t key;
        key=evt->data.evt_sm_confirm_passkey.passkey;

        displayPrintf(DISPLAY_ROW_5, "Passkey %d",key);//print current state
        displayPrintf(DISPLAY_ROW_6, "Confirm with PB0",0);//print current state

        }
        break;

    case sl_bt_evt_sm_bonded_id:
        {
          ble_data.connectionHandle=evt->data.evt_sm_bonded.connection;
                displayPrintf(DISPLAY_ROW_5, " ",0);//print current state
                displayPrintf(DISPLAY_ROW_6, " ",0);//print current state
                displayPrintf(DISPLAY_ROW_4, "Bonded",0);//print current state
                ble_data.isBonded=1;
        }
      break;


    case sl_bt_evt_gatt_server_attribute_value_id:

      if((evt->data.evt_gatt_server_attribute_value.attribute==gattdb_attendance_data_c) )
        {


      employee_report_table[0].status=evt->data.evt_gatt_server_attribute_value.value.data[0];
      employee_report_table[1].status=evt->data.evt_gatt_server_attribute_value.value.data[1];
      employee_report_table[2].status=evt->data.evt_gatt_server_attribute_value.value.data[2];
      employee_report_table[0].Payroll=evt->data.evt_gatt_server_attribute_value.value.data[3];
      employee_report_table[1].Payroll=evt->data.evt_gatt_server_attribute_value.value.data[4];
      employee_report_table[2].Payroll=evt->data.evt_gatt_server_attribute_value.value.data[5];
      //displayPrintf(2,"%d %d %d", employee_report_table[0].Payroll, employee_report_table[1].Payroll, employee_report_table[2].Payroll);
      for(int i=0; i<3 ;i++)
         {
           if(employee_report_table[i].status == 2)
             {
               memset(&employee_report_table[i].attendance_status[0], 0, 11);
               strcpy(employee_report_table[i].attendance_status, "Out");
             }
           else if(employee_report_table[i].status == 1)
             {
               memset(&employee_report_table[i].attendance_status[0], 0, 11);
               strcpy(employee_report_table[i].attendance_status, "In ");
               ble_data.headcount++;
             }
           else
             {
               memset(&employee_report_table[i].attendance_status[0], 0, 11);
               strcpy(employee_report_table[i].attendance_status, "Abs");
             }
       }


       displayPrintf(DISPLAY_ROW_NAME, "Attendance");
       displayPrintf(DISPLAY_ROW_BTADDR, "Monitoring");
       displayPrintf(DISPLAY_ROW_3, "Manager Access",0);// change row

       displayPrintf(5, "Headcount = %d", ble_data.headcount);
       displayPrintf(DISPLAY_ROW_8, " %d      %s     $%d", employee_report_table[0].employee_id, employee_report_table[0].attendance_status, employee_report_table[0].Payroll);
       displayPrintf(9, " %d      %s     $%d", employee_report_table[1].employee_id, employee_report_table[1].attendance_status, employee_report_table[1].Payroll);
       displayPrintf(10, " %d      %s     $%d", employee_report_table[2].employee_id, employee_report_table[2].attendance_status, employee_report_table[2].Payroll);
       displayPrintf(6, "EMPID ATTENDANCE PAY");
       displayPrintf(7, "----- ---------- ---");

       ble_data.headcount=0;
        }

//      if(evt->data.evt_gatt_server_attribute_value.attribute==gattdb_wages)
//        {
//          employee_report_table[0].Payroll=evt->data.evt_gatt_server_attribute_value.value.data[0];
//          employee_report_table[1].Payroll=evt->data.evt_gatt_server_attribute_value.value.data[1];
//          employee_report_table[2].Payroll=evt->data.evt_gatt_server_attribute_value.value.data[2];
//
//
//
////          displayPrintf(DISPLAY_ROW_NAME, "Attendance");
////          displayPrintf(DISPLAY_ROW_BTADDR, "Monitoring");
////          displayPrintf(DISPLAY_ROW_3, "Manager Access",0);// change row
//          displayPrintf(DISPLAY_ROW_4, "Headcount = %d", ble_data.headcount);
////
//          displayPrintf(DISPLAY_ROW_6,  "EMPID ATTENDANCE    ");
//          displayPrintf(DISPLAY_ROW_7,  "----- ---------- ---");
//          displayPrintf(DISPLAY_ROW_8,  " %d       %s    %d ", employee_report_table[0].employee_id, employee_report_table[0].attendance_status,employee_report_table[0].Payroll);
//          displayPrintf(DISPLAY_ROW_9,  " %d       %s    %d ", employee_report_table[1].employee_id, employee_report_table[1].attendance_status,employee_report_table[1].Payroll);
//          displayPrintf(DISPLAY_ROW_10, " %d       %s    %d ", employee_report_table[2].employee_id, employee_report_table[2].attendance_status,employee_report_table[2].Payroll);
////          displayPrintf(DISPLAY_ROW_ASSIGNMENT, "Server");
//          ble_data.headcount=0;
  //      }
      break;


    case sl_bt_evt_system_external_signal_id:
      event=evt->data.evt_system_external_signal.extsignals;
      if(event==PB0PRESS || event==PB0RELEASE)
        {
          //displayPrintf(5,"ext event", 0);

      //LOG_INFO("inside external signal\n\r");



      //for bonding
      if(ble_data.isBonded==0)
        {


          if(event==PB0PRESS)
            {
              //displayPrintf(DISPLAY_ROW_9, "Button Pressed",0);//print current state
              sl_bt_sm_passkey_confirm(ble_data.connectionHandle,true);//confirm passkey
            }
          else if(event==PB0RELEASE)
            {
              //displayPrintf(DISPLAY_ROW_9, "Button Released",0);//print current state
            }

        }


      //for sending button state
      else
      {
          gpioLed1SetOff();


          if(event==PB0PRESS)
            {

              //sendPayrollIndication();
              //displayPrintf(DISPLAY_ROW_9, "Button Pressed",0);//print current state
              state[1]=0x01;
              uint8_t temp_btn_state=1;
              sl_status_t sc = sl_bt_gatt_server_write_attribute_value(
              gattdb_button_state, // handle from gatt_db.h
              0,                              // offset
              1, // length
              &temp_btn_state);    // pointer to buffer where data is
              if (sc != SL_STATUS_OK)
                {
                   LOG_ERROR("sl_bt_gatt_server_write_attribute_value() returned != 0 status=0x%04x", (unsigned int) sc);
                }

            }
      }
  }
//          else if(event==PB0RELEASE)
//            {
//              //displayPrintf(DISPLAY_ROW_9, "Button Released",0);//print current state
//              state[1]=0x00;
//              uint8_t temp_btn_state=0;
//              sl_status_t sc = sl_bt_gatt_server_write_attribute_value(
//             gattdb_button_state, // handle from gatt_db.h
//             0,                              // offset
//             1, // length
//             &temp_btn_state);    // pointer to buffer where data is
//             if (sc != SL_STATUS_OK)
//               {
//                  LOG_ERROR("sl_bt_gatt_server_write_attribute_value() returned != 0 status=0x%04x", (unsigned int) sc);
//               }
 //           }


//          if(ble_data.connection_open==1 && ble_data.ok_to_send_button_indications==1 &&
//             ble_data.isBonded==1 && ble_data.indication_in_flight==0)
//             {
//
//               state[1]=0x01;
//               //sendPayrollIndication();
//               sc = sl_bt_gatt_server_send_indication(ble_data.connectionHandle,
//                                                      gattdb_button_state,
//                                                      sizeof(state),
//                                                      &state[0]);
//               if (sc != SL_STATUS_OK)
//                {
//                   LOG_ERROR("sl_bt_advertiser_start release() returned != 0 status=0x%04x", (unsigned int) sc);
//                }
//
//               else
//                 ble_data.indication_in_flight=1;
//             }
//           else if(ble_data.connection_open==1 && ble_data.ok_to_send_button_indications==1 &&
//                   ble_data.isBonded==1 && ble_data.indication_in_flight==1)
//             {
//               //LOG_INFO("write queue in button\n\r");
//               //write_queue(gattdb_button_state,sizeof(state),state);
//             }
//
//          }
//
//
//
//        }
      break;


    case sl_bt_evt_sm_bonding_failed_id:
      ble_data.connectionHandle=evt->data.evt_sm_bonding_failed.connection;
      sl_bt_sm_bonding_confirm(ble_data.connectionHandle,true);
      break;

    case sl_bt_evt_connection_closed_id:
      {
        ble_data.connectionHandle=evt->data.evt_connection_closed.connection;
        // Start general advertising and enable connections.
        sc = sl_bt_advertiser_start(ble_data.advertisingSetHandle, sl_bt_advertiser_general_discoverable,
                                    sl_bt_advertiser_connectable_scannable);

        if (sc != SL_STATUS_OK)
        {
          LOG_ERROR("sl_bt_advertiser_start() returned != 0 status=0x%04x", (unsigned int) sc);
        }
        ble_data.connection_open=0;
        ble_data.indication_in_flight=0;

        displayPrintf(DISPLAY_ROW_7, "",0);
        displayPrintf(DISPLAY_ROW_4, "Advertising",0);//print current state
        sl_bt_sm_delete_bondings();
        displayPrintf(DISPLAY_ROW_5, " ",0);//print current state
        displayPrintf(DISPLAY_ROW_6, " ",0);//print current state


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
//        LOG_INFO("/rconnection %d interval %d latency %d timeout %d security %d/r/n",param_info.connection,param_info.interval,param_info.latency,param_info.timeout,param_info.security_mode);
        (void) param_info;
      }


        break;


    case sl_bt_evt_gatt_server_characteristic_status_id:

      {
      if(evt->data.evt_gatt_server_characteristic_status.characteristic==gattdb_button_state)
        {
          if(evt->data.evt_gatt_server_characteristic_status.status_flags == sl_bt_gatt_server_client_config )
          {
            if(evt->data.evt_gatt_server_characteristic_status.client_config_flags==gatt_indication )
             {
                ble_data.ok_to_send_button_indications=1;
                gpioLed1SetOn();
             }
            else if(evt->data.evt_gatt_server_characteristic_status.client_config_flags==gatt_disable )
              {
                ble_data.ok_to_send_button_indications=0;
                gpioLed1SetOff();
              }
          }
        }

      if(evt->data.evt_gatt_server_characteristic_status.characteristic==gattdb_temperature_measurement)
        {

          if(evt->data.evt_gatt_server_characteristic_status.status_flags == sl_bt_gatt_server_client_config )
            {
              if(evt->data.evt_gatt_server_characteristic_status.client_config_flags==gatt_indication )
                {
                  ble_data.ok_to_send_htm_indications=1;
                  gpioLed0SetOn();

                }
              else if(evt->data.evt_gatt_server_characteristic_status.client_config_flags==gatt_disable )
                {
                  ble_data.ok_to_send_htm_indications=0;
                  gpioLed0SetOff();
                }

             }
         }

        if(evt->data.evt_gatt_server_characteristic_status.status_flags==sl_bt_gatt_server_confirmation)
          {
            ble_data.indication_in_flight=0;
          }

        if(ble_data.ok_to_send_htm_indications)
          {
            //displayPrintf(DISPLAY_ROW_TEMPVALUE, "Temp=%d C",actual_temp_local);
          }
        else
          {
            //displayPrintf(DISPLAY_ROW_TEMPVALUE, "",0);
          }

      }

          break;

    case sl_bt_evt_gatt_server_indication_timeout_id:

      ble_data.indication_in_flight=0;
      break;



////////////////////common/////////////

    case sl_bt_evt_system_soft_timer_id:
      //LOG_INFO("queue_depth %d\n\r",get_queue_depth());
      displayUpdate();
//      uint16_t conHand;
//      size_t conSize;
//      uint8_t conBuff[5];
//      if(get_queue_depth()!=0 && ble_data.indication_in_flight==0 &&
//          (ble_data.ok_to_send_button_indications==1 || ble_data.ok_to_send_htm_indications))
//        {

//          LOG_INFO(" Inside dequeue %d\n\r",get_queue_depth());
//          read_queue(&conHand,&conSize,&conBuff[0]);
//          sc = sl_bt_gatt_server_send_indication(ble_data.connectionHandle,
//                                                 conHand,
//                                                 conSize,
//                                                 &conBuff[0]);
//          if (sc != SL_STATUS_OK)
//            {
//               LOG_ERROR("sl_bt_gatt_server_send_indication in timer() returned != 0 status=0x%04x", (unsigned int) sc);
//            }
//          else
//            ble_data.indication_in_flight=1;
//        }


      break;

#endif

  } // end - switch

} // handle_ble_event()





void updateGATTDB(uint32_t actual_temp)
{





  actual_temp_local=actual_temp;
  uint8_t htm_temperature_buffer[5];
  uint8_t *p = htm_temperature_buffer;
  uint32_t htm_temperature_flt;
  uint8_t flags =0x00;

  UINT8_TO_BITSTREAM(p, flags);
  htm_temperature_flt = UINT32_TO_FLOAT(actual_temp*1000, -3);
  UINT32_TO_BITSTREAM(p, htm_temperature_flt);


  sl_status_t sc = sl_bt_gatt_server_write_attribute_value(
    gattdb_temperature_measurement, // handle from gatt_db.h
    0,                              // offset
    sizeof(htm_temperature_buffer), // length
    &htm_temperature_buffer[0]);    // pointer to buffer where data is

  if (sc != SL_STATUS_OK)
           LOG_ERROR("GATT DB WRITE ERROR");

  if(ble_data.connection_open==1 && ble_data.ok_to_send_htm_indications==1
      && ble_data.indication_in_flight==0)
    {
      sc = sl_bt_gatt_server_send_indication(ble_data.connectionHandle,
                                                   gattdb_temperature_measurement,
                                                   sizeof(htm_temperature_buffer),
                                                   &htm_temperature_buffer[0]);
      ble_data.indication_in_flight=1;
      if (sc != SL_STATUS_OK)
               LOG_ERROR("GATT INDICATION WRITE ERROR");

    }

}


