#include "app.h"
#include "em_core.h"
#include "sl_power_manager.h"
#include "src/i2c.h"
#include "src/timers.h"
#define INCLUDE_LOG_DEBUG 1
#include "src/log.h"
#include "sl_bt_api.h"
#include "src/scheduler.h"
#include "src/ble.h"
#include "gatt_db.h"
#include "lcd.h"
#include "sl_udelay.h"

#define SECONDS *1000*1000
#define WAGES 5

// DOS: If the caller needs these, shouldn't they be defined in the .h file ???? !!!!!
#define CAPTURE_FLAG        1
#define IDENTIFY_FLAG        2



#define PB0PRESS 8
#define PB0RELEASE 16
#define PB1PRESS 32
#define PB1RELEASE 128
#define TOGGLEINDICATION 64

//#define I2C_COMPLETE_EVENT 4

ble_data_struct_t *ble_data2;

enum myStates_t
{
  IDLE,
  WAIT_FOR_FINGERPRINT,
  IDENTIFY_FINGERPRINT,
  LOG_ATTENDANCE,
  PAYROLL_DISPLAY

};

uint16_t nextEvent=1;
uint8_t i=0;
bool readOp=1;
bool writeOp=0;
uint32_t temperature=0;


uint16_t eventLog=0;

uint8_t headcount=0;

extern uint8_t receiveAck;
extern uint8_t fingerID;

/**************************************************************************//**
 * Function to set the event for read temperature every 3s
 *****************************************************************************/


/**************************************************************************//**
 * Function to set the event for button press
 *****************************************************************************/
void SetPB0Press()
{CORE_DECLARE_IRQ_STATE;
CORE_ENTER_CRITICAL();
  sl_bt_external_signal(PB0PRESS);
  CORE_EXIT_CRITICAL();
}



/**************************************************************************//**
 * Function to set the event for button release
 *****************************************************************************/
void SetPB0Release()
{
  CORE_DECLARE_IRQ_STATE;
  CORE_ENTER_CRITICAL();
  sl_bt_external_signal(PB0RELEASE);
  CORE_EXIT_CRITICAL();
}



/**************************************************************************//**
 * Function to set the event for button press
 *****************************************************************************/
void SetPB1Press()
{CORE_DECLARE_IRQ_STATE;
CORE_ENTER_CRITICAL();
  sl_bt_external_signal(PB1PRESS);
  CORE_EXIT_CRITICAL();
}



/**************************************************************************//**
 * Function to set the event for button release
 *****************************************************************************/
void SetPB1Release()
{
  CORE_DECLARE_IRQ_STATE;
  CORE_ENTER_CRITICAL();
  sl_bt_external_signal(PB1RELEASE);
  CORE_EXIT_CRITICAL();
}


/**************************************************************************//**
 * Function to set the event for indication toggle
 *****************************************************************************/
void setEventToggleIndication()
{
  CORE_DECLARE_IRQ_STATE;
  CORE_ENTER_CRITICAL();
  sl_bt_external_signal(TOGGLEINDICATION);
  GPIO_PinOutToggle(5,4);
  CORE_EXIT_CRITICAL();
}




void setCaptureEvent()
{

  CORE_DECLARE_IRQ_STATE;
  CORE_ENTER_CRITICAL();
  sl_bt_external_signal(CAPTURE_FLAG);
  CORE_EXIT_CRITICAL();

 }



void setIdentifyEvent()
{

  CORE_DECLARE_IRQ_STATE;
  CORE_ENTER_CRITICAL();
  sl_bt_external_signal(IDENTIFY_FLAG);
  CORE_EXIT_CRITICAL();

 }


//void setUFEvent()
//{
// CORE_DECLARE_IRQ_STATE;
// CORE_ENTER_CRITICAL();
// sl_bt_external_signal(UF_EVENT);
//
// CORE_EXIT_CRITICAL();
//
//
//}
//
//
//void setCOMP1Event()
//{
//  CORE_DECLARE_IRQ_STATE;
//  CORE_ENTER_CRITICAL();
//  sl_bt_external_signal(COMP1_EVENT);
//
//  CORE_EXIT_CRITICAL();
//}
//
//
//void setI2CCompleteEvent()
//{
//  CORE_DECLARE_IRQ_STATE;
//  CORE_ENTER_CRITICAL();
//  sl_bt_external_signal(I2C_COMPLETE_EVENT);
//
//  CORE_EXIT_CRITICAL();
//}


//void setSchedulerEvent()
//{
//  if(event==IDLE)
//    {
//
//    }
//}
/**************************************************************************//**
 * Function to read the events log and return an active event through priority
 * and clear the returned event
 *****************************************************************************/
//eventEnum lastEvent1 =IDLE_EVENT;
//eventEnum lastEvent2 =IDLE_EVENT;

uint8_t getEvent()
{

  uint8_t eventToReturn; // DOS

  // #########################
  // This code is broken, see fixed code below
  // #########################

//  if((eventLog & UF_EVENT) && (lastEvent1!=COMP0_UF))
//    {
//      CORE_DECLARE_IRQ_STATE;
//      CORE_ENTER_CRITICAL();
//      eventLog &= ~(1 << (UF_EVENT-1));
//      CORE_EXIT_CRITICAL();
////      nextEvent=2;
//      lastEvent1=COMP0_UF;
//      return COMP0_UF;
//    }
//
//  else if((eventLog & COMP1_EVENT))// && (lastEvent2!=COMP1_UF))
//    {
//       CORE_DECLARE_IRQ_STATE;
//       CORE_ENTER_CRITICAL();
//       eventLog &= ~(1 << (COMP1_EVENT-1));
//       CORE_EXIT_CRITICAL();
//       lastEvent2=COMP1_UF;
//
////       nextEvent=3;
//
//       return COMP1_UF;
//     }
//
//  else if((eventLog & I2C_COMPLETE_EVENT))
//    {
//      CORE_DECLARE_IRQ_STATE;
//      CORE_ENTER_CRITICAL();
//      eventLog &= ~(1 << (I2C_COMPLETE_EVENT-1));
//      CORE_EXIT_CRITICAL();
//
//      eventCompleted++;
//      if(eventCompleted%2==0)
//        {
//          lastEvent1=I2C_TRANSFER_COMPLETE;
//          lastEvent2=I2C_TRANSFER_COMPLETE;
//        }
//      else
//        {
//          lastEvent2=I2C_TRANSFER_COMPLETE;
//        }
//
//
//
////      nextEvent=4;
//
//
//
//      return I2C_TRANSFER_COMPLETE;
//    }
//
//  return 0;

//  if (eventLog & UF_EVENT) {
//      eventToReturn  = UF_EVENT;  // event to return
//      eventLog      ^= UF_EVENT;  // clear the event in our private data structure
//  }
//  else if (eventLog & COMP1_EVENT) {
//      eventToReturn  = COMP1_EVENT;  // event to return
//      eventLog      ^= COMP1_EVENT;  // clear the event in our private data structure
//  }
//  else if (eventLog & I2C_COMPLETE_EVENT) {
//      eventToReturn  = I2C_COMPLETE_EVENT;  // event to return
//      eventLog      ^= I2C_COMPLETE_EVENT;  // clear the event in our private data structure
//  }
//
//  return (eventToReturn);


} // getEvent()

void sendManagerIndication()
{
  uint8_t managerID=0x04;
  sl_status_t sc = sl_bt_gatt_server_write_attribute_value(
    gattdb_employee_id_s, // handle from gatt_db.h
    0,                              // offset
    sizeof(managerID), // length
    &managerID);    // pointer to buffer where data is

  if (sc != SL_STATUS_OK)
           LOG_ERROR("GATT DB WRITE ERROR");

  if(ble_data2->connection_open==1
      && ble_data2->indication_in_flight==0 && ble_data2->ok_to_send_htm_indications)
    {
      sc = sl_bt_gatt_server_send_indication(ble_data2->connectionHandle,
                                             gattdb_employee_id_s,
                                                   sizeof(managerID),
                                                   &managerID);
      ble_data2->indication_in_flight=1;
      if (sc != SL_STATUS_OK)
               LOG_ERROR("GATT INDICATION WRITE ERROR");

    }
}

void sendEmployeeIndication(uint8_t fingerID)
{

    sl_status_t sc = sl_bt_gatt_server_write_attribute_value(
    gattdb_employee_id_s, // handle from gatt_db.h
    0,                              // offset
    sizeof(fingerID), // length
    &fingerID);    // pointer to buffer where data is

  if (sc != SL_STATUS_OK)
           LOG_ERROR("GATT DB WRITE ERROR");

  if(ble_data2->connection_open==1
      && ble_data2->indication_in_flight==0 && ble_data2->ok_to_send_htm_indications)
    {
      sc = sl_bt_gatt_server_send_indication(ble_data2->connectionHandle,
                                             gattdb_employee_id_s,
                                                   sizeof(fingerID),
                                                   &fingerID);
      ble_data2->indication_in_flight=1;
      if (sc != SL_STATUS_OK)
               LOG_ERROR("GATT INDICATION WRITE ERROR");

    }
}

void payRollDisplay()
{

  size_t value_len_s =0;
  uint8_t tempEmployeeID[3];
  sl_status_t sc = sl_bt_gatt_server_read_attribute_value(gattdb_wages,
                                                          0,
                                                          3,
                                                          &value_len_s,
                                                          &tempEmployeeID[0]);

  employee_report_table[0].Payroll=tempEmployeeID[0];
  employee_report_table[1].Payroll=tempEmployeeID[1];
  employee_report_table[2].Payroll=tempEmployeeID[2];



  displayPrintf(DISPLAY_ROW_NAME, "Attendance");
  displayPrintf(DISPLAY_ROW_BTADDR, "Monitoring");
  displayPrintf(DISPLAY_ROW_3, "Manager Access",0);// change row
  displayPrintf(DISPLAY_ROW_4, "Headcount = %d", headcount);

  displayPrintf(DISPLAY_ROW_6, "EMPID ATTENDANCE PAY");
  displayPrintf(DISPLAY_ROW_7, "----- ---------- ---");
  displayPrintf(DISPLAY_ROW_8, " %d    %s $%d", employee_report_table[0].employee_id, employee_report_table[0].attendance_status, employee_report_table[0].Payroll*WAGES);
  displayPrintf(DISPLAY_ROW_9, " %d   %s $%d", employee_report_table[1].employee_id, employee_report_table[1].attendance_status, employee_report_table[1].Payroll*WAGES);
  displayPrintf(DISPLAY_ROW_10, " %d      %s    $%d", employee_report_table[2].employee_id, employee_report_table[2].attendance_status, employee_report_table[2].Payroll*WAGES);
  //displayPrintf(DISPLAY_ROW_ASSIGNMENT, "Client");
}



void employeeDataDisplay()
{

  size_t value_len_s =0;
  uint8_t tempEmployeeID[3];
  sl_status_t sc = sl_bt_gatt_server_read_attribute_value(gattdb_attendance_data_c,
                                                          0,
                                                          3,
                                                          &value_len_s,
                                                          &tempEmployeeID[0]);

  employee_report_table[0].status=tempEmployeeID[0];
  employee_report_table[1].status=tempEmployeeID[1];
  employee_report_table[2].status=tempEmployeeID[2];

  for(int i=0; i<3 ;i++)
    {
      if(employee_report_table[i].status == 0)
        {
          memset(&employee_report_table[i].attendance_status[0], 0, 11);
          strcpy(employee_report_table[i].attendance_status, "Absent");
        }
      else if(employee_report_table[i].status == 1)
        {
          memset(&employee_report_table[i].attendance_status[0], 0, 11);
          strcpy(employee_report_table[i].attendance_status, "Clocked In");
          headcount++;
        }
      else if(employee_report_table[i].status == 2)
        {
          memset(&employee_report_table[i].attendance_status[0], 0, 11);
          strcpy(employee_report_table[i].attendance_status, "Clocked Out");
        }
  }

  displayPrintf(DISPLAY_ROW_NAME, "Attendance");
  displayPrintf(DISPLAY_ROW_BTADDR, "Monitoring");
  displayPrintf(DISPLAY_ROW_3, "Manager Access",0);// change row
  displayPrintf(DISPLAY_ROW_4, "Headcount = %d", headcount);

  displayPrintf(DISPLAY_ROW_6, "EMPID ATTENDANCE    ");
  displayPrintf(DISPLAY_ROW_7, "----- ---------- ---");
  displayPrintf(DISPLAY_ROW_8, " %d    %s    ", employee_report_table[0].employee_id, employee_report_table[0].attendance_status);
  displayPrintf(DISPLAY_ROW_9, " %d   %s    ", employee_report_table[1].employee_id, employee_report_table[1].attendance_status);
  displayPrintf(DISPLAY_ROW_10, " %d      %s    ", employee_report_table[2].employee_id, employee_report_table[2].attendance_status);
  displayPrintf(DISPLAY_ROW_ASSIGNMENT, "Server");
}



void stateMachine(sl_bt_msg_t *evt)
{
  static uint8_t fingerPrint=0;
  ble_data2=getBleDataPtr();
  if(ble_data2->connection_open==1 && ble_data2->isBonded && ble_data2->ok_to_send_htm_indications)
    {

      enum myStates_t currentState=IDLE;
      static enum myStates_t nextState = IDLE;
      uint32_t event=evt->data.evt_system_external_signal.extsignals;
      currentState = nextState;

       switch(currentState)
            {

            case IDLE:
                  //gpioLed1SetOn();
                  fpInit();

                  capturePrint();                 //start capturing finger prints
                  nextState=WAIT_FOR_FINGERPRINT;


              break;


            case WAIT_FOR_FINGERPRINT:

              if(event == 1)
                {
                  //gpioLed0SetOn();
                  identifyFinger();
                  nextState=IDENTIFY_FINGERPRINT;

                }
              break;


            case IDENTIFY_FINGERPRINT:
              //manager access
              if( (event == 2) && (fingerID==0x00))
                {

                  sendManagerIndication();
                  //log out
                  if(ble_data2->managerLoggedIn)
                    {
                      displayPrintf(DISPLAY_ROW_3, "",0);
                      displayPrintf(DISPLAY_ROW_4, "", 0);
                      displayPrintf(DISPLAY_ROW_6, "    ",0);
                      displayPrintf(DISPLAY_ROW_7, "",0);
                      displayPrintf(DISPLAY_ROW_8, "", 0);
                      displayPrintf(DISPLAY_ROW_9, " ", 0);
                      displayPrintf(DISPLAY_ROW_10, "    ", 0);

                      ble_data2->managerLoggedIn=0;
                      nextState=IDLE;
                    }

                  //log in
                  else
                    {
                      displayPrintf(DISPLAY_ROW_3, "Manager Access",0);// change row
                      displayPrintf(DISPLAY_ROW_4, "Getting Data...",0);
                      ble_data2->managerLoggedIn=1;
                      nextState=PAYROLL_DISPLAY;
                    }



                }
              //labor access
              else if( (event == 2) && (fingerID<=0x03) )
                {
                  nextState=LOG_ATTENDANCE;
                  sendEmployeeIndication(fingerID);

                }
              else if((event == 4) && (fingerID>0x03))
                {
                  displayPrintf(DISPLAY_ROW_3, "Unrecognized",0);// change row
                  displayPrintf(DISPLAY_ROW_4, "Try Again",0);
                  nextState=IDLE;
                }
              break;


            case PAYROLL_DISPLAY:
              //if(event==8)
                {
                  employeeDataDisplay();
                  nextState=IDLE;
                }
              break;


            case LOG_ATTENDANCE:
              //if(event==16)
                {
                  displayPrintf(DISPLAY_ROW_3, "Employee %d Logged",fingerID);
                  sl_udelay_wait(5 SECONDS);
                  displayPrintf(DISPLAY_ROW_3, " ",0);
                  nextState=IDLE;
                }
              break;

            } // switch
    }//if

}
