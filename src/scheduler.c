#include "app.h"
#include "em_core.h"
#include "sl_power_manager.h"
#include "src/i2c.h"
#include "src/timers.h"


#define COMP0_EVENT 1
#define COMP1_EVENT 2
#define I2C_COMPLETE_EVENT 4
#define IDLE_EVENT 8
#define EVENTE 16

//uint16_t eventLog;





bool ufEvent=0;
bool comp1Event=0;
bool i2cTransferComplete=0;
bool writeCompleted=0;
bool readCompleted=0;





/**************************************************************************//**
 * Function to set the event for read temperature every 3s
 *****************************************************************************/

void setCOMP0Event()
{
 CORE_DECLARE_IRQ_STATE;
 CORE_ENTER_CRITICAL();
 eventLog|=COMP0_EVENT;

 CORE_EXIT_CRITICAL();


}


void setCOMP1Event()
{
  CORE_DECLARE_IRQ_STATE;
  CORE_ENTER_CRITICAL();
  eventLog|=COMP1_EVENT;

  CORE_EXIT_CRITICAL();
}


void setI2CCompleteEvent()
{
  CORE_DECLARE_IRQ_STATE;
  CORE_ENTER_CRITICAL();
  eventLog|=I2C_COMPLETE_EVENT;

  CORE_EXIT_CRITICAL();
}


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
eventEnum lastEvent =IDLE_EVENT;

uint8_t getEvent()
{
  if((eventLog & COMP0_EVENT) && (lastEvent!=COMP0_UF))
    {
      CORE_DECLARE_IRQ_STATE;
      CORE_ENTER_CRITICAL();
      eventLog &= ~(1 << (COMP0_EVENT-1));
      CORE_EXIT_CRITICAL();
//      nextEvent=2;
      lastEvent=COMP0_UF;
      return COMP0_UF;
    }

  else if((eventLog & COMP1_EVENT))
    {
       CORE_DECLARE_IRQ_STATE;
       CORE_ENTER_CRITICAL();
       eventLog &= ~(1 << (COMP1_EVENT-1));
       CORE_EXIT_CRITICAL();

//       nextEvent=3;

       return COMP1_UF;
     }

  else if(eventLog & I2C_COMPLETE_EVENT )
    {
      CORE_DECLARE_IRQ_STATE;
      CORE_ENTER_CRITICAL();
      eventLog &= ~(1 << (I2C_COMPLETE_EVENT-1));
      CORE_EXIT_CRITICAL();
//      nextEvent=4;



      return I2C_TRANSFER_COMPLETE;
    }








  return 0;
}

void stateMachine()
{

}
