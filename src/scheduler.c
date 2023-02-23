#include "app.h"
#include "em_core.h"
#include "sl_power_manager.h"
#include "src/i2c.h"
#include "src/timers.h"


// DOS: If the caller needs these, shouldn't they be defined in the .h file ???? !!!!!
#define COMP0_EVENT        1
#define COMP1_EVENT        2
#define I2C_COMPLETE_EVENT 4

//#define IDLE_EVENT 8
//#define EVENTE 16

//uint16_t eventLog;


//static int eventCompleted=0;

//bool ufEvent=0;
//bool comp1Event=0;
//bool i2cTransferComplete=0;
//bool writeCompleted=0;
//bool readCompleted=0;



uint16_t eventLog=0;



/**************************************************************************//**
 * Function to set the event for read temperature every 3s
 *****************************************************************************/

void setCOMP0Event()
{
 CORE_DECLARE_IRQ_STATE;
 CORE_ENTER_CRITICAL();
 eventLog |= COMP0_EVENT;

 CORE_EXIT_CRITICAL();


}


void setCOMP1Event()
{
  CORE_DECLARE_IRQ_STATE;
  CORE_ENTER_CRITICAL();
  eventLog |= COMP1_EVENT;

  CORE_EXIT_CRITICAL();
}


void setI2CCompleteEvent()
{
  CORE_DECLARE_IRQ_STATE;
  CORE_ENTER_CRITICAL();
  eventLog |= I2C_COMPLETE_EVENT;

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
//eventEnum lastEvent1 =IDLE_EVENT;
//eventEnum lastEvent2 =IDLE_EVENT;

uint8_t getEvent()
{

  uint8_t eventToReturn; // DOS

  // #########################
  // This code is broken, see fixed code below
  // #########################

//  if((eventLog & COMP0_EVENT) && (lastEvent1!=COMP0_UF))
//    {
//      CORE_DECLARE_IRQ_STATE;
//      CORE_ENTER_CRITICAL();
//      eventLog &= ~(1 << (COMP0_EVENT-1));
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

  if (eventLog & COMP0_EVENT) {
      eventToReturn  = COMP0_EVENT;  // event to return
      eventLog      ^= COMP0_EVENT;  // clear the event in our private data structure
  }
  else if (eventLog & COMP1_EVENT) {
      eventToReturn  = COMP1_EVENT;  // event to return
      eventLog      ^= COMP1_EVENT;  // clear the event in our private data structure
  }
  else if (eventLog & I2C_COMPLETE_EVENT) {
      eventToReturn  = I2C_COMPLETE_EVENT;  // event to return
      eventLog      ^= I2C_COMPLETE_EVENT;  // clear the event in our private data structure
  }

  return (eventToReturn);


} // getEvent()



void stateMachine()
{

}
