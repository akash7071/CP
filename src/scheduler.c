#include "app.h"
#include "em_core.h"


#define READ_TEMP_I2C 1
#define EVENTB 2
#define EVENTC 4
#define EVENTD 8
#define EVENTE 16

/**************************************************************************//**
 * Function to set the event for read temperature every 3s
 *****************************************************************************/

void schedulerSetEvent3s()
{
 CORE_DECLARE_IRQ_STATE;
 CORE_ENTER_CRITICAL();
 eventLog|=READ_TEMP_I2C;
 CORE_EXIT_CRITICAL();


}

/**************************************************************************//**
 * Function to read the events log and return an active event through priority
 * and clear the returned event
 *****************************************************************************/


uint8_t getEvent()
{
  if(eventLog & 1<<0)
    {
      CORE_DECLARE_IRQ_STATE;
      CORE_ENTER_CRITICAL();
      eventLog &= ~(1 << (READ_TEMP_I2C-1));
      CORE_EXIT_CRITICAL();
      return READ_TEMP;
    }

  else if(eventLog & 1<<1)
    {
       eventLog &= ~(1 << (EVENT_B-1));
       return EVENT_B;
     }

  else if(eventLog & 1<<2)
    {
      eventLog &= ~(1 << (EVENT_C-1));
      return EVENT_C;
    }

  else if(eventLog & 1<<3)
    {
      eventLog &= ~(1 << (EVENT_D-1));
      return EVENT_D;
    }


  else
    return EVENT_E;



  return 0;
}
