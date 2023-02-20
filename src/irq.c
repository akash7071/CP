/*
 * @student    Akash Patil, Akash.Patil@Colorado.edu
 * @edit       Energy profiles
 * @date       Feb 3rd 2023
 * @institution University of Colorado Boulder (UCB)
 * @course      ECEN 5823-001: IoT Embedded Firmware (Spring 2023)
 * @instructor  David Sluiter
 * @assignment ecen5823-assignment2-ManagingEnergyModes
 * @resources  Starter code from github classroom, prerecorded lecture.
*/

#define INCLUDE_LOG_DEBUG 1
#include "src/log.h"
#include <app.h>
#include "em_letimer.h"
#include "em_gpio.h"
#include "src/gpio.h"
#include "src/scheduler.h"
#include "src/timers.h"
#include "irq.h"

uint32_t timePassed=0;



//function to handle letimer interrupt
void LETIMER0_IRQHandler()
 {

  uint32_t intSource=LETIMER_IntGetEnabled(LETIMER0);
  if(intSource & LETIMER_IEN_UF)
    {
      LETIMER_IntClear(LETIMER0,4);
      timePassed+=3000;
//      GPIO_PinOutToggle(5,5);

      setCOMP0Event();
    }

  if(intSource & LETIMER_IEN_COMP1)
    {
      LETIMER_IntClear(LETIMER0,2);
//      GPIO_PinOutToggle(5,4);
      setCOMP1Event();
      LETIMER_IntDisable (LETIMER0, LETIMER_IEN_COMP1);

    }


 }


void I2C0_IRQHandler(void)
{

   // this can be locally defined
   I2C_TransferReturn_TypeDef transferStatus;

   // This shepherds the IC2 transfer along,
   // it’s a state machine! see em_i2c.c
   // It accesses global variables :
   // transferSequence
   // cmd_data
   // read_data
   // that we put into the data structure passed
   // to I2C_TransferInit()
//   GPIO_PinOutToggle(5,4);
   transferStatus = I2C_Transfer(I2C0);
   if (transferStatus == i2cTransferDone)
   {
    setI2CCompleteEvent();
   }
   if (transferStatus < 0)
   {
       LOG_ERROR("%x", transferStatus);
   }


} // I2C0_IRQHandler()



uint32_t letimerMilliseconds()
{
  return timePassed;
}
