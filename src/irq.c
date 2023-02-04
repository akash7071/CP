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

#include <src/app.h>
#include "em_letimer.h"
#include "em_gpio.h"
#include "src/gpio.h"
#include "irq.h"



volatile bool shift;                          //flag to alternate between comp load values


//function to handle letimer interrupt
void LETIMER0_IRQHandler()
 {


  LETIMER_IntClear(LETIMER0,6);               //clear both interrupts
  GPIO_PinOutToggle(5,4);                     //toggle LED

  if(shift)                                    //if loop for switching to off counter
    {
      LETIMER_CompareSet(LETIMER0,0,offTime); //load comp0 with offTime counter value
      shift=0;                                //toggle shift flag
    }
  else                                         //else loop for switching to on counter
    {
      LETIMER_CompareSet(LETIMER0,0,onTime);  //load comp0 with onTime counter value
      shift=1;                                //toggle shift flag
    }



 }
