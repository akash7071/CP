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

#include <app.h>
#include "em_letimer.h"
#include "em_gpio.h"
#include "src/gpio.h"
#include "src/scheduler.h"
#include "irq.h"





//function to handle letimer interrupt
void LETIMER0_IRQHandler()
 {


  LETIMER_IntClear(LETIMER0,4);               //clear both interrupts
  //GPIO_PinOutToggle(5,4);                     //toggle LED
  //LETIMER_CompareSet(LETIMER0,0,intTime);


  schedulerSetEvent3s();

 }
