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
#include "timer.h"




void timerInit ()
{
   uint32_t tempData;


   const LETIMER_Init_TypeDef letimerInitData = {
     false,               // enabled later on
     true,                // continue operation during debug
     true,                // load COMP0 into CNT on UF
     true,                // load COMP1 into COMP0 when REP0==0
     0,                   // default output pin 1 value
     0,                   // default output pin 2 value
     letimerUFOANone,     // no underflow output
     letimerUFOANone,     // NO underflow output
     letimerRepeatFree,   // free running mode
     0                    // COMP0(top) Value, loaded later
 };


  // initializing the timer
   LETIMER_Init (LETIMER0, &letimerInitData);


   // calculate and load COMP0 (top)
   uint32_t onTime= LETIMER_PERIOD_MS * (2^15)/(4*1000);
   LETIMER_CompareSet(LETIMER0,0,onTime);                   //first load only


   // Clear all IRQ flags in the LETIMER0 IF
   LETIMER_IntClear (LETIMER0, 0xFFFFFFFF);

   // Set UF and COMP1 in LETIMER0_IEN, so that the timer will generate IRQs to the NVIC.
   tempData = LETIMER_IEN_UF | LETIMER_IEN_COMP1;

   LETIMER_IntEnable (LETIMER0, tempData); // Make sure you have defined the ISR routine LETIMER0_IRQHandler()

   // Enable the timer and start counting
   LETIMER_Enable (LETIMER0, true);


}

