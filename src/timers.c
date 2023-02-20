
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

#include <app.h>
#include "em_letimer.h"
#include <src/timers.h>
#include "src/log.h"

//uint32_t intTime;



void timerInit ()
{
   uint32_t tempData;


   const LETIMER_Init_TypeDef letimerInitData = {
     false,               // enabled later on
     true,                // continue operation during debug
     true,                // load COMP0 into CNT on UF
     true,               // load COMP1 into COMP0 when REP0==0
     0,                   // default output pin 1 value
     0,                   // default output pin 2 value
     letimerUFOANone,     // no underflow output
     letimerUFOANone,     // NO underflow output
     letimerRepeatFree,   // free running mode
     intTime                    // COMP0(top) Value, loaded later
 };


  // initializing the timer
   LETIMER_Init (LETIMER0, &letimerInitData);


   // calculate and load COMP0 (top)
//   uint32_t intTime= LETIMER_PERIOD_MS * (2^15)/(4*1000);
   LETIMER_CompareSet(LETIMER0,0,intTime);                   //first load only


   // Clear all IRQ flags in the LETIMER0 IF
   LETIMER_IntClear (LETIMER0, 0xFFFFFFFF);

   // Set UF and COMP1 in LETIMER0_IEN, so that the timer will generate IRQs to the NVIC.
//   tempData = LETIMER_IEN_UF | LETIMER_IEN_COMP1;
   tempData = LETIMER_IEN_UF ;

   LETIMER_IntEnable (LETIMER0, tempData); // Make sure you have defined the ISR routine LETIMER0_IRQHandler()

   // Enable the timer and start counting
   LETIMER_Enable (LETIMER0, true);


}


/**************************************************************************//**
 * Function to create a delay in multiples of 122 us
 *****************************************************************************/


void timerWaitUs_polled(uint32_t us_wait)
{
  if(us_wait<122 || us_wait>999424)
      {
      LOG_ERROR("wait time out of range");
      return;
      }
  uint32_t actual_wait;

  #if LOWEST_ENERGY_MODE<3                               //for energy modes EM0,EM1,EM2
      {
        actual_wait=us_wait/244;               //delay compute for LXFO
      }
  #else                                                    //for energy modes EM3
        {
          actual_wait=us_wait/1000;
        }
  #endif




    //LOG("Too less time");
  uint32_t startTime=LETIMER_CounterGet(LETIMER0);

  while((startTime - LETIMER_CounterGet(LETIMER0))<actual_wait)
     ;

}


/**************************************************************************//**
 * Function to create a delay in multiples of 122 us
 *****************************************************************************/


void timerWaitUs_irq(uint32_t us_wait)
{

  if(us_wait<122 || us_wait>999424)
      {
      LOG_ERROR("wait time out of range");
      return;
      }
  uint32_t startTime=LETIMER_CounterGet(LETIMER0);
  uint32_t comp1_value;

  //uint32_t actual_wait;

#if LOWEST_ENERGY_MODE<3                               //for energy modes EM0,EM1,EM2
    {
    comp1_value=startTime-(us_wait/244);
    }
#else                                                    //for energy modes EM3
    {
      comp1_value=startTime-(us_wait/1000);
    }
#endif

    LETIMER_CompareSet(LETIMER0,1,comp1_value);

    uint32_t tempData = LETIMER_IEN_COMP1 ;

    LETIMER_IntEnable (LETIMER0, tempData);



}







