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
#include <src/oscillators.h>
#include "em_cmu.h"




void oscillatorInit()
{

#if LOWEST_ENERGY_MODE==3
    {
      CMU_ClockSelectSet(cmuClock_LFA,cmuSelect_ULFRCO);  //Select the appropriate Ultra Low Frequency clock for the LFA clock tree
      CMU_ClockEnable(cmuClock_LFA ,1);                   //Enable clock to LFA tree
      CMU_ClockDivSet(cmuClock_LETIMER0,1);               //set prescalar to 1
    }
#elif LOWEST_ENERGY_MODE>=0 && LOWEST_ENERGY_MODE<=2
    {
      CMU_OscillatorEnable(cmuOsc_LFXO ,1,1);               //Enable LFXO oscillator
      CMU_ClockSelectSet(cmuClock_LFA,cmuSelect_LFXO);         //Select the appropriate Low Frequency clock for the LFA clock tree
      CMU_ClockEnable(cmuClock_LFA ,1);                     //Enable clock to LFA tree
      CMU_ClockDivSet(cmuClock_LETIMER0,8);                 //set prescalar to 8
    }
#endif

//        CMU_OscillatorEnable(cmuOsc_LFXO ,1,1);               //Enable LFXO oscillator
//        CMU_ClockSelectSet(cmuClock_LFA,cmuSelect_LFXO);         //Select the appropriate Low Frequency clock for the LFA clock tree
//        CMU_ClockEnable(cmuClock_LFA ,1);                     //Enable clock to LFA tree
//        CMU_ClockDivSet(cmuClock_LETIMER0,8);                 //set prescalar to 8


        CMU_ClockEnable(cmuClock_LETIMER0 ,1);                    //Enable clock to LETIMER





}

