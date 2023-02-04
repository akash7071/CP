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

#include "em_common.h"

#ifndef SRC_APP_H_
#define SRC_APP_H_

#define LOWEST_ENERGY_MODE 1      //change energy mode here
#define LETIMER_ON_TIME_MS 175    //change on period here
#define LETIMER_PERIOD_MS 2425    //change total period here

extern uint32_t onTime;     //extern variables to calculate onTime value for comp0
extern uint32_t offTime;    //extern variables to calculate offTime value for comp0
#endif /* SRC_APP_H_ */
