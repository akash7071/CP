/***************************************************************************//**
 * @file
 * @brief Application interface provided to main().
 *******************************************************************************
 * # License
 * <b>Copyright 2020 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * SPDX-License-Identifier: Zlib
 *
 * The licensor of this software is Silicon Laboratories Inc.
 *
 * This software is provided 'as-is', without any express or implied
 * warranty. In no event will the authors be held liable for any damages
 * arising from the use of this software.
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 *
 *
 * Editor: Feb 26, 2022, Dave Sluiter
 * Change: Added comment about use of .h files.
 *
 *
 *
 *
 * @student    Akash Patil, Akash.Patil@Colorado.edu
 * @edit       Energy profiles
 * @date       Feb 3rd 2023
 * @institution University of Colorado Boulder (UCB)
 * @course      ECEN 5823-001: IoT Embedded Firmware (Spring 2023)
 * @instructor  David Sluiter
 * @assignment ecen5823-assignment2-ManagingEnergyModes
 * @resources  Starter code from github classroom, prerecorded lecture.
 *
 *
 *
 ******************************************************************************/

// Students: Remember, a header file (a .h file) defines an interface
//           for functions defined within an implementation file (a .c file).
//           The .h file defines what a caller (a user) of a .c file requires.
//           At a minimum, the .h file should define the publicly callable
//           functions, i.e. define the function prototypes. #define and type
//           definitions can be added if the caller requires theses.

#include "em_common.h"

#ifndef SRC_APP_H_
#define SRC_APP_H_

#define LOWEST_ENERGY_MODE 3      //change energy mode here
//#define LETIMER_ON_TIME_MS 175    //change on period here
#define LETIMER_PERIOD_MS 3000    //change total period here

//extern uint32_t onTime;     //extern variables to calculate onTime value for comp0
extern uint32_t intTime;    //extern variables to calculate offTime value for comp0
extern uint16_t eventLog;
typedef enum
{
  READ_TEMP,
  EVENT_B,
  EVENT_C,
  EVENT_D,
  EVENT_E
} eventEnum;

extern eventEnum event;






/**************************************************************************//**
 * Application Init.
 *****************************************************************************/
void app_init(void);

/**************************************************************************//**
 * Application Process Action.
 *****************************************************************************/
void app_process_action(void);

#endif /* SRC_APP_H_ */
