/*
   gpio.h
  
    Created on: Dec 12, 2018
        Author: Dan Walkes

    Updated by Dave Sluiter Sept 7, 2020. moved #defines from .c to .h file.
    Updated by Dave Sluiter Dec 31, 2020. Minor edits with #defines.

    Editor: Feb 26, 2022, Dave Sluiter
    Change: Added comment about use of .h files.

 *
 *
 * @student    Akash Patil, Akash.Patil@Colorado.edu
 *
 * @edit       none
 * @date      Jan 25th 2023
 * @institution University of Colorado Boulder (UCB)
 * @course      ECEN 5823-001: IoT Embedded Firmware (Spring 2023)
 * @instructor  David Sluiter
 * @assignment ecen5823-assignment1-SimplicityStudio
 * @due        Jan 25th, 2023
 * @resources  Utilized Silicon Labs' libraries to implement functionality.
 *
 *
 *
 *
 *
 *
 
 */


// Students: Remember, a header file (a .h file) generally defines an interface
//           for functions defined within an implementation file (a .c file).
//           The .h file defines what a caller (a user) of a .c file requires.
//           At a minimum, the .h file should define the publicly callable
//           functions, i.e. define the function prototypes. #define and type
//           definitions can be added if the caller requires theses.


#ifndef SRC_GPIO_H_
#define SRC_GPIO_H_







// Function prototypes
void gpioInit();
void gpioLed0SetOn();
void gpioLed0SetOff();
void gpioLed1SetOn();
void gpioLed1SetOff();



// testing functions
void gpioPD10On(void);
void gpioPD10Off(void);

//I2C gpio enable and disable functions
void enableI2CGPIO();
void disableI2CGPIO();

//display enable function
void gpioSensorEnSetOn();
void gpioSetDisplayExtcomin(bool value);

#endif /* SRC_GPIO_H_ */
