/*
  gpio.c
 
   Created on: Dec 12, 2018
       Author: Dan Walkes
   Updated by Dave Sluiter Dec 31, 2020. Minor edits with #defines.

   March 17
   Dave Sluiter: Use this file to define functions that set up or control GPIOs.

 *
 *
 * @student    Akash Patil, Akash.Patil@Colorado.edu
 *
 * @edit       Added port and pin numbers for LED0 and LED1
 * @date      Jan 25th 2023
 * @institution University of Colorado Boulder (UCB)
 * @course      ECEN 5823-001: IoT Embedded Firmware (Spring 2023)
 * @instructor  David Sluiter
 * @assignment ecen5823-assignment1-SimplicityStudio
 * @due        Jan 25th, 2023
 * @resources  Utilized Silicon Labs' libraries to implement functionality.
 *
 
 */


// *****************************************************************************
// Students:
// We will be creating additional functions that configure and manipulate GPIOs.
// For any new GPIO function you create, place that function in this file.
// *****************************************************************************

#include <stdbool.h>
#include "em_gpio.h"
#include <string.h>
#include "gatt_db.h"
#include "gpio.h"
#include "src/scheduler.h"
#include "src/lcd.h"
#include "sl_bt_api.h"

#include "src/ble.h"
ble_data_struct_t *ble_data3;


// Include logging specifically for this .c file
#define INCLUDE_LOG_DEBUG 1
#include "log.h"


// Student Edit: Define these, 0's are placeholder values.
// See the radio board user guide at https://www.silabs.com/documents/login/user-guides/ug279-brd4104a-user-guide.pdf
// and GPIO documentation at https://siliconlabs.github.io/Gecko_SDK_Doc/efm32g/html/group__GPIO.html
// to determine the correct values for these.

#define LED0_port  5 // change to correct ports and pins
#define LED0_pin   4
#define LED1_port  5
#define LED1_pin   5


// PD10 traces to Expansion header 7
#define PD_port     gpioPortD
#define PD_port10   10


#define SENSOR_ENABLE 15
#define DISP_ENABLE 15
#define EXTCOMIN 13

bool PB0Pressed=0;
bool PB1Pressed=0;
bool PB0Released=0;
bool PB1Released=0;
bool bothPressed=0;

#define BUTTON0_pin 6
#define BUTTON1_pin 7
#define BUTTON_port gpioPortF



// Set GPIO drive strengths and modes of operation
void gpioInit()
{


  GPIO_PinModeSet(LED1_port, LED1_pin, gpioModePushPull, false);
	GPIO_PinModeSet(LED0_port, LED0_pin, gpioModePushPull, false);
  //pushbutton pins
  GPIO_PinModeSet(BUTTON_port, BUTTON0_pin, gpioModeInputPullFilter, true);
  GPIO_PinModeSet(BUTTON_port, BUTTON1_pin, gpioModeInputPullFilter, true);
  GPIO_ExtIntConfig (BUTTON_port, BUTTON0_pin, BUTTON0_pin, true, true, true);
  GPIO_ExtIntConfig (BUTTON_port, BUTTON1_pin, BUTTON1_pin, true, true, true);



} // gpioInit()


void gpioLed0SetOn()
{
	GPIO_PinOutSet(LED0_port,LED0_pin);
}


void gpioLed0SetOff()
{
	GPIO_PinOutClear(LED0_port,LED0_pin);
}


void gpioLed1SetOn()
{
	GPIO_PinOutSet(LED1_port,LED1_pin);
}


void gpioLed1SetOff()
{
	GPIO_PinOutClear(LED1_port,LED1_pin);
}





/**************************************************************************//**
 * Function to enable GPIO SCL and SDA and set sensor enable.
 *****************************************************************************/



void enableI2CGPIO()
{
  GPIO_PinOutSet(gpioPortD,SENSOR_ENABLE); // turn power on to the 7021




  // DOS: Why are you setting these to pushpull, they have to be set to openDrain???
//  GPIO_PinModeSet(gpioPortC, 10, gpioModePushPull, false);
//  GPIO_PinModeSet(gpioPortC, 11, gpioModePushPull, false);


}

/**************************************************************************//**
 * Function to disable GPIO SCL and SDA and clear sensor enable.
 *****************************************************************************/
void disableI2CGPIO()
{

// DOS: GPIO_PinModeSet(gpioPortC, 10, gpioModeDisabled, false);
//  GPIO_PinModeSet(gpioPortC, 11, gpioModeDisabled, false);



  GPIO_PinOutClear(gpioPortD,SENSOR_ENABLE); // turn power off to the 7021

}


/**************************************************************************//**
 * Function to enable GPIO DISP_ENABLE to LCD
 *****************************************************************************/


void gpioSensorEnSetOn()
{

  GPIO_PinOutSet(gpioPortD,DISP_ENABLE); // turn power on to the LCD
}


/**************************************************************************//**
 * Function to set and reset EXTCOMIN periodically
 *****************************************************************************/
void gpioSetDisplayExtcomin(bool value)
{

  GPIO_PinModeSet(gpioPortD, EXTCOMIN, gpioModePushPull, value);
  LOG_INFO("\r\n1sec");
}
#if DEVICE_IS_BLE_SERVER
/**************************************************************************//**
 * IRQ handler for GPIO external interrupt
 *****************************************************************************/

void GPIO_EVEN_IRQHandler()
 {
  //ble_data3=getBleDataPtr();

  int flags=GPIO_IntGetEnabled();
  GPIO_IntClear(flags);

  if(PB0Pressed==1)
    {
      SetPB0Press();
      PB0Pressed=0;

    }
  else if(PB0Pressed==0)
    {
      SetPB0Release();
      PB0Pressed=1;

    }

 }

#else


/**************************************************************************//**
 * IRQ handler for GPIO PB0 external interrupt
 *****************************************************************************/



void GPIO_EVEN_IRQHandler()
 {

  //ble_data3=getBleDataPtr();


  int flags=GPIO_IntGetEnabled();
  GPIO_IntClear(flags);

  if(PB0Pressed==1)
    {
      SetPB0Release();
      PB0Pressed=0;

      PB0Released=1;

    }
  else if(PB0Pressed==0)
    {
      SetPB0Press();
      PB0Pressed=1;




    }
  if(PB0Released==1 && PB1Released==1 && bothPressed==1)
    {
      setEventToggleIndication();
      bothPressed=0;
    }

 }
  /**************************************************************************//**
   * IRQ handler for GPIO PB1 external interrupt
   *****************************************************************************/

  void GPIO_ODD_IRQHandler()
   {

    //ble_data3=getBleDataPtr();

    int flags=GPIO_IntGetEnabled();
    GPIO_IntClear(flags);

    if(PB1Pressed==1)
      {
        SetPB1Release();
        PB1Pressed=0;
        PB1Released=1;


      }
    else if(PB1Pressed==0)
      {
        SetPB1Press();
        PB1Pressed=1;

      }

    if(PB0Pressed==1 && PB1Pressed==1)
      {
        bothPressed=1;
      }


   }


#endif


