/***************************************************************************//**
 * @file
 * @brief Core application logic.
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
 * Date:        02-25-2022
 * Author:      Dave Sluiter
 * Description: This code was created by the Silicon Labs application wizard
 *              and started as "Bluetooth - SoC Empty".
 *              It is to be used only for ECEN 5823 "IoT Embedded Firmware".
 *              The MSLA referenced above is in effect.
 *
 *
 *
 *
 * @student    Akash Patil, Akash.Patil@Colorado.edu
 *
 * @edit        Added application code for LED blinking in different modes
 * @date        Jan 25th 2023
 * @institution University of Colorado Boulder (UCB)
 * @course      ECEN 5823-001: IoT Embedded Firmware (Spring 2023)
 * @instructor  David Sluiter
 * @assignment ecen5823-assignment1-SimplicityStudio
 * @due        Feb 10th, 2023
 * @resources  Utilized Silicon Labs' libraries and examples to implement
 *              functionality.
 *
 *
 *
 ******************************************************************************/


#include <app.h>
#include "em_common.h"
#include "app_assert.h"
#include "sl_bluetooth.h"
#include "gatt_db.h"
#include <em_usart.h>
#include "src/uart.h"




#define INCLUDE_LOG_DEBUG 1
#define DELAY_TEST 0


// *************************************************
// Students: It is OK to modify this file.
//           Make edits appropriate for each
//           assignment.
// *************************************************



#include "sl_status.h"             // for sl_status_print()
#include "em_gpio.h"
#include "src/ble_device_type.h"

#include "src/gpio.h"
#include "src/lcd.h"
#include <src/oscillators.h>
#include <src/timers.h>
#include "src/irq.h"
#include "src/log.h"
#include "src/i2c.h"
#include "src/scheduler.h"
#include "src/ble.h"

// Students: Here is an example of how to correctly include logging functions in
//           each .c file.
//           Apply this technique to your other .c files.
//           Do not #include "src/log.h" in any .h file! This logging scheme is
//           designed to be included at the top of each .c file that you want
//           to call one of the LOG_***() functions from.

// Include logging specifically for this .c file


//uint32_t onTime=0;    //extern variable init
uint32_t intTime=0;   //extern variable init
//uint16_t eventLog=0; // Whoa!!!! - this is a private data structure for the scheduler!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! no no no












//eventEnum event =IDLE_EVENT;



// *************************************************
// Power Manager
// *************************************************

// See: https://docs.silabs.com/gecko-platform/latest/service/power_manager/overview
#if defined(SL_CATALOG_POWER_MANAGER_PRESENT)

// -----------------------------------------------------------------------------
// defines for power manager callbacks
// -----------------------------------------------------------------------------
// Return values for app_is_ok_to_sleep():
//   Return false to keep sl_power_manager_sleep() from sleeping the MCU.
//   Return true to allow system to sleep when you expect/want an IRQ to wake
//   up the MCU from the call to sl_power_manager_sleep() in the main while (1)
//   loop.
//
// Students: We'll need to modify this for A2 onward so that compile time we
//           control what the lowest EM (energy mode) the MCU sleeps to. So
//           think "#if (expression)".
#if !LOWEST_ENERGY_MODE
#define APP_IS_OK_TO_SLEEP      (false)   //for EM0 mode
#else
#define APP_IS_OK_TO_SLEEP      (true)    //for EM1, EM2, EM3 modes
#endif


// Return values for app_sleep_on_isr_exit():
//   SL_POWER_MANAGER_IGNORE; // The module did not trigger an ISR and it doesn't want to contribute to the decision
//   SL_POWER_MANAGER_SLEEP;  // The module was the one that caused the system wakeup and the system SHOULD go back to sleep
//   SL_POWER_MANAGER_WAKEUP; // The module was the one that caused the system wakeup and the system MUST NOT go back to sleep
//
// Notes:
//       SL_POWER_MANAGER_IGNORE, we see calls to app_process_action() on each IRQ. This is the
//       expected "normal" behavior.
//
//       SL_POWER_MANAGER_SLEEP, the function app_process_action()
//       in the main while(1) loop will not be called! It would seem that sl_power_manager_sleep()
//       does not return in this case.
//
//       SL_POWER_MANAGER_WAKEUP, doesn't seem to allow ISRs to run. Main while loop is
//       running continuously, flooding the VCOM port with printf text with LETIMER0 IRQs
//       disabled somehow, LED0 is not flashing.

#define APP_SLEEP_ON_ISR_EXIT   (SL_POWER_MANAGER_IGNORE)
//#define APP_SLEEP_ON_ISR_EXIT   (SL_POWER_MANAGER_SLEEP)
//#define APP_SLEEP_ON_ISR_EXIT   (SL_POWER_MANAGER_WAKEUP)

#endif // defined(SL_CATALOG_POWER_MANAGER_PRESENT)




// *************************************************
// Power Manager Callbacks
// The values returned by these 2 functions AND
// adding and removing power manage requirements is
// how we control when EM mode the MCU goes to when
// sl_power_manager_sleep() is called in the main
// while (1) loop.
// *************************************************

#if defined(SL_CATALOG_POWER_MANAGER_PRESENT)

bool app_is_ok_to_sleep(void)
{
  return APP_IS_OK_TO_SLEEP;
} // app_is_ok_to_sleep()

sl_power_manager_on_isr_exit_t app_sleep_on_isr_exit(void)
{
  return APP_SLEEP_ON_ISR_EXIT;
} // app_sleep_on_isr_exit()

#endif // defined(SL_CATALOG_POWER_MANAGER_PRESENT)



//int comm_send(uint8_t* pbuf, int nsize, int ntimeout)
//{
//  uint8_t timeout = ntimeout;
//  uint8_t temp;
//
//  while(nsize !=0 && --timeout)
//  {
//    putchar(*pbuf++);
//    nsize --;
//    temp++;
//    //timeout = ntimeout;
//  }
//  return temp;
//}



//void sendOpenCommand()
//{
//  LOG_INFO("\n\rStart\n\r");
//  SB_OEM_PKT pPkt;
//
//  pPkt.Head1=0x55;
//  pPkt.Head2=0xAA;
//  pPkt.wDevId=0x0001;
//  pPkt.nParam=0x00;
//  pPkt.wCmd=0x01;
//  pPkt.wChkSum=oemp_CalcChkSumOfCmdAckPkt(&pPkt);
//  int sent_bytes = comm_send((uint8_t *)&pPkt, 12, 10000);
//  LOG_INFO("\n\rOpen Sent bytes = %d\n\r", sent_bytes);
//  if(sent_bytes < 12) {
//      LOG_INFO("Cannot send open\n\r");
//  }
//}



#include "em_device.h"
#include "em_chip.h"
#include "em_cmu.h"
#include "em_gpio.h"
#include "em_usart.h"
#include "sl_iostream.h"
#include "sl_iostream_uart.h"
#include "sl_iostream_usart.h"
#include "sl_udelay.h"
//#include "sl_iostream_usart_vcom_config.h"














//uint8_t uartRx(void)
//{
//  // Wait for start bit
//  while (GPIO_PinInGet(UART_RX_PORT, UART_RX_PIN));
//
//  // Data bits
//  uint8_t data = 0;
//  for (int i = 0; i < 8; i++) {
//    // Wait for the middle of the bit
//    while (!(LETIMER_IntGet(LETIMER0) & LETIMER_IF_COMP0));
//    LETIMER_IntClear(LETIMER0, LETIMER_IF_COMP0);
//
//    // Read the data bit
//    data |= GPIO_PinInGet(UART_RX_PORT, UART_RX_PIN) << i;
//  }
//
//  // Stop bit
//  while (!(LETIMER_IntGet(LETIMER0) & LETIMER_IF_COMP0));
//  LETIMER_IntClear(LETIMER0, LETIMER_IF_COMP0);
//
//  return data;
//}




















// Size of the buffer for received data

#define LED0_port  5 // change to correct ports and pins
#define LED0_pin   4
#define LED1_port  5
#define LED1_pin   5

// Receive data buffer


// Current position ins buffer
uint32_t inpos = 0;
uint32_t outpos = 0;

// True while receiving data (waiting for CR or BUFLEN characters)
bool receive = true;
//
//uint8_t receive_ack(int len)
//{
//
//  for(int i=0;i<len;i++)
//    {
//      //buffer[i] = USART_RxDataGet(USART0);
//      buffer[i] = USART_Rx(USART0);
//    }
//
//  //gpioLed1SetOn();
//  USART0->CMD |= 1 <_USART_CMD_CLEARRX_SHIFT;
//  return buffer[8];
//
//}






/**************************************************************************//**
 * @brief
 *    The USART0 receive interrupt saves incoming characters.
 *****************************************************************************/
//void USART0_RX_IRQHandler(void)
//{
//
//  //GPIO_PinOutToggle(5,LED1_pin);
//  gpioLed1SetOn();
//  // Get the character just received
//  //for(int i=0;i<12;i++)
//    buffer[inpos] = USART_RxDataGet(USART0);
//    if(buffer[inpos]==0x30)
//      {
//        //gpioLed0SetOn();
//
//
//      }
//
//
//
//
//
//
//
//
////   Exit loop on new line or buffer full
//  if ((buffer[inpos] != '\r') && (inpos < BUFLEN))
//    inpos++;
//  else
//    receive = false;   // Stop receiving on CR
//  if(inpos==12)
//    NVIC_DisableIRQ(USART0_RX_IRQn);
//
//}









/**************************************************************************//**
 * Application Init.
 *****************************************************************************/
SL_WEAK void app_init(void)
{
  // Put your application 1-time initialization code here.
  // This is called once during start-up.
  // Don't call any Bluetooth API functions until after the boot event.

//  intTime=((LETIMER_PERIOD_MS)* 32768)/(8*1000); //computing intTime


#if LOWEST_ENERGY_MODE<3                               //for energy modes EM0,EM1,EM2
    {
      intTime=((LETIMER_PERIOD_MS)* 32768)/(8*1000); //computing offTime
    }
#else                                                    //for energy modes EM3
      {
        intTime=(LETIMER_PERIOD_MS);                 //computing period Time
      }
#endif

#if LOWEST_ENERGY_MODE>=0 && LOWEST_ENERGY_MODE<3
    sl_power_manager_add_em_requirement(LOWEST_ENERGY_MODE);          //adding power requirements for em1 and em2
#endif




    gpioInit();                     //Initializing GPIO
    GPIO_PinOutSet(GT521F52_TX_PORT,GT521F52_TX_PIN);
    oscillatorInit();               //Initializing Oscillators
    //timerInit();                    //Initializing letimer
    //i2cInit();
    //enableI2CGPIO();
    //init_leUART();

    init_uart();
    sl_udelay_wait(10000);
    gpioSensorEnSetOn();
    fpInit();





    // Enable RX interrupt
     USART_IntClear(USART0, USART_IF_RXDATAV);
     USART_IntEnable(USART0, USART_IF_RXDATAV);
     NVIC_ClearPendingIRQ(USART0_RX_IRQn);
     //NVIC_EnableIRQ(USART0_RX_IRQn);
     NVIC_ClearPendingIRQ (LETIMER0_IRQn); //clear pending interrupts in LETIMER
     NVIC_EnableIRQ(LETIMER0_IRQn);        //configure NVIC to allow LETIMER interrupt
     // Enable interrupt handling for even and odd numbered pins
     NVIC_ClearPendingIRQ (GPIO_EVEN_IRQn); //clear pending interrupts in GPIO
     NVIC_EnableIRQ(GPIO_EVEN_IRQn);


     // Enable interrupt handling for even and odd numbered pins
     NVIC_ClearPendingIRQ (GPIO_ODD_IRQn); //clear pending interrupts in GPIO
     NVIC_EnableIRQ(GPIO_ODD_IRQn);


     //USART_Enable(USART0, usartEnable);






    sl_power_manager_add_em_requirement(SL_POWER_MANAGER_EM2);

} // app_init()



/**************************************************************************//**
 * Application Process Action.
 *****************************************************************************/
SL_WEAK void app_process_action(void)
{







} // app_process_action()





/**************************************************************************//**
 * Bluetooth stack event handler.
 * This overrides the dummy weak implementation.
 *
 * @param[in] evt Event coming from the Bluetooth stack.
 *
 * The code here will process events from the Bluetooth stack. This is the only
 * opportunity we will get to act on an event.
 *
 *****************************************************************************/
void sl_bt_on_event(sl_bt_msg_t *evt)
{

   (void) evt;


   handle_ble_event(evt);

  // sequence through states driven by events
   stateMachine(evt);


} // sl_bt_on_event()

