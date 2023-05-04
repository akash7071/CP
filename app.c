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



 uint8_t receiveAck=0;
 uint8_t fingerID=0;








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


// Structure Of Cmd and Ack Packets
typedef struct {
  uint8_t  Head1;
  uint8_t  Head2;
  uint16_t  wDevId;
  uint32_t   nParam;
  uint16_t  wCmd;// or nAck
  uint16_t  wChkSum;
} SB_OEM_PKT;

int comm_send(uint8_t* pbuf, int nsize, int ntimeout)
{
  uint8_t timeout = ntimeout;
  uint8_t temp;

  while(nsize !=0 && --timeout)
  {
    putchar(*pbuf++);
    nsize --;
    temp++;
    //timeout = ntimeout;
  }
  return temp;
}

uint16_t oemp_CalcChkSumOfCmdAckPkt( SB_OEM_PKT* pPkt )
{
  uint16_t wChkSum = 0;
  uint8_t* pBuf = (uint16_t*)pPkt;
  unsigned int i;

  for(i=0;i<(sizeof(SB_OEM_PKT)-2);i++)
    wChkSum += pBuf[i];
  return wChkSum;
}


void sendOpenCommand()
{
  LOG_INFO("\n\rStart\n\r");
  SB_OEM_PKT pPkt;

  pPkt.Head1=0x55;
  pPkt.Head2=0xAA;
  pPkt.wDevId=0x0001;
  pPkt.nParam=0x00;
  pPkt.wCmd=0x01;
  pPkt.wChkSum=oemp_CalcChkSumOfCmdAckPkt(&pPkt);
  int sent_bytes = comm_send((uint8_t *)&pPkt, 12, 10000);
  LOG_INFO("\n\rOpen Sent bytes = %d\n\r", sent_bytes);
  if(sent_bytes < 12) {
      LOG_INFO("Cannot send open\n\r");
  }
}



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

#define GT521F52_UART USART0
#define GT521F52_TX_PORT gpioPortA
#define GT521F52_TX_PIN 0       //default 0
#define GT521F52_RX_PORT gpioPortA
#define GT521F52_RX_PIN 3       //default 1









#include "em_device.h"
#include "em_chip.h"
#include "em_cmu.h"
#include "em_gpio.h"
#include "em_letimer.h"

#define UART_RX_PORT gpioPortA
#define UART_RX_PIN  3

#define BAUDRATE     9600

#define TIMER_FREQ   1000000
#define BIT_TIME_US  (1000000 / BAUDRATE)



void init_leUART()
{
  GPIO_PinModeSet(UART_RX_PORT, UART_RX_PIN, gpioModeInput, 0);    // RX


  // Configure LETIMER for UART bit-banging
//  CMU_ClockEnable(cmuClock_LETIMER0, true);
//  LETIMER_Init_TypeDef letimerInit = LETIMER_INIT_DEFAULT;
//  letimerInit.repMode = letimerRepeatFree;
//  LETIMER_Init(LETIMER0, &letimerInit);
//  uint32_t letimerComp = BIT_TIME_US * 2; // Trigger at middle of each bit
//  LETIMER_CompareSet(LETIMER0, 0, letimerComp);
//  LETIMER_IntEnable(LETIMER0, LETIMER_IEN_COMP0);
//  NVIC_EnableIRQ(LETIMER0_IRQn);

  // Enable global interrupts
  __enable_irq();
}

uint8_t uartRx(void)
{
  // Wait for start bit
  while (GPIO_PinInGet(UART_RX_PORT, UART_RX_PIN));

  // Data bits
  uint8_t data = 0;
  for (int i = 0; i < 8; i++) {
    // Wait for the middle of the bit
    while (!(LETIMER_IntGet(LETIMER0) & LETIMER_IF_COMP0));
    LETIMER_IntClear(LETIMER0, LETIMER_IF_COMP0);

    // Read the data bit
    data |= GPIO_PinInGet(UART_RX_PORT, UART_RX_PIN) << i;
  }

  // Stop bit
  while (!(LETIMER_IntGet(LETIMER0) & LETIMER_IF_COMP0));
  LETIMER_IntClear(LETIMER0, LETIMER_IF_COMP0);

  return data;
}
















void init_uart(void)


{

    CMU_ClockEnable(cmuClock_HFPER, true);
    CMU_ClockEnable(cmuClock_USART0, true);
    CMU_ClockEnable(cmuClock_GPIO, true);

    USART_InitAsync_TypeDef initAsync = USART_INITASYNC_DEFAULT;
    initAsync.baudrate = 9600;
    USART_InitAsync(USART0, &initAsync);




    USART0->ROUTEPEN |= USART_ROUTEPEN_RXPEN | USART_ROUTEPEN_TXPEN;
//    USART0->ROUTELOC0 = (USART0->ROUTELOC0
//                         & ~(_USART_ROUTELOC0_TXLOC_MASK
//                             | _USART_ROUTELOC0_RXLOC_MASK))
//                        | (GT521F52_TX_PIN << _USART_ROUTELOC0_TXLOC_SHIFT)
//                        | (GT521F52_RX_PIN << _USART_ROUTELOC0_RXLOC_SHIFT);

    USART0->ROUTELOC0 =0x00<<_USART_ROUTELOC0_TXLOC_SHIFT | 0x02<<_USART_ROUTELOC0_RXLOC_SHIFT;
    //USART0->ROUTELOC0 =0x03<<_USART_ROUTELOC0_TXLOC_SHIFT;

    GPIO_PinModeSet((GPIO_Port_TypeDef)AF_USART0_TX_PORT(GT521F52_TX_PIN), AF_USART0_TX_PIN(GT521F52_TX_PIN), gpioModePushPull, 1);
    GPIO_PinModeSet((GPIO_Port_TypeDef)AF_USART0_RX_PORT(GT521F52_RX_PIN), AF_USART0_RX_PIN(GT521F52_RX_PIN), gpioModeInput, 0);
    //GPIO_PinModeSet((GPIO_Port_TypeDef)AF_USART0_RX_PORT(GT521F52_RX_PIN), AF_USART0_RX_PIN(GT521F52_RX_PIN), gpioModeInputPull, 0);

    GPIO_PinModeSet(UART_RX_PORT, UART_RX_PIN, gpioModeInput, 0);    // RX
}

uint8_t send_cmd(uint8_t* cmd, uint32_t len)
{
  uint8_t temp;
    for (uint32_t i = 0; i < len; i++) {
        USART_Tx(USART0, cmd[i]);
        //while (!(USART0->STATUS & USART_STATUS_TXC));
        temp++;
    }
    return temp;
}




// Size of the buffer for received data
#define BUFLEN  80
#define LED0_port  5 // change to correct ports and pins
#define LED0_pin   4
#define LED1_port  5
#define LED1_pin   5

// Receive data buffer
uint8_t buffer[BUFLEN];

// Current position ins buffer
uint32_t inpos = 0;
uint32_t outpos = 0;

// True while receiving data (waiting for CR or BUFLEN characters)
bool receive = true;

uint8_t receive_ack(int len)
{

  for(int i=0;i<len;i++)
    {
      //buffer[i] = USART_RxDataGet(USART0);
      buffer[i] = USART_Rx(USART0);
    }

  //gpioLed1SetOn();
  USART0->CMD |= 1 <_USART_CMD_CLEARRX_SHIFT;
  return buffer[8];

}






/**************************************************************************//**
 * @brief
 *    The USART0 receive interrupt saves incoming characters.
 *****************************************************************************/
void USART0_RX_IRQHandler(void)
{

  //GPIO_PinOutToggle(5,LED1_pin);
  gpioLed1SetOn();
  // Get the character just received
  //for(int i=0;i<12;i++)
    buffer[inpos] = USART_RxDataGet(USART0);
    if(buffer[inpos]==0x30)
      {
        //gpioLed0SetOn();


      }








//   Exit loop on new line or buffer full
  if ((buffer[inpos] != '\r') && (inpos < BUFLEN))
    inpos++;
  else
    receive = false;   // Stop receiving on CR
  if(inpos==12)
    NVIC_DisableIRQ(USART0_RX_IRQn);

}




// Header Of Cmd and Ack Packets
#define STX1        0x55  //Header1
#define STX2        0xAA  //Header2
#define PKT_ERR_START -500
#define PKT_COMM_ERR  PKT_ERR_START+1
#define PKT_HDR_ERR   PKT_ERR_START+2
#define PKT_DEV_ID_ERR  PKT_ERR_START+3
#define PKT_CHK_SUM_ERR PKT_ERR_START+4
#define PKT_PARAM_ERR PKT_ERR_START+5
#define SB_OEM_PKT_SIZE     12
#define COMM_DEF_TIMEOUT 10000
#define wDevID 0x0001
uint32_t gCommTimeOut = COMM_DEF_TIMEOUT;



int oemp_SendCmdOrAck(uint32_t nParam, uint16_t wCmdOrAck )
{
  SB_OEM_PKT pkt;
  int nSentBytes;


  pkt.Head1 = (uint8_t)STX1;
  pkt.Head2 = (uint8_t)STX2;
  pkt.wDevId = wDevID;
  pkt.wCmd = wCmdOrAck;
  pkt.nParam = nParam;
  pkt.wChkSum = oemp_CalcChkSumOfCmdAckPkt( &pkt );

  //nSentBytes = comm_send( (uint8_t*)&pkt, SB_OEM_PKT_SIZE, gCommTimeOut );
  nSentBytes = send_cmd((uint8_t*)&pkt, SB_OEM_PKT_SIZE);
  if( nSentBytes != SB_OEM_PKT_SIZE )
    return PKT_COMM_ERR;

  return 0;
}

void cmosLED(bool state)
{
  USART0->CMD |= 1 <_USART_CMD_CLEARRX_SHIFT;
  uint32_t paramLED = 0x00000000;
  paramLED |= state;
  oemp_SendCmdOrAck(paramLED, 0x0012);
  receive_ack(12);
  sl_udelay_wait(50000);
}

void fpInit()
{
  sl_udelay_wait(100000);


  USART0->CMD |= 1 <_USART_CMD_CLEARRX_SHIFT;
  oemp_SendCmdOrAck(0x00002580, 0x0004);  //{0x55, 0xAA, 0x01, 0x00, 0x80, 0x25, 0x00, 0x00, 0x04, 0x00, 0xA9, 0x01};
  receive_ack(12);
  sl_udelay_wait(42000);

  USART0->CMD |= 1 <_USART_CMD_CLEARRX_SHIFT;
  oemp_SendCmdOrAck(00000001, 0x0001); //uint8_t init_cmdb[] = {0x55, 0xAA,   0x01, 0x00,    0x01, 0x00, 0x00, 0x00,    0x01, 0x00,    0x02, 0x01};
  receive_ack(42);
  sl_udelay_wait(76000);


  USART0->CMD |= 1 <_USART_CMD_CLEARRX_SHIFT;
  oemp_SendCmdOrAck(0x00000000, 0x0006);//uint8_t init_cmdc[] = {0x55, 0xAA,   0x01, 0x00,  0x00, 0x00, 0x00, 0x00,   0x06, 0x00,   0x06, 0x01};
  receive_ack(56);
  sl_udelay_wait(100000);
  sl_udelay_wait(100000);
  sl_udelay_wait(100000);


  cmosLED(1);
//  receive_ack(12);
  cmosLED(0);
//  receive_ack(12);

}



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

void identifyFinger()
{

  USART0->CMD |= 1 <_USART_CMD_CLEARRX_SHIFT;
  oemp_SendCmdOrAck(0x00000000, 0x0051);
  //sl_udelay_wait(60000);


  for(int i=0;i<12;i++)
    {
      //buffer[i] = USART_RxDataGet(USART0);
      buffer[i] = USART_Rx(USART0);
    }

  //gpioLed1SetOn();
  USART0->CMD |= 1 <_USART_CMD_CLEARRX_SHIFT;

  if(buffer[8]==0x30)
    {
      fingerID=buffer[4];

    }
  else
    {
      fingerID=0xFF;
    }


  cmosLED(0);
  setIdentifyEvent();

}

void capturePrint()
{
  uint8_t attempts;
CAPTUREAGAIN:
  attempts=0;
  //turn on LED
  cmosLED(1);

  //get enroll count
  USART0->CMD |= 1 <_USART_CMD_CLEARRX_SHIFT;
  oemp_SendCmdOrAck(0x00000000, 0x0020);
  receive_ack(12);
  sl_udelay_wait(100000);

  //capture prints
  USART0->CMD |= 1 <_USART_CMD_CLEARRX_SHIFT;
  oemp_SendCmdOrAck(0x00000000, 0x0060);
  //sl_udelay_wait(60000);
  receiveAck=receive_ack(12);
  //sl_udelay_wait(10000);

  //if no finger pressed, repeat above step

  while( receiveAck!=0x30)
    {
      oemp_SendCmdOrAck(0x00000000, 0x0060);
      //sl_udelay_wait(30000);
      receiveAck=receive_ack(12);
      attempts++;
      if(attempts>15)
        {

          cmosLED(0);
          goto CAPTUREAGAIN;
        }
    }
  setCaptureEvent();



}



/**************************************************************************//**
 * Application Process Action.
 *****************************************************************************/
SL_WEAK void app_process_action(void)
{


  //init fp sensor



//  //turn led on
//  USART0->CMD |= 1 <_USART_CMD_CLEARRX_SHIFT;
//  cmosLED(1);
//  receive_ack(12);
//  sl_udelay_wait(100000);
//
//
//  //get enroll count
//  USART0->CMD |= 1 <_USART_CMD_CLEARRX_SHIFT;
//  oemp_SendCmdOrAck(0x00000000, 0x0020);
//  receive_ack(12);
//  sl_udelay_wait(100000);
//
//
//  //capture enrolled prints count
//  USART0->CMD |= 1 <_USART_CMD_CLEARRX_SHIFT;
//  oemp_SendCmdOrAck(0x00000000, 0x0060);
//  sl_udelay_wait(60000);
//  receiveAck=receive_ack(12);
//  sl_udelay_wait(10000);
//
//  //if no finger pressed, repeat above step
//  if( receiveAck!=0x30)
//  //while( receiveAck!=0x30)
//    {
//      oemp_SendCmdOrAck(0x00000000, 0x0060);
//      sl_udelay_wait(60000);
//      receiveAck=receive_ack(12);
//    }
//  //sl_udelay_wait(60000);


  //if finger pressed, send for identification
//  USART0->CMD |= 1 <_USART_CMD_CLEARRX_SHIFT;
//  oemp_SendCmdOrAck(0x00000000, 0x0051);
//  sl_udelay_wait(60000);
//  fingerID=identifyFinger();
//
//
//
//  //control gpio based on finger ID
//  if(fingerID==0x00)
//    {
//      gpioLed0SetOn();
//      gpioLed1SetOff();
//    }
//  else if(fingerID==0x01)
//    {
//      gpioLed1SetOn();
//      gpioLed0SetOff();
//    }
//  else
//    {
//      gpioLed1SetOff();
//      gpioLed0SetOff();
//    }
//
//  sl_udelay_wait(100000);
//
//
//
//  //turn led off after finger identified
//  cmosLED(0);







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

  // Just a trick to hide a compiler warning about unused input parameter evt.
  (void) evt;

  // For A5 onward:
  // Some events require responses from our application code,
  // and don’t necessarily advance our state machines.
  // For A5 uncomment the next 2 function calls
   handle_ble_event(evt); // put this code in ble.c/.h

  // sequence through states driven by events
   stateMachine(evt);    // put this code in scheduler.c/.h


} // sl_bt_on_event()

