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
  int i;

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
#include "sl_iostream_usart_vcom_config.h"

#define GT521F52_UART USART0
#define GT521F52_TX_PORT gpioPortA
#define GT521F52_TX_PIN 0
#define GT521F52_RX_PORT gpioPortA
#define GT521F52_RX_PIN 1


void init_uart(void)


{

    CMU_ClockEnable(cmuClock_HFPER, true);
    CMU_ClockEnable(cmuClock_USART0, true);
    CMU_ClockEnable(cmuClock_GPIO, true);

    USART_InitAsync_TypeDef initAsync = USART_INITASYNC_DEFAULT;
    initAsync.baudrate = 9600;
    USART_InitAsync(USART0, &initAsync);




    USART0->ROUTEPEN |= USART_ROUTEPEN_RXPEN | USART_ROUTEPEN_TXPEN;
    USART0->ROUTELOC0 = (USART0->ROUTELOC0
                         & ~(_USART_ROUTELOC0_TXLOC_MASK
                             | _USART_ROUTELOC0_RXLOC_MASK))
                        | (GT521F52_TX_PIN << _USART_ROUTELOC0_TXLOC_SHIFT)
                        | (GT521F52_RX_PIN << _USART_ROUTELOC0_RXLOC_SHIFT);

    GPIO_PinModeSet((GPIO_Port_TypeDef)AF_USART0_TX_PORT(GT521F52_TX_PIN), AF_USART0_TX_PIN(GT521F52_TX_PIN), gpioModePushPull, 1);
    GPIO_PinModeSet((GPIO_Port_TypeDef)AF_USART0_RX_PORT(GT521F52_RX_PIN), AF_USART0_RX_PIN(GT521F52_RX_PIN), gpioModeInput, 0);

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

void receive_ack()
{
  uint8_t rec_temp[12];
  memset(rec_temp,0,12);
  //for(int i=0;i<12;i++)
    {

      rec_temp[0]=USART_Rx(USART0);
      if(rec_temp[0]>0)
               gpioLed0SetOn();
      //printf("\n\rrec_temp[%d] %d\n\r",i,rec_temp[i]);
    }
 for(int j=0;j<100;j++);
  //return rec_temp;
}





// Size of the buffer for received data
#define BUFLEN  80

// Receive data buffer
uint8_t buffer[BUFLEN];

// Current position ins buffer
uint32_t inpos = 0;
uint32_t outpos = 0;

// True while receiving data (waiting for CR or BUFLEN characters)
bool receive = true;

/**************************************************************************//**
 * @brief
 *    The USART0 receive interrupt saves incoming characters.
 *****************************************************************************/
void USART0_RX_IRQHandler(void)
{
  // Get the character just received
  buffer[inpos] = USART0->RXDATA;
  if(buffer[inpos]>0)
    gpioLed0SetOn();

  // Exit loop on new line or buffer full
  if ((buffer[inpos] != '\r') && (inpos < BUFLEN))
    inpos++;
  else
    receive = false;   // Stop receiving on CR
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
  uint32_t paramLED = 0x00000000;
  paramLED |= state;
  oemp_SendCmdOrAck(paramLED, 0x0012);
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
    timerInit();                    //Initializing letimer
    i2cInit();
    enableI2CGPIO();
    init_uart();
    //USART_Tx(USART0, 'a');
    //putchar('a');
    //printf("\n\n\n\rstart");


    // Enable NVIC USART sources

    NVIC_ClearPendingIRQ(USART0_RX_IRQn);
    NVIC_EnableIRQ(USART0_RX_IRQn);



    timerWaitUs_polled(10000);
    //uint8_t init_cmda[12] = {0x55, 0xAA, 0x01, 0x00, 0x80, 0x25, 0x00, 0x00, 0x04, 0x00, 0xA9, 0x01};
    oemp_SendCmdOrAck(0x00002580, 0x0004);
    //send_cmd(init_cmda, sizeof(init_cmda));

    // Enable receive data valid interrupt
    USART_IntEnable(USART0, USART_IEN_RXDATAV);
    //receive_ack();
    timerWaitUs_polled(38000);

    //uint8_t init_cmdb[] = {0x55, 0xAA,   0x01, 0x00,    0x01, 0x00, 0x00, 0x00,    0x01, 0x00,    0x02, 0x01};
    oemp_SendCmdOrAck(00000001, 0x0001);
    //send_cmd(init_cmdb, sizeof(init_cmdb));
    timerWaitUs_polled(76000);

    //uint8_t init_cmdc[] = {0x55, 0xAA,   0x01, 0x00,  0x00, 0x00, 0x00, 0x00,   0x06, 0x00,   0x06, 0x01};
    oemp_SendCmdOrAck(0x00000000, 0x0006);
    //send_cmd(init_cmdc, sizeof(init_cmdc));
    timerWaitUs_polled(100000);
    timerWaitUs_polled(100000);


    //uint8_t init_cmdd[] = {0x55, 0xAA,  0x01, 0x00,  0x01, 0x00, 0x00, 0x00,  0x12, 0x00,   0x13, 0x01};
    //oemp_SendCmdOrAck(0x00000001, 0x0012);
    timerWaitUs_polled(100000);
    cmosLED(1);
    //send_cmd(init_cmdd, sizeof(init_cmdd));
    timerWaitUs_polled(100000);
    timerWaitUs_polled(100000);



    //uint8_t init_cmde[] = {0x55, 0xAA,   0x01, 0x00,  0x00, 0x00, 0x00, 0x00,   0x26, 0x00, 0x26, 0x01};
    oemp_SendCmdOrAck(0x00000000, 0x0026);
    //send_cmd(init_cmde, sizeof(init_cmde));
    timerWaitUs_polled(1000000);
    timerWaitUs_polled(100000);

    //uint8_t init_cmdf[] = {0x55, 0xAA, 0x01, 0x00,  0x00, 0x00, 0x00, 0x00,   0x12, 0x00, 0x12, 0x01};
    //oemp_SendCmdOrAck(0x00000000, 0x0012);
    timerWaitUs_polled(100000);
    cmosLED(0);
    //send_cmd(init_cmdf, sizeof(init_cmdf));
    timerWaitUs_polled(100000);
    timerWaitUs_polled(100000);






    NVIC_ClearPendingIRQ (LETIMER0_IRQn); //clear pending interrupts in LETIMER
    NVIC_EnableIRQ(LETIMER0_IRQn);        //configure NVIC to allow LETIMER interrupt
#if DELAY_TEST
    timerWaitUs_irq(100000);
#endif
    sl_power_manager_add_em_requirement(SL_POWER_MANAGER_EM2);

} // app_init()





#if DELAY_TEST
/**************************************************************************//**
 * Function to unit test the us delay function using LED toggle function
 * and verifying it with energy profiling
 *****************************************************************************/

void delayTest()
{
  timerWaitUs_polled(8);
  GPIO_PinOutToggle(5,4);
  timerWaitUs_polled(1);
  GPIO_PinOutToggle(5,4);

  timerWaitUs_polled(80);
  GPIO_PinOutToggle(5,4);
  timerWaitUs_polled(11);
  GPIO_PinOutToggle(5,4);

  timerWaitUs_polled(800);
  GPIO_PinOutToggle(5,4);
  timerWaitUs_polled(110);
  GPIO_PinOutToggle(5,4);

  timerWaitUs_polled(8000);
  GPIO_PinOutToggle(5,4);
  timerWaitUs_polled(1100);
  GPIO_PinOutToggle(5,4);

  timerWaitUs_polled(80000);
  GPIO_PinOutToggle(5,4);
  timerWaitUs_polled(11000);
  GPIO_PinOutToggle(5,4);


  timerWaitUs_polled(800000);
  GPIO_PinOutToggle(5,4);
  timerWaitUs_polled(110000);
  GPIO_PinOutToggle(5,4);


  timerWaitUs_polled(8000000);
  GPIO_PinOutToggle(5,4);
  timerWaitUs_polled(1100000);
  GPIO_PinOutToggle(5,4);


  timerWaitUs_polled(80000000);
  GPIO_PinOutToggle(5,4);
  timerWaitUs_polled(11000000);
  GPIO_PinOutToggle(5,4);


}
#endif



/**************************************************************************//**
 * Application Process Action.
 *****************************************************************************/
SL_WEAK void app_process_action(void)
{



  // Put your application code here for A1 to A4.
  // This is called repeatedly from the main while(1) loop
  // Notice: This function is not passed or has access to Bluetooth stack events.
  // We will create/use a scheme that is far more energy efficient in later assignments.

  // DOS: Note: Why is your state machine not in a function? What about isolation of functionality?
  // Why should main-level code be concerned about all of the details of taking a temp
  // measurement?



  //      stateMachine();

//
//#if DELAY_TEST
//      while (1) {
//          currentState=nextState;
//          event=getEvent();
//
//
//              switch(currentState) {
//
//                case IDLE:
//
//                  if (event & 2) {
//                    timerWaitUs_irq(10000);
//                    gpioLed0SetOn();
//                    nextState=WAIT_FOR_TIMER;
//
//                  }
//                  break;
//
//
//                case WAIT_FOR_TIMER:
//                  if (event & 2) {
//                    timerWaitUs_irq(10000);
//                    gpioLed0SetOff();
//                    nextState=IDLE;
//
//                  }
//                  break;
//
//
//              } // switch
//
//          } // while (1)
//#endif
//
//
//
//
//


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
   //handle_ble_event(evt); // put this code in ble.c/.h

  // sequence through states driven by events
  // stateMachine(evt);    // put this code in scheduler.c/.h


} // sl_bt_on_event()

