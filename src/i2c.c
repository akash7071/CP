#define INCLUDE_LOG_DEBUG 1

#include "src/log.h"
#include "gatt_db.h"
#include "em_gpio.h"
#include "sl_i2cspm.h"
#include "src/timers.h"
#include "src/gpio.h"

//#include "app.h"




#define SI7021_I2C_BUS_ADDRESS           0x40               /**< I2C bus address                        */
#define MEASURE_TEMP_CMD_NOBUSHOLD       0xF3


 I2C_TransferSeq_TypeDef transferSequence;
 uint8_t                 cmd_data;
 uint16_t                read_data;

 float    actual_temp;

/**************************************************************************//**
 * Function to init I2C periphral
 *****************************************************************************/


void i2cInit()
{



I2CSPM_Init_TypeDef I2C_Config = {
 .port = I2C0,
 .sclPort = gpioPortC,
 .sclPin = 10,
 .sdaPort = gpioPortC,
 .sdaPin = 11,
 .portLocationScl = 14,
 .portLocationSda = 16,
 .i2cRefFreq = 0,
 .i2cMaxFreq = I2C_FREQ_STANDARD_MAX,
 .i2cClhr = i2cClockHLRStandard
 };


  I2CSPM_Init(&I2C_Config);
// uint32_t i2c_bus_frequency = I2C_BusFreqGet (I2C0);

}

/**************************************************************************//**
 * Function to command the sensor to start temp measurement
 *****************************************************************************/


void startMeasurement()
{

    I2C_TransferReturn_TypeDef transferStatus; // this can be locally defined

    i2cInit();

    cmd_data                     = MEASURE_TEMP_CMD_NOBUSHOLD;
    transferSequence.addr        = SI7021_I2C_BUS_ADDRESS << 1;
    transferSequence.flags       = I2C_FLAG_WRITE;
    transferSequence.buf[0].data = (uint8_t*) &cmd_data; // pointer to cmd to write
    transferSequence.buf[0].len  = sizeof(cmd_data);

    // config NVIC to generate an IRQ for the I2C0 module.
    // ==>> disable this NVIC interrupt when the I2C transfer has completed
    NVIC_EnableIRQ(I2C0_IRQn);

//    LOG_INFO("Start I2C Write");
    transferStatus = I2C_TransferInit (I2C0, &transferSequence);
    if (transferStatus < 0)
    {
        LOG_ERROR("I2C_TransferInit() Write error = %x", transferStatus);
    }

}

/**************************************************************************//**
 * Function to command the sensor to read temp measurement
 *****************************************************************************/


void readMeasurement()
{

  I2C_TransferReturn_TypeDef transferStatus; // make this global for IRQs in A4
//  I2C_TransferSeq_TypeDef transferSequence; // this one can be local
  //uint8_t cmd_data; // make this global for IRQs in A4
  //read_data; // make this global for IRQs in A4
  //cmd_data = 0xE0;
//  uint8_t read_data[2];

  i2cInit(); // DOS: I added, just like for startMeasurement()

  transferSequence.addr        = SI7021_I2C_BUS_ADDRESS << 1; // shift device address left
  transferSequence.flags       = I2C_FLAG_READ;
  transferSequence.buf[0].data = (uint8_t*) &read_data; // pointer to memory where address read data is placed
  transferSequence.buf[0].len  = sizeof(read_data);

  NVIC_EnableIRQ(I2C0_IRQn);

//  LOG_INFO("Start I2C Read #bytes=%d", sizeof(read_data));
  transferStatus = I2C_TransferInit (I2C0, &transferSequence);
  if (transferStatus < 0)
    {
        LOG_ERROR("I2C_TransferInit() Read error = %d", transferStatus);
    }

}

uint32_t dispTemperature()
{
  //code for converting read data to celsius degrees and print on serial output

  uint32_t tempValue;



  tempValue = ((read_data & 0xFF00) >> 8) | // DOS
              ((read_data & 0x00FF) << 8);



  actual_temp = (((tempValue) * 175.72) / 65536) - 46.85;





  LOG_INFO("temperature:%f\r\n", actual_temp);

  return (uint32_t)actual_temp;




}




/**************************************************************************//**
 * Function to execute the control flow to read temperature from the sensor
 * through I2C
 *****************************************************************************/
void read_temp_from_si7021()
{
  enableI2CGPIO();            //enable GPIO to the sensor
  timerWaitUs_irq(80000);          //wait for sensor to boot up
  startMeasurement();         //send command to start temp measurement on I2C
  timerWaitUs_irq(11000);          //wait for successful read
  readMeasurement();          //read temp from buffer
  disableI2CGPIO();           //disable GPIO  for LPM


}



