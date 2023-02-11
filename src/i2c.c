#define INCLUDE_LOG_DEBUG 1
#include "src/log.h"
#include "em_gpio.h"
#include "sl_i2cspm.h"




#define SI7021_I2C_BUS_ADDRESS           0x40               /**< I2C bus address                        */



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
 //uint32_t i2c_bus_frequency = I2C_BusFreqGet (I2C0);

}

/**************************************************************************//**
 * Function to command the sensor to start temp measurement
 *****************************************************************************/


void startMeasurement()
{
    I2C_TransferReturn_TypeDef transferStatus; // make this global for IRQs in A4
    I2C_TransferSeq_TypeDef transferSequence; // this one can be local
    uint8_t cmd_data; // make this global for IRQs in A4
    //read_data; // make this global for IRQs in A4
    cmd_data = 0xF3;
    transferSequence.addr = SI7021_I2C_BUS_ADDRESS << 1; // shift device address left
    transferSequence.flags = I2C_FLAG_WRITE;
    transferSequence.buf[0].data = &cmd_data; // pointer to data to write
    transferSequence.buf[0].len = sizeof(cmd_data);
    transferStatus = I2CSPM_Transfer (I2C0, &transferSequence);

    if (transferStatus != i2cTransferDone)
      {
     LOG_ERROR ("Write error\n\r");
      }

}

/**************************************************************************//**
 * Function to command the sensor to read temp measurement
 *****************************************************************************/


void readMeasurement()
{

  I2C_TransferReturn_TypeDef transferStatus; // make this global for IRQs in A4
  I2C_TransferSeq_TypeDef transferSequence; // this one can be local
  //uint8_t cmd_data; // make this global for IRQs in A4
  //read_data; // make this global for IRQs in A4
  //cmd_data = 0xE0;
  uint8_t read_data[2];
  transferSequence.addr = SI7021_I2C_BUS_ADDRESS << 1; // shift device address left
  transferSequence.flags = I2C_FLAG_READ;
  transferSequence.buf[0].data = &read_data[0]; // pointer to data to write
  transferSequence.buf[0].len = sizeof(read_data);
  transferStatus = I2CSPM_Transfer (I2C0, &transferSequence);

  if (transferStatus != i2cTransferDone)
    {
   LOG_ERROR ("read error\n\r");
   return;
    }

  //code for converting read data to celsius degrees and print on serial output

  uint32_t tempValue;
  float actual_temp;
  uint32_t rounded_temp;

  // Formula to decode read Temperature from the Si7021 Datasheet
  tempValue = ((uint32_t) read_data[0] << 8) + (read_data[1] & 0xfc);
  actual_temp = (((tempValue) * 175.72) / 65536) - 46.85;

  // Round the temperature to an integer value
  rounded_temp = (uint32_t)(actual_temp + 0.5 - (actual_temp < 0));

  LOG_INFO("temperature:%d\n\r",rounded_temp);

}


