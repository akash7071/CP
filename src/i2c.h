/*
 * i2c.h
 *
 *  Created on: Feb 10, 2023
 *      Author: akash
 */

#ifndef SRC_I2C_H_
#define SRC_I2C_H_


void i2cInit();
void enableI2CGPIO();
void disableI2CGPIO();
void startMeasurement();
void dispTemperature();
void readMeasurement();
void read_temp_from_si7021();

#endif /* SRC_I2C_H_ */
