/*
 * scheduler.h
 *
 *  Created on: Feb 10, 2023
 *      Author: akash
 */

#ifndef SRC_SCHEDULER_H_
#define SRC_SCHEDULER_H_

#include "sl_bt_api.h"


void schedulerSetEvent3s();
uint8_t getEvent();
void setUFEvent();
void setCOMP1Event();
void sendPayrollIndication();
void SetPB0Press();
void SetPB0Release();
void SetPB1Press();
void SetPB1Release();
void setCOMP1Event();
void setCaptureEvent();
void setIdentifyEvent();
void setEventToggleIndication();


void setI2CCompleteEvent();
void stateMachine(sl_bt_msg_t *evt);


#endif /* SRC_SCHEDULER_H_ */
