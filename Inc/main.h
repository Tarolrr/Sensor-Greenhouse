/*
 * main.h
 *
 *  Created on: Oct 15, 2016
 *      Author: Tarolrr
 */

#ifndef MAIN_H_
#define MAIN_H_

#include <stdlib.h>
#include <stdbool.h>
#include "pin_description.h"
//#include "SensorDesc.h"
//#include "ActuatorDesc.h"
#include "GUID.h"
#include "ChannelDesc.h"

const uint64_t GUID[] = {
	0x848B04992EC34F28,
	0x95DB3576EFA594A0
};

#define LoRaDeviceType_Sensor		0x0000

const uint8_t NetworkID = 0x01;
const uint16_t LoRaAddress = 4 | LoRaDeviceType_Sensor;

#define ChannelNumber 				1
#define NormalWakeupPeriodInMinutes		10
#define AlertWakeupPeriodInMinutes		1
#define MinimumRxTimeout 500
#define RetryCount 5


#endif /* MAIN_H_ */
