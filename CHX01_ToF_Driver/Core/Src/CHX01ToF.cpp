/*
 * CHX01ToF.cpp
 *
 *  Created on: Aug 2, 2024
 *      Author: nikhil
 */

#include "CHX01ToF.h"
#include <algorithm>

//I2cStm32L1 i2c_com(&hi2c2);
extern I2C_HandleTypeDef hi2c2;
// GpioStm32L1 RST_PIN(GPIO_PORT_B_3NFW, 12);

// GpioStm32L1 PROG_PIN(GPIO_PORT_B_3NFW, 13);
// GpioStm32L1 INT_PIN(GPIO_PORT_B_3NFW, 14);
// GpioStm32L1 DIR_PIN(GPIO_PORT_B_3NFW, 15);

GpioPin RST_PIN = {GPIOB, 12};
GpioPin PROG_PIN = {GPIOB, 13};
GpioPin INT_PIN = {GPIOB, 14};
GpioPin DIR_PIN = {GPIOB, 15};


CHX01_ToF tof(hi2c2, &RST_PIN, &PROG_PIN, &INT_PIN, &DIR_PIN, EXTI15_10_IRQn, GPIO_PIN_14);

CHX01_ToF::CHX01_ToF(I2C_HandleTypeDef& I2cBus, GpioPin* reset_pin, GpioPin* prog_pin, GpioPin* int_pin, GpioPin* dir_pin, IRQn_Type int_line, uint16_t int_pin_num) {
				//mode            interval	range	reso	addr	i2cbus		reset		prog		int			dir			int_line	int_pin_num		rsv1	rsv2
	gChirpDev = {CH_MODE_FREERUN,     500,	750,	1,	   0x30,	I2cBus,	reset_pin, 	prog_pin,	int_pin,	dir_pin,	int_line,	int_pin_num,	0,		0};

}

void CHX01_ToF::init(uint16_t sensorType, uint16_t maxRange_mm, uint16_t interval, float offset_cm, float scaleFactor){
	setOffset(offset_cm);
	setScaleFactor(scaleFactor);

	USR_GPIO_SetPullDown(gChirpDev.int_pin);	// Ensure noise doesn't trigger interrupt

	int8_t	ret;
	ret = CH_API_Init(sensorType);
	ret = CH_API_Config(CH_MODE_FREERUN, maxRange_mm, interval);

	if (ret == RET_OK) {
	  DEBUG_PRINT_STR("\nConfiguration is succeeded\n");
	} else {
	  DEBUG_PRINT_STR("\nConfiguration is failed\n");
	}
	CH_API_MeasStart();
}

void CHX01_ToF::handleCallBack(uint16_t GPIO_Pin){
	if(GPIO_Pin == gChirpDev.int_pin_num){
		gChirpDev.IRQFlag = 1;	// Raise flag to indicate interrupt has been triggered
	}
}

// Instead return an std::optional<float> to signify if target was detected?
float CHX01_ToF::readDist(tdeDistUnit distUnit){

	int8_t	ret_ch = RET_NG;

	chirp_result_t  chResult;

	ret_ch = CH_API_GetResult(&chResult);

	if (ret_ch != RET_FINISH){	// Data not available yet
		return -1.0f;
	}
	if (chResult.range == CH_NO_TARGET){	// No target found
		return -1.0f;
	}

	float dist = (__convertToUnit(chResult.range, distUnit) /_scaleFactor) - _offset_cm;
	static float filteredVal;
	if(_filterStart){
		filteredVal = dist;
		_filterStart = false;
	}
	filteredVal = filteredVal*(1.0 - _filterConstant) + dist * _filterConstant;
	return  _filterEnabled? filteredVal : dist;
}

float CHX01_ToF::readDistMedian(tdeDistUnit distUnit, uint8_t times, uint16_t delayMs){
	float arr[times];

	float oneMeasurement = RET_NG;

	const int failsAllowed = 5;	// How many CONSECTUTIVE times reading is allowed to fail before returning -1
	int failCount = 0;
	int i = 0;
	while (i < times) {

		oneMeasurement = readDist(distUnit);

		if (oneMeasurement == RET_NG){	// Data not available yet
//			return -1.0f;
			failCount++;
			if(failCount > failsAllowed){
				return -1.0f;
			}
			HAL_Delay(delayMs);
			continue;
		}
		else failCount = 0;


		HAL_Delay(delayMs);

		arr[i] = oneMeasurement;
		 ++i;
	}
	std::sort(arr, arr + times);
	if(times & 1)	return  arr[times/2];	// Times is odd

	float dist = (arr[times/2] + arr[uint8_t(times/2) - 1]) / 2.0;

//	static float filteredVal;
//	if(_filterStart){
//		filteredVal = dist;
//		_filterStart = false;
//	}
//	filteredVal = filteredVal*(1.0 - _filterConstant) + dist * _filterConstant;
//	return  _filterEnabled? filteredVal : dist;
	return dist;
}

float CHX01_ToF::readDistAvg(tdeDistUnit distUnit, uint8_t times, uint16_t delayMs){

	float oneMeasurement = RET_NG;

	float avg = 0;
	for (int i = 0; i < times; ++i) {

		oneMeasurement = readDist(distUnit);
		if (oneMeasurement == RET_NG){	// Data not available yet
			return -1.0f;
		}

		HAL_Delay(delayMs);

		avg += oneMeasurement;
	}

	return avg/(float)times;
}

float CHX01_ToF::__convertToUnit(float val, tdeDistUnit distUnit){
	switch(distUnit){
	case DIST_UNIT_MM:
		return val/3.2f;
	case DIST_UNIT_CM:
		return val/32.0f;
	case DIST_UNIT_M:
		return val/3200.0f;
	case DIST_UNIT_INCH:
		return val/3.2/25.4f;
	}
	return -1.0f;
}

// Setters and getters

void CHX01_ToF::enableFilter(bool enable){
	_filterEnabled = enable;
	if(_filterEnabled){
		_filterStart = true;	// trigger start
	}
}

bool CHX01_ToF::isFilteredEnabled() const{
	return _filterEnabled;
}


void CHX01_ToF::setFilterConstant(float filterConstant){
	_filterConstant = filterConstant;
}
float CHX01_ToF::getFilterConstant()	const{
	return _filterConstant;
}


void CHX01_ToF::setOffset(float offset_cm){
	_offset_cm = offset_cm;
}
float CHX01_ToF::getOffset()	const{
	return _offset_cm;
}


void CHX01_ToF::setScaleFactor(float scaleFactor){
	_scaleFactor = scaleFactor;
}
float CHX01_ToF::setScaleFactor()	const{
	return _scaleFactor;
}
