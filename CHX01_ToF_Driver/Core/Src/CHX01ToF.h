/*
 * CHX01ToF.h
 *
 *  Created on: Aug 2, 2024
 *      Author: Nikhil
 */

#ifndef SRC_CHX01TOF_H_
#define SRC_CHX01TOF_H_

#include "Chirp/usr_def.h"
#include "Chirp/constant.h"
#include "Chirp/ch_lib.h"
#include "Chirp/usr_bsp.h"

typedef enum{
	DIST_UNIT_MM = 0x00,
	DIST_UNIT_CM = 0x01,
	DIST_UNIT_M = 0x02,
	DIST_UNIT_INCH = 0x03
} tdeDistUnit;


class CHX01_ToF {
public:
	CHX01_ToF(
		I2C_HandleTypeDef& I2cBus,
		GpioPin& reset_pin,
		GpioPin& prog_pin,
		GpioPin&  int_pin, 
		GpioPin&  dir_pin, 
		IRQn_Type int_line, 
		uint16_t int_pin_num
	);

	/**
	 * @brief Burns firmware on CHX01 device and starts measurement in FREE_RUN mode
	 *
	 * @param sensorType	Should be either CHIRP_TYPE_CH101 or CHIRP_TYPE_CH102
	 * @param maxRange_mm 	Supposedly defines the max range device can measure,
	 * hasn't worked in my experience. Set between 40-1200 for CH101 and 40-5000 for CH102
	 * @param interval_ms	How often to sample using readDist (must be at least 10ms)
	 * @param offset_cm		See setOffset
	 * @param scaleFactor	See setScaleFactor
	 */
	void init(uint16_t sensorType, uint16_t maxRange_mm = 500, uint16_t interval_ms = 50, float offset_cm = 0.0, float scaleFactor = 1.0);


	/**
	 * @brief Sets device's IRQFlag, MUST be called in HAL_GPIO_EXTI_Callback
	 * HAL_GPIO_EXTI_Callback is found in stm32l1xx_hal_gpio.c
	 */
	static void handleCallBack(uint16_t GPIO_Pin);

	/**
	 * @brief Retrieves a single distance measurement from CHX01 device in distUnit
	 *	Should be called only every *interval* milliseconds (set in init method)
	 *	@param distUnit	Unit the distance is measured in (mm, cm, m or inch)
	 *	@return	Returns distance measured by sensor in distUnit
	 */
	float readDist(tdeDistUnit distUnit = DIST_UNIT_CM);

	// Same as readDist but measures for a number of times to calculate a median
	// Should be used most often for best data
	float readDistMedian(tdeDistUnit distUnit = DIST_UNIT_CM, uint8_t times = 1, uint16_t delay_ms = 50);

	// Same as readDist but measures for a number of times to calculate an average
	float readDistAvg(tdeDistUnit distUnit = DIST_UNIT_CM, uint8_t times = 1, uint16_t delay_ms = 50);

	/**
	 * @brief Determines whether call to readDist returns raw or filtered value
	 * @param enable	True enables the filter, false disables it
	 */
	void enableFilter(bool enable);
	bool isFilteredEnabled() const;

	/*
	 * @param filterConstant	Determines much of the current value to consider
	 * E.g. 0.01 means 1% of the current + 99% of previous filtered value
	 * Should be between 0.0 - 1.0
	*/
	void setFilterConstant(float filterConstant);
	float getFilterConstant()	const;

	// Offset subtracts from the reading value AFTER scaling
	void setOffset(float offset_cm);
	float getOffset()	const;

	// Reading value is DIVIDED by scale factor
	void setScaleFactor(float scaleFactor);
	float setScaleFactor()	const;


	//	float getSoftness()
	//	setStaticTargetRejectionRange(uint16_t range_cm)
	//	getStaticTargetRejectionRange
private:
	// Takes value from sensor and converts it to the desired unit
	float __convertToUnit(float val, tdeDistUnit distUnit = DIST_UNIT_MM);

	float _offset_cm;
	float _scaleFactor;
	bool _filterStart = false, _filterEnabled = false;
	float _filterConstant = 0.008;

};

extern CHX01_ToF tof;
#endif /* SRC_CHX01TOF_H_ */
