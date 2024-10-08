/*
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
 * SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION
 * OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN
 * CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 *
 */

/**
 * @file ch_drv.c
 * @version $Id: ch_drv.c $
 * @author  Software Solutions Dev. Dept.  \n
 *          Advanced Products Development Center  \n
 *          Technology & Intellectual Property HQ  \n
 * @copyright  Copyright(C) 2022 TDK Corporation All rights reserved.
 *
 * @brief Module to control the ultrasonic sensor system
 * 
 **/

//#include "ch_types.h"
#include "ch101_gpr.h"
#include "constant.h"
#include "ch_com.h"
#include "usr_bsp.h"
#include "usr_def.h"
#include "ch_drv_ch101.h"

/*
 * Defines
 */

/*
 * External variable declaration
 */
static chirp_calib_t sCalib;

/*
 * Prototype declaration
 */
static uint16_t DRV_mmToSamples_CH101(uint16_t num_mm);
static void DRV_GenPulseMS_CH101(uint16_t pulse_width);


/*=========================================================================================
 * \brief	Load firmware
 * \param	none
 * \return	RET_OK succeeded, RET_NG otherwise
============================================================================================*/
int8_t DRV_FWLoad_CH101(void)
{
	int8_t ret = RET_NG;
	uint8_t i2c_addr_app;
	uint16_t write_val;
	const uint8_t* pt_ram = &ram_ch101_gpr_init[0];
	const uint8_t* pt_fw = &ch101_gpr_fw[0];

	/* PROG pin assert */
	USR_GPIO_Set(gChirpDev.prog_pin, HIGH);

	/* transfer ram value */
	ret = COM_WriteMemBurst(CH101_RAM_INIT_ADDRESS, pt_ram, CH101_RAM_INIT_WRITE_SIZE);

	/* transfer firmware */
	if (ret == RET_OK) {
		ret = COM_WriteMemBurst(CH101_PROG_MEM_ADDR, pt_fw, CH101_FW_SIZE);
	}
	if (ret == RET_OK) {		
		DEBUG_PRINT_STR("FW Load: Successful\n");
	} else {
		DEBUG_PRINT_STR("FW Load: Failed\n");
	}

	/* asic reset and halt */
	if (ret == RET_OK) {
		ret = COM_WriteRegProg(CH_PROG_REG_CPU, 0x40);	// reset asic
	}
	if (ret == RET_OK) {
		ret = COM_WriteRegProg(CH_PROG_REG_CPU, 0x11);	// halt asic and disable watchdog
	}

	/* transfer an I2C address to the sensor */
	if (ret == RET_OK) {
		i2c_addr_app = gChirpDev.i2c_addr;
		ret = COM_WriteMem(0x01C5, (uint16_t)i2c_addr_app, 1);
	}

	/* Run charge pumps */
	if (ret == RET_OK) {
		write_val = 0x0200;
		ret  = COM_WriteMem(0x01A6, write_val, 2); // PMUT.CNTRL4 = HVVSS_FON
		USR_WaitMS(5);
	}
	if (ret == RET_OK) {
		write_val = 0x0600;
		ret = COM_WriteMem(0x01A6, write_val, 2); // PMUT.CNTRL4 = (HVVSS_FON | HVVDD_FON)
		USR_WaitMS(5);
	}
	if (ret == RET_OK) {
		write_val = 0x0000;
		ret = COM_WriteMem(0x01A6, write_val, 2); // PMUT.CNTRL4 = 0
	}

	/* asic run */
	if (ret == RET_OK) {
		ret = COM_WriteRegProg(CH_PROG_REG_CPU, 2); // Exit programming mode and run the chip
	}

	/* PROG pin de-assert */
	USR_GPIO_Set(gChirpDev.prog_pin, LOW);

	return ret;
}


/*=========================================================================================
 * \brief	Wait for an individual sensor to finish start-up procedure.
 * \param	timeout_ms	timeout waiting for confirmation
 * \return	RET_OK succeeded, RET_NG otherwise
============================================================================================*/
int8_t DRV_WaitForLock_CH101(uint16_t timeout_ms)
{
	int8_t ret = RET_NG;
	int8_t com;
	uint16_t count = (timeout_ms / 10u) + 1u;
	uint16_t ready_value = 0;

	while (count > 0u) {
		USR_WaitMS(10);
		count--;

		/* Check the status of the sensor */
		com = COM_ReadReg(gChirpDev.i2c_addr, CH101_COMMON_REG_READY, &ready_value, 1);
		if (com == RET_OK) {
			if ((ready_value & CH101_COMMON_READY_FREQ_LOCKED) != 0u) {
				ret = RET_OK;
				break;
			}
		}
	}

	if (ret == RET_OK) {
		DEBUG_PRINT_STR("Ready  : Successful\n");
	} else {
		DEBUG_PRINT_STR("Ready  : Failed\n");
	}

	return ret;
}


/*=========================================================================================
 * \brief	Calibrate the sensor real-time clock against the host microcontroller clock.
 * \param	none
 * \return	RET_OK succeeded, RET_NG otherwise
============================================================================================*/
int8_t DRV_MeasRTC_CH101(void)
{
	uint16_t calc_val;
	uint32_t count;
	uint16_t read_val;
	uint32_t num;
	uint32_t den;
	int8_t ret = RET_NG;

	/* Preparing the trigger pulse */
	ret = COM_WriteRegApp(gChirpDev.i2c_addr, CH101_COMMON_REG_CAL_TRIG, 0x00, 1);


	/* Trigger a pulse on the INT pin */
	if (ret == RET_OK) {
		DRV_GenPulseMS_CH101(RTC_CAL_PULSE_MS);

		/* Keep the IO low for at least 50us to allow the ASIC FW to deactivate the PT logic */
		USR_WaitMS(100);
	}

	/* Get calibration result */
	if (ret == RET_OK) {
		ret = COM_ReadReg(gChirpDev.i2c_addr, CH101_GPR_OPEN_REG_CAL_RESULT, &read_val, 2); //read rtc_cal_result

		if (ret == RET_OK) {
			sCalib.rtc_result = read_val;

			count = ((uint32_t)read_val * 1000u) / RTC_CAL_PULSE_MS;
			calc_val = (uint16_t)((uint32_t)CH101_GPR_OPEN_CTR * 16U * CH101_COMMON_FREQCOUNTERCYCLES / count);
			ret = COM_WriteRegApp(gChirpDev.i2c_addr, CH101_GPR_OPEN_REG_CALC, calc_val, 2);

		}
	}

	/* Get frequency */
	if (ret == RET_OK) {
		ret = COM_ReadReg(gChirpDev.i2c_addr, CH101_COMMON_REG_TOF_SF, &read_val, 2); //read raw_freq

		if (ret == RET_OK) {
			num = ((((uint32_t)sCalib.rtc_result)*1000U) / (16U * CH101_COMMON_FREQCOUNTERCYCLES)) * (uint32_t)(read_val);
			den = (uint32_t)RTC_CAL_PULSE_MS;
			sCalib.freq = num / den;
		}
	}

	/* Get scale factor */
	if (ret == RET_OK) {
		ret = COM_ReadReg(gChirpDev.i2c_addr, CH101_COMMON_REG_TOF_SF, &read_val, 2); //read scale_factor
		if (ret == RET_OK) {
			sCalib.scale_factor = read_val;
		} else {
			sCalib.scale_factor = 0;
		}
	}


	DEBUG_PRINTF("Freq             =  %ld\n",sCalib.freq);
	DEBUG_PRINTF("RTC calib result =  %d\n",sCalib.rtc_result);
	DEBUG_PRINTF("Scale Factor     =  %d\n",sCalib.scale_factor);
	
	return ret;
}


/*=========================================================================================
 * \brief	Set the mode of the sensor.
 * \param	mode	mode of the sensor
 * \return	RET_OK succeeded, RET_NG otherwise
============================================================================================*/
int8_t DRV_SetMode_CH101(uint8_t mode)
{
	int8_t ret = RET_NG;

	switch (mode) {
		case CH_MODE_IDLE:
			ret = COM_WriteRegApp(gChirpDev.i2c_addr, CH101_COMMON_REG_OPMODE, CH_MODE_IDLE, 1);

			if (ret == RET_OK) {
				ret = COM_WriteRegApp(gChirpDev.i2c_addr, CH101_COMMON_REG_PERIOD, 0x00, 1);

			}
			if (ret == RET_OK) {
				ret = COM_WriteRegApp(gChirpDev.i2c_addr, CH101_COMMON_REG_TICK_INTERVAL, 2048, 2);

			}
			break;

		case CH_MODE_FREERUN:
			ret = COM_WriteRegApp(gChirpDev.i2c_addr, CH101_COMMON_REG_OPMODE, CH_MODE_FREERUN, 1);

			break;

		case CH_MODE_TRIGGERED_TX_RX:
			ret = COM_WriteRegApp(gChirpDev.i2c_addr, CH101_COMMON_REG_OPMODE, CH_MODE_TRIGGERED_TX_RX, 1);

			break;

		case CH_MODE_TRIGGERED_RX_ONLY:
			ret = COM_WriteRegApp(gChirpDev.i2c_addr, CH101_COMMON_REG_OPMODE, CH_MODE_TRIGGERED_RX_ONLY, 1);

			break;

		default:
			ret = RET_NG;
			break;
	}

	return ret;
}


/*=========================================================================================
 * \brief	Set the max range of the sensor.
 * \param	range	max range of the sensor
 * \return	RET_OK succeeded, RET_NG otherwise
============================================================================================*/
int8_t DRV_SetMaxRange_CH101(uint16_t range)
{
	int8_t ret = RET_NG;
	uint16_t num_samples;

	num_samples = DRV_mmToSamples_CH101(range);

	if (num_samples > CH101_GPR_OPEN_MAX_SAMPLES) {
		num_samples = CH101_GPR_OPEN_MAX_SAMPLES;
	}

	ret = COM_WriteRegApp(gChirpDev.i2c_addr, CH101_COMMON_REG_MAX_RANGE, num_samples, 1);

	return ret;
}

/*=========================================================================================
 * \brief	Set the static range of the sensor.
 * \param	samples		target sampling position
 * \return	RET_OK succeeded, RET_NG otherwise
============================================================================================*/
int8_t DRV_SetStaticRange_CH101(uint16_t samples)
{
	int8_t ret = RET_NG;

	ret = COM_WriteRegApp(gChirpDev.i2c_addr, CH101_COMMON_REG_STAT_RANGE, samples, 1);

	if (ret == RET_OK) {
		ret = COM_WriteRegApp(gChirpDev.i2c_addr, CH101_COMMON_REG_STAT_COEFF, CH101_COMMON_STAT_COEFF_DEFAULT, 1);
	}

	return ret;
}


/*=========================================================================================
 * \brief	Set the measurement interval.
 * \param	interval	interval time
 * \return	RET_OK succeeded, RET_NG otherwise
============================================================================================*/
int8_t DRV_SetSampleInterval_CH101(uint16_t interval)
{
	int8_t ret = RET_OK;
	uint32_t sample_interval;
	uint32_t period;
	uint32_t tick_interval;

	sample_interval = (uint32_t)sCalib.rtc_result * (uint32_t)interval / RTC_CAL_PULSE_MS;
	
	if (interval != 0u) {
		period = (sample_interval / 2048u) + 1u;
		if (period > UINT8_MAX_VALUE) {	/* check if result fits in register */
			ret = RET_NG;
		}
	} else {
		period = 0;
	}

	if (ret == RET_OK) {
		if (period != 0u) {
			tick_interval = sample_interval / period;
		} else {
			tick_interval = 0;
		}


		ret = COM_WriteRegApp(gChirpDev.i2c_addr, CH101_COMMON_REG_PERIOD, (uint16_t) period, 1);

		if (ret == RET_OK) {
			ret = COM_WriteRegApp(gChirpDev.i2c_addr, CH101_COMMON_REG_TICK_INTERVAL, (uint16_t) tick_interval, 2);

		}
	}
	
	return ret;
}


/*=========================================================================================
 * \brief	Convert range to sampling position.
 * \param	num_mm		range
 * \return	sampling position
============================================================================================*/
static uint16_t DRV_mmToSamples_CH101(uint16_t num_mm)
{
//	uint8_t  ret = RET_NG;
	uint16_t scale_factor;
	uint32_t num_samples = 0;
	uint32_t divisor1;
	uint32_t divisor2 = (RTC_CAL_PULSE_MS * CH_SPEEDOFSOUND_MPS);

	divisor1 = 0x2000;			// (4*16*128)  XXX need define(s)

	scale_factor = sCalib.scale_factor;
	num_samples = (((uint32_t)sCalib.rtc_result * scale_factor) + (divisor1 - 1u)) / divisor1;
	num_samples = ((num_samples * num_mm)  + (divisor2 - 1u)) / divisor2;		// oversample is not used in this firmware

	if (num_samples > UINT16_MAX_VALUE) {
		num_samples = 0;	// return zero if error
	} 

	return (uint16_t)num_samples;
}

#ifdef _NO_USE_ // no use
/*=========================================================================================
 * \brief	Convert sampling position to range.
 * \param	num_samples		sampling position
 * \return	range
============================================================================================*/
uint16_t DRV_SamplesTomm(uint16_t num_samples)
{
	uint32_t num_mm = 0;
	uint32_t op_freq;

	op_freq = sCalib.freq;
	if (op_freq != 0u) {
		num_mm = ((uint32_t)num_samples * CH_SPEEDOFSOUND_MPS * 8u * 1000u) / (op_freq * 2u);
	}
	
	return (uint16_t)num_mm;
}
#endif
// 

/*=========================================================================================
 * \brief	Get range
 * \param	none
 * \return	range
============================================================================================*/
uint32_t DRV_GetRange_CH101(void)
{
	uint32_t range = CH_NO_TARGET;
	uint16_t time_of_flight;
	uint16_t scale_factor;
	uint32_t num;
	uint32_t den;
	int8_t ret;

	ret = COM_ReadReg(gChirpDev.i2c_addr, CH101_COMMON_REG_TOF, &time_of_flight, 2);

	if ((ret == RET_OK) && (time_of_flight != UINT16_MAX_VALUE)) { // If object detected
		scale_factor = sCalib.scale_factor;
		if (scale_factor != 0u) {
			num = (CH_SPEEDOFSOUND_MPS * RTC_CAL_PULSE_MS * (uint32_t)time_of_flight);
			den = ((uint32_t)sCalib.rtc_result * (uint32_t)scale_factor) >> 11;
			range = num / den;
			range /= 2u;		// range_type = CH_RANGE_ECHO_ONE_WAY
		}
	}
	
	return range;
}

/*=========================================================================================
 * \brief	Get amplitude
 * \param	none
 * \return	amplitude
============================================================================================*/
uint16_t DRV_GetAmplitude_CH101(void)
{
	uint16_t amplitude = 0;
	int8_t ret;

	ret = COM_ReadReg(gChirpDev.i2c_addr, CH101_COMMON_REG_AMPLITUDE, &amplitude, 2);

	if (ret != RET_OK) {
		amplitude = 0;
	}

	return amplitude;
}


/*=========================================================================================
 * \brief	Generate pulse
 * \param	pulse_width		pulse width(msec)
 * \return	none
============================================================================================*/
static void DRV_GenPulseMS_CH101(uint16_t pulse_width)
{
	/* INT pin is the output setting */
	USR_GPIO_Set(gChirpDev.dir_pin, DIR_HOST_TO_SENSOR);
	USR_GPIO_SetDir(gChirpDev.int_pin, GPIO_OUT);

	/* Generate pulse */
	USR_GPIO_Set(gChirpDev.int_pin, HIGH);
	USR_WaitMS(pulse_width);
	USR_GPIO_Set(gChirpDev.int_pin, LOW);

	/* INT pin is the input setting */
	USR_GPIO_Set(gChirpDev.dir_pin, DIR_SENSOR_TO_HOST);
	USR_GPIO_SetDir(gChirpDev.int_pin, GPIO_IN);
	USR_GPIO_SetPullDown(gChirpDev.int_pin);
}

/*=========================================================================================
 * \brief	Generate pulse
 * \param	pulse_width		pulse width(usec)
 * \return	none
============================================================================================*/
void DRV_GenPulseUS_CH101(uint16_t pulse_width)	// Unused!
{
	/* INT pin is the output setting */
	USR_GPIO_Set(gChirpDev.dir_pin, DIR_HOST_TO_SENSOR);
	USR_GPIO_SetDir(gChirpDev.int_pin, GPIO_OUT);

	/* Generate pulse */
	USR_GPIO_Set(gChirpDev.int_pin, HIGH);
	USR_WaitMS(pulse_width);
	USR_GPIO_Set(gChirpDev.int_pin, LOW);

	/* INT pin is the input setting */
	USR_GPIO_Set(gChirpDev.dir_pin, DIR_SENSOR_TO_HOST);
	USR_GPIO_SetDir(gChirpDev.int_pin, GPIO_IN);
	USR_GPIO_SetPullDown(gChirpDev.int_pin);
}
