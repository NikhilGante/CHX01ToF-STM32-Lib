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
 * @file usr_bsp.c
 * @version $Id: usr_bsp.c $
 * @author  Software Solutions Dev. Dept.  \n
 *          Advanced Products Development Center  \n
 *          Technology & Intellectual Property HQ  \n
 * @copyright  Copyright(C) 2022 TDK Corporation All rights reserved.
 *
 * @brief  BSP interface for RP2040(Raspberry Pi Pico)
 * 
 **/

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
//#include "pico/stdlib.h"
//#include "class/cdc/cdc_device.h"
#include <usr_bsp.h>

#include "constant.h"
#include "usr_def.h"
#include "stm32l1xx_hal_gpio.h"

/*
 * External variable declaration
 */
// struct repeating_timer sPicoTimer;	// Unused!
// static uint8_t sTimerOccuredCount = 0;	// Unused!
// static uint8_t sTimerAcceptCount = 0;	// Unused!
// static uint8_t gChirpDev.IRQFlag = 0;

/*
 * Prototype declaration
 */
// bool USR_TimerCallback(struct repeating_timer *t);
void USR_IntrCallback(uint gpio, uint32_t events);
#define UNUSED_VARIABLE(x) (void)(x)

/*=========================================================================================
 * \brief	Periodic timer callback routine
 * \param	t
 * \return	
============================================================================================*/
// bool USR_TimerCallback(struct repeating_timer *t)
// {
// 	UNUSED_VARIABLE(t);
// 	sTimerOccuredCount++;
// 	return(true);
// }

/*=========================================================================================
 * \brief	Enable timer to start measurement
 * \param	sample_interval		measurement interval
 * \return	none
============================================================================================*/
// void USR_TimerEnable(uint16_t sample_interval)
// {
// 	(void)add_repeating_timer_ms((int32_t)sample_interval, USR_TimerCallback,  NULL, &sPicoTimer); 
// }

/*=========================================================================================
 * \brief	Disable timer to stop measurement
 * \param	none
 * \return	none
============================================================================================*/
// void USR_TimerDisable(void)
// {
// 	(void)cancel_repeating_timer (&sPicoTimer);
// }


/*=========================================================================================
 * \brief	Get timer status flag
 * \param	none
 * \return	timer finished or not
============================================================================================*/
// uint8_t USR_TimerCallbackOccurred(void)		// Unused!
// {
// 	if (sTimerAcceptCount != sTimerOccuredCount) {
// 		sTimerAcceptCount = sTimerOccuredCount;
// 		return RET_FINISH;
// 	}
	
// 	return RET_NOT_FINISH;
// }

/*=========================================================================================
 * \brief	GPIO interrupts callback routine
 * \param	gpio	GPIO pin where the interrupt occurred
 *			events	events that occurred
 * \return	none
============================================================================================*/
// void USR_IntrCallback(uint gpio, uint32_t events)
// {
// 	UNUSED_VARIABLE(events);
// 	if (gpio == gChirpDev.int_pin) {
// 		gChirpDev.IRQFlag= 1;
// 	}
// }

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  /* Prevent unused argument(s) compilation warning */
	CHX01_ToF::handleCallBack(GPIO_Pin);
  /* NOTE : This function Should not be modified, when the callback is needed,
            the HAL_GPIO_EXTI_Callback could be implemented in the user file
   */
}


/*=========================================================================================
 * \brief	Enable GPIO interrupts
 * \param	pin		GPIO pin
 * \return	none
============================================================================================*/
void USR_IntrEnable()
{
	// static uint8_t sPassed = 0;
	
	// if ((int8_t)sPassed == 0) {
	// 	sPassed = 1;
	// 	// 0x8u means interrupt line is HIGH (should be able to use rising edge event as well)
	// 	gpio_set_irq_enabled_with_callback(pin, 0x8u, true, USR_IntrCallback);		/* posedge */

	// } else {
	//     gpio_set_irq_enabled(pin, 0x8u, true);		/* posedge */
	// }

	HAL_NVIC_EnableIRQ(gChirpDev.int_line);	// NOTE: this enables the entire line, so other ints on this line will also be enabled
}

/*=========================================================================================
 * \brief	Disable GPIO interrupts
 * \param	pin		GPIO pin
 * \return	none
============================================================================================*/
void USR_IntrDisable()
{
    // gpio_set_irq_enabled(pin, 0x8u, false);
	gChirpDev.IRQFlag = 0;
	HAL_NVIC_DisableIRQ(gChirpDev.int_line);	// NOTE: this disables the entire line, so other ints on this line will also be disabled
	// __HAL_GPIO_EXTI_CLEAR_IT(INT_PIN_NUM);
}

/*=========================================================================================
 * \brief	Get interrupt status flag
 * \param	none
 * \return	status flag
============================================================================================*/
uint8_t USR_IntrCallbackOccured(void)
{
	uint8_t ret;

	ret = gChirpDev.IRQFlag;
	// ret = __HAL_GPIO_EXTI_GET_IT(INT_PIN_NUM);

	return ret;
}

/*=========================================================================================
 * \brief	Clear interrupt status flag
 * \param	none
 * \return	none
============================================================================================*/
void USR_IntrCallbackRefresh(void)
{
	/* Interrupt_disable */
	// USR_IntrDisable();

	gChirpDev.IRQFlag = 0; // Flag now gets cleared in USR_IntrDisable
	
	/* Interrupt_enable */
	// USR_IntrEnable();
}

/*=========================================================================================
 * \brief	Initialize  I/O pins
 * \param	none
 * \return	none
============================================================================================*/
void USR_GPIO_Init(void)
{

	gChirpDev.reset_pin->setMode(GPIO_MODE_OUTPUT_PP_CUSTOM, GPIO_PIN_RESET);
	gChirpDev.prog_pin->setMode(GPIO_MODE_OUTPUT_PP_CUSTOM, GPIO_PIN_RESET);
	gChirpDev.int_pin->setMode(GPIO_MODE_OUTPUT_PP_CUSTOM, GPIO_PIN_RESET);
	gChirpDev.dir_pin->setMode(GPIO_MODE_OUTPUT_PP_CUSTOM, GPIO_PIN_SET/*DIR_HOST_TO_SENSOR*/);



//	gChirpDev.dir_pin->setMode(GPIO_MODE_OUTPUT_PP_3NFW, GPIO_STATE_LOW_3NFW/*DIR_SENSOR_TO_HOST*/);
}

/*=========================================================================================
 * \brief	Set GPIO pin to High or Low
 * \param	pin		GPIO pin
 *			level	HIGH or LOW
 * \return	none
============================================================================================*/
void USR_GPIO_Set(GpioPin& pin, uint8_t level)
{
	bool val=false;
	if (level!=0u)
	{
		val=true;
	}
	else
	{
		val=false;
	}
//	gpio_put(pin, val);
	HAL_GPIO_WritePin(pin.port, pin.pin, (GPIO_PinState)val);
	// pin->setOutput((tdeGpioState)val);
}

/*=========================================================================================
 * \brief	Get the level of the GPIO pin
 * \param	pin		GPIO pin
 * \return	the level of the GPIO pin
============================================================================================*/
// uint8_t USR_GPIO_Get(uint8_t pin)
// {
// 	uint8_t ret=0;
// 	if ( gpio_get(pin) == true )
// 	{
// 		ret=1;
// 	}
// 	else
// 	{
// 		ret=0;
// 	}

// 	return ret;
// }

/*=========================================================================================
 * \brief	Set the direction of GPIO pin (either Ext interrupt with rising edge (0) or output (1)
 * \param	pin		GPIO pin
 *			dir		IN or OUT
 * \return	none
============================================================================================*/
void USR_GPIO_SetDir(GpioPin& pin, uint8_t dir)
{
//	bool val=false;
	if (dir==(uint8_t)GPIO_IN)
	{
//		val = false;
		USR_IntrEnable();
		pin->setMode(GPIO_MODE_IT_RISING_CUSTOM, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(pin.port, pin.pin, GPIO_PIN_RESET);
	}
	else
	{
//		val = true;
		USR_IntrDisable();
		pin->setMode(GPIO_MODE_OUTPUT_PP_CUSTOM, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(pin.port, pin.pin, GPIO_PIN_SET);
	} 
//	gpio_set_dir(pin, val);
}

/*=========================================================================================
 * \brief	Set pull up of GPIO pin
 * \param	pin		GPIO pin
 * \return	none
============================================================================================*/
// void USR_GPIO_SetPullUp(uint8_t pin)	// Unused!
// {
// 	gpio_pull_up(pin);
// }

/*=========================================================================================
 * \brief	Set pull down of GPIO pin
 * \param	pin		GPIO pin
 * \return	none
============================================================================================*/
void USR_GPIO_SetPullDown(GpioPin& pin)	// WARNING: sets pin to low because of how stm init works
{
	pin->setMode(GPIO_MODE_INPUT_PD_CUSTOM, GPIO_PIN_RESET);

	// GPIO_InitTypeDef GPIO_InitStruct;
	// GPIO_InitStruct.Pin = pin.pin;
	// GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	// GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	// GPIO_InitStruct.Pull = GPIO_PULLDOWN;

	// HAL_GPIO_Init(pin.port, &GPIO_InitStruct);

	// HAL_GPIO_WritePin(pin.port, pin.pin, GPIO_PIN_RESET);
}

/*=========================================================================================
 * \brief	Wait time
 * \param	time	wait time(msec)
 * \return	none
============================================================================================*/
void USR_WaitMS(uint16_t t_ms)
{
	HAL_Delay(t_ms);
}

/*=========================================================================================
 * \brief	Wait time
 * \param	time	wait time(usec)
 * \return	none
============================================================================================*/
// void USR_WaitUS(uint16_t t_us)	// Unused!
// {
// 	busy_wait_us(t_us);
// }
