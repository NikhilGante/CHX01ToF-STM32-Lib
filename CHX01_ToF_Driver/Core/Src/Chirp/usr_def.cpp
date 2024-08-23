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
 * @file usr_def.c
 * @version $Id: usr_def.c $
 * @author  Software Solutions Dev. Dept.  \n
 *          Advanced Products Development Center  \n
 *          Technology & Intellectual Property HQ  \n
 * @copyright  Copyright(C) 2022 TDK Corporation All rights reserved.
 *
 * @brief  User Defined Ultrasonic Sensor System
 * 
 **/

//#include "pico/binary_info.h"

#include "usr_def.h"
#include "constant.h"

/*
 * External variable declaration
 */
/* Setting information for each sensor */
/* Note: If multiple sensors are connected to a level shifter,
   the same DIR pin definition is acceptable. */


                       	   //mode             intval	range	reso	add		prog		int				dir					rsv1	rsv2
//chirp_dev_t gChirpDev101 = {CH_MODE_FREERUN,     500,	750,	1,	   0x30,	&PROG_PIN,		&INT_PIN,		&DIR_PIN,		0,		0};
//
//chirp_dev_t gChirpDev201 = {CH_MODE_FREERUN,     500,	1500,	1,	   0x30,	&PROG_PIN,		&INT_PIN,		&DIR_PIN,		0,		0};

chirp_dev_t gChirpDev;

// Forward declarations
void HAL_GPIO_Init(GPIO_TypeDef  *GPIOx, GPIO_InitTypeDef *GPIO_Init);
void HAL_GPIO_WritePin(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState);


void GpioPin::setMode(tdeGpioMode eMode, uint8_t eInitState){
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.Pin = pin;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

	switch (eMode) {
		case GPIO_MODE_OUTPUT_PP_CUSTOM:
			GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
			break;
		case GPIO_MODE_INPUT_PD_CUSTOM:
			GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
			GPIO_InitStruct.Pull = GPIO_PULLDOWN;
			break;
		case GPIO_MODE_IT_RISING_CUSTOM:
			GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
			break;
	}
	HAL_GPIO_Init(port, &GPIO_InitStruct);
	HAL_GPIO_WritePin(port, pin, eInitState);
}

/*
 * Prototype declaration
 */
/* none */


