/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */


// Chirp stuff
#include <stdio.h>
#include <string.h>
#include <cmath>
//#include "class/cdc/cdc_device.h"
#include "Chirp/usr_def.h"
#include "Chirp/constant.h"
#include "Chirp/ch_lib.h"
#include "Chirp/usr_bsp.h"

#include "CHX01ToF.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Chirp

#define MEAS_BUF_SIZE            (10u)	/* Median calculate buffer */

#define SENSOR_TYPE         (CHIRP_TYPE_CH101)
#define RANGE_LIMIT_UPPER   (320.0)
#define RANGE_LIMIT_LOWER   (10.0)
#define AMP_MAX             (30000u)

typedef float float32_t;

typedef struct {
	float32_t   range;				// from ch_get_range()
	uint16_t	amplitude;			// from ch_get_amplitude()
	uint16_t	reserved;			// alignment */
} stMeasResult_t;

#define	MMODE_MOFMOF		(0U)
#define	MMODE_DISTANCE		(1U)
static	uint8_t	sMeasureMode = MMODE_DISTANCE;

stMeasResult_t	sMdn_buf[MEAS_BUF_SIZE];
uint8_t			sMdn_widx;
uint8_t			sMdn_cnt;

void	execute_cmd( uint8_t cmdcode );

float32_t	measure_mof( chirp_result_t *ch_result );
float32_t	measure_dist( chirp_result_t *ch_result );

void	clrMedian( void );
int8_t	calcMedian(stMeasResult_t *onemeas, float32_t *range, uint16_t *amp);

int __io_putchar(int ch)
{
 // Write character to ITM ch.0
 ITM_SendChar(ch);
 return(ch);
}

//float32_t dist = 0.0;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C2_Init();
  MX_I2C1_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  for (uint8_t i = 1; i < 128; ++i) {	// I2C scan

	  if (HAL_I2C_IsDeviceReady(&hi2c2, i << 1, 1, 100)) {
		  HAL_Delay(10);
		  HAL_GPIO_WritePin(LedB_GPIO_Port, LedB_Pin, GPIO_PIN_SET);
//		  break;
	  }
  }
//  HAL_Delay(500);

  // Chirp Stuff
  chirp_result_t  ch_result;

  int8_t       	ret_ch = RET_NG;

  // My init

//  GpioStm32L1 RST_PIN(GPIO_PORT_B_3NFW, 12);
//
//  GpioStm32L1 PROG_PIN(GPIO_PORT_B_3NFW, 13);
//  GpioStm32L1 INT_PIN(GPIO_PORT_B_3NFW, 14);
//  GpioStm32L1 DIR_PIN(GPIO_PORT_B_3NFW, 15);

//  CHX01_ToF tof(&i2c_com, &RST_PIN, &PROG_PIN, &INT_PIN, &DIR_PIN, EXTI15_10_IRQn, GPIO_PIN_14);
//  tof.init(1, 500, 50, -0.416, 1.26);
  tof.init(1, 500, 50, 0.0);
  tof.setFilterConstant(0.3);
  HAL_Delay(500);
//  int8_t	ret;
//  ret = CH_API_Init(SENSOR_TYPE);
//  ret = CH_API_Config(CH_MODE_FREERUN, 500, 50);
////  CH_API_Config(CH_MODE_FREERUN, 500, 400);
//   if (ret == RET_OK) {
//      DEBUG_PRINT_STR("\nConfiguration is succeeded\n");
//   } else {
//      DEBUG_PRINT_STR("\nConfiguration is failed\n");
//   }
//   CH_API_MeasStart();
  HAL_GPIO_WritePin(LedB_GPIO_Port, LedB_Pin, GPIO_PIN_RESET);
  float dist;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

        //Get result if data is ready
//        ret_ch = CH_API_GetResult(&ch_result);
//		if ( ret_ch != RET_FINISH ){
//			continue;
//		} else { /* else nothing */ }
//
//		if ( ch_result.range == CH_NO_TARGET ){
//			// DEBUG_PRINT_STR("no target found\n");
//			dist = ch_result.range;
//			continue;
//		} else { /* else nothing */ }
//
//		switch( sMeasureMode ){
//		case MMODE_MOFMOF:
//			dist = measure_mof( &ch_result );
//			break;
//		case MMODE_DISTANCE:
//		default:
//			dist = measure_dist( &ch_result );
//			break;
//		}
//		CH_API_MeasStop();

//	  	dist = tof.readDist(DIST_UNIT_CM);
	  	dist = tof.readDistMedian(DIST_UNIT_CM, 20, 50);
//	  	dist = tof.readDistAverage(DIST_UNIT_CM, 10, 10);

//		static float last = dist;
////		float alpha = fabs(dist - last) > 5.0 ? 0.5 : 0.05; // Tuning value
//		float alpha = 0.01;
//		last = last*(1.0 - alpha) + dist * alpha;

	  	uint8_t buf[30];
//	  	sprintf((char*)buf, "dist: %.2fcm | %.2fcm\n", dist, last);
//	  	sprintf((char*)buf, "dist: %.2fcm\n", dist);
	  	sprintf((char*)buf, "%.2f\n", dist);
	  	HAL_UART_Transmit(&huart1, buf, strlen((char*)buf), HAL_MAX_DELAY);

		HAL_Delay(50);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV4;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/*=========================================================================================
 * \brief	command execution
 * \param	uint8_t cmdcode
 * \return	void
============================================================================================*/
void execute_cmd( uint8_t cmdcode )
{
//	int8_t	ret;
//
//	switch( cmdcode ){
//	case SEN_CMDCODE_STOP: /* \r */
//		// DEBUG_PRINT_STR("---------------------------------------\n");
//	    CH_API_MeasStop();
//		break;
//	case SEN_CMDCODE_SET: /* set */
//		// DEBUG_PRINT_STR("---------------------------------------\n");
//        (void)CH_API_Init(SENSOR_TYPE);
//		break;
//	case SEN_CMDCODE_MOF: /* mof */
//		sMeasureMode = MMODE_MOFMOF;
//		ret = CH_API_Config(CH_MODE_FREERUN, 500, 400);
//        if (ret == RET_OK) {
//        	// DEBUG_PRINT_STR("\nConfiguration is succeeded\n");
//		} else {
//			// DEBUG_PRINT_STR("\nConfiguration is failed\n");
//		}
//		CH_API_MeasStart();
//		break;
//	case SEN_CMDCODE_DIST:	/* dist */
//		sMeasureMode = MMODE_DISTANCE;
//		ret = CH_API_Config(CH_MODE_FREERUN, 500, 400);
//		if (ret == RET_OK) {
//			// DEBUG_PRINT_STR("\nConfiguration is succeeded\n");
//		} else {
//			// DEBUG_PRINT_STR("\nConfiguration is failed\n");
//		}
//		CH_API_MeasStart();
//		break;
//	case SEN_CMDCODE_CLR:
//		clrMedian( );
//        // DEBUG_PRINT_STR("Data Buffer is cleared\n");
//		break;
//	default: /* include SEN_CMDCODE_UNKNOWN */
//		break;
//	}
//	return;
}

/*=========================================================================================
 * \brief	measure softness
 * \param	chirp_result_t *   measurement result
 * \return	void
============================================================================================*/
float32_t measure_mof( chirp_result_t *ch_result )
{
    float32_t meas_range   = 0.0f;
    float32_t median_range = -1.0f;
    uint16_t  median_amp   = 0;
    float32_t softness     = 0.0f;

	stMeasResult_t	onemeas;
	int8_t	data_count;

	meas_range = ((float32_t)ch_result->range)/32.0f;
	if(meas_range >= RANGE_LIMIT_UPPER){
		// DEBUG_PRINT_STR("too far \n");	/* Not included in median calculation */
	}
	else if (meas_range < RANGE_LIMIT_LOWER){
		// DEBUG_PRINT_STR("too near\n");	/* Not included in median calculation */
	}
	else{
		onemeas.range = meas_range;
		onemeas.amplitude = ((ch_result->amplitude > AMP_MAX) ? AMP_MAX : (ch_result->amplitude) );
		data_count = calcMedian( &onemeas , &median_range, &median_amp );
		if ( data_count < 0 ){
//			DEBUG_PRINTF("%d data is measured, need %d more data\n", (data_count*(-1)), ((int8_t)MEAS_BUF_SIZE - (data_count*(-1))) );
		}else{
			softness = ( ((float32_t)AMP_MAX - (float32_t)median_amp)/(float32_t)AMP_MAX ) * (float32_t)100;
//			DEBUG_PRINTF("Range: %0.1f mm  Amp: %u  Softness: %0.1f%%\n", median_range, median_amp, softness);
		}
	}
	return softness;
}

/*=========================================================================================
 * \brief	printf measurement result
 * \param	chirp_result_t *   measurement result
 * \return	void
============================================================================================*/
float32_t measure_dist( chirp_result_t *ch_result )
{
//	DEBUG_PRINTF("Range: %5.1f mm  Amp: %5u\n", ((float32_t)ch_result->range)/32.0f, ch_result->amplitude);
    return ((float32_t)ch_result->range)/3.2;
}

/*=========================================================================================
 * \brief	calculate median value
 * \param	stMeasResult_t*     new data for median value calculation
 *          *range   calculated range
 *          *amp     calculated amp
 * \return	zero
============================================================================================*/
int8_t calcMedian(stMeasResult_t *onemeas, float32_t *range, uint16_t *amp)
{
	uint8_t	i;
	uint8_t	j;
    int8_t ret_code;
	stMeasResult_t tmp;
	stMeasResult_t sort_buf[MEAS_BUF_SIZE];
	uint8_t cnt_meas_buf;

	sMdn_buf[sMdn_widx] = *onemeas;
	sMdn_widx ++;

	if ( sMdn_widx >= MEAS_BUF_SIZE ){
		sMdn_widx = 0;
	}
	if ( sMdn_cnt < MEAS_BUF_SIZE ){
		sMdn_cnt ++;
		ret_code = (int8_t)(sMdn_cnt) * -1;
	}
	else{ /* ( sMdn_cnt == MEAS_BUF_SIZE ) */
		ret_code = (int8_t)sMdn_cnt;

		/* Copy to buffer for sorting  */
		(void)memcpy( sort_buf , sMdn_buf, sizeof(sMdn_buf) );

		/* sort by amplitude */
		for (i=0; i<MEAS_BUF_SIZE; ++i) {
			for (j=i+1u; j< MEAS_BUF_SIZE; ++j) {
				if (sort_buf[i].amplitude > sort_buf[j].amplitude) {
					tmp =  sort_buf[i];
					sort_buf[i] = sort_buf[j];
					sort_buf[j] = tmp;
				}
			}
		}
		cnt_meas_buf = MEAS_BUF_SIZE;
		if ((cnt_meas_buf % 2u) == 0u) {
			*range = (sort_buf[MEAS_BUF_SIZE / 2u].range + sort_buf[(MEAS_BUF_SIZE / 2u) - 1u].range) / 2.0f;
			*amp   = (sort_buf[MEAS_BUF_SIZE / 2u].amplitude + sort_buf[(MEAS_BUF_SIZE / 2u) - 1u].amplitude) / 2u;
		} else {
			*range = sort_buf[MEAS_BUF_SIZE / 2u].range;
			*amp   = sort_buf[MEAS_BUF_SIZE / 2u].amplitude;
		}
	}
	return ( ret_code );
}

/*=========================================================================================
 * \brief	clear buffer for median value calculation
 * \param	void
 * \return	void
============================================================================================*/
void clrMedian( void  )
{
	sMdn_widx = 0;
	sMdn_cnt = 0;
	return;
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
