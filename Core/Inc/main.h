/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/*-----------------------------------------------------------------------
    INCLUDES
-----------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

#include <string.h>
typedef enum
{
	TOCABUZZ,
	DESLBUZZ,
	INIBUZZ,
	IDLE,

} enum_stt_buzzer;

/*-----------------------------------------------------------------------
    EXTERN VARIABLES
-----------------------------------------------------------------------*/
extern volatile int8_t Crono[];            // vetor com vals dec do cronometro
extern volatile int8_t ValAdc[];           // vetor com vals decimais do ADC
extern volatile int8_t ExCrono[];          // vetor externo vals dec do crono
extern volatile int8_t ExValAdc[];         // vetor externo vals dec do ADC
extern enum_stt_buzzer sttBUZZER;
extern int32_t contbuzzer;

/*-----------------------------------------------------------------------
    GLOBAL FUNCTIONS PROTOTYPES
-----------------------------------------------------------------------*/

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void);

/**
  * @brief  : Callback do systick, ajusta o cronometro
  * 		  e dispara conversão do ADC
  * @param  : None
  * @retval : None
  */
void Ajusta_Crono_ADC(void);


#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
