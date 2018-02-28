/**
  ******************************************************************************
  * File Name          : ADC.c
  * Description        : This file provides code for the configuration
  *                      of the ADC instances.
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "adc.h"

#include "gpio.h"

/* USER CODE BEGIN 0 */
#include <string.h>
#include "esp8266.h"
#include "message.h"
#define COLLECT_COUNT_VALUE 6
uint8_t adc_value[32*COLLECT_COUNT_VALUE + 5];
/* USER CODE END 0 */

ADC_HandleTypeDef hadc1;

/* ADC1 init function */
void MX_ADC1_Init(void)
{

    /**Common config 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

void HAL_ADC_MspInit(ADC_HandleTypeDef* adcHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if(adcHandle->Instance==ADC1)
  {
  /* USER CODE BEGIN ADC1_MspInit 0 */

  /* USER CODE END ADC1_MspInit 0 */
    /* ADC1 clock enable */
    __HAL_RCC_ADC1_CLK_ENABLE();
  
    /**ADC1 GPIO Configuration    
    PA0-WKUP     ------> ADC1_IN0
    PA1     ------> ADC1_IN1
    PB0     ------> ADC1_IN8 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = VBAT_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    HAL_GPIO_Init(VBAT_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN ADC1_MspInit 1 */

  /* USER CODE END ADC1_MspInit 1 */
  }
}

void HAL_ADC_MspDeInit(ADC_HandleTypeDef* adcHandle)
{

  if(adcHandle->Instance==ADC1)
  {
  /* USER CODE BEGIN ADC1_MspDeInit 0 */

  /* USER CODE END ADC1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_ADC1_CLK_DISABLE();
  
    /**ADC1 GPIO Configuration    
    PA0-WKUP     ------> ADC1_IN0
    PA1     ------> ADC1_IN1
    PB0     ------> ADC1_IN8 
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_0|GPIO_PIN_1);

    HAL_GPIO_DeInit(VBAT_GPIO_Port, VBAT_Pin);

  /* USER CODE BEGIN ADC1_MspDeInit 1 */

  /* USER CODE END ADC1_MspDeInit 1 */
  }
} 

/* USER CODE BEGIN 1 */
uint32_t Get_Battery_Value(void)
{
    uint32_t value;
	ADC_ChannelConfTypeDef sConfig;
    sConfig.Channel = ADC_CHANNEL_8;
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
    HAL_ADC_ConfigChannel(&hadc1, &sConfig);
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 50);
    
    if(HAL_IS_BIT_SET(HAL_ADC_GetState(&hadc1), HAL_ADC_STATE_REG_EOC))
    {
        value = HAL_ADC_GetValue(&hadc1);
    }
    return value;
}
void Collect_Adc_Value(uint16_t *value)
{
    uint8_t i;
    ADC_ChannelConfTypeDef sConfig;
#if defined (LEFT_FOOT)
    sConfig.Channel = ADC_CHANNEL_0;
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
    HAL_ADC_ConfigChannel(&hadc1, &sConfig);
    for(i = 0;i < 16;i++)
    {
        switch(i)
        {
        case 0:/*6-1*/
            HAL_GPIO_WritePin(CH4_GPIO_Port,CH4_Pin,GPIO_PIN_RESET);
            HAL_GPIO_WritePin(CH5_GPIO_Port,CH5_Pin,GPIO_PIN_SET);
            HAL_GPIO_WritePin(CH6_GPIO_Port,CH6_Pin,GPIO_PIN_SET);
            
            HAL_GPIO_WritePin(CH1_GPIO_Port,CH1_Pin,GPIO_PIN_SET);
            HAL_GPIO_WritePin(CH2_GPIO_Port,CH2_Pin,GPIO_PIN_RESET);
            HAL_GPIO_WritePin(CH3_GPIO_Port,CH3_Pin,GPIO_PIN_RESET);
            break;
        case 1:/*6-2*/
            HAL_GPIO_WritePin(CH4_GPIO_Port,CH4_Pin,GPIO_PIN_RESET);
            HAL_GPIO_WritePin(CH5_GPIO_Port,CH5_Pin,GPIO_PIN_SET);
            HAL_GPIO_WritePin(CH6_GPIO_Port,CH6_Pin,GPIO_PIN_SET);
            
            HAL_GPIO_WritePin(CH1_GPIO_Port,CH1_Pin,GPIO_PIN_RESET);
            HAL_GPIO_WritePin(CH2_GPIO_Port,CH2_Pin,GPIO_PIN_SET);
            HAL_GPIO_WritePin(CH3_GPIO_Port,CH3_Pin,GPIO_PIN_RESET);
            break;
        case 2:/*6-3*/
            HAL_GPIO_WritePin(CH4_GPIO_Port,CH4_Pin,GPIO_PIN_RESET);
            HAL_GPIO_WritePin(CH5_GPIO_Port,CH5_Pin,GPIO_PIN_SET);
            HAL_GPIO_WritePin(CH6_GPIO_Port,CH6_Pin,GPIO_PIN_SET);
            
            HAL_GPIO_WritePin(CH1_GPIO_Port,CH1_Pin,GPIO_PIN_SET);
            HAL_GPIO_WritePin(CH2_GPIO_Port,CH2_Pin,GPIO_PIN_SET);
            HAL_GPIO_WritePin(CH3_GPIO_Port,CH3_Pin,GPIO_PIN_RESET);
            break;
        case 3:/*6-4*/
            HAL_GPIO_WritePin(CH4_GPIO_Port,CH4_Pin,GPIO_PIN_RESET);
            HAL_GPIO_WritePin(CH5_GPIO_Port,CH5_Pin,GPIO_PIN_SET);
            HAL_GPIO_WritePin(CH6_GPIO_Port,CH6_Pin,GPIO_PIN_SET);
            
            HAL_GPIO_WritePin(CH1_GPIO_Port,CH1_Pin,GPIO_PIN_RESET);
            HAL_GPIO_WritePin(CH2_GPIO_Port,CH2_Pin,GPIO_PIN_RESET);
            HAL_GPIO_WritePin(CH3_GPIO_Port,CH3_Pin,GPIO_PIN_SET);
            break;
        case 4:/*1-1*/
            HAL_GPIO_WritePin(CH4_GPIO_Port,CH4_Pin,GPIO_PIN_SET);
            HAL_GPIO_WritePin(CH5_GPIO_Port,CH5_Pin,GPIO_PIN_RESET);
            HAL_GPIO_WritePin(CH6_GPIO_Port,CH6_Pin,GPIO_PIN_RESET);
            
            HAL_GPIO_WritePin(CH1_GPIO_Port,CH1_Pin,GPIO_PIN_SET);
            HAL_GPIO_WritePin(CH2_GPIO_Port,CH2_Pin,GPIO_PIN_RESET);
            HAL_GPIO_WritePin(CH3_GPIO_Port,CH3_Pin,GPIO_PIN_RESET);
            break;case 5:/*1-2*/
            HAL_GPIO_WritePin(CH4_GPIO_Port,CH4_Pin,GPIO_PIN_SET);
            HAL_GPIO_WritePin(CH5_GPIO_Port,CH5_Pin,GPIO_PIN_RESET);
            HAL_GPIO_WritePin(CH6_GPIO_Port,CH6_Pin,GPIO_PIN_RESET);
            
            HAL_GPIO_WritePin(CH1_GPIO_Port,CH1_Pin,GPIO_PIN_RESET);
            HAL_GPIO_WritePin(CH2_GPIO_Port,CH2_Pin,GPIO_PIN_SET);
            HAL_GPIO_WritePin(CH3_GPIO_Port,CH3_Pin,GPIO_PIN_RESET);
            break;
        case 6:/*1-3*/
            HAL_GPIO_WritePin(CH4_GPIO_Port,CH4_Pin,GPIO_PIN_SET);
            HAL_GPIO_WritePin(CH5_GPIO_Port,CH5_Pin,GPIO_PIN_RESET);
            HAL_GPIO_WritePin(CH6_GPIO_Port,CH6_Pin,GPIO_PIN_RESET);
            
            HAL_GPIO_WritePin(CH1_GPIO_Port,CH1_Pin,GPIO_PIN_SET);
            HAL_GPIO_WritePin(CH2_GPIO_Port,CH2_Pin,GPIO_PIN_SET);
            HAL_GPIO_WritePin(CH3_GPIO_Port,CH3_Pin,GPIO_PIN_RESET);
            break;
        case 7:/*1-4*/
            HAL_GPIO_WritePin(CH4_GPIO_Port,CH4_Pin,GPIO_PIN_SET);
            HAL_GPIO_WritePin(CH5_GPIO_Port,CH5_Pin,GPIO_PIN_RESET);
            HAL_GPIO_WritePin(CH6_GPIO_Port,CH6_Pin,GPIO_PIN_RESET);
            
            HAL_GPIO_WritePin(CH1_GPIO_Port,CH1_Pin,GPIO_PIN_RESET);
            HAL_GPIO_WritePin(CH2_GPIO_Port,CH2_Pin,GPIO_PIN_RESET);
            HAL_GPIO_WritePin(CH3_GPIO_Port,CH3_Pin,GPIO_PIN_SET);
            break;
        case 8:/*5-4*/
            HAL_GPIO_WritePin(CH4_GPIO_Port,CH4_Pin,GPIO_PIN_SET);
            HAL_GPIO_WritePin(CH5_GPIO_Port,CH5_Pin,GPIO_PIN_RESET);
            HAL_GPIO_WritePin(CH6_GPIO_Port,CH6_Pin,GPIO_PIN_SET);
            
            HAL_GPIO_WritePin(CH1_GPIO_Port,CH1_Pin,GPIO_PIN_RESET);
            HAL_GPIO_WritePin(CH2_GPIO_Port,CH2_Pin,GPIO_PIN_RESET);
            HAL_GPIO_WritePin(CH3_GPIO_Port,CH3_Pin,GPIO_PIN_SET);
            break;
        case 9:/*2-2*/
            HAL_GPIO_WritePin(CH4_GPIO_Port,CH4_Pin,GPIO_PIN_RESET);
            HAL_GPIO_WritePin(CH5_GPIO_Port,CH5_Pin,GPIO_PIN_SET);
            HAL_GPIO_WritePin(CH6_GPIO_Port,CH6_Pin,GPIO_PIN_RESET);
            
            HAL_GPIO_WritePin(CH1_GPIO_Port,CH1_Pin,GPIO_PIN_RESET);
            HAL_GPIO_WritePin(CH2_GPIO_Port,CH2_Pin,GPIO_PIN_SET);
            HAL_GPIO_WritePin(CH3_GPIO_Port,CH3_Pin,GPIO_PIN_RESET);
            break;
        case 10:/*2-3*/
            HAL_GPIO_WritePin(CH4_GPIO_Port,CH4_Pin,GPIO_PIN_RESET);
            HAL_GPIO_WritePin(CH5_GPIO_Port,CH5_Pin,GPIO_PIN_SET);
            HAL_GPIO_WritePin(CH6_GPIO_Port,CH6_Pin,GPIO_PIN_RESET);
            
            HAL_GPIO_WritePin(CH1_GPIO_Port,CH1_Pin,GPIO_PIN_SET);
            HAL_GPIO_WritePin(CH2_GPIO_Port,CH2_Pin,GPIO_PIN_SET);
            HAL_GPIO_WritePin(CH3_GPIO_Port,CH3_Pin,GPIO_PIN_RESET);
            break;
        case 11:/*2-4*/
            HAL_GPIO_WritePin(CH4_GPIO_Port,CH4_Pin,GPIO_PIN_RESET);
            HAL_GPIO_WritePin(CH5_GPIO_Port,CH5_Pin,GPIO_PIN_SET);
            HAL_GPIO_WritePin(CH6_GPIO_Port,CH6_Pin,GPIO_PIN_RESET);
            
            HAL_GPIO_WritePin(CH1_GPIO_Port,CH1_Pin,GPIO_PIN_RESET);
            HAL_GPIO_WritePin(CH2_GPIO_Port,CH2_Pin,GPIO_PIN_RESET);
            HAL_GPIO_WritePin(CH3_GPIO_Port,CH3_Pin,GPIO_PIN_SET);
            break;
        case 12:/*4-2*/
            HAL_GPIO_WritePin(CH4_GPIO_Port,CH4_Pin,GPIO_PIN_RESET);
            HAL_GPIO_WritePin(CH5_GPIO_Port,CH5_Pin,GPIO_PIN_RESET);
            HAL_GPIO_WritePin(CH6_GPIO_Port,CH6_Pin,GPIO_PIN_SET);
            
            HAL_GPIO_WritePin(CH1_GPIO_Port,CH1_Pin,GPIO_PIN_RESET);
            HAL_GPIO_WritePin(CH2_GPIO_Port,CH2_Pin,GPIO_PIN_SET);
            HAL_GPIO_WritePin(CH3_GPIO_Port,CH3_Pin,GPIO_PIN_RESET);
            break;
        case 13:/*4-3*/
            HAL_GPIO_WritePin(CH4_GPIO_Port,CH4_Pin,GPIO_PIN_RESET);
            HAL_GPIO_WritePin(CH5_GPIO_Port,CH5_Pin,GPIO_PIN_RESET);
            HAL_GPIO_WritePin(CH6_GPIO_Port,CH6_Pin,GPIO_PIN_SET);
            
            HAL_GPIO_WritePin(CH1_GPIO_Port,CH1_Pin,GPIO_PIN_SET);
            HAL_GPIO_WritePin(CH2_GPIO_Port,CH2_Pin,GPIO_PIN_SET);
            HAL_GPIO_WritePin(CH3_GPIO_Port,CH3_Pin,GPIO_PIN_RESET);
            break;
        case 14:/*3-2*/
            HAL_GPIO_WritePin(CH4_GPIO_Port,CH4_Pin,GPIO_PIN_SET);
            HAL_GPIO_WritePin(CH5_GPIO_Port,CH5_Pin,GPIO_PIN_SET);
            HAL_GPIO_WritePin(CH6_GPIO_Port,CH6_Pin,GPIO_PIN_RESET);
            
            HAL_GPIO_WritePin(CH1_GPIO_Port,CH1_Pin,GPIO_PIN_RESET);
            HAL_GPIO_WritePin(CH2_GPIO_Port,CH2_Pin,GPIO_PIN_SET);
            HAL_GPIO_WritePin(CH3_GPIO_Port,CH3_Pin,GPIO_PIN_RESET);
            break;
        case 15:/*3-3*/
            HAL_GPIO_WritePin(CH4_GPIO_Port,CH4_Pin,GPIO_PIN_SET);
            HAL_GPIO_WritePin(CH5_GPIO_Port,CH5_Pin,GPIO_PIN_SET);
            HAL_GPIO_WritePin(CH6_GPIO_Port,CH6_Pin,GPIO_PIN_RESET);
            
            HAL_GPIO_WritePin(CH1_GPIO_Port,CH1_Pin,GPIO_PIN_SET);
            HAL_GPIO_WritePin(CH2_GPIO_Port,CH2_Pin,GPIO_PIN_SET);
            HAL_GPIO_WritePin(CH3_GPIO_Port,CH3_Pin,GPIO_PIN_RESET);
            break;
        }
    }
#else
    sConfig.Channel = ADC_CHANNEL_1;
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
    HAL_ADC_ConfigChannel(&hadc1, &sConfig);
    for(i = 0;i < 16;i++)
    {
        switch(i)
        {
        case 0:/*6-1*/
            HAL_GPIO_WritePin(CH1_GPIO_Port,CH1_Pin,GPIO_PIN_RESET);
            HAL_GPIO_WritePin(CH2_GPIO_Port,CH2_Pin,GPIO_PIN_SET);
            HAL_GPIO_WritePin(CH3_GPIO_Port,CH3_Pin,GPIO_PIN_SET);
            
            HAL_GPIO_WritePin(CH4_GPIO_Port,CH4_Pin,GPIO_PIN_SET);
            HAL_GPIO_WritePin(CH5_GPIO_Port,CH5_Pin,GPIO_PIN_RESET);
            HAL_GPIO_WritePin(CH6_GPIO_Port,CH6_Pin,GPIO_PIN_RESET);
            break;
        case 1:/*6-2*/
            HAL_GPIO_WritePin(CH1_GPIO_Port,CH1_Pin,GPIO_PIN_RESET);
            HAL_GPIO_WritePin(CH2_GPIO_Port,CH2_Pin,GPIO_PIN_SET);
            HAL_GPIO_WritePin(CH3_GPIO_Port,CH3_Pin,GPIO_PIN_SET);
            
            HAL_GPIO_WritePin(CH4_GPIO_Port,CH4_Pin,GPIO_PIN_RESET);
            HAL_GPIO_WritePin(CH5_GPIO_Port,CH5_Pin,GPIO_PIN_SET);
            HAL_GPIO_WritePin(CH6_GPIO_Port,CH6_Pin,GPIO_PIN_RESET);
            break;
        case 2:/*6-3*/
            HAL_GPIO_WritePin(CH1_GPIO_Port,CH1_Pin,GPIO_PIN_RESET);
            HAL_GPIO_WritePin(CH2_GPIO_Port,CH2_Pin,GPIO_PIN_SET);
            HAL_GPIO_WritePin(CH3_GPIO_Port,CH3_Pin,GPIO_PIN_SET);
            
            HAL_GPIO_WritePin(CH4_GPIO_Port,CH4_Pin,GPIO_PIN_SET);
            HAL_GPIO_WritePin(CH5_GPIO_Port,CH5_Pin,GPIO_PIN_SET);
            HAL_GPIO_WritePin(CH6_GPIO_Port,CH6_Pin,GPIO_PIN_RESET);
            break;
        case 3:/*6-4*/
            HAL_GPIO_WritePin(CH1_GPIO_Port,CH1_Pin,GPIO_PIN_RESET);
            HAL_GPIO_WritePin(CH2_GPIO_Port,CH2_Pin,GPIO_PIN_SET);
            HAL_GPIO_WritePin(CH3_GPIO_Port,CH3_Pin,GPIO_PIN_SET);
            
            HAL_GPIO_WritePin(CH4_GPIO_Port,CH4_Pin,GPIO_PIN_RESET);
            HAL_GPIO_WritePin(CH5_GPIO_Port,CH5_Pin,GPIO_PIN_RESET);
            HAL_GPIO_WritePin(CH6_GPIO_Port,CH6_Pin,GPIO_PIN_SET);
            break;
        case 4:/*1-1*/
            HAL_GPIO_WritePin(CH1_GPIO_Port,CH1_Pin,GPIO_PIN_SET);
            HAL_GPIO_WritePin(CH2_GPIO_Port,CH2_Pin,GPIO_PIN_RESET);
            HAL_GPIO_WritePin(CH3_GPIO_Port,CH3_Pin,GPIO_PIN_RESET);
            
            HAL_GPIO_WritePin(CH4_GPIO_Port,CH4_Pin,GPIO_PIN_SET);
            HAL_GPIO_WritePin(CH5_GPIO_Port,CH5_Pin,GPIO_PIN_RESET);
            HAL_GPIO_WritePin(CH6_GPIO_Port,CH6_Pin,GPIO_PIN_RESET);
            break;
        case 5:/*1-2*/
            HAL_GPIO_WritePin(CH1_GPIO_Port,CH1_Pin,GPIO_PIN_SET);
            HAL_GPIO_WritePin(CH2_GPIO_Port,CH2_Pin,GPIO_PIN_RESET);
            HAL_GPIO_WritePin(CH3_GPIO_Port,CH3_Pin,GPIO_PIN_RESET);
            
            HAL_GPIO_WritePin(CH4_GPIO_Port,CH4_Pin,GPIO_PIN_RESET);
            HAL_GPIO_WritePin(CH5_GPIO_Port,CH5_Pin,GPIO_PIN_SET);
            HAL_GPIO_WritePin(CH6_GPIO_Port,CH6_Pin,GPIO_PIN_RESET);
            break;
        case 6:/*1-3*/
            HAL_GPIO_WritePin(CH1_GPIO_Port,CH1_Pin,GPIO_PIN_SET);
            HAL_GPIO_WritePin(CH2_GPIO_Port,CH2_Pin,GPIO_PIN_RESET);
            HAL_GPIO_WritePin(CH3_GPIO_Port,CH3_Pin,GPIO_PIN_RESET);
            
            HAL_GPIO_WritePin(CH4_GPIO_Port,CH4_Pin,GPIO_PIN_SET);
            HAL_GPIO_WritePin(CH5_GPIO_Port,CH5_Pin,GPIO_PIN_SET);
            HAL_GPIO_WritePin(CH6_GPIO_Port,CH6_Pin,GPIO_PIN_RESET);
            break;
        case 7:/*1-4*/
            HAL_GPIO_WritePin(CH1_GPIO_Port,CH1_Pin,GPIO_PIN_SET);
            HAL_GPIO_WritePin(CH2_GPIO_Port,CH2_Pin,GPIO_PIN_RESET);
            HAL_GPIO_WritePin(CH3_GPIO_Port,CH3_Pin,GPIO_PIN_RESET);
            
            HAL_GPIO_WritePin(CH4_GPIO_Port,CH4_Pin,GPIO_PIN_RESET);
            HAL_GPIO_WritePin(CH5_GPIO_Port,CH5_Pin,GPIO_PIN_RESET);
            HAL_GPIO_WritePin(CH6_GPIO_Port,CH6_Pin,GPIO_PIN_SET);
            break;
        case 8:/*5-4*/
            HAL_GPIO_WritePin(CH1_GPIO_Port,CH1_Pin,GPIO_PIN_SET);
            HAL_GPIO_WritePin(CH2_GPIO_Port,CH2_Pin,GPIO_PIN_RESET);
            HAL_GPIO_WritePin(CH3_GPIO_Port,CH3_Pin,GPIO_PIN_SET);
            
            HAL_GPIO_WritePin(CH4_GPIO_Port,CH4_Pin,GPIO_PIN_RESET);
            HAL_GPIO_WritePin(CH5_GPIO_Port,CH5_Pin,GPIO_PIN_RESET);
            HAL_GPIO_WritePin(CH6_GPIO_Port,CH6_Pin,GPIO_PIN_SET);
            break;
        case 9:/*2-2*/
            HAL_GPIO_WritePin(CH1_GPIO_Port,CH1_Pin,GPIO_PIN_RESET);
            HAL_GPIO_WritePin(CH2_GPIO_Port,CH2_Pin,GPIO_PIN_SET);
            HAL_GPIO_WritePin(CH3_GPIO_Port,CH3_Pin,GPIO_PIN_RESET);
            
            HAL_GPIO_WritePin(CH4_GPIO_Port,CH4_Pin,GPIO_PIN_RESET);
            HAL_GPIO_WritePin(CH5_GPIO_Port,CH5_Pin,GPIO_PIN_SET);
            HAL_GPIO_WritePin(CH6_GPIO_Port,CH6_Pin,GPIO_PIN_RESET);
            break;
        case 10:/*2-3*/
            HAL_GPIO_WritePin(CH1_GPIO_Port,CH1_Pin,GPIO_PIN_RESET);
            HAL_GPIO_WritePin(CH2_GPIO_Port,CH2_Pin,GPIO_PIN_SET);
            HAL_GPIO_WritePin(CH3_GPIO_Port,CH3_Pin,GPIO_PIN_RESET);
            
            HAL_GPIO_WritePin(CH4_GPIO_Port,CH4_Pin,GPIO_PIN_SET);
            HAL_GPIO_WritePin(CH5_GPIO_Port,CH5_Pin,GPIO_PIN_SET);
            HAL_GPIO_WritePin(CH6_GPIO_Port,CH6_Pin,GPIO_PIN_RESET);
            break;
        case 11:/*2-4*/
            HAL_GPIO_WritePin(CH1_GPIO_Port,CH1_Pin,GPIO_PIN_RESET);
            HAL_GPIO_WritePin(CH2_GPIO_Port,CH2_Pin,GPIO_PIN_SET);
            HAL_GPIO_WritePin(CH3_GPIO_Port,CH3_Pin,GPIO_PIN_RESET);
            
            HAL_GPIO_WritePin(CH4_GPIO_Port,CH4_Pin,GPIO_PIN_RESET);
            HAL_GPIO_WritePin(CH5_GPIO_Port,CH5_Pin,GPIO_PIN_RESET);
            HAL_GPIO_WritePin(CH6_GPIO_Port,CH6_Pin,GPIO_PIN_SET);
            break;
        case 12:/*4-2*/
            HAL_GPIO_WritePin(CH1_GPIO_Port,CH1_Pin,GPIO_PIN_RESET);
            HAL_GPIO_WritePin(CH2_GPIO_Port,CH2_Pin,GPIO_PIN_RESET);
            HAL_GPIO_WritePin(CH3_GPIO_Port,CH3_Pin,GPIO_PIN_SET);
            
            HAL_GPIO_WritePin(CH4_GPIO_Port,CH4_Pin,GPIO_PIN_RESET);
            HAL_GPIO_WritePin(CH5_GPIO_Port,CH5_Pin,GPIO_PIN_SET);
            HAL_GPIO_WritePin(CH6_GPIO_Port,CH6_Pin,GPIO_PIN_RESET);
            break;
        case 13:/*4-3*/
            HAL_GPIO_WritePin(CH1_GPIO_Port,CH1_Pin,GPIO_PIN_RESET);
            HAL_GPIO_WritePin(CH2_GPIO_Port,CH2_Pin,GPIO_PIN_RESET);
            HAL_GPIO_WritePin(CH3_GPIO_Port,CH3_Pin,GPIO_PIN_SET);
            
            HAL_GPIO_WritePin(CH4_GPIO_Port,CH4_Pin,GPIO_PIN_SET);
            HAL_GPIO_WritePin(CH5_GPIO_Port,CH5_Pin,GPIO_PIN_SET);
            HAL_GPIO_WritePin(CH6_GPIO_Port,CH6_Pin,GPIO_PIN_RESET);
            break;
        case 14:/*3-2*/
            HAL_GPIO_WritePin(CH1_GPIO_Port,CH1_Pin,GPIO_PIN_SET);
            HAL_GPIO_WritePin(CH2_GPIO_Port,CH2_Pin,GPIO_PIN_SET);
            HAL_GPIO_WritePin(CH3_GPIO_Port,CH3_Pin,GPIO_PIN_RESET);
            
            HAL_GPIO_WritePin(CH4_GPIO_Port,CH4_Pin,GPIO_PIN_RESET);
            HAL_GPIO_WritePin(CH5_GPIO_Port,CH5_Pin,GPIO_PIN_SET);
            HAL_GPIO_WritePin(CH6_GPIO_Port,CH6_Pin,GPIO_PIN_RESET);
            break;
        case 15:/*3-3*/
            HAL_GPIO_WritePin(CH1_GPIO_Port,CH1_Pin,GPIO_PIN_SET);
            HAL_GPIO_WritePin(CH2_GPIO_Port,CH2_Pin,GPIO_PIN_SET);
            HAL_GPIO_WritePin(CH3_GPIO_Port,CH3_Pin,GPIO_PIN_RESET);
            
            HAL_GPIO_WritePin(CH4_GPIO_Port,CH4_Pin,GPIO_PIN_SET);
            HAL_GPIO_WritePin(CH5_GPIO_Port,CH5_Pin,GPIO_PIN_SET);
            HAL_GPIO_WritePin(CH6_GPIO_Port,CH6_Pin,GPIO_PIN_RESET);
            break;
        }
    }
#endif
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 50);
    
    if(HAL_IS_BIT_SET(HAL_ADC_GetState(&hadc1), HAL_ADC_STATE_REG_EOC))
    {
        *value = HAL_ADC_GetValue(&hadc1);
    }
}
void send_data(void)
{
    static uint8_t collect_count = 0;
    adc_value[0] = 0xA5;
    adc_value[1] = (32*COLLECT_COUNT_VALUE)%256;
    adc_value[2] = (32*COLLECT_COUNT_VALUE)/256;
    adc_value[3] = Message_Form_Opcode_Send_Data;
    adc_value[32*COLLECT_COUNT_VALUE+4] = 0x80;
    
    if (g_System_Param.Collect_Rate >= 300)
    {
	    memset(&adc_value[4]+collect_count*32,collect_count,32);
        collect_count++;
        if (collect_count == COLLECT_COUNT_VALUE)
        {
            collect_count = 0;
            ESP8266_SendData((uint8_t*)adc_value, 32*COLLECT_COUNT_VALUE+5);
        }
    }
    else
    {

    }
}
/* USER CODE END 1 */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
