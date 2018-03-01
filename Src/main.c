/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
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
#include "main.h"
#include "stm32f1xx_hal.h"
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include <string.h>
#include "esp8266.h"
#include "message.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

uint32_t g_Event_status = 0;
st_System_Param_t g_System_Param = 
    {
    .Collect_Rate = 300,
    };

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
  ESP8266_ConnectionInfoTypeDef ConnectionInfo;
  ESP8266_StatusTypeDef Status = ESP8266_OK; 
  uint32_t last_tick;
  uint32_t battery_value;
  uint32_t charge_completed_count = 0;
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();

  /* USER CODE BEGIN 2 */
  HAL_ADCEx_Calibration_Start(&hadc1);
  HAL_ADCEx_Calibration_Start(&hadc2);
  
//  battery_value = Get_Battery_Value();
//   while(battery_value < 2304)
//  {
//    if ((HAL_GetTick() - last_tick)> 1000)
//    {
//        last_tick = HAL_GetTick();
//        battery_value = Get_Battery_Value();
//    }
//    if(HAL_GPIO_ReadPin(CHG_STATUS_GPIO_Port,CHG_STATUS_Pin) == GPIO_PIN_RESET)
//	{
//        break;
//	}
//  }
//   //led 初始化
   HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_SET);
   HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
   HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
//   //wifi 初始化
//  Status = ESP8266_Init();
//  if (Status != ESP8266_OK)
//  {
//    _Error_Handler("esp8266 init failed!",Status);
//  }

//  Message_Uart_IO_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
//  while (1)
//  {
//  /* USER CODE END WHILE */

//  /* USER CODE BEGIN 3 */
//  
//    /*采集压力数据*/
//    if ((g_Event_status & EVENT_COLLECT_PRESSURE_VALUE)!=0)
//    {
//		send_data();
//        g_Event_status &= ~EVENT_COLLECT_PRESSURE_VALUE; 
//    }
//    /*开始工作*/
//    if ((g_Event_status & EVENT_WORK_BEGIN)!= 0)
//    {
//        HAL_TIM_Base_Start_IT(&htim1);
//        HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
//        g_Event_status &= ~EVENT_WORK_BEGIN;
//    }
//    /*停止工作*/
//    if ((g_Event_status & EVENT_WORK_END)!= 0)
//    {
//        HAL_TIM_Base_Stop(&htim1);
//        HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
//        g_Event_status &= ~EVENT_WORK_END;
//    }
//    /*设置tcp连接*/
//    if ((g_Event_status & EVENT_WIFI_SET_TCP_CONNECT)!= 0)
//    {
//        memset(&ConnectionInfo, '\0', sizeof (ESP8266_ConnectionInfoTypeDef));

//        ConnectionInfo.connectionType = ESP8266_TCP_CONNECTION;
//        ConnectionInfo.ipAddress = g_System_Param.Host_IP;
//        ConnectionInfo.isServer = ESP8266_FALSE;
//        ConnectionInfo.port = g_System_Param.Host_Port;
//        Status = ESP8266_EstablishConnection(&ConnectionInfo);
//        if (Status == ESP8266_OK)
//        {//连接成功
//        
//        }
//        else
//        {//连接失败

//        }
//        g_Event_status &= ~EVENT_WIFI_SET_TCP_CONNECT;
//    }
//    /*设置接入点*/
//    if ((g_Event_status & EVENT_WIFI_SET_JOIN_ACCESS_POINT)!= 0)
//    {
//        Status = ESP8266_JoinAccessPoint(g_System_Param.SSID,g_System_Param.Password);
//        if (Status == ESP8266_OK)
//        {//连接接入点成功
//        
//        }
//        else
//        {//连接接入点失败

//        }
//      g_Event_status &= ~EVENT_WIFI_SET_JOIN_ACCESS_POINT;
//    }
//    /*进入透传模式*/
//    if((g_Event_status & EVENT_ENTERY_SERIAL_NET) != 0)
//    {
//        Status = ESP8266_Entery_Moode_One();
//        if (Status != ESP8266_OK)
//        {
//            ESP8266_Entery_Moode_One();
//        }
//        g_Event_status &= ~EVENT_ENTERY_SERIAL_NET;
//    }
//    /*离开透传模式*/
//    if((g_Event_status & EVENT_LEAVE_SERIAL_NET) != 0)
//    {
//        ESP8266_Leave_Mode_One();
//        g_Event_status &= ~EVENT_LEAVE_SERIAL_NET;
//    }
//    /*查询本地IP*/
//    if ((g_Event_status & EVENT_WIFI_GET_LOCAL_IP)!= 0)
//    {
//      g_Event_status &= ~EVENT_WIFI_GET_LOCAL_IP;
//    }
//    /*设置采样率*/
//    if ((g_Event_status & EVENT_SET_RATE)!= 0)
//    {
//      htim1.Init.Period = PRESSURE_COLLECT_RATE_HZ(g_System_Param.Collect_Rate);
//      HAL_TIM_Base_Init(&htim1);
//      g_Event_status &= ~EVENT_SET_RATE;
//    }
//    /*请求电池电量*/
//    if ((g_Event_status & EVENT_REQUEST_BATTERY_VALUE)!= 0)
//    {
//     g_Event_status &= ~EVENT_REQUEST_BATTERY_VALUE;
//    }

//    /*处理通信数据*/
//    /*处理串口数据*/
//    Message_Process_Uart();
//    if (Serial_Net_flag)
//    {
//        /*透传模式下处理WIFI数据*/
//        Message_Process_Wifi();
//    }
//    if ((HAL_GetTick() - last_tick)> 3000)
//    {
//        last_tick = HAL_GetTick();
//        battery_value = Get_Battery_Value();
//		if(HAL_GPIO_ReadPin(CHG_STATUS_GPIO_Port,CHG_STATUS_Pin) == GPIO_PIN_RESET)
//		{
//			if(battery_value < 2688)
//			{//充电中
//				HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin);
//			}
//			else
//			{//充满
//				HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_RESET);
//			}
//			HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
//		}
//		else
//		{
//            HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_SET);
//			if (battery_value > 2304)
//			{
//			    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
//			}
//			else
//			{//3.6v低电压
//			    HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
//			}
//		}
//    }
//  }
HAL_GPIO_WritePin(CH1_GPIO_Port,CH1_Pin,GPIO_PIN_RESET);
HAL_GPIO_WritePin(CH2_GPIO_Port,CH2_Pin,GPIO_PIN_RESET);
HAL_GPIO_WritePin(CH3_GPIO_Port,CH3_Pin,GPIO_PIN_RESET);

HAL_GPIO_WritePin(CH4_GPIO_Port,CH4_Pin,GPIO_PIN_SET);
HAL_GPIO_WritePin(CH5_GPIO_Port,CH5_Pin,GPIO_PIN_SET);
HAL_GPIO_WritePin(CH6_GPIO_Port,CH6_Pin,GPIO_PIN_RESET);
uint16_t value;
uint8_t buffer[30];
  while(1)
  {
        __NOP();
        __NOP();
        __NOP();
        __NOP();
        HAL_ADC_Start(&hadc1);
        HAL_ADC_PollForConversion(&hadc1, 50);
        
        if(HAL_IS_BIT_SET(HAL_ADC_GetState(&hadc1), HAL_ADC_STATE_REG_EOC))
        {
            value = HAL_ADC_GetValue(&hadc1);
	        sprintf(buffer,"value %d\n",value);
            HAL_UART_Transmit(&huart1, buffer, strlen(buffer), HAL_MAX_DELAY);
        }
		HAL_Delay(500);
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
