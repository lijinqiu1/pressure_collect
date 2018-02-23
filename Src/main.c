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

  /* USER CODE END 1 */
  uint32_t last_tick;
  uint32_t battery_value;
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
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  MX_USART2_UART_Init();

  /* USER CODE BEGIN 2 */
  HAL_ADCEx_Calibration_Start(&hadc1);
  Status = ESP8266_Init();
  if (Status != ESP8266_OK)
  {
    _Error_Handler("esp8266 init failed!",Status);
  }

  Message_Uart_IO_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
  
    /*�ɼ�ѹ������*/
    if ((g_Event_status & EVENT_COLLECT_PRESSURE_VALUE)!=0)
    {
		send_data();
        g_Event_status &= ~EVENT_COLLECT_PRESSURE_VALUE; 
    }
    /*��ʼ����*/
    if ((g_Event_status & EVENT_WORK_BEGIN)!= 0)
    {
        HAL_TIM_Base_Start_IT(&htim1);
        g_Event_status &= ~EVENT_WORK_BEGIN;
    }
    /*ֹͣ����*/
    if ((g_Event_status & EVENT_WORK_END)!= 0)
    {
        HAL_TIM_Base_Stop(&htim1);
        g_Event_status &= ~EVENT_WORK_END;
    }
    /*����tcp����*/
    if ((g_Event_status & EVENT_WIFI_SET_TCP_CONNECT)!= 0)
    {
        memset(&ConnectionInfo, '\0', sizeof (ESP8266_ConnectionInfoTypeDef));

        ConnectionInfo.connectionType = ESP8266_TCP_CONNECTION;
        ConnectionInfo.ipAddress = g_System_Param.Host_IP;
        ConnectionInfo.isServer = ESP8266_FALSE;
        ConnectionInfo.port = g_System_Param.Host_Port;
        Status = ESP8266_EstablishConnection(&ConnectionInfo);
        if (Status == ESP8266_OK)
        {//���ӳɹ�
        
        }
        else
        {//����ʧ��

        }
        g_Event_status &= ~EVENT_WIFI_SET_TCP_CONNECT;
    }
    /*���ý����*/
    if ((g_Event_status & EVENT_WIFI_SET_JOIN_ACCESS_POINT)!= 0)
    {
        Status = ESP8266_JoinAccessPoint(g_System_Param.SSID,g_System_Param.Password);
        if (Status == ESP8266_OK)
        {//���ӽ����ɹ�
        
        }
        else
        {//���ӽ����ʧ��

        }
      g_Event_status &= ~EVENT_WIFI_SET_JOIN_ACCESS_POINT;
    }
    /*����͸��ģʽ*/
    if((g_Event_status & EVENT_ENTERY_SERIAL_NET) != 0)
    {
        Status = ESP8266_Entery_Moode_One();
        if (Status != ESP8266_OK)
        {
            ESP8266_Entery_Moode_One();
        }
        g_Event_status &= ~EVENT_ENTERY_SERIAL_NET;
    }
    /*�뿪͸��ģʽ*/
    if((g_Event_status & EVENT_LEAVE_SERIAL_NET) != 0)
    {
        ESP8266_Leave_Mode_One();
        g_Event_status &= ~EVENT_LEAVE_SERIAL_NET;
    }
    /*��ѯ����IP*/
    if ((g_Event_status & EVENT_WIFI_GET_LOCAL_IP)!= 0)
    {
      g_Event_status &= ~EVENT_WIFI_GET_LOCAL_IP;
    }
    /*���ò�����*/
    if ((g_Event_status & EVENT_SET_RATE)!= 0)
    {
      htim1.Init.Period = PRESSURE_COLLECT_RATE_HZ(g_System_Param.Collect_Rate);
      HAL_TIM_Base_Init(&htim1);
      g_Event_status &= ~EVENT_SET_RATE;
    }
    /*�����ص���*/
    if ((g_Event_status & EVENT_REQUEST_BATTERY_VALUE)!= 0)
    {
     g_Event_status &= ~EVENT_REQUEST_BATTERY_VALUE;
    }

    /*����ͨ������*/
    /*����������*/
    Message_Process_Uart();
    if (Serial_Net_flag)
    {
        /*͸��ģʽ�´���WIFI����*/
        Message_Process_Wifi();
    }
    if ((HAL_GetTick() - last_tick)> 30000)
    {
        battery_value = Get_Battery_Value();
    }
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
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
