/**
  ******************************************************************************
  * File Name          : main.hpp
  * Description        : This file contains the common defines of the application
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H
  /* Includes ------------------------------------------------------------------*/

/* Includes ------------------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define WIFI_RST_Pin GPIO_PIN_4
#define WIFI_RST_GPIO_Port GPIOA
#define WIFI_EN_Pin GPIO_PIN_5
#define WIFI_EN_GPIO_Port GPIOA
#define WIFI_RES_Pin GPIO_PIN_6
#define WIFI_RES_GPIO_Port GPIOA
#define CHG_STATUS_Pin GPIO_PIN_7
#define CHG_STATUS_GPIO_Port GPIOA
#define VBAT_Pin GPIO_PIN_0
#define VBAT_GPIO_Port GPIOB
#define CH6_Pin GPIO_PIN_10
#define CH6_GPIO_Port GPIOB
#define CH5_Pin GPIO_PIN_12
#define CH5_GPIO_Port GPIOB
#define CH4_Pin GPIO_PIN_13
#define CH4_GPIO_Port GPIOB
#define CH3_Pin GPIO_PIN_14
#define CH3_GPIO_Port GPIOB
#define CH2_Pin GPIO_PIN_15
#define CH2_GPIO_Port GPIOB
#define CH1_Pin GPIO_PIN_8
#define CH1_GPIO_Port GPIOA
#define LED2_Pin GPIO_PIN_4
#define LED2_GPIO_Port GPIOB
#define LED1_Pin GPIO_PIN_5
#define LED1_GPIO_Port GPIOB
#define LED0_Pin GPIO_PIN_6
#define LED0_GPIO_Port GPIOB

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1U */

/* USER CODE BEGIN Private defines */
#define WIFI_SSID       "AndroidAP"
#define WIFI_PASSWORD   "welcome0"
#define HOST_ADDRESS    "192.168.43.1"
#define HOST_PORT       8080
#define PRESSURE_COLLECT_DEFAULT_RATE_HZ    300

#define PRESSURE_COLLECT_TIME_PRESCALER     72
#define PRESSURE_COLLECT_RATE_HZ(x)         ((72000000 / 72)/x - 1)

#define EVENT_WORK_BEGIN                    (uint32_t)(0x00000001 << 0)  /*开始采集*/
#define EVENT_WORK_END                      (uint32_t)(0x00000001 << 1)  /*停止采集*/
#define EVENT_WIFI_SET_TCP_CONNECT          (uint32_t)(0x00000001 << 2)  /*设置TCP连接*/
#define EVENT_WIFI_SET_JOIN_ACCESS_POINT    (uint32_t)(0x00000001 << 3)  /*设置接入点*/
#define EVENT_WIFI_GET_LOCAL_IP             (uint32_t)(0x00000001 << 4)  /*获取本地IP*/
#define EVENT_SET_RATE                      (uint32_t)(0x00000001 << 5)  /*设置采样率*/
#define EVENT_SET_SYSTEM_PARAM              (uint32_t)(0x00000001 << 6)  /*设置系统参数，左右脚等*/
#define EVENT_REQUEST_BATTERY_VALUE         (uint32_t)(0x00000001 << 7)  /*获取电池电量*/
#define EVENT_COLLECT_PRESSURE_VALUE        (uint32_t)(0x00000001 << 8)  /*采集压力数据*/
#define EVENT_ENTERY_SERIAL_NET             (uint32_t)(0x00000001 << 9)  /*进入透传模式*/
#define EVENT_LEAVE_SERIAL_NET              (uint32_t)(0x00000001 <<10)  /*离开透传模式*/

typedef struct System_Param
{
    uint8_t SSID[20];
    uint8_t Password[20];
    uint8_t Host_IP[20];
    uint8_t Host_Port[20];
    uint8_t Which_foot;
    uint16_t Collect_Rate;
}st_System_Param_t;

extern st_System_Param_t g_System_Param;
extern uint32_t g_Event_status;
/* USER CODE END Private defines */

#ifdef __cplusplus
 extern "C" {
#endif
void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
#ifdef __cplusplus
}
#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

#endif /* __MAIN_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
