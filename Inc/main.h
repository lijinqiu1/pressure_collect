/**
  ******************************************************************************
  * File Name          : main.h
  * Description        : This file contains the common defines of the application
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H
  /* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

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

typedef struct System_Param
{
    uint8_t SSID[20];
    uint8_t Password[20];
    uint8_t Host_IP[20];
    uint16_t Host_Port;
    uint8_t Which_foot;
    uint16_t Collect_Rate;
}st_System_Param_t;

extern st_System_Param_t g_System_Param;
extern uint32_t g_Event_status;
/* USER CODE END Private defines */

void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)

/**
  * @}
  */ 

/**
  * @}
*/ 

#endif /* __MAIN_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
