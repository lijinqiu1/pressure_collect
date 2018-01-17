#ifndef __MESSAGE_H__
#define __MESSAGE_H__
#include <stdint.h>

#define MESSAGE_FORM_HEAD         0xA5
#define MESSAGE_FORM_END          0x80

typedef enum Message_Form
{
    Message_Form_Head,
    Message_Form_Length,
    Message_Form_Opcode,
    Message_Form_Data,
    Message_Form_End,
}en_Message_Form_t;

typedef enum Message_Form_Opcode
{
    Message_Form_Opcode_Begin,                  /*开始工作*/
    Message_Form_Opcode_End,                    /*停止工作*/
    Message_Form_Opcode_Get_LOCAL_IP,           /*获取IP*/
    Message_Form_Opcode_Set_TCP_Connect,        /*设置tcp连接*/
    Message_Form_Opcode_Join_Access_Point,      /*接入路由*/
    Message_Form_Opcode_Set_RATE,               /*设置采样率*/
    Message_Form_Opcode_Send_Data,              /*发送数据*/
    Message_Form_Opcode_Set_System_Param,       /*设置系统参数 左右脚区分*/
    Message_Form_Opcode_Request_Battery_Value,  /*请求电池电压*/
    Message_Form_Opcode_Send_Battery_Value,     /*发送电池电量*/
}en_Message_Form_Opcode_t;


void Message_Process_Uart(void);
void Message_Process_Wifi(void);
void Message_UART_RxCpltCallback(void *handle);
void Message_UART_ErrorCallback(void *handle);

#endif

