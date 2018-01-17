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
    Message_Form_Opcode_Begin,                  /*��ʼ����*/
    Message_Form_Opcode_End,                    /*ֹͣ����*/
    Message_Form_Opcode_Get_LOCAL_IP,           /*��ȡIP*/
    Message_Form_Opcode_Set_TCP_Connect,        /*����tcp����*/
    Message_Form_Opcode_Join_Access_Point,      /*����·��*/
    Message_Form_Opcode_Set_RATE,               /*���ò�����*/
    Message_Form_Opcode_Send_Data,              /*��������*/
    Message_Form_Opcode_Set_System_Param,       /*����ϵͳ���� ���ҽ�����*/
    Message_Form_Opcode_Request_Battery_Value,  /*�����ص�ѹ*/
    Message_Form_Opcode_Send_Battery_Value,     /*���͵�ص���*/
}en_Message_Form_Opcode_t;


void Message_Process_Uart(void);
void Message_Process_Wifi(void);
void Message_UART_RxCpltCallback(void *handle);
void Message_UART_ErrorCallback(void *handle);

#endif

