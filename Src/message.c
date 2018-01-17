#include "message.h"
#include "usart.h"
#include <string.h>

#define MessageUartHandle    huart2
#define UART_BUFFER_SIZE     1*1024
#define WIFI_BUFFER_SIZE     1*1024

typedef struct
{
  uint8_t  data[UART_BUFFER_SIZE];
  uint16_t tail; 				
  uint16_t head;
}UartBuffer_t;

static UartBuffer_t MessageUartRxBuffer;
static uint8_t UartBuffer[UART_BUFFER_SIZE];
static uint8_t WifiBuffer[WIFI_BUFFER_SIZE];

static void Message_Analyse(uint8_t * Message)
{
    //opcode
    switch(Message[0])
    {
    case Message_Form_Opcode_Begin:
        g_Event_status |= EVENT_WORK_BEGIN;
    break;
    case Message_Form_Opcode_End:
        g_Event_status |= EVENT_WORK_END;
    break;
    case Message_Form_Opcode_Get_LOCAL_IP:
        g_Event_status |= EVENT_WIFI_GET_LOCAL_IP;
    break;
    case Message_Form_Opcode_Set_TCP_Connect:
        strcpy(g_System_Param.Host_IP,&Message[1]);
        g_System_Param.Host_Port = *(uint16_t *)&Message[strlen(&Message[1]) + 1];
        g_Event_status |= EVENT_WIFI_SET_TCP_CONNECT;
    break;
    case Message_Form_Opcode_Join_Access_Point:
        strcpy(g_System_Param.SSID,&Message[1]);
        strcpy(g_System_Param.Password,&Message[strlen(&Message[1]) + 1]);
        g_Event_status |= EVENT_WIFI_SET_JOIN_ACCESS_POINT;
    break;
    case Message_Form_Opcode_Set_RATE:
        g_System_Param.Collect_Rate = *(uint16_t *)&Message[1];
        g_Event_status |= EVENT_SET_RATE;
    break;
    case Message_Form_Opcode_Send_Data:
    break;
    case Message_Form_Opcode_Set_System_Param:
        g_System_Param.Which_foot = Message[1];
        g_Event_status |= EVENT_SET_SYSTEM_PARAM;
    break;
    case Message_Form_Opcode_Request_Battery_Value:
        g_Event_status |= EVENT_REQUEST_BATTERY_VALUE;
    break;
    }

}

static void Message_UART_Receive(uint8_t * buf, uint32_t * length)
{
    *length = 0;
    while(MessageUartRxBuffer.head != MessageUartRxBuffer.tail)
    {
        *buf++ = MessageUartRxBuffer.data[MessageUartRxBuffer.head++];
        *length ++;
        if (MessageUartRxBuffer.head == UART_BUFFER_SIZE)
        {
            MessageUartRxBuffer.head = 0;
        }
    }
}


/* 处理串口报文 */
void Message_Process_Uart(void)
{
    uint32_t length = 0;
    uint32_t i = 0;
    static uint8_t message_len = 0;
    static uint8_t message_count = 0;
    static uint8_t message_buffer[100] = 0;
    static en_Message_Form_t Message_Form_Status = Message_Form_Head;
    Message_UART_Receive(UartBuffer, &length);
    for (i = 0;i<length;i++)
    {
        switch (Message_Form_Status)
        {
        case Message_Form_Head:
            if(UartBuffer[i] == MESSAGE_FORM_HEAD)
            {
                Message_Form_Status = Message_Form_Length;
            }
        break;
        case Message_Form_Length:
            message_len = UartBuffer[i];
            message_count = 0;
            Message_Form_Status = Message_Form_Data;
        break;
        case Message_Form_Data:
            message_buffer[message_count++] = UartBuffer[i];
            if (message_count == message_len)
            {
                Message_Analyse(message_buffer);
                message_count = 0;
                message_len = 0;
                Message_Form_Status = Message_Form_Head;
            }
        break;
        }
    }
}

/* 处理wifi报文 */
void Message_Process_Wifi(void)
{
    uint32_t i = 0;
    uint32_t length = 0;
    
    static uint8_t message_len = 0;
    static uint8_t message_count = 0;
    static uint8_t message_buffer[100] = 0;
    static en_Message_Form_t Message_Form_Status = Message_Form_Head;
    ESP8266_ReceiveData(WifiBuffer, WIFI_BUFFER_SIZE, *length);
    for (i = 0;i<length;i++)
    {
        switch (Message_Form_Status)
        {
        case Message_Form_Head:
            if(WifiBuffer[i] == MESSAGE_FORM_HEAD)
            {
                Message_Form_Status = Message_Form_Length;
            }
        break;
        case Message_Form_Length:
            message_len = WifiBuffer[i];
            message_count = 0;
            Message_Form_Status = Message_Form_Data;
        break;
        case Message_Form_Data:
            message_buffer[message_count++] = WifiBuffer[i];
            if (message_count == message_len)
            {
                Message_Analyse(message_buffer);
                message_count = 0;
                message_len = 0;
                Message_Form_Status = Message_Form_Head;
            }
        break;
        }
    }
}

void Message_UART_RxCpltCallback(void *handle)
{
    /* If ring buffer end is reached reset tail pointer to start of buffer */
    if(++MessageUartRxBuffer.tail >= UART_BUFFER_SIZE)
    {
        MessageUartRxBuffer.tail = 0;   
    }

    HAL_UART_Receive_IT(handle, (uint8_t *)&MessageUartRxBuffer.data[MessageUartRxBuffer.tail], 1);
}

void Message_UART_ErrorCallback(void *handle)
{

}
