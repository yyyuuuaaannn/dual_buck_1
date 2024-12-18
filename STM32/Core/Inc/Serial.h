#ifndef __SERIAL_H
#define __SERIAL_H
#include "stm32f3xx.h"                  // Device header

typedef struct Serial_TX_Buffer_Structure
{
	float TX_Data[4];
	uint8_t tail[4];
}Serial_TX_Buffer_Typedef;

//typedef struct Serial_RX_Buffer_Structure
//{
//	float data;
//	uint8_t addr;
//	uint8_t flag;
//}Serial_RX_Buffer_Typedef;		//not used

void Serial_RX(void);
void CUST_UART_IdleCallback(UART_HandleTypeDef *huart);

#endif
