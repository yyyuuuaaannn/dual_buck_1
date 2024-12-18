#include "main.h"
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "PID.h"
#include "Serial.h"

uint8_t RX_Data[6];		//RX_Data [0~3]=Data [4]=AddressNumber [5]=Flag
float * RX_Addr[7];
Serial_TX_Buffer_Typedef Serial_TX_Buffer={{0},{0x00, 0x00, 0x80, 0x7f}};
//Serial_RX_Buffer_Typedef Serial_RX_Buffer; //not used

void Serial_RX(void)
{
	if(RX_Data[5] == 0xAF)
	{
		*RX_Addr[RX_Data[4]] = *(float *)(&RX_Data[0]);
		RX_Data[5]=0;
//		HAL_UART_Receive_DMA(&huart2, RX_Data, 6);
	}
//	if(Serial_RX_Buffer.flag)
//	{
//		uint8_t tmp = sizeof(Serial_RX_Buffer);
//		Serial_RX_Buffer.flag = 0;
//		*RX_Addr[Serial_RX_Buffer.addr] = Serial_RX_Buffer.data;
//		
//		HAL_UART_Receive_DMA(&huart2, (uint8_t*)&Serial_RX_Buffer, tmp);
//	}
	return;
}

void CUST_UART_IdleCallback(UART_HandleTypeDef *huart)
{	
	if(__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE))
	{
		__HAL_UART_CLEAR_IDLEFLAG(huart);
		Serial_RX();
		HAL_UARTEx_ReceiveToIdle_DMA(huart, RX_Data, 6);
	}
}
