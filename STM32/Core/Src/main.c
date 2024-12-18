/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "hrtim.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "PID.h"
#include "Serial.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define HRTIM1_MASTER_TIMER_PERIOD 57600
#define PHASE_SHIFT 28800
#define MAX_VOLTAGE ((float)42.3)
#define BASE_VOLTAGE ((float)18.5)
#define VOLTAGE_OUT_LINEAR_A (0.0102601752)
#define VOLTAGE_OUT_LINEAR_B (0.5932726908)
#define CURRENT_1_LINEAR_A (-0.004751058)
#define CURRENT_1_LINEAR_B (12.48739301)
#define CURRENT_OUT_LINEAR_A (-0.005096203)
#define CURRENT_OUT_LINEAR_B (13.41522621)
#define CURRENT_ADC_FILTER 5
//#define VOLTAGE_OFFSET 0
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
extern uint8_t RX_Data[6];		//RX_Data [3:0]=Data [4]=AddressNumber [5]=Flag=0xAF
extern float * RX_Addr[7];
PID pid_voltage, pid_current;
extern Serial_TX_Buffer_Typedef Serial_TX_Buffer;
//extern Serial_RX_Buffer_Typedef Serial_RX_Buffer;		//not used
uint16_t ADC_Buffer[3];
float duty1, duty2;
float Voltage_Out_Set = 8;
float ctrl_1 = 8;
float ctrl_2 = 8;
float Voltage_Out;
float Current_Out, Current_1, Current_2;
float Current_2_Set;
uint8_t mode;
float mode_f;
float Current_Ratio = 1;
/*
mode = 0: voltage PID adjust
mode = 1: current PID adjust
*/
uint8_t mode_changed_flag;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

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
  MX_HRTIM1_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

	HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
	HAL_Delay(200);
	HAL_ADC_Start_DMA(&hadc1,(uint32_t *)ADC_Buffer,sizeof(uint16_t)*3);

	RX_Addr[0] = &Current_Ratio;
	RX_Addr[1] = &(pid_voltage.kp);
	RX_Addr[2] = &(pid_voltage.ki);
	RX_Addr[3] = &(pid_voltage.kd);
	RX_Addr[4] = &mode_f;
	RX_Addr[5] = &Voltage_Out_Set;
	
	pid_voltage.kp=0.04;
	pid_voltage.ki=0;
	pid_voltage.kd=0.12;
	pid_voltage.max_integral=MAX_VOLTAGE;
	pid_voltage.max_output=MAX_VOLTAGE;
	
	pid_current.kp=0.06;
	pid_current.ki=0;
	pid_current.kd=0.04;
	pid_current.max_integral=MAX_VOLTAGE;
	pid_current.max_output=MAX_VOLTAGE;
	
	HAL_UARTEx_ReceiveToIdle_DMA(&huart1, RX_Data, 6);
	
	HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TA2 | HRTIM_OUTPUT_TB1 | HRTIM_OUTPUT_TB2);
	HAL_HRTIM_WaveformCounterStart(&hhrtim1, HRTIM_TIMERID_MASTER | HRTIM_TIMERID_TIMER_A | HRTIM_TIMERID_TIMER_B);
	
	__HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_MASTER, HRTIM_COMPAREUNIT_1, PHASE_SHIFT);
	__HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_COMPAREUNIT_1, 14400);
	__HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, HRTIM_COMPAREUNIT_1, 14400);
	
	//HAL_TIM_Base_Start_IT(&htim3);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_HRTIM1|RCC_PERIPHCLK_USART1
                              |RCC_PERIPHCLK_ADC12;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12PLLCLK_DIV1;
  PeriphClkInit.Hrtim1ClockSelection = RCC_HRTIM1CLK_PLLCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	static uint16_t Tx_Counter;
	static uint16_t Current_ADC_Filter_Counter;
	static uint32_t Current_Out_ADC_Buffer, Current_2_ADC_Buffer;
	if(htim->Instance == TIM3)
	{
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
		
//		Voltage_Out = ADC_Buffer[0]*MAX_VOLTAGE/4095;
		Voltage_Out = ADC_Buffer[1] * VOLTAGE_OUT_LINEAR_A + VOLTAGE_OUT_LINEAR_B;
		
		Current_Out_ADC_Buffer += ADC_Buffer[2];
		Current_2_ADC_Buffer += ADC_Buffer[0];
		Current_ADC_Filter_Counter++;
		if(Current_ADC_Filter_Counter == CURRENT_ADC_FILTER)
		{
			Current_ADC_Filter_Counter = 0;
//			Current_Out = ((float)ADC_Buffer[2]/4095*3.3-2.5)/-0.185;
//			Current_In = ((float)ADC_Buffer[2]/4095*3.3-2.5)/-0.185;
			Current_Out = (float)Current_Out_ADC_Buffer / CURRENT_ADC_FILTER * CURRENT_OUT_LINEAR_A + CURRENT_OUT_LINEAR_B;
			Current_2 = (float)Current_2_ADC_Buffer / CURRENT_ADC_FILTER * CURRENT_1_LINEAR_A + CURRENT_1_LINEAR_B;

			Current_Out_ADC_Buffer = 0;
			Current_2_ADC_Buffer = 0;
		}
		Current_1 = Current_Out - Current_2;

		if(mode != (int)mode_f)
		{
			mode = mode_f;
			mode_changed_flag = 1;
			pid_voltage.integral = 0;
			pid_current.integral = 0;
			
			if(mode == 0)
			{
				RX_Addr[1] = &(pid_voltage.kp);
				RX_Addr[2] = &(pid_voltage.ki);
				RX_Addr[3] = &(pid_voltage.kd);
			}		
			if(mode == 1)
			{
				RX_Addr[1] = &(pid_current.kp);
				RX_Addr[2] = &(pid_current.ki);
				RX_Addr[3] = &(pid_current.kd);
			}
		}

		if(mode == 0 || mode == 1)
		{
			Current_2_Set = Current_1 * Current_Ratio;
			
			if(mode_changed_flag)
			{
				pid_voltage.last_error =	Voltage_Out_Set - Voltage_Out;
				pid_current.last_error =	Current_2_Set - Current_2;
				mode_changed_flag = 0;
			}
			
			PID_Calc(&pid_voltage, Voltage_Out_Set - Voltage_Out);
			PID_Calc(&pid_current, Current_2_Set - Current_2);
			ctrl_1 += pid_voltage.output;
			ctrl_2 += pid_current.output;
			
//			ctrl = Voltage_Out_Set;  //***************************** TEST CODE *****************************
			
			LIMIT(ctrl_1, 0.5f, 2*MAX_VOLTAGE);
			LIMIT(ctrl_2, 0.5f, 2*MAX_VOLTAGE);
		}
		

		duty1 = ctrl_1 / BASE_VOLTAGE;
		duty2 = ctrl_2 / BASE_VOLTAGE;
		LIMIT(duty1, 0.05f, 0.95f);	
		LIMIT(duty2, 0.05f, 0.95f);			
		
		
		
		__HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_COMPAREUNIT_1, duty1*HRTIM1_MASTER_TIMER_PERIOD);
		__HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, HRTIM_COMPAREUNIT_1, duty2*HRTIM1_MASTER_TIMER_PERIOD);
		
		
		
		Tx_Counter++;
		if(Tx_Counter==300)
		{
			Tx_Counter=0;
			
			if(mode == 0)
			{
				Serial_TX_Buffer.TX_Data[0] = Voltage_Out;
				Serial_TX_Buffer.TX_Data[1] = Voltage_Out_Set;
				Serial_TX_Buffer.TX_Data[2] = Current_2;
				Serial_TX_Buffer.TX_Data[3] = Current_1;
//				Serial_TX_Buffer.TX_Data[2] = ADC_Buffer[0];
//				Serial_TX_Buffer.TX_Data[3] = ADC_Buffer[1];
			}
			if(mode == 1)
			{
				Serial_TX_Buffer.TX_Data[0] = Voltage_Out;
				Serial_TX_Buffer.TX_Data[1] = Current_2_Set;
				Serial_TX_Buffer.TX_Data[2] = Current_2;
				Serial_TX_Buffer.TX_Data[3] = ctrl_2;
			}
			
			HAL_UART_Transmit_DMA(&huart1,(uint8_t *)&Serial_TX_Buffer,sizeof(Serial_TX_Buffer));
		}
		
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
	}
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
