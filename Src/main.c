/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
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
#include "light_control.h"
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdbool.h>
/* USER CODE END Includes */
/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart1_tx;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
uint8_t RX_BUFFER[256] ;
uint8_t LightCalOLD[20] = {0};
//uint8_t SERSOR_FRAME[] = {0xA5,0x45,0x01,
//													0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
//													0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
//													0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
//													0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
//													0X55,0XAA}; 

//uint8_t LightOutputTemp1[9]={0};
//uint8_t LightOutputTemp2[9]={0};
//uint8_t LightOutputTemp3[9]={0};
//uint8_t LightOutputTemp4[9]={0};
//uint8_t LightOutputTemp5[9]={0};
//uint8_t LightOutputTemp6[9]={0};
//uint8_t LightOutputTemp7[9]={0};
//uint8_t LightOutputTemp8[9]={0};
//uint8_t LightOutputTemp9[9]={0};
//uint8_t LightOutputTemp10[9]={0};
//uint8_t LightOutputTemp11[9]={0};
//uint8_t LightOutputTemp12[9]={0};
//uint8_t LightOutputTemp13[9]={0};
//uint8_t LightOutputTemp14[9]={0};
//uint8_t LightOutputTemp15[9]={0};

uint8_t LAMP_FRAME[] = {0XA5,0x08,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X55,0XAA}; 
uint8_t GW_FRAME[] = {0XA5 ,0XFF, 0XFF, 0X00, 0X04 ,0X09 ,0X03,0X55, 0XAA};  
uint8_t SET_FRAME[] = {'A','T','+','A','0','0','5'} ; 
//源地址   目标地址    包长 功能码   
#define SOURCE_ADD 0X0004
#define GATEWAY_ADD 0XFFFF
#define HEADER 0XA5 
#define DATA_POINTER 7
#define SRC_POINTER 1
#define TGT_POINTER 3
#define SERSOR_FRAMELENGTH 15
#define GW_FRAMELENGTH  9
#define TAIL   0XAA
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);

uint8_t mesgGet(void);
uint8_t mesgDecode(char *message);
int FindChar(const char *pbuff,char ch);
/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
//像素点数据存放数组
bool SensorData[SensorNumMax][64];
uint8_t DistanceData[2];
uint16_t LightCalTemp[9];
/******************************灯具-传感器 对应表********************************/
/************灯Id 传感器id1、2、3 传感器点1、2、3 权值百分比1、2、3**************/
extern unsigned char LightSensorTable[9][16];
/*******************************顶灯-射灯/台灯 对应表*********************************/
extern unsigned char LightLightTable[5][3];
/*******************************灯具输出 对应表*********************************/
extern uint8_t LightOutputTable[20];


static unsigned int sys_tick ;

// 1ms 周期执行任务

void HAL_SYSTICK_Callback(void)
{
	sys_tick++ ; 
	if(sys_tick > 4000000000) sys_tick = 0 ; 	
}

void delay_ms(int ms)
{
	unsigned int _tick ;
	_tick = sys_tick ; 
	while(sys_tick - _tick < ms) ; 
}


void uart1_send_str(const char *str)
{
	uint16_t len;
	len = strlen(str);
	while(len--)
	{
		//HAL_UART_Transmit(&huart1,(uint8_t *)str++,1,0xFF);
		HAL_UART_Transmit_IT(&huart1,(uint8_t *)str++,1);
//		delay_ms(5);
	}
}
/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
	uint8_t i,j;
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
//  MX_USART3_UART_Init();

  /* USER CODE BEGIN 2 */
    __HAL_UART_ENABLE_IT( &huart2, UART_IT_IDLE);    //使能串口2空闲中断
    HAL_UART_Receive_IT(&huart2,RX_BUFFER,UART2_RECV_LEN);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
		
		for(i = 0;i < LightNumMax;i++)
		{
			LightCalOLD[i+11] = LightOutputTable[i+11];
			LightCalTemp[i] = ( HumanCalcula(LightSensorTable[i][1],LightSensorTable[i][6]) * LightSensorTable[i][11] 
														+ HumanCalcula(LightSensorTable[i][2],LightSensorTable[i][7]) * LightSensorTable[i][12] 
														+ HumanCalcula(LightSensorTable[i][3],LightSensorTable[i][8]) * LightSensorTable[i][13] 
														+ HumanCalcula(LightSensorTable[i][4],LightSensorTable[i][9]) * LightSensorTable[i][14] 
														+ HumanCalcula(LightSensorTable[i][5],LightSensorTable[i][10]) * LightSensorTable[i][15] );
			if(i < 5)
				LightCalTemp[i] += 40;
			if(LightCalTemp[i] > 255)
				LightOutputTable[i+11] = 255;
			else
				LightOutputTable[i+11] = LightCalTemp[i];
			
			if(LightOutputTable[i+11] < LightCalOLD[i+11])
				LightOutputTable[i+11] = LightCalOLD[i+11] - 3;
			if(LightOutputTable[i+11] < 0 )
				LightOutputTable[i+11] = 0;
		}
			/*********************************************************/
/*		  LightOutputTemp1[i] = LightOutputTemp2[i];
			LightOutputTemp2[i] = LightOutputTemp3[i];
			LightOutputTemp3[i] = LightOutputTemp4[i];
			LightOutputTemp4[i] = LightOutputTemp5[i];
			LightOutputTemp5[i] = LightOutputTemp6[i];
			LightOutputTemp6[i] = LightOutputTemp7[i];
			LightOutputTemp7[i] = LightOutputTemp8[i];
			LightOutputTemp8[i] = LightOutputTemp9[i];
			LightOutputTemp9[i] = LightOutputTemp10[i];
			LightOutputTemp10[i] = LightOutputTemp11[i];
			LightOutputTemp11[i] = LightOutputTemp12[i];
			LightOutputTemp12[i] = LightOutputTemp13[i];
			LightOutputTemp13[i] = LightOutputTemp14[i];
			LightOutputTemp14[i] = LightOutputTemp15[i];
			
			LightOutputTemp15[i] = ( HumanCalcula(LightSensorTable[i][1],LightSensorTable[i][4]) * LightSensorTable[i][7] 
														+ HumanCalcula(LightSensorTable[i][2],LightSensorTable[i][5]) * LightSensorTable[i][8] 
														+ HumanCalcula(LightSensorTable[i][3],LightSensorTable[i][6]) * LightSensorTable[i][9] );
		  LightOutputTable[i+11] = ( LightOutputTemp1[i] + LightOutputTemp2[i] + LightOutputTemp3[i] + LightOutputTemp4[i]
																+ LightOutputTemp5[i] + LightOutputTemp6[i] + LightOutputTemp7[i] + LightOutputTemp8[i]
																+ LightOutputTemp9[i] + LightOutputTemp10[i] + LightOutputTemp11[i] + LightOutputTemp12[i]
																+ LightOutputTemp13[i] + LightOutputTemp14[i] + LightOutputTemp15[i]  ) / 5;
		
		}
		LightOutputTable[11] = 0x00;
		LightOutputTable[13] = 0x00;
		LightOutputTable[14] = 0x00;
		LightOutputTable[15] = 0x00;
		LightOutputTable[16] = 0x00;
		LightOutputTable[18] = 0x00;
		LightOutputTable[19] = 0x00;
*/
/****************************************************************************/		
		
//		for(i = 0;i < LightShowMax;i++)
//		{
//			j = LightLightTable[i][0];
//			LightOutputTable[j+2] =  LightOutputTable[j+2] - LightShowTh * LightOutputTable[LightLightTable[i][1]+2] 
//																								 - LightShowTh * LightOutputTable[LightLightTable[i][2]+2];
//			if(LightOutputTable[j+2] < 0)
//			{
//				LightOutputTable[j+2] = 0;
//			}
//		}
		
		
//		LightOutputTable[3] =  HumanCalcula(LightSensorTable[0][1],LightSensorTable[0][4]) * LightSensorTable[0][7] ;
	//	LightOutputTable[5] = 0x30;
		HAL_UART_Transmit_DMA(&huart1,LightOutputTable,sizeof(LightOutputTable));//uart1_send_str(LightOutputTable);
		/* 数据输入处理和输出 */
		HAL_Delay(100);
		HAL_UART_DMAStop(&huart1);
		/* 数据输入处理和输出结束 */
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
	RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;//RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
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

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;//9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;//UART_WORDLENGTH_9B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;//UART_PARITY_EVEN;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }

}

/* USART3 init function */
static void MX_USART3_UART_Init(void)
{

  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
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
