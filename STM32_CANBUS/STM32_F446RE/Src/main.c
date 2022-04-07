
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2021 STMicroelectronics
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
#include "stm32f4xx_hal.h"

/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "stdlib.h"
#include "KEYPAD.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

CAN_HandleTypeDef hcan1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
CAN_TxHeaderTypeDef		TxHeader;
CAN_RxHeaderTypeDef 	RxHeader;
uint8_t								TxData[8];
uint8_t								RxData[8];
uint32_t							TxMailbox;

int16_t value=0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART2_UART_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
static void MX_CAN1_UserInit(void);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

void CanWriteData(uint16_t ID);

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
	{
			if(hadc->Instance==ADC1)
				{
						value=HAL_ADC_GetValue(hadc);
				}
	}






#ifdef __GNUC__
 /* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
 set to 'Yes') calls __io_putchar() */
 #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
 #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

/**
 * @brief Retargets the C library printf function to the USART.
 * @param None
 * @retval None
 */
PUTCHAR_PROTOTYPE 
{
 /* Place your implementation of fputc here */
 /* e.g. write a character to the USART */
 HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 100);


 return ch;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

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
  MX_CAN1_Init();
  MX_ADC1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
	HAL_ADC_Start_IT(&hadc1);
	MX_CAN1_UserInit();
	KeypadInit();
	// CAN start
	if (HAL_CAN_Start(&hcan1) != HAL_OK)
	{
		/* Start Error */
		Error_Handler();
		//HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
	} //else {
		//HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
	//}
	
	// CAN notifications (interrupts)
	if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_TX_MAILBOX_EMPTY) != HAL_OK)
	{
		/* Notification Error */
		Error_Handler();
		//HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
	} //else {
		//HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
	//}
	
//	/* CAN TX messager */
//	TxHeader.StdId = 0x200;
//	TxHeader.ExtId = 0x00;
//	TxHeader.RTR = CAN_RTR_DATA;
//	TxHeader.IDE = CAN_ID_STD;
//	TxHeader.DLC = 8;
//	TxHeader.TransmitGlobalTime = DISABLE;
//	TxData[0] = 1;
//	TxData[1] = 2;
//	TxData[2] = 3;
//	TxData[3] = 4;
//	TxData[4] = 5;
//	TxData[5] = 6;
//	TxData[6] = 7;
//	TxData[7] = 8;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	HAL_CAN_Start(&hcan1);
  HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING);
	uint16_t x=0;
  while (1)
  {

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
		

		
		
		
		HAL_ADC_Start_IT(&hadc1);
		if(value>3200)
			{
				CanWriteData(0x50);
			}
		
		x=QuetPhim();
		if(x!=0)
		{
			
			CanWriteData(x);
			HAL_Delay(100);
		}
		

		if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_9)==0)
			{
				HAL_Delay(200);
				CanWriteData(0x00);
			}
//			if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13)==0)
//				{
//						HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, 0);
//				}
//				else HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, 1);
  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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

/* ADC1 init function */
static void MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* CAN1 init function */
static void MX_CAN1_Init(void)
{

  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 10;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_13TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
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
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6|GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PC6 PC8 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PC9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PB8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void CanWriteData(uint16_t ID)
{
	int i=0;
	for(i=0 ; i<8 ; i++)
	{
		TxData[i]= rand()%0xff;
	}


  /* transmit */
	
	TxHeader.StdId = ID;				// CAN message address
	//TxHeader.ExtId = 0x00;			 	// Only used if (TxHeader.IDE == CAN_ID_EXT)
	TxHeader.RTR = CAN_RTR_DATA;	// Data request, not remote request
	TxHeader.IDE = CAN_ID_STD;		// Standard CAN, not Extended
	TxHeader.DLC = 8;							// Data size in bytes
	TxHeader.TransmitGlobalTime = DISABLE;
	
		HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);

		//TxData[7] = TxData[7] + 1;
    
}





static void MX_CAN1_UserInit(void) 
{
	CAN_FilterTypeDef			sFilterConfig;
	
	// CAN filter structure items configuration
	sFilterConfig.FilterBank = 0;		// Use first filter bank
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;		// Look for specific can messages
	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	sFilterConfig.FilterIdHigh = 0x00;		 	// Filter registers need to be shifted left 5 bits
	sFilterConfig.FilterIdLow = 0x0000;				// Filter registers need to be shifted left 5 bits
	sFilterConfig.FilterMaskIdHigh = 0x0000;	// Unused
	sFilterConfig.FilterMaskIdLow = 0x0000;		// Unused
	sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
	sFilterConfig.FilterActivation = ENABLE;
	sFilterConfig.SlaveStartFilterBank = 0;		// Set all filter banks for CAN1
	
	// CAN filter configuration function call
	if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK)
	{
		/* Filter Configuration Error */
		Error_Handler();
	} 
}



// Transmission Mailbox 0 complete callback.
void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan) {
	if(TxHeader.StdId==0x10)	//nut *
		{
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, 1);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, 1);
					printf("ID : 0x0%x\n", RxHeader.StdId);
					int i=0;
					for(i=0 ; i<8 ; i++) printf("%x  ", RxData[i]);
					printf("\n");
			printf("bat tat ca quat & den\r\n");
		}
	if(TxHeader.StdId==0x11)	//nut#
		{
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, 0);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, 0);
					printf("ID : 0x0%x\n", RxHeader.StdId);
					int i=0;
					for(i=0 ; i<8 ; i++) printf("%x  ", RxData[i]);
					printf("\n");
			printf("tat tat  ca quat & den\r\n");
		}
	if(TxHeader.StdId==0x03)
				{
					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, 1);
					
					printf("ID : 0x0%x\n", RxHeader.StdId);
					int i=0;
					for(i=0 ; i<8 ; i++) printf("%x  ", RxData[i]);
					printf("\n");
			printf("bat tat ca den\r\n");		
				}
	if(TxHeader.StdId==0x04)
				{
					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, 0);
					
					printf("ID : 0x0%x\n", RxHeader.StdId);
					int i=0;
					for(i=0 ; i<8 ; i++) printf("%x  ", RxData[i]);
					printf("\n");
			printf("tat tat ca den\r\n");		
				}
	if(TxHeader.StdId==0x05)
		{
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, 1);

			printf("ID : 0x0%x\n", RxHeader.StdId);
			int i=0;
			for(i=0 ; i<8 ; i++) printf("%x  ", RxData[i]);
			printf("\n");
			printf("bat tat ca quat\r\n");
		}
	if(TxHeader.StdId==0x06)
		{
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, 0);
			
			printf("ID : 0x0%x\n", RxHeader.StdId);
			int i=0;
			for(i=0 ; i<8 ; i++) printf("%x  ", RxData[i]);
			printf("\n");
			printf("tat tat ca quat\r\n");
		}
		
		//IR
		if(TxHeader.StdId==0x00)
		{
			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6|GPIO_PIN_8);
			
			printf("ID : 0x0%x\n", RxHeader.StdId);
			int i=0;
			for(i=0 ; i<8 ; i++) printf("%x  ", RxData[i]);
			printf("\n");
			printf("dao trang thai thiet bi\r\n");
		}
		//mq2 446
		if(TxHeader.StdId==0x50)
		{
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6|GPIO_PIN_8, 1);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, 0);
			printf("ID : 0x0%x\n", RxHeader.StdId);
			int i=0;
			for(i=0 ; i<8 ; i++) printf("%x  ", RxData[i]);
			printf("\n");
			printf("phat hien khi Gas\r\n");
			printf("dieu khien module sim\r\n");
		}
		
		
		//tat coi
		if(TxHeader.StdId==0x100)
		{
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6|GPIO_PIN_8, 0);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, 1);
			printf("ID : 0x0%x\n", RxHeader.StdId);
			int i=0;
			for(i=0 ; i<8 ; i++) printf("%x  ", RxData[i]);
			printf("\n");
			printf("tat tat ca thiet bi\r\n");
		}

		
		

		
		
		
}






// CAN Rx Callback
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
	HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader, RxData);
	
	if(RxHeader.StdId==0x10) //nut *
		{
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, 1);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, 1);
			
			//truyen hercules
			printf("ID : 0x%x\n", RxHeader.StdId);
			int i=0;
			for(i=0 ; i<8 ; i++) printf("%x  ", RxData[i]);
			printf("\n");
			printf("bat tat ca quat & den\r\n");
		}	
		if(RxHeader.StdId==0x11) //nut #
		{
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, 0);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, 0);
			
			printf("ID : 0x%x\n", RxHeader.StdId);
			int i=0;
			for(i=0 ; i<8 ; i++) printf("%x  ", RxData[i]);
			printf("\n");
			printf("tat tat ca quat & den\r\n");
		}	
		if(RxHeader.StdId==0x01)
				{
					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, 1);
					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, 1);
					printf("ID : 0x0%x\n", RxHeader.StdId);
					int i=0;
					for(i=0 ; i<8 ; i++) printf("%x  ", RxData[i]);
					printf("\n");
					printf("bat tat ca CAN Node nhan\r\n");
				}	
		if(RxHeader.StdId==0x02)
				{
					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, 0);
					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, 0);
					printf("ID : 0x0%x\n", RxHeader.StdId);
					int i=0;
					for(i=0 ; i<8 ; i++) printf("%x  ", RxData[i]);
					printf("\n");
					printf("tat tat ca CAN Node nhan\r\n");
				}	
		if(RxHeader.StdId==0x03)
				{
					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, 1);
					
					printf("ID : 0x0%x\n", RxHeader.StdId);
					int i=0;
					for(i=0 ; i<8 ; i++) printf("%x  ", RxData[i]);
					printf("\n");
					printf("bat tat ca den\r\n");
				}	
if(RxHeader.StdId==0x04)
		{
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, 0);
			
			printf("ID : 0x0%x\n", RxHeader.StdId);
			int i=0;
			for(i=0 ; i<8 ; i++) printf("%x  ", RxData[i]);
			printf("\n");
			printf("tat tat ca den\r\n");
		}	
if(RxHeader.StdId==0x05)
		{
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, 1);

			printf("ID : 0x0%x\n", RxHeader.StdId);
			int i=0;
			for(i=0 ; i<8 ; i++) printf("%x  ", RxData[i]);
			printf("\n");
			printf("bat tat ca quat\r\n");
		}	
if(RxHeader.StdId==0x06)
		{
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, 0);
			
			printf("ID : 0x0%x\n", RxHeader.StdId);
			int i=0;
			for(i=0 ; i<8 ; i++) printf("%x  ", RxData[i]);
			printf("\n");
			printf("tat tat ca quat\r\n");
		}
//mq2 446
		if(RxHeader.StdId==0x50)
		{
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6|GPIO_PIN_8, 1);
			
			printf("ID : 0x0%x\n", RxHeader.StdId);
			int i=0;
			for(i=0 ; i<8 ; i++) printf("%x  ", RxData[i]);
			printf("\n");
			printf("phat hien khi gas 446\n dieu khien module sim\r\n");
			}
//mq2 103
		if(RxHeader.StdId==0x51)
		{
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6|GPIO_PIN_8, 1);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, 0);
			printf("ID : 0x0%x\n", RxHeader.StdId);
			int i=0;
			for(i=0 ; i<8 ; i++) printf("%x  ", RxData[i]);
			printf("\n");
			}		
//tat coi
		if(RxHeader.StdId==0x100)
		{
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6|GPIO_PIN_8, 0);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, 1);
			printf("ID : 0x0%x\n", RxHeader.StdId);
			int i=0;
			for(i=0 ; i<8 ; i++) printf("%x  ", RxData[i]);
			printf("\n");
			printf("tat coi\n");
		}
//mq2 476
		if(RxHeader.StdId==0x476)
		{
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6|GPIO_PIN_8, 1);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, 0);
			printf("ID : 0x0%x\n", RxHeader.StdId);
			int i=0;
			for(i=0 ; i<8 ; i++) printf("%x  ", RxData[i]);
			printf("\n");
			printf("phat hien khi gas 476\n dieu khien module sim\r\n");
			}			
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
