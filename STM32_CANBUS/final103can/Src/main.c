/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "stdlib.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;

/* USER CODE BEGIN PV */
CAN_TxHeaderTypeDef		TxHeader;
CAN_RxHeaderTypeDef 	RxHeader;
uint8_t								TxData[8];
uint8_t								RxData[8];
uint32_t							TxMailbox;
uint16_t value=0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void MX_CAN1_UserInit(void);
void CanWriteData(uint16_t ID);
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
  MX_CAN_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	
	MX_CAN1_UserInit();
	
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
	
	/* CAN TX messager */
//	TxHeader.StdId = 0x244;
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
	//HAL_ADC_Start_IT(&hadc1);
  HAL_CAN_Start(&hcan1);
  HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING);
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

  /** Initializes the CPU, AHB and APB busses clocks 
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
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
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
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

  /* USER CODE END CAN_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA4 PA6 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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

// CAN Tx Callback
void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan) {
	
//IR
if(TxHeader.StdId==0x00)
		{
			HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_6|GPIO_PIN_7);
		}

//if(TxHeader.StdId==0x51) //nut #
//		{
//			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6|GPIO_PIN_7, 1);
//			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 0);
//		}	
//		
}





// CAN Rx Callback
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
	HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader, RxData);
	
	if(RxHeader.StdId==0x10) //nut *
		{
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, 1);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, 1);
		}	
		if(RxHeader.StdId==0x11) //nut #
		{
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, 0);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, 0);
		}	
		if(RxHeader.StdId==0x01)
				{
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, 1);
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, 1);
				}	
		if(RxHeader.StdId==0x02)
				{
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, 0);
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, 0);
				}	
		if(RxHeader.StdId==0x03)
				{
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, 1);
				}	
if(RxHeader.StdId==0x04)
		{
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, 0);
		}	
if(RxHeader.StdId==0x05)
		{
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, 1);
		}	
if(RxHeader.StdId==0x06)
		{
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, 0);
		}

//IR
if(RxHeader.StdId==0x00)
		{
			HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_6|GPIO_PIN_7);
		}		
//mq2 446
		if(RxHeader.StdId==0x50)
		{
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6|GPIO_PIN_7, 1);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 0);
		}		
//tat coi      nut 0
		if(RxHeader.StdId==0x100)
		{
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6|GPIO_PIN_7, 0);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 1);
			
		}	
//mq2 476
		if(RxHeader.StdId==0x476)
		{
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6|GPIO_PIN_7, 1);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 0);
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
