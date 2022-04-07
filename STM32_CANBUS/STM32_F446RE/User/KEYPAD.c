#include "KEYPAD.h"

void KeypadInit(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	
  __HAL_RCC_GPIOC_CLK_ENABLE();	
	
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12, GPIO_PIN_RESET);
	
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
	
	

  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);	
	
	COT_1(1);
	COT_2(1);
	COT_3(1);

}

uint16_t QuetPhim(void)
{
	uint16_t Key=0;
	
	COT_1(0);//quet cot 1
	if(HANG_1==0) Key=0x01;
	if(HANG_2==0) Key=0x04;
	if(HANG_3==0) Key=0x07;
	if(HANG_4==0) Key=0x10;
	COT_1(1);//ngung quet cot
	
	COT_2(0);//quet cot 2
	if(HANG_1==0) Key=0x02;
	if(HANG_2==0) Key=0x05;
	if(HANG_3==0) Key=0x08;
	if(HANG_4==0) Key=0x100;
	COT_2(1);//ngung quet cot	
	
	COT_3(0);//quet cot 3
	if(HANG_1==0) Key=0x03;
	if(HANG_2==0) Key=0x06;
	if(HANG_3==0) Key=0x09;
	if(HANG_4==0) Key=0x11;
	COT_3(1);//ngung quet cot	
	
	
	return Key;
}


