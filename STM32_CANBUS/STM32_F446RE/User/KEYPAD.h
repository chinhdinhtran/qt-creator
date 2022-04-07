#ifndef	KEYPAD_H
#define	KEYPAD_H

#include "stm32f4xx_hal.h"

//ket noi cac cot phim
#define COT_1(x)	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, x)
#define COT_2(x)	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, x)
#define COT_3(x)	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, x)


//ket noi cac hang phim
#define HANG_1 		HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_0)
#define HANG_2		HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_1)
#define HANG_3		HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_2)
#define HANG_4		HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_3)

void KeypadInit(void);
uint16_t QuetPhim(void);

#endif