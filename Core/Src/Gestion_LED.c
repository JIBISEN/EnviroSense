/*
 * Gestion_LED.c
 *
 *  Created on: Jul 4, 2024
 *      Author: DASILVAM
 */

# include <stdio.h>
#include <main.h>

void GestionLed(int LedState)
{

	switch(LedState)
	{
	case 0:// Affichage de la temperature
		  HAL_GPIO_TogglePin(GPIOB,L0_Pin);
		  HAL_GPIO_WritePin(GPIOB, L1_Pin, 0);
		  HAL_GPIO_WritePin(GPIOB, L2_Pin, 0);
		  HAL_GPIO_WritePin(GPIOB, L3_Pin, 0);
		  HAL_GPIO_WritePin(GPIOB, L4_Pin, 0);
	break;

	case 1:// Affichage de la consigne
		  HAL_GPIO_WritePin(GPIOB, L0_Pin, 0);
		  HAL_GPIO_TogglePin(GPIOB,L1_Pin);
		  HAL_GPIO_WritePin(GPIOB, L2_Pin, 0);
		  HAL_GPIO_WritePin(GPIOB, L3_Pin, 0);
		  HAL_GPIO_WritePin(GPIOB, L4_Pin, 0);
	break;

	case 2:// Affichage de l'alarme temp
		  HAL_GPIO_WritePin(GPIOB, L0_Pin, 0);
		  HAL_GPIO_WritePin(GPIOB, L1_Pin, 0);
		  HAL_GPIO_TogglePin(GPIOB,L2_Pin);
		  HAL_GPIO_WritePin(GPIOB, L3_Pin, 0);
		  HAL_GPIO_WritePin(GPIOB, L4_Pin, 0);
	break;

	case 3:// Affichage de l'alarm accel
		  HAL_GPIO_WritePin(GPIOB, L0_Pin, 0);
		  HAL_GPIO_WritePin(GPIOB, L1_Pin, 0);
		  HAL_GPIO_WritePin(GPIOB, L2_Pin, 0);
		  HAL_GPIO_WritePin(GPIOB, L3_Pin, 1);
		  HAL_GPIO_WritePin(GPIOB, L4_Pin, 0);
	break;

	case 4:// Extinction des leds
		  HAL_GPIO_WritePin(GPIOB, L0_Pin, 0);
		  HAL_GPIO_WritePin(GPIOB, L1_Pin, 0);
		  HAL_GPIO_WritePin(GPIOB, L2_Pin, 0);
		  HAL_GPIO_WritePin(GPIOB, L3_Pin, 0);
		  HAL_GPIO_WritePin(GPIOB, L4_Pin, 0);
	break;

	case 5:// Extinction des leds
		  HAL_GPIO_WritePin(GPIOB, L0_Pin, 1);
		  HAL_GPIO_WritePin(GPIOB, L1_Pin, 1);
		  HAL_GPIO_WritePin(GPIOB, L2_Pin, 1);
		  HAL_GPIO_WritePin(GPIOB, L3_Pin, 1);
		  HAL_GPIO_WritePin(GPIOB, L4_Pin, 1);
	break;

	}
}
