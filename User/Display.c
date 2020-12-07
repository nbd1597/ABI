/*
 * Display.c
 *
 *  Created on: Nov 9, 2020
 *      Author: Duy
 */
#include "ILI9341_GFX.h"
#include "ILI9341_STM32_Driver.h"
#include <stdio.h>


void LCD_Layout()
{
	ILI9341_Fill_Screen(BLACK);
	ILI9341_Set_Rotation(SCREEN_VERTICAL_2);
	ILI9341_Draw_Text("ABI", 0, 0, RED, 2, BLACK);
	ILI9341_Draw_Text("Left arm (pressure 0)", 0, 20, RED, 1, BLACK);
	ILI9341_Draw_Text("Pulse 0", 0, 40, RED, 1, BLACK);
	ILI9341_Draw_Text("Right arm (pressure 1)", 0, 60, RED, 1, BLACK);
	ILI9341_Draw_Text("Pulse 1", 0, 80, RED, 1, BLACK);
	ILI9341_Draw_Text("Left leg (pressure 2)", 0, 100, RED, 1, BLACK);
	ILI9341_Draw_Text("Pulse 2", 0, 120, RED, 1, BLACK);
	ILI9341_Draw_Text("Right leg (pressure 3)", 0, 140, RED, 1, BLACK);
	ILI9341_Draw_Text("Pulse 3", 0, 160, RED, 1, BLACK);
}

void LCD_Update_adc(uint16_t* adc)
{
////	ILI9341_Draw_Text("Test", 0, 240, RED, 1, BLACK);
//
//	for(int i = 0; i < 8; i++)
//	{
//		sprintf(temp, "%6d", adc[i]);
////		ILI9341_Draw_Text("Test", 0, 240, RED, 1, BLACK);
//		ILI9341_Draw_Text(temp, 60, 20 + i*20, RED, 1, BLACK);
//	}

}

