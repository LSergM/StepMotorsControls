/* Includes ------------------------------------------------------------------*/
#include "pwm_pcb.h"

__flash unsigned char volatile __io*  IN_PORT[INn] = {&PWM_D0_PORT, &PWM_D1_PORT, &PWM_D2_PORT, &PWM_D3_PORT, &END_STOP_PORT};
__flash const unsigned char IN_PIN[INn]  = {PWM_D0_PIN,  PWM_D1_PIN,  PWM_D2_PIN,  PWM_D3_PIN, END_STOP_PIN};
__flash const unsigned char IN_TIME[INn] = {PWM_D0_TIME, PWM_D1_TIME, PWM_D2_TIME, PWM_D3_TIME, END_STOP_TIME};
/*
void LcdPinOff(LDC_TypeDef lcdpin)
{
  LCD_PORT[lcdpin]->BRR = LCD_PIN[lcdpin];   
}

void LcdPinOn(LDC_TypeDef lcdpin)
{
  LCD_PORT[lcdpin]->BSRR = LCD_PIN[lcdpin];   
}

void LcdDataPinsOff(void)
{
	LcdPinOff(LCD_D0);
	LcdPinOff(LCD_D1);
	LcdPinOff(LCD_D2);
	LcdPinOff(LCD_D3);
}

void LcdDataPinsOn(void)
{
	LcdPinOn(LCD_D0);
	LcdPinOn(LCD_D1);
	LcdPinOn(LCD_D2);
	LcdPinOn(LCD_D3);
}

void LcdDataPinsOut(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
  GPIO_InitStructure.GPIO_Pin = LCD_PIN[LCD_D0];
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(LCD_PORT[LCD_D0], &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
  GPIO_InitStructure.GPIO_Pin = LCD_PIN[LCD_D1];
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(LCD_PORT[LCD_D1], &GPIO_InitStructure);	
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
  GPIO_InitStructure.GPIO_Pin = LCD_PIN[LCD_D2];
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(LCD_PORT[LCD_D2], &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
  GPIO_InitStructure.GPIO_Pin = LCD_PIN[LCD_D3];
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(LCD_PORT[LCD_D3], &GPIO_InitStructure);	
}

void LcdDataPinsIn(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
  GPIO_InitStructure.GPIO_Pin = LCD_PIN[LCD_D0];
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(LCD_PORT[LCD_D0], &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
  GPIO_InitStructure.GPIO_Pin = LCD_PIN[LCD_D1];
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(LCD_PORT[LCD_D1], &GPIO_InitStructure);	
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
  GPIO_InitStructure.GPIO_Pin = LCD_PIN[LCD_D2];
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(LCD_PORT[LCD_D2], &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
  GPIO_InitStructure.GPIO_Pin = LCD_PIN[LCD_D3];
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(LCD_PORT[LCD_D3], &GPIO_InitStructure);	
}


void LcdPinsOut(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
  GPIO_InitStructure.GPIO_Pin = LCD_PIN[LCD_D0];
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(LCD_PORT[LCD_D0], &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
  GPIO_InitStructure.GPIO_Pin = LCD_PIN[LCD_D1];
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(LCD_PORT[LCD_D1], &GPIO_InitStructure);	
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
  GPIO_InitStructure.GPIO_Pin = LCD_PIN[LCD_D2];
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(LCD_PORT[LCD_D2], &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
  GPIO_InitStructure.GPIO_Pin = LCD_PIN[LCD_D3];
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(LCD_PORT[LCD_D3], &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
  GPIO_InitStructure.GPIO_Pin = LCD_PIN[LCD_RS];
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(LCD_PORT[LCD_RS], &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
  GPIO_InitStructure.GPIO_Pin = LCD_PIN[LCD_RW];
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(LCD_PORT[LCD_RW], &GPIO_InitStructure);	
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
  GPIO_InitStructure.GPIO_Pin = LCD_PIN[LCD_E];
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(LCD_PORT[LCD_E], &GPIO_InitStructure);	
}

void InsInit(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_InitStructure.GPIO_Pin = IN_PIN[IN_KEY_UP];
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(IN_PORT[IN_KEY_UP], &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_InitStructure.GPIO_Pin = IN_PIN[IN_KEY_DOWN];
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(IN_PORT[IN_KEY_DOWN], &GPIO_InitStructure);	
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_InitStructure.GPIO_Pin = IN_PIN[IN_KEY_ENTER];
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(IN_PORT[IN_KEY_ENTER], &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_InitStructure.GPIO_Pin = IN_PIN[IN_KEY_ESC];
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(IN_PORT[IN_KEY_ESC], &GPIO_InitStructure);
	
}
*/

   
