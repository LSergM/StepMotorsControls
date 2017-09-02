/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __pwm_pcb
#define __pwm_pcb

#ifdef __cplusplus
 extern "C" {
#endif 

//#include <iotiny2313.h>
#include "ioavr.h"   
	 
/* Init LCD PINS	*/	
	 
#define INn           5
typedef enum 
{	 
  PWM_D0    = 0,
  PWM_D1    = 1,
  PWM_D2    = 2,
  PWM_D3    = 3,
  END_STOP  = 4
}PWM_TypeDef;

#define PWM_D0_PIN                      PORTD3
#define PWM_D0_PORT                   	PIND
#define PWM_D0_TIME			20

#define PWM_D1_PIN                      PORTD5
#define PWM_D1_PORT                 	PIND
#define PWM_D1_TIME			20

#define PWM_D2_PIN                      PORTD6
#define PWM_D2_PORT                     PIND
#define PWM_D2_TIME			20

#define PWM_D3_PIN                   	PORTD7
#define PWM_D3_PORT                	PIND
#define PWM_D3_TIME			20

#define END_STOP_PIN                   	PORTB4
#define END_STOP_PORT                	PINB
#define END_STOP_TIME			20

extern __flash unsigned char volatile __io* IN_PORT[INn];
extern __flash const unsigned char IN_PIN[INn];
extern __flash const unsigned char IN_TIME[INn];

#define RE_OFF  PORTB|= (1<<PORTB0)
#define RE_ON   PORTB&=~(1<<PORTB0)
#define DE_OFF  PORTD&=~(1<<PORTD2)
#define DE_ON   PORTD|= (1<<PORTD2) 

#define SET_STEP_PORT_ON    PORTB |= (1<<PORTB6)
#define SET_STEP_PORT_OFF   PORTB &= ~(1<<PORTB6)
#define SET_DRV_DIR_FRW     PORTB |= (1<<PORTB7)
#define SET_DRV_DIR_REV     PORTB &= ~(1<<PORTB7)
#define SET_DRV_EN_ON       PORTC &= ~(1<<PORTC5)
#define SET_DRV_EN_OFF      PORTC |= (1<<PORTC5)
#define SET_DRV_M0_ON       PORTC |= (1<<PORTC4)
#define SET_DRV_M0_OFF      PORTC &= ~(1<<PORTC4)
#define SET_DRV_M1_ON       PORTC |= (1<<PORTC3)
#define SET_DRV_M1_OFF      PORTC &= ~(1<<PORTC3)
#define SET_DRV_M2_ON       PORTC |= (1<<PORTC2)
#define SET_DRV_M2_OFF      PORTC &= ~(1<<PORTC2)  
#define SET_DRV_RST_ON      PORTB &= ~(1<<PORTB3)
#define SET_DRV_RST_OFF     PORTB |=  (1<<PORTB3)

#define GET_DRV_FAULT      !(PINC & (1 << PORTC6))


/*
void LcdPinOff(LDC_TypeDef lcdpin);
void LcdPinOn(LDC_TypeDef lcdpin);
void LcdDataPinsOff(void);
void LcdDataPinsOn(void);
void LcdDataPinsOut(void);
void LcdDataPinsIn(void);
void LcdPinsOut(void);
void InsInit(void);
*/
#ifdef __cplusplus
}
#endif


#endif
