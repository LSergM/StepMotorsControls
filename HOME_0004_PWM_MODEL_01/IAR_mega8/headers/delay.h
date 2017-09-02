/*******************************************************************
Файл delay.h
Организация задержек
01.06.2006
*******************************************************************/

#ifndef DELAY_H
#define DELAY_H

#include <inavr.h>

#define CPU_CLK_Hz 14745600                                 //Частота кварца в Гц
#define CPU_CLK_kHz (unsigned long)(CPU_CLK_Hz/1000)       //Частота кварца в кГц

#define delay_ns(x) __delay_cycles(x*(CPU_CLK_kHz/1000000000)) //Задержка нс
#define delay_us(x) __delay_cycles(x*(CPU_CLK_Hz/1000000)) //Задержка мкс
#define delay_ms(x) __delay_cycles(x*(CPU_CLK_Hz/1000))    //Задержка с
#define delay_s(x)  __delay_cycles(x*CPU_CLK_Hz)

#endif
