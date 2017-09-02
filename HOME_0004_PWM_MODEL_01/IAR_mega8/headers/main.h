#ifndef _MAIN_H_
#define _MAIN_H_

//14 мсек
#define ICR1_DEF 14000
//Сколько элекр-х градусов на 1мксек


#define MAX_PWM     2300
#define MIN_PWM     900
#define DEF_PWM     900
#define DELTA      (70/10)
#define ANGLE_MAX   75

#define CAL ((MAX_PWM-MIN_PWM)/ANGLE_MAX)
//(ICR1_DEF/1000)

void InitCPU(void);

#endif

