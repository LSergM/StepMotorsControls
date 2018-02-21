#ifndef _UN_DREBEZG_H_
#define _UN_DREBEZG_H_

#include "pwm_pcb.h"

struct Data_IN {
    unsigned char ckeym[INn];
    unsigned char Flags_Temp_State_IN; //16 ???
    unsigned char fin; //16 ???
    unsigned char ckey0[INn]; //8 ???
    unsigned char ckey1[INn]; //8 ???
};

extern struct Data_IN Test_IN_0;

extern void Test_Drebezg_IN(void);

#endif
