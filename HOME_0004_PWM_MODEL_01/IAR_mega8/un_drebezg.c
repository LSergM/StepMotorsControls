
#include "un_drebezg.h"

struct Data_IN Test_IN_0 = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

void Test_Drebezg_IN(void) 
{
  unsigned char i;
  for (i = 0; i < INn; i++) 
  {
        if (*IN_PORT[i]&(1<<IN_PIN[i]))
        //if(GPIO_ReadInputDataBit(IN_PORT[i],IN_PIN[i]))
	{
            Test_IN_0.ckey1[i]++;			// "1"
        }
	else 
	    {
		Test_IN_0.ckey0[i]++;	// "0"
  	    } 
        Test_IN_0.ckeym[i]++; 
        if (Test_IN_0.ckeym[i]>IN_TIME[i])
        {
            Test_IN_0.ckeym[i] = 0;
            if (Test_IN_0.ckey1[i] > Test_IN_0.ckey0[i])
            {
                if(IN_TIME[i]-Test_IN_0.ckey1[i]<(IN_TIME[i]>>3)) //(Test_IN_0.ckey1[i]>=IN_TIME[i]*1)//0.92
                {
                    Test_IN_0.fin |= (1 << i);
		} 
            }
            else
		{
  		  Test_IN_0.fin &= ~(1 << i);
		}
            Test_IN_0.ckey1[i] = 0;
            Test_IN_0.ckey0[i] = 0;
	}
  }
}
