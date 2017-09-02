#ifndef _MODBUS_H_
#define _MODBUS_H_

//#include <iotiny2313.h>
#include "ioavr.h"
#include "pwm_pcb.h"

#define UARTBUF                     40
#define NWBUF                       UARTBUF
#define NUMBER_OF_R_REGS            6
#define NUMBER_OF_RW_REGS           6
#define NUMBER_OF_RW_BIT_VARIABLES  8
#define ADRESS_DEVICE               200
#define CNT_SPEED                   240
#define FUNCTION_MODBUS		    1	


//  Flags_Link_To_PC
#define RECIVED_POCET_FROM_PC 0

//ModBUS_r_reg
#define ADC_0            0
#define ADC_1            1
#define CNT_0            2
#define DRV_STATE        3   //Регистр состояния драйвера
/*
    0 - 8 - Последнее значение, кол-ва шагов
    9 - DIR направление движения
    10 - Сигнал "Перегрузка"
    11 - Сигнал "Сброс"
    12 - Сигнал "EN" Разрешение работы
    13-15 Разрешающая способность М0, М1, М2
*/
    #define DRV_DIR        9   //Регистр состояния драйвера
    #define DRV_FAULT      10   //Регистр состояния драйвера
    #define DRV_RST        11   //Регистр состояния драйвера
    #define DRV_EN         12   //Регистр состояния драйвера
    #define DRV_M0         13   //Регистр состояния драйвера
    #define DRV_M1         14   //Регистр состояния драйвера
    #define DRV_M2         15U   //Регистр состояния драйвера
#define STATE_IN       4
#define CURRENT_POSITION       5

//ModBUS_rw_reg

/*#define PWM_DATA_1_2        0
#define PWM_DATA_3_4        1
#define PWM_DATA_5_6        2
#define PWM_DATA_7_8        3
#define PWM_DATA_9_10       4
*/
#define PWM_DATA            0
//#define PINS                1
#define FLAGS               1
        #define CTRL_SOURS      0
#define PWM_CALC            2
#define DRV_CTRL            3   //Регистр управления драйвером
    #define CMD_GOOD            0
    #define MAX_STEP            511
    #define MAX_DEGREE          180
//    #define STEP_DEGREE         18
    #define CMD_EN_ON           6401
    #define CMD_EN_OFF          6402
    #define CMD_DIR_FRW         6403
    #define CMD_DIR_REV         6404
    #define CMD_DIV_STEP_FULL   6405
    #define CMD_DIV_STEP_2      6406
    #define CMD_DIV_STEP_4      6407
    #define CMD_DIV_STEP_8      6408
    #define CMD_DIV_STEP_16     6409
    #define CMD_DIV_STEP_32     6410
    #define CMD_RST_ON          6411
    #define CMD_RST_OFF         6412
    #define CMD_AUTO_HOME       6413
    #define CMD_BAD             65535

//  0     - команда выполнена успещно, готовность к выполнению следующей команды
//  65535 - команда не выполнена, готовность к выполнению следующей команды
#define SIZE_STEP        4
#define MAX_POSITION     5


struct D_pc_str
{
	unsigned char Num_Byte_RW_To_PC;
	unsigned char Flags_Link_To_PC;
	unsigned char mr[UARTBUF];
	unsigned char mw[UARTBUF];
	unsigned char Counter_Byte_To_PC;
	unsigned int ModBUS_rw_reg[NUMBER_OF_RW_REGS]; 
	unsigned char ModBUS_rwbit[NUMBER_OF_RW_BIT_VARIABLES >> 3]; //байтовый массив для обмена битовыми
	//переменными rw по ModBUS
	unsigned int ModBUS_r_reg[NUMBER_OF_R_REGS];
	unsigned char ModBUS_rw_reg_write_flags[(NUMBER_OF_RW_REGS + 7 >> 3)+(NUMBER_OF_RW_BIT_VARIABLES >> 3)];//byte массив битовых флагов прихода
//данных по ModBUS для rw регистров
} ;

extern struct D_pc_str D_pc;

unsigned int GetCRC16(unsigned char *buf, unsigned char bufsize);
void Link_To_PC(void);
#endif 
