#ifndef _EEMEM_H_
#define _EEMEM_H_

//#include <iotiny2313.h>
#include "ioavr.h"
#include <intrinsics.h>

#define EE_BUF_NUM 5

extern void write_buffers_byte_to_EE(void); 
extern void Write_Transmit_Change_Data(unsigned char *data, unsigned char size_data, unsigned int Adr_EE);
extern void eeprom_write_byte(unsigned int EE_ADR, unsigned char EE_DATA);
extern unsigned char eeprom_read_byte(unsigned int EE_ADR);
#endif
