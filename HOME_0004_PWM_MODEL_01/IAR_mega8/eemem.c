//#include "ioavr.h"
#include "eemem.h"
//nclude <iotiny2313.inc>


unsigned char ee_buf_adr[EE_BUF_NUM];
unsigned char ee_buf_data[EE_BUF_NUM];

unsigned int eeb_h=0, eeb_t=0;

void Write_Transmit_Change_Data(unsigned char *data, unsigned char size_data, unsigned int Adr_EE)
{
	unsigned char k=0;

	for(k=0; k<size_data; k++)
	{
	  eeprom_write_byte(Adr_EE, data[k]);	
          Adr_EE++;
	}
}
//************************************************************
void write_buffers_byte_to_EE()
{
  unsigned char EE_ADR;
  unsigned char EE_DATA;
  
  if(eeb_h==eeb_t || EECR & 1<<EEWE) return;
  
  EE_ADR=ee_buf_adr[eeb_t];
  EE_DATA=ee_buf_data[eeb_t];
  
  __disable_interrupt(); //cli();
  if(!(EECR & 1<<EEWE))
  {
    //Начать запись в EEPROM
    EEAR = EE_ADR;				//Запись адреса
    EEDR = EE_DATA;				//Запись данных
    EECR = 1<<EEMWE;			//Установить флаг "EEMWE"
    EECR = 1<<EEWE;
    eeb_t++;
  }  
  __enable_interrupt(); //sei();
  
  if(eeb_t>=EE_BUF_NUM) eeb_t=0;
}
//////////////////////////////////////////////

//****************************************************************
//		ЗАПИСЬ ДАННЫХ В EEPROM
//****************************************************************
void eeprom_write_byte(unsigned int EE_ADR, unsigned char EE_DATA)
{
  ee_buf_adr[eeb_h]=EE_ADR;
  ee_buf_data[eeb_h++]=EE_DATA;
          
  if(eeb_h>=EE_BUF_NUM) eeb_h=0; 
}

//****************************************************************
//		ЧТЕНИЕ ДАННЫХ ИЗ EEPROM
//****************************************************************
unsigned char eeprom_read_byte(unsigned int EE_ADR)
{
  while(EECR & 1<<EEWE);	        //Ждать завершения предыдущей записи	
  __disable_interrupt(); //cli();
  while(EECR & 1<<EEWE);	        //Ждать завершения предыдущей записи
  EEAR = EE_ADR;			//Запись адреса
  EECR |= 1<<EERE;			//Начать чтение из EEPROM
  __enable_interrupt(); //sei();

  return EEDR;
}

