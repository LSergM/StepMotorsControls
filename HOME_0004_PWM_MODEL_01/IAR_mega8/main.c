#include "main.h"
#include "delay.h"
#include <ioavr.h>
#include <stdint.h>
#include <intrinsics.h>
#include "eemem.h"
#include "modbus.h"
#include "un_drebezg.h"

unsigned char fl=0,ch=0, fadc=0, tch=0;
unsigned int nn=0, n1=0, tadc=0;
uint16_t steps_drv = 0;
uint16_t position = 0;

const uint16_t DELAY_OF_START = 100;
const uint8_t  IS_NOT_HOME_POSITION  = 0;
const uint8_t  IS_HOME_POSITION      = 1;

  #define ms1     0
  #define ech     1

uint16_t MedianFilter(uint16_t datum, uint8_t nch);
void update_drv(void);
void UpdateFaultState(void);
uint8_t CheckHomePosition(void);

void main( void )
{
  uint8_t din = 0;
  uint8_t pwm = 0;
  uint8_t sp = 0;
  uint8_t delstart = 0;  
  
  InitCPU();

  __enable_interrupt();
  while(1)
  {
//    __watchdog_reset();
    
    __disable_interrupt();
    if(fadc)
    {      
      uint16_t ttadc = tadc;
      uint8_t  ttch  = tch;
      fadc = 0;
      D_pc.ModBUS_r_reg[ADC_0 + ttch] = MedianFilter(ttadc, ttch);
    }
     __enable_interrupt();
    
    if(fl & (1 << ms1)) 
    {      
      fl &= ~(1 << ms1);
      
      if (delstart < DELAY_OF_START) 
      {
        delstart++;
      }
                  
      Test_Drebezg_IN();
      
      if (steps_drv && (delstart >= DELAY_OF_START))
      {                
        if(IS_NOT_HOME_POSITION == CheckHomePosition())
        {      
          if(steps_drv) 
          {
            steps_drv--;
          }
          
          if(sp)
          {
            SET_STEP_PORT_ON;
            sp = 0;
          }
          else 
          {
            SET_STEP_PORT_OFF;
            sp = 1;
          }
          
          if (0xFFFF != position)
          {
            if( D_pc.ModBUS_r_reg[DRV_STATE] & (1 << DRV_DIR) )
            {
              if (position >= D_pc.ModBUS_rw_reg[SIZE_STEP])
              {
                position = position - D_pc.ModBUS_rw_reg[SIZE_STEP];
              }
//                    else 
//                      while(1);
            }
            else
            {
              position = position + D_pc.ModBUS_rw_reg[SIZE_STEP];
            }
          }                
        }
      }            
    }
    
    D_pc.ModBUS_r_reg[CURRENT_POSITION] = position;
    D_pc.ModBUS_r_reg[STATE_IN] = Test_IN_0.fin;
    
    if(nn >= 71)
    {      
      nn = 0;
      D_pc.ModBUS_r_reg[CNT_0] = n1 + TCNT0;
      TCNT0 = 0;
      n1 = 0;
    }
    
    if (    D_pc.ModBUS_rw_reg[DRV_CTRL] != CMD_GOOD
        &&  D_pc.ModBUS_rw_reg[DRV_CTRL] != CMD_BAD)
    {
      update_drv();
    }
    
    UpdateFaultState(); 
        
    Link_To_PC();
    write_buffers_byte_to_EE();
    
    din = ~Test_IN_0.fin & 0x0F;    

    if(D_pc.ModBUS_rw_reg[FLAGS] & (1 << CTRL_SOURS))  
    {
      pwm = DELTA * din;//*((unsigned char*)&D_pc.ModBUS_rw_reg+data);
    }
    else  
    {
      pwm = D_pc.ModBUS_rw_reg[PWM_DATA];
    }
    
    if(pwm > ANGLE_MAX) 
    {
      pwm = 0;
    }
    
    D_pc.ModBUS_rw_reg[PWM_CALC] = pwm;
    
    OCR1A = MAX_PWM - pwm*CAL;     
  }  
}

uint8_t CheckHomePosition(void)
{
  if (    !(Test_IN_0.fin & (1 << END_STOP)) 
      &&   (D_pc.ModBUS_r_reg[DRV_STATE] & (1 << DRV_DIR)) 
  )          
  {
    steps_drv = 0;
    position  = 0;
    
    return IS_HOME_POSITION;
  }
  
  return IS_NOT_HOME_POSITION;
}

void UpdateFaultState(void)
{
    if (GET_DRV_FAULT)
    {
          D_pc.ModBUS_r_reg[DRV_STATE] |= (1 << DRV_FAULT);
    }
    else  
    {
          D_pc.ModBUS_r_reg[DRV_STATE] &= ~(1 << DRV_FAULT);
    }
}

void update_drv(void)
{
    uint8_t fwree = 1;
  
    D_pc.ModBUS_rw_reg[SIZE_STEP] = D_pc.ModBUS_rw_reg[SIZE_STEP] & 0x00FF;
    
    if (D_pc.ModBUS_rw_reg[DRV_CTRL])
    {
      if (D_pc.ModBUS_rw_reg[DRV_CTRL] <= MAX_STEP)
      {
        if (D_pc.ModBUS_rw_reg[DRV_CTRL] > D_pc.ModBUS_rw_reg[MAX_POSITION])
        {
          D_pc.ModBUS_rw_reg[DRV_CTRL] = D_pc.ModBUS_rw_reg[MAX_POSITION];
        }
        
        if (position != D_pc.ModBUS_rw_reg[DRV_CTRL])
        {
          if(D_pc.ModBUS_rw_reg[SIZE_STEP])
          {
             steps_drv = D_pc.ModBUS_rw_reg[DRV_CTRL] * 10;          
             if (position > steps_drv)
             {
               steps_drv = (position - steps_drv)/D_pc.ModBUS_rw_reg[SIZE_STEP];
               D_pc.ModBUS_r_reg[DRV_STATE] |= (1 << DRV_DIR);
             }
             else
             {
               steps_drv = (steps_drv - position)/D_pc.ModBUS_rw_reg[SIZE_STEP];
               D_pc.ModBUS_r_reg[DRV_STATE] &= ~(1 << DRV_DIR);
             }
          }
        }
        
        D_pc.ModBUS_r_reg[DRV_STATE] &= ~0x01FF;
          D_pc.ModBUS_r_reg[DRV_STATE] |= D_pc.ModBUS_rw_reg[DRV_CTRL];
      }
      else
      {
        if (steps_drv)
        {
          D_pc.ModBUS_rw_reg[DRV_CTRL] = CMD_BAD;
           fwree = 0;
        }
        else  
        {
          fwree = 1;
            
          switch(D_pc.ModBUS_rw_reg[DRV_CTRL])
          {
          case CMD_EN_ON: D_pc.ModBUS_r_reg[DRV_STATE] |= (1 << DRV_EN);                        
            break;
          case CMD_EN_OFF: D_pc.ModBUS_r_reg[DRV_STATE] &= ~(1 << DRV_EN);                        
            break;               
          case CMD_DIR_FRW: D_pc.ModBUS_r_reg[DRV_STATE] |= (1 << DRV_DIR);                        
            break;
          case CMD_DIR_REV: D_pc.ModBUS_r_reg[DRV_STATE] &= ~(1 << DRV_DIR);                        
            break;  
            
          case CMD_DIV_STEP_FULL: 
                D_pc.ModBUS_r_reg[DRV_STATE] &= ~(1 << DRV_M0 | 1 << DRV_M1 | 1 << DRV_M2);                        
            break;
          case CMD_DIV_STEP_2: 
                D_pc.ModBUS_r_reg[DRV_STATE] &= ~(1 << DRV_M0 | 1 << DRV_M1 | 1 << DRV_M2);
                  D_pc.ModBUS_r_reg[DRV_STATE] |= (1 << DRV_M0);
            break;           
          case CMD_DIV_STEP_4: 
                D_pc.ModBUS_r_reg[DRV_STATE] &= ~(1 << DRV_M0 | 1 << DRV_M1 | 1 << DRV_M2);
                  D_pc.ModBUS_r_reg[DRV_STATE] |= (1 << DRV_M1);
            break;           
          case CMD_DIV_STEP_8: 
                D_pc.ModBUS_r_reg[DRV_STATE] &= ~(1 << DRV_M0 | 1 << DRV_M1 | 1 << DRV_M2);
                  D_pc.ModBUS_r_reg[DRV_STATE] |= (1 << DRV_M0 | 1 << DRV_M1);
            break;           
          case CMD_DIV_STEP_16: 
                D_pc.ModBUS_r_reg[DRV_STATE] &= ~(1 << DRV_M0 | 1 << DRV_M1 | 1 << DRV_M2);
                  D_pc.ModBUS_r_reg[DRV_STATE] |= (uint16_t)(1 << DRV_M2);
            break;           
          case CMD_DIV_STEP_32: 
                D_pc.ModBUS_r_reg[DRV_STATE] &= ~(1 << DRV_M0 | 1 << DRV_M1 | 1 << DRV_M2);
                  D_pc.ModBUS_r_reg[DRV_STATE] |= (1 << DRV_M0 | 1 << DRV_M2);
            break;     
          case CMD_RST_ON: D_pc.ModBUS_r_reg[DRV_STATE] |= (1 << DRV_RST);                        
            break;
          case CMD_RST_OFF: D_pc.ModBUS_r_reg[DRV_STATE] &= ~(1 << DRV_RST);                        
            break;   
          case CMD_AUTO_HOME: 
                  D_pc.ModBUS_r_reg[DRV_STATE] |= (1 << DRV_DIR);  
                   steps_drv = 511;
            break;                                           
            default:
                D_pc.ModBUS_rw_reg[DRV_CTRL] = CMD_BAD;
                fwree = 0;
              break;
          }
          if (fwree)
          {
            eeprom_write_byte(NUMBER_OF_RW_REGS * 2,      D_pc.ModBUS_r_reg[DRV_STATE] >> 8);					
             eeprom_write_byte(NUMBER_OF_RW_REGS * 2 + 1, D_pc.ModBUS_r_reg[DRV_STATE]);
          }
        }
      }
      if (D_pc.ModBUS_rw_reg[DRV_CTRL] != CMD_BAD)
      {
        D_pc.ModBUS_rw_reg[DRV_CTRL] = CMD_GOOD;
      }
    }
    
    if (D_pc.ModBUS_r_reg[DRV_STATE] & (1 << DRV_DIR))
    {
          SET_DRV_DIR_FRW;
    }
    else  
    {
          SET_DRV_DIR_REV;
    }
    
    if (D_pc.ModBUS_r_reg[DRV_STATE] & (1 << DRV_EN))
    {
          SET_DRV_EN_ON;
    }
    else  
    {
          SET_DRV_EN_OFF;
    }
    
    if (D_pc.ModBUS_r_reg[DRV_STATE] & (1 << DRV_M0))
    {
          SET_DRV_M0_ON;
    }
    else  
    {
          SET_DRV_M0_OFF;
    }
    
    if (D_pc.ModBUS_r_reg[DRV_STATE] & (1 << DRV_M1))
    {
          SET_DRV_M1_ON;
    }
    else  
    {
          SET_DRV_M1_OFF;
    }   

    if (D_pc.ModBUS_r_reg[DRV_STATE] & (uint16_t)(1 << DRV_M2))
    {
          SET_DRV_M2_ON;
    }
    else  
    {
          SET_DRV_M2_OFF;
    }
   
    if (D_pc.ModBUS_r_reg[DRV_STATE] & (1 << DRV_RST))
    {
          SET_DRV_RST_ON;
    }
    else  
    {
          SET_DRV_RST_OFF;
    }    
}

void InitCPU(void)
{
    //DDRA|=0x03;
    DDRB   =  (1 << PORTB0 | 1 << PORTB1 | 1 << PORTB2 | 1 << PORTB3 | 0 << PORTB4 | 1 << PORTB5 | 1 << PORTB6 | 1 << PORTB7);
    PORTB =  (1 << PORTB0 | 1 << PORTB1 | 0 << PORTB2 | 0 << PORTB3 | 1 << PORTB4 | 0 << PORTB5 | 0 << PORTB6 | 0 << PORTB7);
    
    DDRD   =  (0 << PORTD0 | 1 << PORTD1 | 1 << PORTD2 | 0 << PORTD3 | 0 << PORTD4 | 0 << PORTD5 | 0 << PORTD6 | 0 << PORTD7);
    PORTD = 0x71;
    
    DDRC   =  (0 << PORTC0 | 0 << PORTC1 | 1 << PORTC2 | 1 << PORTC3 | 1 << PORTC4 | 1 << PORTC5 | 0 << PORTC6);
    PORTC  =  (0 << PORTC0 | 0 << PORTC1 | 1 << PORTC2 | 1 << PORTC3 | 1 << PORTC4 | 1 << PORTC5 | 1 << PORTC6);

    
    TCCR1A = (1<<COM1A1|1<<COM1A0|1<<COM1B1|0<<COM1B0|1<<WGM11|0<<WGM10); //Режимы ШИМ -14 Fast PWM
    TCCR1B = (0<<ICNC1|0<<ICES1|1<<WGM13|1<<WGM12|0<<CS12|0<<CS11|0<<CS10); //Таймер - 0 "Остановлен"
    ICR1 = ICR1_DEF;   //При 1МГц тактировании, период 14мс
    OCR1A = 0;
    OCR1B = 73;
    TIMSK|=1<<OCIE1B|1<<TOIE0|1<<TOIE2;//
    
    TCCR0=1<<CS02|1<<CS01|0<<CS00;
  
    UCSRA=(1<<U2X|0<<MPCM);
    UCSRB=(1<<RXCIE|0<<TXCIE|0<<UDRIE|1<<RXEN|1<<TXEN|0<<UCSZ2|0<<RXB8|0<<TXB8);
    UCSRC=(         0<<UMSEL|0<<UPM1|0<<UPM0|0<<USBS|1<<UCSZ1|1<<UCSZ0|0<<UCPOL);
    UBRRH = 0;
    UBRRL = 51;//??? 19200 bit/s
          
    ADMUX=1<<REFS0;
    ADCSR=1<<ADIE|1<<ADPS2|1<<ADPS1|0<<ADPS0;
    
    unsigned char adr = 0;
    for(unsigned char i=0; i<NUMBER_OF_RW_REGS; i++)
    {
      D_pc.ModBUS_rw_reg[i]=eeprom_read_byte(adr)<<8|eeprom_read_byte(adr+1);      
      adr += 2;
    }
    
    D_pc.ModBUS_r_reg[DRV_STATE] = eeprom_read_byte(NUMBER_OF_RW_REGS * 2)<<8 | eeprom_read_byte(NUMBER_OF_RW_REGS * 2 + 1);  
    D_pc.ModBUS_r_reg[DRV_STATE] &= ~0x01FF;
    D_pc.ModBUS_r_reg[DRV_STATE] |= (1 << DRV_DIR); //Go to home
    D_pc.ModBUS_rw_reg[DRV_CTRL] = CMD_GOOD;  
    update_drv();
    steps_drv = 511;
    position = 0xFFFF;
    
    TCCR1B |= (1 << CS11);  //Старт ШИМ
    ADCSR  |= (1 << ADEN) | (1 << ADSC);    
}

#pragma vector = USART_TXC_vect
__interrupt void Usart1TxVect(void) 
{
	D_pc.Counter_Byte_To_PC++;
	if (D_pc.Counter_Byte_To_PC<D_pc.Num_Byte_RW_To_PC) 
                UDR=D_pc.mw[D_pc.Counter_Byte_To_PC];
	else 
            {   
                D_pc.Counter_Byte_To_PC = 0;
                DE_OFF;
		UCSRB &= ~(1<<TXEN|1<<TXCIE);
                RE_ON;
		UCSRB |= (1<<RXEN|1<<RXCIE);
		
                //fl|=1<<ech;
                //TCNT0 = CNT_SPEED; //237    //Обнулить таймер 0 1,25 мс
	        //TCCR0B = 0x05;//0B00000101; //Запустить таймер 0 c предделителем /1024	
	    }	
}

#pragma vector = USART_RXC_vect
__interrupt void Usart1RxVect(void) 
{
	TCCR2 = 0; //Остановить таймер 0
	
	if (D_pc.Counter_Byte_To_PC<UARTBUF)  
        {
          D_pc.mr[D_pc.Counter_Byte_To_PC]=UDR;
          D_pc.Counter_Byte_To_PC++;          
        }
        else UCSRB &= ~(1<<RXEN|1<<RXCIE);

	TCNT2 = CNT_SPEED; //237    //Обнулить таймер 0 1,25 мс
	TCCR2 = 0x07;//0B00000101; //Запустить таймер 0 c предделителем /1024	
}

#pragma vector = TIMER0_OVF_vect
__interrupt void Timer0OverFVect(void) 
{
  if(nn) n1=n1+255;  
}

#pragma vector = TIMER2_OVF_vect
__interrupt void Timer2OverFVect(void) 
{
	TCCR2 = 0;
/*        if(fl&(1<<ech))
        {
          fl&=~(1<<ech);
          	DE_OFF;
		D_pc.Counter_Byte_To_PC = 0;
		UCSRB |= (1<<RXEN|1<<RXCIE);
		RE_ON;
        }
        else
        {
*/        
          RE_OFF;
          UCSRB &= ~(1<<RXEN|1<<RXCIE); //	Отключить приемник UART ModBus																										//	Запретить прерывание приемника UARt
          D_pc.Flags_Link_To_PC |= (1<<RECIVED_POCET_FROM_PC);
//        }
}

#pragma vector = TIMER1_COMPB_vect
__interrupt void Timer1CompBVect(void) 
{  
  nn++;
  fl|=1<<ms1;
  //TCNT1=65535-2960;
}

#pragma vector = ADC_vect
__interrupt void ADCvect(void) 
{
  tch=ch; tadc=ADC;

  fadc=1;
  //D_pc.ModBUS_r_reg[ADC_0+ch]=ADC;
  ch++;
  ch&=1;
  ADMUX=(ADMUX&~0x0F)|ch;
  //asm("nop");
  ADCSR|=1<<ADSC;
}

#define NULL         0
#define STOPPER   0 /* Smaller than any datum */
#define MEDIAN_FILTER_SIZE 25
#define NCH_ADC       2
 
uint16_t MedianFilter(uint16_t datum, uint8_t nch)
{
  
 __enable_interrupt();
struct pair{
  struct pair *point; /* Pointers forming list linked in sorted order */
  uint16_t value; /* Values to sort */
};
  
/* Buffer of nwidth pairs */
 static struct pair buffer[NCH_ADC][MEDIAN_FILTER_SIZE] = {0};
/* Pointer into circular buffer of data */
 static struct pair *datpoint[NCH_ADC] = {
                                            &buffer[0][0],
                                            &buffer[1][0],
                                         };
/* Chain stopper */
 static struct pair small[NCH_ADC] = {
                                        {NULL, STOPPER},
                                        {NULL, STOPPER}
                                     };
/* Pointer to head (largest) of linked list.*/
 static struct pair big[NCH_ADC] = {
                            {&small[0], 0},
                            {&small[1], 0}
                           };
 
/* Pointer to successor of replaced data item */
 struct pair *successor[NCH_ADC];
/* Pointer used to scan down the sorted list */
 struct pair *scan[NCH_ADC];
/* Previous value of scan */
 struct pair *scanold[NCH_ADC];
/* Pointer to median */
 struct pair *median[NCH_ADC];
 uint16_t i;
 
if (datum == STOPPER){
   datum = STOPPER + 1; /* No stoppers allowed. */
}
 
 if ( (++datpoint[nch] - buffer[nch]) >= MEDIAN_FILTER_SIZE){
   datpoint[nch] = buffer[nch]; /* Increment and wrap data in pointer.*/
 }
 
 datpoint[nch]->value = datum; /* Copy in new datum */
 successor[nch] = datpoint[nch]->point; /* Save pointer to old value's successor */
 median[nch] = &big[nch]; /* Median initially to first in chain */
 scanold[nch] = NULL; /* Scanold initially null. */
 scan[nch] = &big[nch]; /* Points to pointer to first (largest) datum in chain */
 
 /* Handle chain-out of first item in chain as special case */
 if (scan[nch]->point == datpoint[nch]){
   scan[nch]->point = successor[nch];
 }
 
 scanold[nch] = scan[nch]; /* Save this pointer and */
 scan[nch] = scan[nch]->point ; /* step down chain */
 
 /* Loop through the chain, normal loop exit via break. */
 for (i = 0 ; i < MEDIAN_FILTER_SIZE; ++i){
  /* Handle odd-numbered item in chain */
  if (scan[nch]->point == datpoint[nch]){
    scan[nch]->point = successor[nch]; /* Chain out the old datum.*/
  }
 
  if (scan[nch]->value < datum){ /* If datum is larger than scanned value,*/
    datpoint[nch]->point = scanold[nch]->point; /* Chain it in here. */
    scanold[nch]->point = datpoint[nch]; /* Mark it chained in. */
    datum = STOPPER;
  };
 
  /* Step median pointer down chain after doing odd-numbered element */
  median[nch] = median[nch]->point; /* Step median pointer. */
  if (scan[nch] == &small[nch]){
    break; /* Break at end of chain */
  }
  scanold[nch] = scan[nch]; /* Save this pointer and */
  scan[nch] = scan[nch]->point; /* step down chain */
 
  /* Handle even-numbered item in chain. */
  if (scan[nch]->point == datpoint[nch]){
    scan[nch]->point = successor[nch];
  }
 
  if (scan[nch]->value < datum){
    datpoint[nch]->point = scanold[nch]->point;
    scanold[nch]->point = datpoint[nch];
    datum = STOPPER;
  }
 
  if (scan[nch] == &small[nch]){
    break;
  }
 
  scanold[nch] = scan[nch];
  scan[nch] = scan[nch]->point;
}
  
return median[nch]->value;
}