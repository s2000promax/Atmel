/*
 * Cooling_OWI_v2.c
 *
 * Created: 05.02.2017 20:13:11
 * Author : venger
 */ 

 #define F_CPU 16000000UL

 #define NEWLINESTR "\r\n"
 #define MAX_STRING_SIZE			192

#include <avr/io.h>
#include <stdio.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <util/delay.h>
#include <math.h>
#include <string.h>
#include <stdint.h>


#include "common_files\compilers.h"

#include "OWIPolled.h"
#include "OWIHighLevelFunctions.h"
#include "OWIBitFunctions.h"
#include "common_files\OWIcrc.h"

#include "u8g.h"

#include "pic_bmp_logo.h"
#include "pic_bmp_model.h"
#include "pic_bmp_logo_small.h"
#include "pic_bmp_Attention.h"
#include "pic_bmp_Error.h"
#include "pic_bmp_snowball1.h"
#include "pic_bmp_snowball2.h"
#include "pic_bmp_up.h"
#include "pic_bmp_down.h"




//код семейства и коды команд датчика DS18B20
#define DS18B20_FAMILY_ID                0x28
#define DS18B20_CONVERT_T                0x44
#define DS18B20_READ_SCRATCHPAD          0xbe
#define DS18B20_WRITE_SCRATCHPAD         0x4e
#define DS18B20_COPY_SCRATCHPAD          0x48
#define DS18B20_RECALL_E                 0xb8
#define DS18B20_READ_POWER_SUPPLY        0xb4

//вывод, к которому подключены 1Wire устройства
#define BUS   OWI_PIN_7

//прототипы функций
unsigned char DS18B20_ReadTemperature(unsigned char bus, unsigned char * id, unsigned int* temperature);
void DS18B20_PrintTemperature(unsigned int temperature);


//количество устройств на шине 1Wire
#define MAX_DEVICES       0x02

//коды ошибок для функции чтения температуры
#define READ_SUCCESSFUL   0x00
#define READ_CRC_ERROR    0x01

#define SEARCH_SENSORS 0x00
#define SENSORS_FOUND 0xff

OWI_device allDevices[MAX_DEVICES];
unsigned char rom[8];


#define KEY_NONE 0
#define KEY_MENU 1
#define KEY_BACK 2
#define KEY_PREV 3
#define KEY_LEFT 4
#define KEY_NEXT 5
#define KEY_RIGHT 6
#define KEY_HOME 7
#define KEY_SELECT 8

uint8_t sys_key_first = KEY_NONE;
uint8_t sys_key_second = KEY_NONE;
uint8_t sys_key_code = KEY_NONE;
uint8_t last_key_code = KEY_NONE;

char txt[80], str[80];


//Global variables
u8g_t u8g;

static volatile uint8_t adc_value = 0;
uint16_t rarely_count = 0;
uint8_t KeyNumber=0, MenuPosition=0, SubMenuPosition=0, SubMenuSelect=0, SubMenuPID=0, SubMenuAlarm=0, SubMenuAbout=0,
Sub_SubMenuPID = 0, Sub_SubmenuAlarm = 0;

//unsigned char keyC, keyCsame, bitE0,bitE1,bitE2,bitE3,bitE4,bitE5,bitE6,bitE7;
//unsigned char bitF0,bitF1,bitF2,bitF3,bitF4,bitF5,bitF6,bitF7;
//unsigned char noDevices=0, OK_Flag = 0;

uint8_t AlarmTemp1 = 1, AlarmTemp2 = 1, Sens1_fault = 0, Sens2_fault = 0 ;

unsigned int counter=0;
//unsigned int i=0;
// счетчики
uint16_t CountScale = 0, CountScale2 = 0, B1 = 0, B2 = 0, i_count = 0, i_count2 = 0;



float T1 = 0, T2 = 0;
//float T1_old =0, T2_old = 0;
float display_power1 = 0, display_power2 = 0, OCR1AL_double =0, OCR1BL_double = 0;

// PID
//struct PID {
// init
float	Kp;		// -100-100 percent/degree
float	Ti;		// 1-1000 sec
float	Td;		// 0-100 sec
float	Ts;		// sampling time
//float	Sp;		// setpoint
// process
float	Out1, Out2;	// 1-100 percent
// intrinsic
float	Ki;
float	Kd;
float	Uset1[2], Uset2[2];
float	Err1[3], Err2[3];
//};

//Уставки
float SetPointI; float eeSetPointI EEMEM;

float SetPointII; float eeSetPointII EEMEM;

float Set_Alarm1_min; float eeSet_Alarm1_min EEMEM;

float Set_Alarm1_max; float eeSet_Alarm1_max EEMEM;

float Set_Alarm2_min; float eeSet_Alarm2_min EEMEM;

float Set_Alarm2_max; float eeSet_Alarm2_max EEMEM;

char txt[80], str[80];


//unsigned char romcode1 = 0, romcode2 = 0, romcode3 = 0, romcode4 = 0, romcode5 = 0, romcode6 = 0, romcode7 = 0, romcode8 = 0, rc9=0, rc10=0;



uint8_t swap_value = 0; uint8_t eeSwap_value EEMEM;
uint8_t flag1 = 0, flag2 = 0, flagEE = 0;


//Переменные из Timer2

//static uint8_t i1;
uint8_t i;  // , j, crc8, data_crc, u, temp, temp_int, temp_float;
//uint8_t flag_0xCC = 1, flag_timout_0xCC = 0, flag_match = 0, flag_timeout_end = 0, count_timeout_0xCC = 0, count_timeout_end = 0;


float point_T1 = 0, point_T2 = 0;

//unsigned char Temp_H = 0,Temp_L = 0,temp_flag;

unsigned char tempint1 = 0,tempint2 = 0; // переменные для целого значения температуры

unsigned char temppoint1 = 0, temppoint2 = 0; // переменные для дробного значения температуры

	  unsigned int temperature = 0, temperature_uart = 0;
	  unsigned char searchFlag = SEARCH_SENSORS;
	  unsigned char crcFlag = 0;
	  unsigned char num = 0;


uint8_t GetKey()
{
	uint8_t key = 0;

	PORTE |= 0xE0;
	asm ("nop");
	
	if ((PINF & (1<<PF1))||(PINF & (1<<PF2))||(PINF & (1<<PF3)))
	{
		//PORTE &= 0x9F;
		PORTE &= 0x1F;
		PORTE |= 0x80;
		asm ("nop");
		//keyCsame = 0xEE;
		if (PINF & (1<<PF1)) {key = 7; PORTE &= 0x1F;}
		if (PINF & (1<<PF2)) {key = 8; PORTE &= 0x1F;}
		if (PINF & (1<<PF3)) {key = 9; PORTE &= 0x1F;}
		PORTE &= 0x7F;

		PORTE |= 0x40;
		asm ("nop");
		if (PINF & (1<<PF1)) {key = 4; PORTE &= 0x1F;}
		if (PINF & (1<<PF2)) {key = 5; PORTE &= 0x1F;}
		if (PINF & (1<<PF3)) {key = 6; PORTE &= 0x1F;}
		PORTE &= 0xBF;

		PORTE |= 0x20;
		asm ("nop");
		if (PINF & (1<<PF1)) {key = 1; PORTE &= 0x1F;}
		if (PINF & (1<<PF2)) {key = 2; PORTE &= 0x1F;}
		if (PINF & (1<<PF3)) {key = 3; PORTE &= 0x1F;}
		PORTE &= 0xDF;
		asm ("nop");

	} else PORTE &= 0x1F;

	return key;
}

uint8_t sys_get_key(void)
{
	uint8_t result = KEY_NONE;

	if (KeyNumber==0) result = KEY_NONE;
	if (KeyNumber==1) result = KEY_MENU;
	if (KeyNumber==2) result = KEY_BACK;
	if (KeyNumber==3) result = KEY_NEXT;
	if (KeyNumber==4) result = KEY_LEFT;
	if (KeyNumber==5) result = KEY_PREV;
	if (KeyNumber==6) result = KEY_RIGHT;
	if (KeyNumber==7) result = KEY_HOME;
	if (KeyNumber==8) result = KEY_SELECT;
	
	return result;
}

void sys_debounce_key(void)
{
	sys_key_second = sys_key_first;
	sys_key_first = sys_get_key();
	
	if ( sys_key_second == sys_key_first )
	sys_key_code = sys_key_first;
	else
	sys_key_code = KEY_NONE;
}

void u8g_setup(void)
{
	
	u8g_Init8Bit(&u8g,&u8g_dev_ks0108_128x64,PN(2, 0), PN(2, 1), PN(2, 2), PN(2, 3), PN(2, 4), PN(2, 5), PN(2, 6), PN(2, 7),
	PN(0, 6), PN(0, 3), PN(0, 4), PN(0, 7), PN(6, 2), U8G_PIN_NONE);
	
	
	/* flip screen, if required */
	//u8g_SetRot180(&u8g);

	/* assign default color value */
	
	if ( u8g_GetMode(&u8g) == U8G_MODE_R3G3B2 )
	u8g_SetColorIndex(&u8g, 255);     /* white */
	else if ( u8g_GetMode(&u8g) == U8G_MODE_GRAY2BIT )
	u8g_SetColorIndex(&u8g, 3);         /* max intensity */
	else if ( u8g_GetMode(&u8g) == U8G_MODE_BW )
	u8g_SetColorIndex(&u8g, 1);         /* pixel on */

}


ISR(TIMER0_OVF_vect)
{
	        KeyNumber = GetKey();
	        sys_debounce_key();

			if (sys_key_code==KEY_MENU) {MenuPosition = 1; SubMenuPosition = 0; SubMenuPID = 0; SubMenuAlarm = 0; }
			if ((sys_key_code==KEY_HOME)||(sys_key_code==KEY_BACK)&&(MenuPosition == 1)) { MenuPosition = 0; SubMenuPosition = 0; SubMenuPID = 0; SubMenuAlarm = 0; }
			
			
			switch ( MenuPosition ) {
				case 0:// Отрисовка главного экрана
				  u8g_FirstPage(&u8g);
				  do
				  {
					  draw_mainscreen();
				  } while ( u8g_NextPage(&u8g) );
				break;
				
				case 1://Отрисовка главного меню
				  u8g_FirstPage(&u8g);
				  do
				  {
					  draw_MainMenuScreen();
				  } while ( u8g_NextPage(&u8g) );

				  if (sys_key_code==KEY_NEXT) { if ((SubMenuPosition < 2)&&(SubMenuPosition >= 0)) SubMenuPosition++; }
				  if (sys_key_code==KEY_PREV) { if ((SubMenuPosition <= 2)&&(SubMenuPosition > 0)) SubMenuPosition--; }
				  if (sys_key_code==KEY_SELECT) {MenuPosition=MenuPosition+SubMenuPosition+1;}
				break;
				

				case 2://Отрисовка Меню настроек ПИД
				u8g_FirstPage(&u8g);
				do
				{
					draw_SubMenuPID();
				} while ( u8g_NextPage(&u8g) );

				if (sys_key_code==KEY_BACK) { MenuPosition=1; SubMenuPID = 0; }
				if (sys_key_code==KEY_NEXT) { if ((SubMenuPID < 2)&&(SubMenuPID >= 0)) SubMenuPID++; }
				if (sys_key_code==KEY_PREV) { if ((SubMenuPID <= 2)&&(SubMenuPID > 0)) SubMenuPID--; }
				if (sys_key_code==KEY_SELECT) {MenuPosition=MenuPosition+SubMenuPID+3;}
				break;

				case 3://Отрисовка Меню настроек Alarm-ов
				u8g_FirstPage(&u8g);
				do
				{
					draw_SubMenuAlarm();
				} while ( u8g_NextPage(&u8g) );

				if (sys_key_code==KEY_BACK) { MenuPosition = 1; SubMenuAlarm = 0; }
				if (sys_key_code==KEY_NEXT) { if ((SubMenuAlarm < 1)&&(SubMenuAlarm >= 0)) SubMenuAlarm++; }
				if (sys_key_code==KEY_PREV) { if ((SubMenuAlarm <= 1)&&(SubMenuAlarm > 0)) SubMenuAlarm--; }
				if (sys_key_code==KEY_SELECT) {MenuPosition=MenuPosition+SubMenuAlarm+5;}
				break;

				case 4: //Отрисовка экрана About
		
				u8g_FirstPage(&u8g);
				do
				{
					draw_SubMenuAbout();
				} while ( u8g_NextPage(&u8g) );

				if (sys_key_code==KEY_BACK) { MenuPosition = 1; }
				if (sys_key_code==KEY_HOME) { MenuPosition = 0; SubMenuPosition = 0; }
				break;

				case 5: //Отрисовка экрана setpoint I
				
				u8g_FirstPage(&u8g);
				do
				{
					draw_SetpointI();
				} while ( u8g_NextPage(&u8g) );

				if (sys_key_code==KEY_BACK) {   MenuPosition = 2;}
				if (sys_key_code==KEY_HOME) {  while (EEWE != 0); MenuPosition = 0; SubMenuPosition = 0; Sub_SubMenuPID = 0; }
				if (sys_key_code==KEY_PREV) {SetPointI = SetPointI + 0.1;  flagEE = 1;}
				if (sys_key_code==KEY_NEXT) {SetPointI = SetPointI - 0.1;  flagEE = 1;}
				break;

				case 6: //Отрисовка экрана setpoint II
				
				u8g_FirstPage(&u8g);
				do
				{
					draw_SetpointII();
				} while ( u8g_NextPage(&u8g) );

				if (sys_key_code==KEY_BACK) { MenuPosition = 2; }
				if (sys_key_code==KEY_HOME) { MenuPosition = 0; SubMenuPosition = 0; Sub_SubMenuPID = 0; }
				if (sys_key_code==KEY_PREV) {SetPointII = SetPointII + 0.1; flagEE = 1;}
				if (sys_key_code==KEY_NEXT) {SetPointII = SetPointII - 0.1; flagEE = 1;}
				break;

				case 7: //Отрисовка экрана swap sensors
				
				u8g_FirstPage(&u8g);
				do
				{
					draw_SwapSensors();
				} while ( u8g_NextPage(&u8g) );

				if (sys_key_code==KEY_BACK) { MenuPosition = 2; }
				if (sys_key_code==KEY_HOME) { MenuPosition = 0; SubMenuPosition = 0; Sub_SubMenuPID = 0; }
				if (sys_key_code==KEY_PREV) {swap_value = 1; flagEE = 1;}
				if (sys_key_code==KEY_NEXT) {swap_value = 0; flagEE = 1;}
				break;

				case 8: //Отрисовка экрана Stage I min
				
				u8g_FirstPage(&u8g);
				do
				{
					draw_Alarm1min();
				} while ( u8g_NextPage(&u8g) );

				if (sys_key_code==KEY_BACK) { MenuPosition = 3; }
				if (sys_key_code==KEY_HOME) { MenuPosition = 0; SubMenuPosition = 0; Sub_SubmenuAlarm = 0; }
				if (sys_key_code==KEY_PREV) {Set_Alarm1_min = Set_Alarm1_min + 0.1; flagEE = 1;}
				if (sys_key_code==KEY_NEXT) {Set_Alarm1_min = Set_Alarm1_min - 0.1; flagEE = 1;}
				break;

				case 9: //Отрисовка экрана Stage I max
				
				u8g_FirstPage(&u8g);
				do
				{
					draw_Alarm1max();
				} while ( u8g_NextPage(&u8g) );

				if (sys_key_code==KEY_BACK) { MenuPosition = 3; }
				if (sys_key_code==KEY_HOME) { MenuPosition = 0; SubMenuPosition = 0; Sub_SubmenuAlarm = 0; }
				if (sys_key_code==KEY_PREV) {Set_Alarm1_max = Set_Alarm1_max + 0.1; flagEE = 1;}
				if (sys_key_code==KEY_NEXT) {Set_Alarm1_max = Set_Alarm1_max - 0.1; flagEE = 1;}
				break;
				
				case 10: //Отрисовка экрана Stage II min
				
				u8g_FirstPage(&u8g);
				do
				{
					draw_Alarm2min();
				} while ( u8g_NextPage(&u8g) );

				if (sys_key_code==KEY_BACK) { MenuPosition = 3; }
				if (sys_key_code==KEY_HOME) { MenuPosition = 0; SubMenuPosition = 0; Sub_SubmenuAlarm = 0; }
				break;

				case 11: //Отрисовка экрана Stage II max
				
				u8g_FirstPage(&u8g);
				do
				{
					draw_Alarm2max();
				} while ( u8g_NextPage(&u8g) );

				if (sys_key_code==KEY_BACK) { MenuPosition = 3; }
				if (sys_key_code==KEY_HOME) { MenuPosition = 0; SubMenuPosition = 0; Sub_SubmenuAlarm = 0; }
				break;
					
						
				default:
				//Код, который выполнится, если ни одно из константых значений не соответствует значению в переменной variable
				break;

				
			}
			
			/*
				 if (rarely_count==2)
				 {
					 
					 counter++;
					 if(counter>65)  counter=0;
					 rarely_count = 0;
				 }
				 
				 rarely_count++;
*/
				 CountScale++;
				 if (CountScale > 100) CountScale = 0;
				 
				 if (CountScale%4==0) if (B1==0) B1 = 1; else B1 = 0;

				 CountScale2++;
				 if (CountScale > 99) CountScale = 0;

				 if (CountScale2%6==0) if (B2==0) B2 = 1; else B2 = 0;

				 


if ((T1>=Set_Alarm1_min)&&(T1<=Set_Alarm1_max)) {AlarmTemp1 = 0; } else {AlarmTemp1 = 1;  }
if ((T2>=Set_Alarm1_min)&&(T2<=Set_Alarm1_max)) {AlarmTemp2 = 0; } else {AlarmTemp2 = 1;  }


if ((AlarmTemp1)||(AlarmTemp2)) {if (B1) {PORTA &=~0x04;} else {PORTA |=0x04;}} else PORTA |=0x04; //Желтый светодиод
if ((Sens1_fault)||(Sens2_fault)) PORTA &= ~0x01; else PORTA |= 0x01;
if (Sens1_fault) T1 = 0;
if (Sens2_fault) T2 = 0;
		
	if ((AlarmTemp1 == 0)&&(AlarmTemp2 == 0)&&(Sens1_fault == 0)&&(Sens2_fault == 0))
		{
		PORTD |= 0x60;
		}
		else
		{
		PORTD &= ~0x60;
		}
}

ISR(TIMER2_OVF_vect)
{
static uint16_t temp_cnt = 0;
float znak = 0;
uint8_t pos = 0;

if (temp_cnt==3)
{
   if (swap_value == 0) pos = 0; else pos = 1;
    
    crcFlag = DS18B20_ReadTemperature(BUS, allDevices[pos].id, &temperature);
    if (crcFlag != READ_CRC_ERROR){ 
	temperature_uart = temperature;
      sprintf(txt,"T1=",1);
	  uprintf(txt);
      DS18B20_PrintTemperature(temperature_uart);
	  if ((temperature & 0x8000) == 0){
		
		  znak = 1;
	  }
	  else{
		  temperature = ~temperature + 1;
		  znak = -1;
	  }
	  
	  //выводим значение целое знач. температуры
	  tempint1 = (unsigned char)(temperature>>4); 
	  //выводим дробную часть знач. температуры
	  temppoint1 = (unsigned char)(temperature&15);
	  temppoint1 = (temppoint1>>1) + (temppoint1>>3);
	
	 point_T1 = (float)(temppoint1);
	 point_T1 = point_T1 / 10;
	 T1 = (float)tempint1;
	 T1 = T1 + point_T1;
     T1 = T1 * znak;	

	 Sens1_fault = 0;
	  	  
    }
    else{
      
      searchFlag = SEARCH_SENSORS;
	  Sens1_fault = 1;
    }
    
	pos++; if (pos>1) pos=0;
        
    crcFlag = DS18B20_ReadTemperature(BUS, allDevices[pos].id, &temperature);
    if (crcFlag != READ_CRC_ERROR){ 
	temperature_uart = temperature;
	  	sprintf(txt,"T2=",2);
	  	uprintf(txt);
		 DS18B20_PrintTemperature(temperature_uart);
		 	  if ((temperature & 0x8000) == 0){
			 	  
			 	  znak = 1;
		 	  }
		 	  else{
			 	  temperature = ~temperature + 1;
			 	  znak = -1;
		 	  }
		 	  
		 	  //выводим значение целое знач. температуры
		 	  tempint2 = (unsigned char)(temperature>>4);
		 	  //выводим дробную часть знач. температуры
		 	  temppoint2 = (unsigned char)(temperature&15);
		 	  temppoint2 = (temppoint2>>1) + (temppoint2>>3);
		 	  
		 	 point_T2 = (float)(temppoint2);
		 	 point_T2 = point_T2 / 10;
		 	 T2 = (float)tempint2;
		 	 T2 = T2 + point_T2;
		 	 T2 = T2 * znak;

			 Sens2_fault = 0;
    }
    else{
      
      searchFlag = SEARCH_SENSORS;
	  Sens2_fault = 1;
    }

	temp_cnt = 0;
}	
temp_cnt++;

}

ISR(TIMER1_OVF_vect)
{



static uint16_t i_w = 0;



if (i_w < 2) {
	i_w++;
	} else {
	flag1 = 1;
	i_w = 0;
	//PORTA &= ~0x01;
     }

counter = i_w;

 if (flag1 == 1)
	                 {
					
					Uset1[1] = Uset1[0];
					Err1[2] = Err1[1];
					Err1[1] = Err1[0];
					Err1[0] = SetPointI - T1;
					
					// Calc Uset
					Uset1[0] = Uset1[1] + Kp*((Err1[0]-Err1[1]) +
					Ki*Err1[0] + Kd*(Err1[0]-2*Err1[1]+Err1[2]));
					
					Out1 = Uset1[0];
					if (Out1 > 100) Out1 = 100;
					if (Out1 < 0) Out1 = 0;

					OCR1AL = (uint8_t)(Out1*2.55);


					Uset2[1] = Uset2[0];
					Err2[2] = Err2[1];
					Err2[1] = Err2[0];
					Err2[0] = SetPointII - T2;
						
						// Calc Uset
					Uset2[0] = Uset2[1] + Kp*((Err2[0]-Err2[1]) +
					Ki*Err2[0] + Kd*(Err2[0]-2*Err2[1]+Err2[2]));
						
					Out2 = Uset2[0];
					if (Out2 > 100) Out2 = 100;
					if (Out2 < 0) Out2 = 0;
					
					OCR1BL = (uint8_t)(Out2*2.55);
					 
	             	 flag1 = 0;			
		
	                 }
}


ISR(TIMER3_OVF_vect)
{		
		//TCNT0 = 0x83;

		

	

	

//OCR0 = i;	
}



/*
ISR(ADC_vect)
{
    adc_value = ADCH;
    ADCSRA |= _BV(ADSC);
	OCR0 = adc_value;
}
*/


/*******************************************************************************
 * Send data buffer
 *******************************************************************************/
void UARTWrite(uint8_t const* data, uint8_t len)
{
	while (len--) {

		while(!(UCSR1A&_BV(UDRE1))) ;
		UCSR1A |= _BV(TXC1);
		UDR1 = *data++;
	}
	
	while(!(UCSR1A&_BV(TXC1))) ;	
}

/*******************************************************************************
 * UART print
 *******************************************************************************/
void uprintf(const char *pFormat, ...)
{
	char pStr[MAX_STRING_SIZE];
	char pWarn[] = "WARNING: Increase MAX_STRING_SIZE\r\n";
	uint32_t len = 0;
	va_list ap;

	va_start(ap, pFormat);
	len = vsnprintf(pStr, sizeof(pStr), pFormat, ap);
	va_end(ap);

	UARTWrite((uint8_t*)pStr, strlen(pStr));
	if (len == sizeof(pStr)) {
		UARTWrite((uint8_t*)pWarn, strlen(pWarn));
	}
}

ISR(USART_RXC_vect)
{
	static uint8_t rx_framebuff[8];
	rx_framebuff[0] = UDR1;
	uprintf("%c", rx_framebuff);
}


void draw_mainscreen(void)
{
   	char buffer0[20];
	char buffer1[20];
	char buffer2[20];
	char buffer3[20];
	char buffer4[20];
	char buffer5[20];
	char buffer6[20];
		
	u8g_DrawFrame(&u8g, 0,0,128,64);
	u8g_SetFont(&u8g, u8g_font_6x10);
	u8g_DrawLine(&u8g,64,0,64,64);
	u8g_DrawLine(&u8g,0,10,128,10);	
	
	u8g_DrawStr(&u8g, 12, 9, "STAGE I");
	u8g_DrawStr(&u8g, 72, 9, "STAGE II");
	
	sprintf(buffer0,"%4.1f", SetPointI); // Уставка I ступени
	sprintf(buffer1,"%4.1f", SetPointII); // Уставка II ступени
	//if (T1<0) sprintf(buffer2,"%4.1f", T1); else sprintf(buffer2,"+%4.1f", T1); // Температура I ступени
	//if (T2<0) sprintf(buffer3,"%4.1f", T2); else sprintf(buffer3,"+%4.1f", T2); // Температура II ступени
	sprintf(buffer2,"%4.1f", T1);
	sprintf(buffer3,"%4.1f", T2);
	
	u8g_DrawStr(&u8g, 20, 61, buffer0); //Отображение Уставки I ступени
    u8g_DrawStr(&u8g, 84, 61, buffer1); //Отображение Уставки II ступени
	
	u8g_SetFont(&u8g, u8g_font_10x20);
    
	u8g_DrawStr(&u8g, 2, 50, buffer2);
    u8g_DrawStr(&u8g, 66, 50, buffer3);
	
	OCR1AL_double = (float)(OCR1AL);
	OCR1BL_double = (float)(OCR1BL);

	display_power1 = (OCR1AL_double/255)*100;
	display_power2 = (OCR1BL_double/255)*100;

	sprintf(buffer4,"%3.0f%%", display_power1); // Мощность ШИМ I ступени
	sprintf(buffer5,"%3.0f%%", display_power2); // Мощность ШИМ II ступени

	u8g_SetFont(&u8g, u8g_font_5x7);     //Отрисовка шкалы мощности
	u8g_DrawStr(&u8g, 44, 19, buffer4);
	u8g_DrawStr(&u8g, 106, 19, buffer5);
	u8g_SetFont(&u8g, u8g_font_6x10);
	
	u8g_DrawLine(&u8g, 62, 21, 62, 61); //Отрисовка контура шкалы I ступени
	u8g_DrawLine(&u8g, 43, 21, 62, 21);
	u8g_DrawLine(&u8g, 43, 21, 43, 30);
	u8g_DrawLine(&u8g, 43, 30, 53, 36);
	u8g_DrawLine(&u8g, 53, 36, 53, 61);
	u8g_DrawLine(&u8g, 53, 61, 62, 61);

	u8g_DrawLine(&u8g, 61+64, 21, 61+64, 61); //Отрисовка контура шкалы II ступени
	u8g_DrawLine(&u8g, 42+64, 21, 61+64, 21);
	u8g_DrawLine(&u8g, 42+64, 21, 42+64, 30);
	u8g_DrawLine(&u8g, 42+64, 30, 52+64, 36);
	u8g_DrawLine(&u8g, 52+64, 36, 52+64, 61);
	u8g_DrawLine(&u8g, 52+64, 61, 61+64, 61);

	//Прототип шкалы
	if (OCR1AL >= 0) u8g_DrawLine(&u8g, 55, 59, 60, 59);//1 5%
	if (OCR1AL >= 25) u8g_DrawLine(&u8g, 55, 57, 60, 57);//2 10%
	if (OCR1AL >= 37) u8g_DrawLine(&u8g, 55, 55, 60, 55);//3 15%
	if (OCR1AL >= 50) u8g_DrawLine(&u8g, 55, 53, 60, 53);//4 20%
	if (OCR1AL >= 62) u8g_DrawLine(&u8g, 55, 51, 60, 51);//5 25%
	if (OCR1AL >= 75) u8g_DrawLine(&u8g, 55, 49, 60, 49);//6 30%
	if (OCR1AL >= 87) u8g_DrawLine(&u8g, 55, 47, 60, 47);//7 35%
	if (OCR1AL >= 100) u8g_DrawLine(&u8g, 55, 45, 60, 45);//8 40%
	if (OCR1AL >= 112) u8g_DrawLine(&u8g, 55, 43, 60, 43);//9 45%
	if (OCR1AL >= 125) u8g_DrawLine(&u8g, 55, 41, 60, 41);//10 50%
	if (OCR1AL >= 137) u8g_DrawLine(&u8g, 55, 39, 60, 39);//11 55%
	if (OCR1AL >= 150) u8g_DrawLine(&u8g, 55, 37, 60, 37);//12 60%
	if (OCR1AL >= 162) u8g_DrawLine(&u8g, 55, 35, 60, 35);//13 65%

	if (OCR1AL >= 175) u8g_DrawLine(&u8g, 52, 33, 60, 33);//14 70%
	if (OCR1AL >= 187) u8g_DrawLine(&u8g, 49, 31, 60, 31);//15 75%
	if (OCR1AL >= 200) u8g_DrawLine(&u8g, 45, 29, 60, 29);//16 80%
	if (OCR1AL >= 212) u8g_DrawLine(&u8g, 45, 27, 60, 27);//17 85%
	if (OCR1AL >= 225) u8g_DrawLine(&u8g, 45, 25, 60, 25);//18 90%
	if (OCR1AL >= 237) u8g_DrawLine(&u8g, 45, 23, 60, 23);//19 95%
	if (OCR1AL >= 254) u8g_DrawLine(&u8g, 45, 21, 60, 21);//20 100%
    //=================================
	
	//Прототип шкалы
	if (OCR1BL >= 0) u8g_DrawLine(&u8g, 55+63, 59, 60+63, 59);
	if (OCR1BL >= 25) u8g_DrawLine(&u8g, 55+63, 57, 60+63, 57);
	if (OCR1BL >= 37) u8g_DrawLine(&u8g, 55+63, 55, 60+63, 55);
	if (OCR1BL >= 50) u8g_DrawLine(&u8g, 55+63, 53, 60+63, 53);
	if (OCR1BL >= 62) u8g_DrawLine(&u8g, 55+63, 51, 60+63, 51);
	if (OCR1BL >= 75) u8g_DrawLine(&u8g, 55+63, 49, 60+63, 49);
	if (OCR1BL >= 87) u8g_DrawLine(&u8g, 55+63, 47, 60+63, 47);
	if (OCR1BL >= 100) u8g_DrawLine(&u8g, 55+63, 45, 60+63, 45);
	if (OCR1BL >= 112) u8g_DrawLine(&u8g, 55+63, 43, 60+63, 43);
	if (OCR1BL >= 125) u8g_DrawLine(&u8g, 55+63, 41, 60+63, 41);
	if (OCR1BL >= 137) u8g_DrawLine(&u8g, 55+63, 39, 60+63, 39);
	if (OCR1BL >= 150) u8g_DrawLine(&u8g, 55+63, 37, 60+63, 37);
	if (OCR1BL >= 162) u8g_DrawLine(&u8g, 55+63, 35, 60+63, 35);

	if (OCR1BL >= 175) u8g_DrawLine(&u8g, 52+63, 33, 60+63, 33);
	if (OCR1BL >= 187) u8g_DrawLine(&u8g, 49+63, 31, 60+63, 31);
	if (OCR1BL >= 200) u8g_DrawLine(&u8g, 45+63, 29, 60+63, 29);
	if (OCR1BL >= 212) u8g_DrawLine(&u8g, 45+63, 27, 60+63, 27);
	if (OCR1BL >= 225) u8g_DrawLine(&u8g, 45+63, 25, 60+63, 25);
	if (OCR1BL >= 237) u8g_DrawLine(&u8g, 45+63, 23, 60+63, 23);
	if (OCR1BL >= 254) u8g_DrawLine(&u8g, 45+63, 21, 60+63, 21);
    //=================================

	//Отрисовка Alarm-ов
	//u8g_DrawXBM(&u8g,4,14,16,16,pic_bmp_Attention);
	if (AlarmTemp1) if (B1) {/*PORTA &=~0x04;*/  u8g_DrawXBM(&u8g,24,14,16,16,pic_bmp_Attention); } else {/*PORTA |=0x04;*/ }
	if (AlarmTemp2) if (B1) {/*PORTA &=~0x04;*/  u8g_DrawXBM(&u8g,24+63,14,16,16,pic_bmp_Attention);} else {/*PORTA |=0x04;*/ }

	if (B2) { u8g_DrawXBM(&u8g,4,14,16,16,pic_bmp_SnowBall1); u8g_DrawXBM(&u8g,4+63,14,16,16,pic_bmp_SnowBall1); } else { u8g_DrawXBM(&u8g,4,14,16,16,pic_bmp_SnowBall2); u8g_DrawXBM(&u8g,4+63,14,16,16,pic_bmp_SnowBall2); }
	//u8g_DrawXBM(&u8g,24,14,16,16,pic_bmp_Error);

	//u8g_DrawXBM(&u8g,4+63,14,16,16,pic_bmp_Attention);

	//u8g_DrawXBM(&u8g,24+63,14,16,16,pic_bmp_Error);

	if (flagEE)
	{
		eeprom_write_float(&eeSetPointI, SetPointI);
		eeprom_write_float(&eeSetPointII, SetPointII);
		eeprom_write_float(&eeSet_Alarm1_min, Set_Alarm1_min);
		eeprom_write_float(&eeSet_Alarm1_max, Set_Alarm1_max);
		eeprom_write_byte(&eeSwap_value, swap_value);

		flagEE = 0;
	}
}

void draw_MainMenuScreen(void)
{
	
	char buffer0[20];
	char buffer1[20];
	char buffer2[20];
	char buffer3[20];
	char buffer4[20];

	
	u8g_DrawFrame(&u8g, 0,0,128,64);
	u8g_SetFont(&u8g, u8g_font_6x10);

	sprintf(buffer0,"%f", SetPointI);
	//u8g_DrawStr(&u8g,65,62,buffer0);

	sprintf(buffer1,"%s", "Main settings");
	sprintf(buffer2,"%s", "Alarm settings");
	sprintf(buffer3,"%s", "About");
	
	u8g_DrawStr(&u8g,36,9, "Main menu");
	u8g_DrawLine(&u8g,10,10,118,10);	
	
	sprintf(buffer4,"%d", SubMenuPosition);
	//u8g_DrawStr(&u8g,2,62,buffer4);
	
	switch (SubMenuPosition)
	{
	case 0:
	       u8g_DrawBox(&u8g, 10, 22, 108, 10);	  
	       u8g_SetDefaultBackgroundColor(&u8g);
	       u8g_DrawStr(&u8g,20,30,buffer1);
	       u8g_SetDefaultForegroundColor(&u8g);
	
       	   u8g_DrawStr(&u8g,20,40,buffer2);
	       u8g_DrawStr(&u8g,20,50,buffer3);
	break;

	case 1:
	       u8g_DrawStr(&u8g,20,30,buffer1);
	
	       u8g_DrawBox(&u8g, 10, 32, 108, 10);
	       u8g_SetDefaultBackgroundColor(&u8g);
	       u8g_DrawStr(&u8g,20,40,buffer2);
	       u8g_SetDefaultForegroundColor(&u8g);

	       u8g_DrawStr(&u8g,20,50,buffer3);
	break;

	case 2:
	       u8g_DrawStr(&u8g,20,30,buffer1);
	       u8g_DrawStr(&u8g,20,40,buffer2);
	
       	   u8g_DrawBox(&u8g, 10, 42, 108, 10);
	       u8g_SetDefaultBackgroundColor(&u8g);
	       u8g_DrawStr(&u8g,20,50,buffer3);
	       u8g_SetDefaultForegroundColor(&u8g);
	break;
	}
	

	

}

void draw_SubMenuPID(void)
{
	
	char buffer0[20];
	char buffer1[20];
	char buffer2[20];
	char buffer3[20];
	char buffer4[20];
	char buffer5[20];

	
	u8g_DrawFrame(&u8g, 0,0,128,64);
	u8g_SetFont(&u8g, u8g_font_6x10);

	sprintf(buffer0,"%u",counter*16);
	//u8g_DrawStr(&u8g,120,62,buffer0);

	sprintf(buffer1,"%s", "Setpoint I");
	sprintf(buffer2,"%s", "Setpoint II");
	sprintf(buffer3,"%s", "Swap sensor");
	sprintf(buffer4,"%s", " ");
	
	u8g_DrawStr(&u8g,26,9, "Main settings");
	u8g_DrawLine(&u8g,10,10,118,10);
	
	sprintf(buffer5,"%d", SubMenuPID);
	//u8g_DrawStr(&u8g,2,62,buffer5);
	
	switch (SubMenuPID)
	{
		case 0:
		u8g_DrawBox(&u8g, 10, 22, 108, 10);
		u8g_SetDefaultBackgroundColor(&u8g);
		u8g_DrawStr(&u8g,20,30,buffer1);
		u8g_SetDefaultForegroundColor(&u8g);
		
		u8g_DrawStr(&u8g,20,40,buffer2);
		u8g_DrawStr(&u8g,20,50,buffer3);
		u8g_DrawStr(&u8g,20,60,buffer4);
		break;

		case 1:
		u8g_DrawStr(&u8g,20,30,buffer1);
		
		u8g_DrawBox(&u8g, 10, 32, 108, 10);
		u8g_SetDefaultBackgroundColor(&u8g);
		u8g_DrawStr(&u8g,20,40,buffer2);
		u8g_SetDefaultForegroundColor(&u8g);

		u8g_DrawStr(&u8g,20,50,buffer3);
		u8g_DrawStr(&u8g,20,60,buffer4);
		break;

		case 2:
		u8g_DrawStr(&u8g,20,30,buffer1);
		u8g_DrawStr(&u8g,20,40,buffer2);
		
		u8g_DrawBox(&u8g, 10, 42, 108, 10);
		u8g_SetDefaultBackgroundColor(&u8g);
		u8g_DrawStr(&u8g,20,50,buffer3);
		u8g_SetDefaultForegroundColor(&u8g);

		u8g_DrawStr(&u8g,20,60,buffer4);
		break;
		/*
		case 3:
		u8g_DrawStr(&u8g,20,30,buffer1);
		u8g_DrawStr(&u8g,20,40,buffer2);
		
		
		u8g_DrawStr(&u8g,20,50,buffer3);
		
		u8g_DrawBox(&u8g, 10, 52, 108, 10);
		u8g_SetDefaultBackgroundColor(&u8g);
		u8g_DrawStr(&u8g,20,60,buffer4);
		u8g_SetDefaultForegroundColor(&u8g);
		break;
		*/
	}
}

void draw_SubMenuAlarm(void)
{
	
	char buffer0[20];
	char buffer1[20];
	char buffer2[20];
	char buffer3[20];
	char buffer4[20];
	char buffer5[20];

	
	u8g_DrawFrame(&u8g, 0,0,128,64);
	u8g_SetFont(&u8g, u8g_font_6x10);

	sprintf(buffer0,"%u",counter*16);
	//u8g_DrawStr(&u8g,120,62,buffer0);

	sprintf(buffer1,"%s", "Alarm level MIN");
	sprintf(buffer2,"%s", "Alarm level MAX");
	//sprintf(buffer3,"%s", "Stage II min");
	//sprintf(buffer4,"%s", "Stage II max");
	
	u8g_DrawStr(&u8g,23,9, "Alarm settings");
	u8g_DrawLine(&u8g,10,10,118,10);
	
	sprintf(buffer5,"%d", SubMenuAlarm);
	//u8g_DrawStr(&u8g,2,62,buffer5);
	
	switch (SubMenuAlarm)
	{
		case 0:
		u8g_DrawBox(&u8g, 10, 22, 108, 10);
		u8g_SetDefaultBackgroundColor(&u8g);
		u8g_DrawStr(&u8g,20,30,buffer1);
		u8g_SetDefaultForegroundColor(&u8g);
		
		u8g_DrawStr(&u8g,20,40,buffer2);
		//u8g_DrawStr(&u8g,20,50,buffer3);
		//u8g_DrawStr(&u8g,20,60,buffer4);
		break;

		case 1:
		u8g_DrawStr(&u8g,20,30,buffer1);
		
		u8g_DrawBox(&u8g, 10, 32, 108, 10);
		u8g_SetDefaultBackgroundColor(&u8g);
		u8g_DrawStr(&u8g,20,40,buffer2);
		u8g_SetDefaultForegroundColor(&u8g);

		//u8g_DrawStr(&u8g,20,50,buffer3);
		//u8g_DrawStr(&u8g,20,60,buffer4);
		break;

		case 2:
		u8g_DrawStr(&u8g,20,30,buffer1);
		u8g_DrawStr(&u8g,20,40,buffer2);
		
		u8g_DrawBox(&u8g, 10, 42, 108, 10);
		u8g_SetDefaultBackgroundColor(&u8g);
		u8g_DrawStr(&u8g,20,50,buffer3);
		u8g_SetDefaultForegroundColor(&u8g);

		u8g_DrawStr(&u8g,20,60,buffer4);
		break;
		
		case 3:
		u8g_DrawStr(&u8g,20,30,buffer1);
		u8g_DrawStr(&u8g,20,40,buffer2);
		
		
		u8g_DrawStr(&u8g,20,50,buffer3);
		
		u8g_DrawBox(&u8g, 10, 52, 108, 10);
		u8g_SetDefaultBackgroundColor(&u8g);
		u8g_DrawStr(&u8g,20,60,buffer4);
		u8g_SetDefaultForegroundColor(&u8g);
		break;
	}
}

void draw_SubMenuAbout(void)
{
	u8g_DrawFrame(&u8g, 0,0,128,64);
	u8g_SetFont(&u8g, u8g_font_6x10);

	u8g_DrawStr(&u8g,50,9, "About");
	u8g_DrawLine(&u8g,10,10,118,10);

    u8g_DrawXBM(&u8g,2,15,29,29,pic_bmp_logo_small_29);

	u8g_DrawStr(&u8g,35,23,"SAMPLE COOLING");
	u8g_DrawStr(&u8g,41,33,"ETL GSM 2050");
	u8g_DrawStr(&u8g,38,43,"Software v3.1");
	u8g_DrawStr(&u8g,20,58,"Evrotechlab Ltd.");	
}

void draw_SetpointI(void)
{
    char buffer0[20]; 
    char buffer1[20];

	u8g_DrawFrame(&u8g, 0,0,128,64);
	u8g_SetFont(&u8g, u8g_font_6x10);

	u8g_DrawStr(&u8g,35,9, "Setpoint I");
	u8g_DrawLine(&u8g,10,10,118,10);

	u8g_DrawXBM(&u8g,106,16,18,18,pic_bmp_Up);
	u8g_DrawXBM(&u8g,106,41,18,18,pic_bmp_Down);

	u8g_SetFont(&u8g, u8g_font_10x20);
	sprintf(buffer0, "%4.1f°C", SetPointI);
	u8g_DrawStr(&u8g, 25, 42, buffer0);
	u8g_SetFont(&u8g, u8g_font_6x10);


}

void draw_SetpointII(void)
{
    char buffer0[20];
    char buffer1[20];

	u8g_DrawFrame(&u8g, 0,0,128,64);
	u8g_SetFont(&u8g, u8g_font_6x10);

	u8g_DrawStr(&u8g,35,9, "Setpoint II");
	u8g_DrawLine(&u8g,10,10,118,10);

	u8g_DrawXBM(&u8g,106,16,18,18,pic_bmp_Up);
	u8g_DrawXBM(&u8g,106,41,18,18,pic_bmp_Down);

	u8g_SetFont(&u8g, u8g_font_10x20);
	sprintf(buffer0, "%4.1f°C", SetPointII);
	u8g_DrawStr(&u8g, 25, 42, buffer0);
	u8g_SetFont(&u8g, u8g_font_6x10);
}

void draw_SwapSensors(void)
{
	char buffer0[20];
	char buffer1[20];
	char buffer20[10];
	char buffer21[10];
	char buffer22[10];
	char buffer23[10];
	char buffer24[10];
	char buffer25[10];
	char buffer26[10];
	char buffer27[10];

	char buffer30[10];
	char buffer31[10];
	char buffer32[10];
	char buffer33[10];
	char buffer34[10];
	char buffer35[10];
	char buffer36[10];
	char buffer37[10];

	uint8_t num_i = 0;

	if (swap_value == 0) num_i = 0; else num_i = 1;


	u8g_DrawFrame(&u8g, 0,0,128,64);
	u8g_SetFont(&u8g, u8g_font_6x10);

	u8g_DrawStr(&u8g,29,9, "Swap sensors");
	u8g_DrawLine(&u8g,10,10,118,10);

	//u8g_DrawXBM(&u8g,106,14,18,18,pic_bmp_Up);
	//u8g_DrawXBM(&u8g,106,40,18,18,pic_bmp_Down);

	u8g_SetFont(&u8g, u8g_font_6x10);
	sprintf(buffer0, "T1=%4.1f°C", T1);
	u8g_DrawStr(&u8g,12,21, buffer0);

	sprintf(buffer20, "%X", allDevices[num_i].id[0]);
	u8g_DrawStr(&u8g,12,32, buffer20);

	sprintf(buffer21, "%X", allDevices[num_i].id[1]);
	u8g_DrawStr(&u8g,26,32, buffer21);

	sprintf(buffer22, "%X", allDevices[num_i].id[2]);
	u8g_DrawStr(&u8g,39,32, buffer22);

	sprintf(buffer23, "%X", allDevices[num_i].id[3]);
	u8g_DrawStr(&u8g,53,32, buffer23);

	sprintf(buffer24, "%X", allDevices[num_i].id[4]);
	u8g_DrawStr(&u8g,67,32, buffer24);

	sprintf(buffer25, "%X", allDevices[num_i].id[5]);
	u8g_DrawStr(&u8g,81,32, buffer25);

	sprintf(buffer26, "%X", allDevices[num_i].id[6]);
	u8g_DrawStr(&u8g,95,32, buffer26);

	sprintf(buffer27, "%X", allDevices[num_i].id[7]);
	u8g_DrawStr(&u8g,109,32, buffer27);
	

	u8g_DrawLine(&u8g,1,35,127,35);

	if (swap_value == 0) num_i = 1; else num_i = 0;

	u8g_SetFont(&u8g, u8g_font_6x10);
	sprintf(buffer1, "T2=%4.1f°C", T2);
	u8g_DrawStr(&u8g,12,47, buffer1);
	
	sprintf(buffer30, "%X", allDevices[num_i].id[0]);
	u8g_DrawStr(&u8g,12,58, buffer30);

	sprintf(buffer31, "%X", allDevices[num_i].id[1]);
	u8g_DrawStr(&u8g,26,58, buffer31);

	sprintf(buffer32, "%X", allDevices[num_i].id[2]);
	u8g_DrawStr(&u8g,39,58, buffer32);

	sprintf(buffer33, "%X", allDevices[num_i].id[3]);
	u8g_DrawStr(&u8g,53,58, buffer33);

	sprintf(buffer34, "%X", allDevices[num_i].id[4]);
	u8g_DrawStr(&u8g,67,58, buffer34);

	sprintf(buffer35, "%X", allDevices[num_i].id[5]);
	u8g_DrawStr(&u8g,81,58, buffer35);

	sprintf(buffer36, "%X", allDevices[num_i].id[6]);
	u8g_DrawStr(&u8g,95,58, buffer36);

	sprintf(buffer37, "%X", allDevices[num_i].id[7]);
	u8g_DrawStr(&u8g,109,58, buffer37);
	



}

void draw_Alarm1min(void)
{
    char buffer0[20];
    char buffer1[20];

	u8g_DrawFrame(&u8g, 0,0,128,64);
	u8g_SetFont(&u8g, u8g_font_6x10);

	u8g_DrawStr(&u8g,18,9, "Alarm level MIN");
	u8g_DrawLine(&u8g,10,10,118,10);

	u8g_DrawXBM(&u8g,106,16,18,18,pic_bmp_Up);
	u8g_DrawXBM(&u8g,106,41,18,18,pic_bmp_Down);

	u8g_SetFont(&u8g, u8g_font_10x20);
	sprintf(buffer0, "%4.1f°C", Set_Alarm1_min);
	u8g_DrawStr(&u8g, 25, 42, buffer0);
	u8g_SetFont(&u8g, u8g_font_6x10);

}

void draw_Alarm1max(void)
{
    char buffer0[20];
    char buffer1[20];

	u8g_DrawFrame(&u8g, 0,0,128,64);
	u8g_SetFont(&u8g, u8g_font_6x10);

	u8g_DrawStr(&u8g,18,9, "Alarm level MAX");
	u8g_DrawLine(&u8g,10,10,118,10);

	u8g_DrawXBM(&u8g,106,16,18,18,pic_bmp_Up);
	u8g_DrawXBM(&u8g,106,41,18,18,pic_bmp_Down);

	u8g_SetFont(&u8g, u8g_font_10x20);
	sprintf(buffer0, "%4.1f°C", Set_Alarm1_max);
	u8g_DrawStr(&u8g, 25, 42, buffer0);
	u8g_SetFont(&u8g, u8g_font_6x10);
}

void draw_Alarm2min(void)
{
	u8g_DrawFrame(&u8g, 0,0,128,64);
	u8g_SetFont(&u8g, u8g_font_6x10);

	u8g_DrawStr(&u8g,29,9, "Stage II min");
	u8g_DrawLine(&u8g,10,10,118,10);

}

void draw_Alarm2max(void)
{
	u8g_DrawFrame(&u8g, 0,0,128,64);
	u8g_SetFont(&u8g, u8g_font_6x10);

	u8g_DrawStr(&u8g,29,9, "Stage II max");
	u8g_DrawLine(&u8g,10,10,118,10);

}



int main(void)
{
    /* Replace with your application code */

	  
	  
	  // Конфикурация портов ввода-вывода

	  DDRA=(1<<PA0) | (1<<PA1) | (1<<PA2) | (1<<PA5);
	  PORTA |= 0x25; //Подсветка и светодиод Power

	  // Port B initialization
	  // Function: Bit7=In Bit6=Out Bit5=Out Bit4=In Bit3=In Bit2=In Bit1=In Bit0=In
	  DDRB=(0<<PB7) | (1<<PB6) | (1<<PB5) | (0<<PB4) | (0<<PB3) | (0<<PB2) | (0<<PB1) | (0<<PB0);
	  // State: Bit7=T Bit6=0 Bit5=0 Bit4=T Bit3=T Bit2=T Bit1=T Bit0=T
	  PORTB=(0<<PB7) | (0<<PB6) | (0<<PB5) | (0<<PB4) | (0<<PB3) | (0<<PB2) | (0<<PB1) | (0<<PB0);


	  DDRD = 0x70;
	  PORTD &= ~0x70;

	  DDRE = 0xE0; // Предварительно, PORTE весь сконфигурирован на выход.
	  PORTE = 0x00;// Предварительное обнуление.

	  DDRF=0x00;
	  PORTF=0x00;


	  // Разрешение прерывания TIMER0, TIMER2, TIMER2
	  TIMSK=(0<<OCIE2) | (1<<TOIE2) | (0<<TICIE1) | (0<<OCIE1A) | (0<<OCIE1B) | (1<<TOIE1) | (0<<OCIE0) | (1<<TOIE0);
	  
	  //TCCR0 = 0x00; //Timer0 stop
	  TCCR0 = 0x01; // Running. CLK
	  // TCCR0 = 0x02; // Running. CLK/8
	  //TCCR0 = 0x03; // Running. CLK/32
	  //TCCR0 = 0x04; // Running. CLK/64
	  //TCCR0 = 0x05; // Running. CLK/128
	  //TCCR0 = 0xC5; // Running. CLK/128
	  //TCCR0 = 0x6F; // Running. CLK/128
	  //TCCR0 = 0x06; // Running. CLK/256
	  //TCCR0 = 0x07; // Running. CLK/1024
	  OCR0 = 0xFF;
	  
	  //Configuration Timet1
	  //TCCR1B = 0x00; //Timer1 stop
	  //   TCCR1B = 0x01; // Running. No prescaling
	  //   TCCR1B = 0x02; // Running. CLK/8
	  //TCCR1B = 0x03; // Running. CLK/64
	  //TCCR1B = 0x04; // Running. CLK/256
	  //TCCR1B = 0x05; // Running. CLK/1024
	  TCCR1A=(1<<COM1A1) | (0<<COM1A0) | (1<<COM1B1) | (0<<COM1B0) | (0<<COM1C1) | (0<<COM1C0) | (0<<WGM11) | (1<<WGM10);
	  TCCR1B=(0<<ICNC1) | (0<<ICES1) | (0<<WGM13) | (1<<WGM12) | (0<<CS12) | (1<<CS11) | (0<<CS10);
	  TCNT1H=0x00;
	  TCNT1L=0x00;
	  ICR1H=0x00;
	  ICR1L=0x00;

	  OCR1AH=0x00;
	  OCR1AL=0x00;

	  OCR1BH=0x00;
	  OCR1BL=0x00;

	  OCR1CH=0x00;
	  OCR1CL=0x00;
	  
	  //Configuration Timet2
	  //TCCR2 = 0x00; //Timer2 stop
	  //TCCR2 = 0x01; // Running. No prescaling
	  //TCCR2 = 0x02; // Running. CLK/8
	  // TCCR2 = 0x03; // Running. CLK/64
	  //TCCR2 = 0x04; // Running. CLK/256
	  TCCR2 = 0x05; // Running. CLK/1024
	  OCR2 = 0xFF;
	  
	  //Configuration Timet3
	  ETIMSK=(0<<TICIE3) | (0<<OCIE3A) | (0<<OCIE3B) | (1<<TOIE3) | (0<<OCIE3C) | (0<<OCIE1C);
	  //  TCCR3B = 0x00; //Timer3 stop
	  //   TCCR3B = 0x01; // Running. No prescaling
	  //   TCCR3B = 0x02; // Running. CLK/8
	  TCCR3B = 0x03; // Running. CLK/64
	  //TCCR3B = 0x04; // Running. CLK/256
	  //TCCR3B = 0x05; // Running. CLK/1024
	  




	  // USART1 initialization
	  // Communication Parameters: 8 Data, 1 Stop, No Parity
	  // USART1 Receiver: Off
	  // USART1 Transmitter: On
	  // USART1 Mode: Asynchronous
	  // USART1 Baud Rate: 38400
	  UCSR1A=(0<<RXC1) | (0<<TXC1) | (0<<UDRE1) | (0<<FE1) | (0<<DOR1) | (0<<UPE1) | (0<<U2X1) | (0<<MPCM1);
	  UCSR1B=(0<<RXCIE1) | (1<<TXCIE1) | (0<<UDRIE1) | (0<<RXEN1) | (1<<TXEN1) | (0<<UCSZ12) | (0<<RXB81) | (0<<TXB81);
	  UCSR1C=(0<<UMSEL1) | (0<<UPM11) | (0<<UPM10) | (0<<USBS1) | (1<<UCSZ11) | (1<<UCSZ10) | (0<<UCPOL1);
	  UBRR1H=0x00;
	  UBRR1L=0x19;

	  uprintf(NEWLINESTR);
	  uprintf("========================"NEWLINESTR);
	  uprintf("ETL GSM 2050"NEWLINESTR);



u8g_setup(); //Настройка и инициализация экрана


  // Загрузка уставок из памяти
 /*
 eeprom_write_float(&eeSetPointI, 2.0);
 eeprom_write_float(&eeSetPointII, 2.0);
 eeprom_write_float(&eeSet_Alarm1_min, 0.5); 
 eeprom_write_float(&eeSet_Alarm1_max, 4.0);
 eeprom_write_float(&eeSet_Alarm2_min, 0.5);
 eeprom_write_float(&eeSet_Alarm1_max, 4.0);
 eeprom_write_byte(&eeSwap_value, 0);
*/


SetPointI = eeprom_read_float(&eeSetPointI);
 SetPointII = eeprom_read_float(&eeSetPointII);

 Set_Alarm1_min = eeprom_read_float(&eeSet_Alarm1_min);
 Set_Alarm1_max = eeprom_read_float(&eeSet_Alarm1_max);

 Set_Alarm2_min = eeprom_read_float(&eeSet_Alarm2_min);
 Set_Alarm2_max = eeprom_read_float(&eeSet_Alarm2_max);

 swap_value = eeprom_read_byte(&eeSwap_value);

 Kp = -3;
 Ti = 400;
 Td = 0;
 Ts = 1;

 Uset1[0] = 0; Uset1[1] = 0;
 Err1[0] = 0; Err1[1] = 0; Err1[2] = 0;

 Uset2[0] = 0; Uset2[1] = 0;
 Err2[0] = 0; Err2[1] = 0; Err2[2] = 0;

 Ki = Ts/Ti;	// i.e. 1/(Ti/Ts);
 Kd = Td/Ts;
	
	//========================================
	//Блок вывода Логотипа-заставки
	  while (counter<65)
	  {
		  u8g_FirstPage(&u8g);
		  do
		  {
				u8g_DrawXBM(&u8g,0,0,128,counter,pic_bmp_logo);
				counter++;
		  } while ( u8g_NextPage(&u8g) );
		  u8g_Delay(20);		  
	 }
	 
	 _delay_ms(1000);

	// PORTD &= ~0x70;
	 
	 counter = 0;
		 	 
		 {
			 u8g_FirstPage(&u8g);
			 do
			 {
				 u8g_DrawXBM(&u8g,0,0,128,64,pic_bmp_model);
			 } while ( u8g_NextPage(&u8g) );
			 u8g_Delay(20);
		 }
		 
		 _delay_ms(100);		 
		// PORTD |= 0x70;	 

		










	  OWI_Init(BUS);
/*-флаг сброшен - выполнить поиск 1Wire устройств
    -если количество заданных устройсв совпадает с
    колличеством найденных - устанавливаем флаг, 
    чтобы функция поиска больше не запускалась
    -отобразить количество найденных устройств*/
    if (searchFlag == SEARCH_SENSORS){
      num = 0;
      crcFlag = OWI_SearchDevices(allDevices, MAX_DEVICES, BUS, &num);
      //LCD_Goto(14,1);
      //BCD_1Lcd(num);
	   	sprintf(str,"Number %d:",num);
	   	uprintf(str);
	   	uprintf(NEWLINESTR);
      if ((num == MAX_DEVICES)&&(crcFlag != SEARCH_CRC_ERROR)){
         searchFlag = SENSORS_FOUND;  
      }
    }

	 _delay_ms(100);
	sei();







    while (1) 
    {

	/*
    	  u8g_FirstPage(&u8g);
    	  do
    	  {
	    	  draw_mainscreen();
    	  } while ( u8g_NextPage(&u8g) );

		  asm("nop");
		  u8g_Delay(50);
   
    */
  
  

    };
  
  return 0;




    
}









/*****************************************************************************
*   Function name :   DS18B20_ReadTemperature
*   Returns :       коды - READ_CRC_ERROR, если считанные данные не прошли проверку
*                          READ_SUCCESSFUL, если данные прошли проверку    
*   Parameters :    bus - вывод микроконтроллера, который выполняет роль 1WIRE шины
*                   *id - имя массива из 8-ми элементов, в котором хранится
*                         адрес датчика DS18B20
*                   *temperature - указатель на шестнадцати разрядную переменную
*                                в которой будет сохранено считанного зн. температуры
*   Purpose :      Адресует датчик DS18B20, дает команду на преобразование температуры
*                  ждет, считывает его память - scratchpad, проверяет CRC,
*                  сохраняет значение температуры в переменной, возвращает код ошибки             
*****************************************************************************/
unsigned char DS18B20_ReadTemperature(unsigned char bus, unsigned char * id, unsigned int* temperature)
{
    unsigned char scratchpad[9];
    unsigned char i;
  
    /*подаем сигнал сброса
    команду для адресации устройства на шине
    подаем команду - запук преобразования */
    OWI_DetectPresence(bus);
    OWI_MatchRom(id, bus);
    OWI_SendByte(DS18B20_CONVERT_T ,bus);

    /*ждем, когда датчик завершит преобразование*/ 
    while (!OWI_ReadBit(bus));

    /*подаем сигнал сброса
    команду для адресации устройства на шине
    команду - чтение внутренней памяти
    затем считываем внутреннюю память датчика в массив
    */
    OWI_DetectPresence(bus);
    OWI_MatchRom(id, bus);
    OWI_SendByte(DS18B20_READ_SCRATCHPAD, bus);
    for (i = 0; i<=8; i++){
      scratchpad[i] = OWI_ReceiveByte(bus);
    }
    
    if(OWI_CheckScratchPadCRC(scratchpad) != OWI_CRC_OK){
      return READ_CRC_ERROR;
    }
    
    *temperature = (unsigned int)scratchpad[0];
    *temperature |= ((unsigned int)scratchpad[1] << 8);
    
    return READ_SUCCESSFUL;
}

/*****************************************************************************
*   Function name :  DS18B20_PrintTemperature 
*   Returns :         нет       
*   Parameters :     temperature - температура датчика DS18B20     
*   Purpose :        Выводит значение температуры датчика DS18B20
*                    на LCD. Адрес знакоместа нужно выставлять заранее.
*****************************************************************************/

void DS18B20_PrintTemperature(unsigned int temperature)
{
  unsigned char tmp = 0;
  /*выводим знак температуры
  *если она отрицательная 
  *делаем преобразование*/  
  if ((temperature & 0x8000) == 0){
    //LCD_WriteData('+');
	  	//sprintf(str,"+",temperature);
	  	uprintf("+");
	  	//uart_puts(NEWLINESTR);
  }
  else{
    //LCD_WriteData('-');
	  	//sprintf(str,"T2=%d",temperature);
	  	uprintf("-");
	  	//uart_puts(NEWLINESTR);
    temperature = ~temperature + 1;
  }
        
  //выводим значение целое знач. температуры      
  tmp = (unsigned char)(temperature>>4);
  if (tmp<100){
    //BCD_2Lcd(tmp);
	  	sprintf(str,"%d",tmp);
	  	uprintf(str);
	  	//uart_puts(NEWLINESTR);
  }
  else{
    //BCD_3Lcd(tmp);
	  	sprintf(str,"%d",tmp);
	  	uprintf(str);
	  	//uart_puts(NEWLINESTR);    
  }
        
  //выводим дробную часть знач. температуры
  tmp = (unsigned char)(temperature&15);
  tmp = (tmp>>1) + (tmp>>3);
    	sprintf(str,".%d",tmp);
    	uprintf(str);
    	uprintf(NEWLINESTR);
  //LCD_WriteData('.');
  //BCD_1Lcd(tmp);
}