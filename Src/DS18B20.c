/*
 * DS18B20.c
 *
 *  Created on: 13 oct. 2017
 *      Author: Farouk madjoub
 */
#include "stm32l4xx_hal.h"




   extern int TEMPVALUE;


#define  ONEWIRE_OUT        GPIO_PIN_9
#define  ONEWIRE_IN         GPIO_PIN_8
#define  ONEWIRE_PORT  		GPIOB

#define  ONEWIRE_INPUT_READ				HAL_GPIO_ReadPin(ONEWIRE_PORT,ONEWIRE_IN)
#define  ONEWIRE_OUTPUT_HIGH			HAL_GPIO_WritePin(ONEWIRE_PORT,ONEWIRE_OUT,GPIO_PIN_SET)   // 5v
#define  ONEWIRE_OUTPUT_LOW				HAL_GPIO_WritePin(ONEWIRE_PORT,ONEWIRE_OUT,GPIO_PIN_RESET) // 0v


#define  READ_ROM			0x33
#define  SKIP_ROM			0xCC
#define  READ_SCRATCHPAD	0xBE
#define  CONVERT_TEMP		0x44
#define  WRITE_SCRATCHPAD   0x4E



uint8_t						pad[9];
uint8_t						rom[8];
uint8_t						res[2];

float tempDS;

void				 		DelayDS(uint32_t);
static void					PingDS(void);
static void					InitDS(void);
static void					SendInit(void);
static void					SendDS(uint8_t  val);
static uint8_t  			ReadDS(void);
static void					ReportScratchpad(void);
static void					StartConversion(void);
static void					ReportTemperature(void);
static void					ReportROM(void);


void  InitDS(void)
{
	 GPIO_InitTypeDef GPIO_InitStruct;

	  __HAL_RCC_GPIOB_CLK_ENABLE();	// route the clocks

    GPIO_InitStruct.Pin = ONEWIRE_OUT;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = ONEWIRE_IN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}


/*--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/

void  delayDS(uint32_t  usecs)
{
	uint32_t			n;

	n = usecs;

	while (n)
	{
		n--;
		asm("nop");
	}
}




float checkDS(void)
{

	InitDS();
    PingDS();
	ReportROM();
    ReportScratchpad();
    StartConversion();
	delayDS(270000);
	ReportTemperature();

return tempDS;


}



static void  PingDS(void)
{
	uint8_t				response;

	response = 1;
	ONEWIRE_OUTPUT_LOW;
	delayDS(335);

	while (1)
	{
		SendInit();
		response = HAL_GPIO_ReadPin(ONEWIRE_PORT,ONEWIRE_IN);		// device pulls line low in response
		delayDS(1512);
        if (response == 0)  break;
	}

}



static void  StartConversion(void)
{

	SendInit();
	delayDS(335);


	SendDS(SKIP_ROM);
	SendDS(CONVERT_TEMP);
}


static void  ReportTemperature(void)
{
	uint32_t			val;
    uint8_t				n;


	SendInit();
	delayDS(360);

	SendDS(SKIP_ROM);
	SendDS(READ_SCRATCHPAD);
	for (n=0; n<9; n++)
	{
		pad[n] = ReadDS();
	}
	val = (pad[1] * 256 + pad[0]);			
	tempDS=val*(0.0625);  // 12bit resolution

}



static void  ReportROM(void)
{
	uint8_t					n;

	SendInit();
	delayDS(335);//100

	SendDS(READ_ROM);
	for (n=0; n<8; n++)
	{
		rom[n] = ReadDS();
	}

}



static void  ReportScratchpad(void)
{
	uint8_t					n;

	SendInit();
	delayDS(335);//100


	SendDS(SKIP_ROM);
	SendDS(READ_SCRATCHPAD);
	for (n=0; n<9; n++)
	{
		pad[n] = ReadDS();
	}
}



static void  SendInit(void)
{
		ONEWIRE_OUTPUT_HIGH;
	
	    delayDS(1700);//500

	    ONEWIRE_OUTPUT_LOW;
	    delayDS(1700);//500

	    ONEWIRE_OUTPUT_HIGH;

	    delayDS(165);//50

}


static void  SendDS(uint8_t  val)
{
	uint8_t				n;

	for (n=0; n<8; n++)
	{
		ONEWIRE_OUTPUT_LOW;
		delayDS(10);//5


		if (val & 1) { ONEWIRE_OUTPUT_HIGH;}
		delayDS(315);//95


		ONEWIRE_OUTPUT_HIGH;
		delayDS(0);//2


		val = val >> 1;
	}
}



static uint8_t  ReadDS(void)
{
	uint8_t				n;
	uint8_t				val;

	val = 0;
	for (n=0; n<8; n++)
	{
		val = val >> 1;
		ONEWIRE_OUTPUT_LOW;

		delayDS(45);//15
        ONEWIRE_OUTPUT_HIGH;
        delayDS(20); //10

		if (ONEWIRE_INPUT_READ)  {val = val | 0x80;}
		delayDS(198);//65
	}
	return  val;
}
