#include "LoraEp.h"
#include "Init.h"

void main()
{	
	LoraInit();
	while(1)
	{
		WDT_CLR();		
		if(LoraStatus==Lora_ON)
			Led_LoraON();
		if(LowVolt==1) //Ç·Ñ¹×´Ì¬,ºìµÆÁÁ
		{
			Led_LowVolt();
		}
			EnterSleepMode();
		}
			
}
