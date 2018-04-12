#include "LoraEp.h"
#include "Init.h"

void main()
{
	LoraInit();		
	LpTimConfig();
	while(1)
	{
		WDT_CLR();
		EnterStopMode();
	}
	
}
