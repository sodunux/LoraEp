#include "init.h"

void clk_out_ctl(INT8U option)
{
	switch(option)
	{
		case 0:
			XTCTRL1 = (TX_Buf[1]<<3);
			XTCTRL2 = TX_Buf[2];
			IPWRCTRL = (TX_Buf[3]<<4) | TX_Buf[4];
		
			IO4OUTEN |= BIT5;  //GPIO45 OUT CLK
			TESTCON = clko_xtlf | clko_en; 				
		break;
		case 1:
			IO4OUTEN |= BIT5;  //GPIO45 OUT CLK
			TESTCON = clko_lsclk | clko_en; 			
		break;		
		case 2:
			RCCTRL1 = TX_Buf[1];
			RCCTRL2 = TX_Buf[2];
		
			IO4OUTEN |= BIT5;  //GPIO45 OUT CLK
			TESTCON = clko_rchf_16 | clko_en; 			
		break;
		case 3:
			XTCTRL1 = (TX_Buf[1]<<3);
			XTCTRL2 = TX_Buf[2];
			IPWRCTRL = (TX_Buf[3]<<4) | TX_Buf[4];
		
			IO4OUTEN |= BIT5;  //GPIO45 OUT CLK
			TESTCON = clko_rtc_1s | clko_en; 			
		break;
		case 4:
			IO4OUTEN |= BIT5;  //GPIO45 OUT CLK
			TESTCON = clko_coreclk | clko_en; 			
		break;
		case 5:
			XTCTRL1 = (TX_Buf[1]<<3);
			XTCTRL2 = TX_Buf[2];
			IPWRCTRL = (TX_Buf[3]<<4) | TX_Buf[4];			
		
			IO4OUTEN |= BIT5;  //GPIO45 OUT CLK
			TESTCON = clko_rtc_20s | clko_en; 			
		break;		
		
		default:
			break;
	}
}
