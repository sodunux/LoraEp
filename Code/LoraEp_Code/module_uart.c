#include "init.h"
#include "fm375_uart.h"



/****************************************************************/
/* 函数名：CalFrameBCC										*/
/* 功能说明：计算发送数据缓冲区Uart.TX_Buf的BCC校验码			*/
/* 入口参数：Uart.TX_Buf,Uart.TX_Buf[2]=len 					*/
/* 出口参数：校验和 									*/
/* 时间:														*/
/* 作者:														*/
/****************************************************************/
void CalFrameBCC(void)
{
	INT16U  i;
	
	i = TX_Buf[4];
	//i = (TX_Buf[3]<<8)+TX_Buf[4];
	TX_Buf[i+7] = 0xAA;
	TX_Buf[i+8] = 0x16;

}

/****************************************************************/
/* 函数名：Comm_Proc										*/
/* 功能说明：串口通信解帧处理函数				*/
/* 入口参数：N/A											*/
/* 出口参数：N/A											*/
/* 时间:  														*/
/* 作者:  														*/
/****************************************************************/
void Comm_Proc(void)
{
	switch (Commandh)
	{
		case SYS:
			Pro_SYS();
		break;
		
		case PMU:
			Pro_PMU();
		break;		
		
		case RTC:
			Pro_RTC();
		break;		
		
		case SPI:
			Pro_SPI();
		break;
		
		case DMA:
			Pro_DMA();
		break;		
		
		case I2C:
			Pro_I2C();
		break;	
		
		case U7816:
			Pro_U7816();
		break;		

		case FLSCTRL:
			Pro_FLSCTRL();
		break;		
		
		case MOD:
			Pro_MOD();
		break;		
		
		case ANA:
			Pro_ANA();
		break;	

		case RST:
			Pro_WDT();
		break;

		case CLKGEN:
			Pro_CLKGEN(); 
		break;		
		
		case FLIP:
			Pro_FLIP();
		break;		
		
		case PAD:
			Pro_PAD();
		break;		
		
		case CRC:
			Pro_CRC();
		break;	
		
		case LPTIM:
			Pro_LPTIM();
		break;	
			
		case RAM:
			Pro_RAM();
		break;

		case DISP:
			Pro_DISP();
		break;	

		case IDD:
			Pro_IDD();
		break;



		default:
		break;
	}
	
//填返回数据头部

	TX_Buf[0] = 0x68;				//HEAD1
	TX_Buf[1] = UART_ADDRH;		//地址字节H  
	TX_Buf[2] = UART_ADDRL;		//地址字节L
	TX_Buf[3] = 0x97;		 		//HEAD2,数据区起始标志
//	TX_Buf[4] = 0x00;				//数据区长度
	TX_Buf[5] = Commandh;
	TX_Buf[6] = Commandl;
	
//填返回数据尾部	
	CalFrameBCC();
	
}

/****************************************************************/
/* 函数名：Uart_Proc											*/
/* 功能说明：Uart串口通信解帧处理					*/
/* 入口参数：N/A											*/
/* 出口参数：N/A											*/
/* 时间:  														*/
/* 作者:  														*/
/****************************************************************/
void Uart_Proc(void)
{	  

	Comm_Proc();						//通信命令处理

	switch(UART_ADDRL)
	{
		case 0:

			Uart3.SendStatus = 0x0;     //为防止SM3运算改变此变量，此处再次对其初始化
			Uart3.RecvOKFLAG = FALSE; 			//清有命令处理标志状态
			Uart3.pRecvPoint = 0;					//接收指针清零
			Uart3.RecvStatus = 0;					//接收状态复位为开始接收装态
			if (Uart3.SendStatus == 0x00)
			{

				//启动发送数据
				Uart3.SendStatus = 0xFF;				//树发送标志
				Uart3.pSendPoint = 1;
				Uart3.bSendOverFlag = FALSE;			//清发送数据结束标志

				Uart3.pSendLenth = TX_Buf[4]+9;
//Uart3.pSendLenth = (TX_Buf[3]<<8)+TX_Buf[4]+9;
				ACC = 0x68;             //帧头
				TXREG3 = ACC;						//Uart0串口发
				
// 				while(!(UARTIF & txif3));
				UARTIE |= txie3;			//ENABLE TX INTERRUPT
			}			
		break;		
	}
}