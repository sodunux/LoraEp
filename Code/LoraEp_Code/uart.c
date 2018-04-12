#include "init.h"
char volatile usart_temp[50];
/************************************************************************/
/* 函数名：Init_UART_PAD													*/
/* 功能说明：UART管脚初始化子程序 							*/
/* 入口参数：N/A													*/
/* 出口参数：N/A													*/
/************************************************************************/
void Init_UART_PAD(void)
{

	PFFCR1 &= ~BIT4;
	PFFCR2 |= BIT4;//{PFFCR2[4]:PFFCR1[4]}=2'b10,alternate function 
	AFSELF |= afself4_txd0;//TXD0功能	

	PFFCR1 &= ~BIT3;
	PFFCR2 |= BIT3;//{PFFCR2[3]:PFFCR1[3]}=2'b10,alternate function 
	AFSELF |= afself3_rxd0;//RXD0功能	
		
	
	PAFCR1 &= ~BIT1;
	PAFCR2 |= BIT1;//{PAFCR2[1]:PAFCR1[1]}=2'b10,alternate function 
	AFSELA |= afsela1_txd1;//TXD1功能	

	PAFCR1 &= ~BIT0;
	PAFCR2 |= BIT0;//{PAFCR2[0]:PAFCR1[0]}=2'b10,alternate function 
	AFSELA |= afsela0_rxd1;//RXD1功能		
	
	PDFCR1 &= ~BIT2;
	PDFCR2 |= BIT2;//{PDFCR2[2]:PDFCR1[2]}=2'b10,alternate function 
	AFSELD &= ~BIT2;//RXD2功能	

	PDFCR1 &= ~BIT3;
	PDFCR2 |= BIT3;//{PDFCR2[3]:PDFCR1[3]}=2'b10,alternate function 
	AFSELD &= ~BIT3;//TXD2功能		
	
	PGFCR1 &= ~BIT6;
	PGFCR2 |= BIT6;//{PGFCR2[6]:PGFCR1[6]}=2'b10,alternate function 
	AFSELG |= afselg6_txd3;//TXD3功能	

	PGFCR1 &= ~BIT5;
	PGFCR2 |= BIT5;//{PGFCR2[5]:PGFCR1[5]}=2'b10,alternate function 
	AFSELG |= afselg5_rxd3;//RXD3功能			
}

/************************************************************************/
/* 函数名：Init_UART												    */
/* 功能说明：uart 模块初始化子程序      					    		*/
/* 入口参数：N/A													    */
/* 出口参数：N/A													    */
/************************************************************************/
void Init_UART(void)
{
	//Uart0	
//	RTXCON0 |= rxdflag | txdflag;
	RXSTA0 = rxen | pdsel_8_n;	//8位数据，无奇偶校验,接收使能
	TXSTA0 = txen;		//发送使能 txen
	
	SPBRGH0 = 0x03;	
 	SPBRGL0 = 0x40;//9600,8M MCLK
// 	SPBRGH0 = 0x06;	
//  	SPBRGL0 = 0x81;//9600,16M MCLK
// 	SPBRGH0 = 0x00;	
//  	SPBRGL0 = 0xcf;//9600,2M MCLK
// 	SPBRGH0 = 0x08;	
//  	SPBRGL0 = 0x22;//9600,20M MCLK
	
	RXSTA1 = rxen | pdsel_8_n;	//8位数据，无奇偶校验,接收使能
	TXSTA1 = txen;		//发送使能 txen
	
	SPBRGH1 = 0x03;	
 	SPBRGL1 = 0x40;//9600,8M MCLK	

//==== UART2控制34401，波特率1200,2个停止位 =====//
	RXSTA2 = rxen | pdsel_8_n;	//8位数据，无奇偶校验,接收使能
	TXSTA2 = txen | stopsel;		//发送使能 txen
	
	SPBRGH2 = 0x1A;	
 	SPBRGL2 = 0x0A;//1200,8M MCLK	


	RXSTA3 = rxen | pdsel_8_n;	//8位数据，无奇偶校验,接收使能
	TXSTA3 = txen;		//发送使能 txen
	
	SPBRGH3 = 0x03;	
 	SPBRGL3 = 0x40;//9600,8M MCLK		

	UARTIF = 0;
	UARTIE |= rxie3;		//接收中断使能
	RXSTA3 |= uarterrie;  //允许错误中断 
	
	Commandh = 0x0;
	Commandl = 0x0;

	//uart3
	Uart3.pRecvPoint = 0;
	Uart3.pRecvLenth = 0;
	Uart3.pSendPoint = 0;
	Uart3.pSendLenth = 0;
	Uart3.RecvStatus = 0x0;
	Uart3.SendStatus = 0x0;
	Uart3.RecvOKFLAG = FALSE; 		//清有命令处理标志状态
	Uart3.bUpRecordFlag = FALSE;		//清有数据上传标志
	Uart3.pTmOutVal = 00;				//接收最大字节间延时100*4=400毫秒
	Uart3.bCRC = 0;
	Uart_test_int = 0;
}


/****************************************************************/
/* 函数名：UartTx_0										*/
/* 功能说明：串口字节发送				*/
/* 入口参数：value,待发送数据											*/
/* 出口参数：N/A											*/
/* 时间:  														*/
/* 作者:  														*/
/****************************************************************/												
void UartTx_0(INT8U Value)
{

			TXREG0 = Value;
			while(!(UARTIF & txif0));

}

/****************************************************************/
/* 函数名：UartTx_2									*/
/* 功能说明：串口字节发送				*/
/* 入口参数：value,待发送数据											*/
/* 出口参数：N/A											*/
/* 时间:  														*/
/* 作者:  														*/
/****************************************************************/												
void UartTx_2(INT8U Value)
{

			TXREG2 = Value;
			while(!(UARTIF & txif2));

}

/****************************************************************/
/* 函数名：UartTx_3									*/
/* 功能说明：串口字节发送				*/
/* 入口参数：value,待发送数据											*/
/* 出口参数：N/A											*/
/* 时间:  														*/
/* 作者:  														*/
/****************************************************************/												
void UartTx_3(INT8U Value)
{

			TXREG3 = Value;
			while(!(UARTIF & txif3));

}

/* 函数名：Uart2_SendStr										*/
/* 功能说明：串口字符串发送										*/
/* 入口参数：len,发送数据长度；ptr,发送数据缓冲区首址     		*/
/* 出口参数：N/A												*/
/* 时间:  														*/
/* 作者:  														*/
/****************************************************************/	
void Uart2_SendStr(INT8U len, INT8U *ptr)
{
	INT8U i;
	for(i=0; i<len; i++)
	{
		UartTx_2(ptr[i]);
	}
}

/****************************************************************/
/* 函数名：UartRec_0										*/
/* 功能说明：串口字节接收				*/
/* 入口参数：										*/
/* 出口参数：value											*/
/* 时间:  														*/
/* 作者:  														*/
/****************************************************************/												
/*INT8U UartRec_0(void)
{
	INT8U i;
	while(!(UARTIF & rxif0));
	i = RXREG0;
	return i;
}
*/


/* 函数名：Uart0_SendStr										*/
/* 功能说明：串口字符串发送										*/
/* 入口参数：len,发送数据长度；ptr,发送数据缓冲区首址     		*/
/* 出口参数：N/A												*/
/* 时间:  														*/
/* 作者:  														*/
/****************************************************************/	
// void Uart0_SendStr(INT8U len, INT8U *ptr)
// {
// 	INT8U i;
// 	for(i=0; i<len; i++)
// 	{
// 		UartTx_0(ptr[i]);
// 	}
// }

/****************************************************************/
/* 函数名：UartTx_1										*/
/* 功能说明：串口字节发送				*/
/* 入口参数：value,待发送数据											*/
/* 出口参数：N/A											*/
/* 时间:  														*/
/* 作者:  														*/
/****************************************************************/												
void UartTx_1(INT8U Value)
{

			TXREG1 = Value;
			while(!(UARTIF & txif1));

}

/****************************************************************/
/* 函数名：UartSendInfo									*/
/* 功能说明：串口字符串发送				*/
/* 入口参数：len,发送数据长度；ptr,发送数据缓冲区首址     */
/* 出口参数：N/A											*/
/* 时间:  														*/
/* 作者:  														*/
/****************************************************************/	
void Uart1_SendInfo(INT8U len, INT8U *ptr)
{
	INT8U i;
	for(i=0; i<len; i++)
	{
		UartTx_1(ptr[i]);
	}
}

void UART1_receiveStr(uchar length)
{
	uchar i=0;

	while (i<length)
	{
		while(!(UARTIF & rxif1)) //UART1接收中断标志位为1时退出while
		{
			WDT_CLR();
		} 
		usart_temp[i++]= RXREG1;	
  }	
}

void UART2_receiveStr(uchar length)
{
	uchar i=0;

	while (i<length)
	{
		while(!(UARTIF & rxif2)) //UART2接收中断标志位为1时退出while
		{
			WDT_CLR();
		} 
		usart_temp[i++]= RXREG2;	
  }	
}



/****************************************************************/
/* 函数名：UartSendInfo									*/
/* 功能说明：串口字符串发送				*/
/* 入口参数：len,发送数据长度；ptr,发送数据缓冲区首址     */
/* 出口参数：N/A											*/
/* 时间:  														*/
/* 作者:  														*/
/****************************************************************/	
void UartSendInfo(INT8U len, INT8U *ptr)
{
	INT8U i;
	for(i=0; i<len; i++)
	{
		UartTx_3(ptr[i]);
	}
}


// void Init_UART2(void)
// {

// 	RXSTA2 = rxen | pdsel_8_n;	//8位数据，无奇偶校验,接收使能
// 	TXSTA2 = txen;		//发送使能 txen
// 	
// 	SPBRGH2 = 0x03;	
//  	SPBRGL2 = 0x40;//9600,8M MCLK
// 	
// }

void Uart0_TX_DMA(INT8U near *ptr, INT8U len)
{

	DMA_Start(CH0,(INT16U)ptr,len,uart0_tx);
	while(!(UARTIF & txif0));
	//while(TXFIFOSTA0 & txff);  //最后一个字节要等待UART发送结束
}

void Uart0_RX_DMA(INT8U near *ptr, INT8U len)
{
	DMA_Start(CH0,(INT16U)ptr,len,uart0_rx);
}

// void Uart2_TX_DMA(INT8U near *ptr, INT8U len)
// {
// 	DMA_Start(CH0,ptr,len,uart2_tx);
// 	while(!(UARTIF & txif2));
// 	//while(TXFIFOSTA2 & txff);  //最后一个字节要等待UART发送结束
// }
