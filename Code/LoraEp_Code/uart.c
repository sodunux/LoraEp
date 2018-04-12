#include "init.h"
char volatile usart_temp[50];
/************************************************************************/
/* ��������Init_UART_PAD													*/
/* ����˵����UART�ܽų�ʼ���ӳ��� 							*/
/* ��ڲ�����N/A													*/
/* ���ڲ�����N/A													*/
/************************************************************************/
void Init_UART_PAD(void)
{

	PFFCR1 &= ~BIT4;
	PFFCR2 |= BIT4;//{PFFCR2[4]:PFFCR1[4]}=2'b10,alternate function 
	AFSELF |= afself4_txd0;//TXD0����	

	PFFCR1 &= ~BIT3;
	PFFCR2 |= BIT3;//{PFFCR2[3]:PFFCR1[3]}=2'b10,alternate function 
	AFSELF |= afself3_rxd0;//RXD0����	
		
	
	PAFCR1 &= ~BIT1;
	PAFCR2 |= BIT1;//{PAFCR2[1]:PAFCR1[1]}=2'b10,alternate function 
	AFSELA |= afsela1_txd1;//TXD1����	

	PAFCR1 &= ~BIT0;
	PAFCR2 |= BIT0;//{PAFCR2[0]:PAFCR1[0]}=2'b10,alternate function 
	AFSELA |= afsela0_rxd1;//RXD1����		
	
	PDFCR1 &= ~BIT2;
	PDFCR2 |= BIT2;//{PDFCR2[2]:PDFCR1[2]}=2'b10,alternate function 
	AFSELD &= ~BIT2;//RXD2����	

	PDFCR1 &= ~BIT3;
	PDFCR2 |= BIT3;//{PDFCR2[3]:PDFCR1[3]}=2'b10,alternate function 
	AFSELD &= ~BIT3;//TXD2����		
	
	PGFCR1 &= ~BIT6;
	PGFCR2 |= BIT6;//{PGFCR2[6]:PGFCR1[6]}=2'b10,alternate function 
	AFSELG |= afselg6_txd3;//TXD3����	

	PGFCR1 &= ~BIT5;
	PGFCR2 |= BIT5;//{PGFCR2[5]:PGFCR1[5]}=2'b10,alternate function 
	AFSELG |= afselg5_rxd3;//RXD3����			
}

/************************************************************************/
/* ��������Init_UART												    */
/* ����˵����uart ģ���ʼ���ӳ���      					    		*/
/* ��ڲ�����N/A													    */
/* ���ڲ�����N/A													    */
/************************************************************************/
void Init_UART(void)
{
	//Uart0	
//	RTXCON0 |= rxdflag | txdflag;
	RXSTA0 = rxen | pdsel_8_n;	//8λ���ݣ�����żУ��,����ʹ��
	TXSTA0 = txen;		//����ʹ�� txen
	
	SPBRGH0 = 0x03;	
 	SPBRGL0 = 0x40;//9600,8M MCLK
// 	SPBRGH0 = 0x06;	
//  	SPBRGL0 = 0x81;//9600,16M MCLK
// 	SPBRGH0 = 0x00;	
//  	SPBRGL0 = 0xcf;//9600,2M MCLK
// 	SPBRGH0 = 0x08;	
//  	SPBRGL0 = 0x22;//9600,20M MCLK
	
	RXSTA1 = rxen | pdsel_8_n;	//8λ���ݣ�����żУ��,����ʹ��
	TXSTA1 = txen;		//����ʹ�� txen
	
	SPBRGH1 = 0x03;	
 	SPBRGL1 = 0x40;//9600,8M MCLK	

//==== UART2����34401��������1200,2��ֹͣλ =====//
	RXSTA2 = rxen | pdsel_8_n;	//8λ���ݣ�����żУ��,����ʹ��
	TXSTA2 = txen | stopsel;		//����ʹ�� txen
	
	SPBRGH2 = 0x1A;	
 	SPBRGL2 = 0x0A;//1200,8M MCLK	


	RXSTA3 = rxen | pdsel_8_n;	//8λ���ݣ�����żУ��,����ʹ��
	TXSTA3 = txen;		//����ʹ�� txen
	
	SPBRGH3 = 0x03;	
 	SPBRGL3 = 0x40;//9600,8M MCLK		

	UARTIF = 0;
	UARTIE |= rxie3;		//�����ж�ʹ��
	RXSTA3 |= uarterrie;  //��������ж� 
	
	Commandh = 0x0;
	Commandl = 0x0;

	//uart3
	Uart3.pRecvPoint = 0;
	Uart3.pRecvLenth = 0;
	Uart3.pSendPoint = 0;
	Uart3.pSendLenth = 0;
	Uart3.RecvStatus = 0x0;
	Uart3.SendStatus = 0x0;
	Uart3.RecvOKFLAG = FALSE; 		//����������־״̬
	Uart3.bUpRecordFlag = FALSE;		//���������ϴ���־
	Uart3.pTmOutVal = 00;				//��������ֽڼ���ʱ100*4=400����
	Uart3.bCRC = 0;
	Uart_test_int = 0;
}


/****************************************************************/
/* ��������UartTx_0										*/
/* ����˵���������ֽڷ���				*/
/* ��ڲ�����value,����������											*/
/* ���ڲ�����N/A											*/
/* ʱ��:  														*/
/* ����:  														*/
/****************************************************************/												
void UartTx_0(INT8U Value)
{

			TXREG0 = Value;
			while(!(UARTIF & txif0));

}

/****************************************************************/
/* ��������UartTx_2									*/
/* ����˵���������ֽڷ���				*/
/* ��ڲ�����value,����������											*/
/* ���ڲ�����N/A											*/
/* ʱ��:  														*/
/* ����:  														*/
/****************************************************************/												
void UartTx_2(INT8U Value)
{

			TXREG2 = Value;
			while(!(UARTIF & txif2));

}

/****************************************************************/
/* ��������UartTx_3									*/
/* ����˵���������ֽڷ���				*/
/* ��ڲ�����value,����������											*/
/* ���ڲ�����N/A											*/
/* ʱ��:  														*/
/* ����:  														*/
/****************************************************************/												
void UartTx_3(INT8U Value)
{

			TXREG3 = Value;
			while(!(UARTIF & txif3));

}

/* ��������Uart2_SendStr										*/
/* ����˵���������ַ�������										*/
/* ��ڲ�����len,�������ݳ��ȣ�ptr,�������ݻ�������ַ     		*/
/* ���ڲ�����N/A												*/
/* ʱ��:  														*/
/* ����:  														*/
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
/* ��������UartRec_0										*/
/* ����˵���������ֽڽ���				*/
/* ��ڲ�����										*/
/* ���ڲ�����value											*/
/* ʱ��:  														*/
/* ����:  														*/
/****************************************************************/												
/*INT8U UartRec_0(void)
{
	INT8U i;
	while(!(UARTIF & rxif0));
	i = RXREG0;
	return i;
}
*/


/* ��������Uart0_SendStr										*/
/* ����˵���������ַ�������										*/
/* ��ڲ�����len,�������ݳ��ȣ�ptr,�������ݻ�������ַ     		*/
/* ���ڲ�����N/A												*/
/* ʱ��:  														*/
/* ����:  														*/
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
/* ��������UartTx_1										*/
/* ����˵���������ֽڷ���				*/
/* ��ڲ�����value,����������											*/
/* ���ڲ�����N/A											*/
/* ʱ��:  														*/
/* ����:  														*/
/****************************************************************/												
void UartTx_1(INT8U Value)
{

			TXREG1 = Value;
			while(!(UARTIF & txif1));

}

/****************************************************************/
/* ��������UartSendInfo									*/
/* ����˵���������ַ�������				*/
/* ��ڲ�����len,�������ݳ��ȣ�ptr,�������ݻ�������ַ     */
/* ���ڲ�����N/A											*/
/* ʱ��:  														*/
/* ����:  														*/
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
		while(!(UARTIF & rxif1)) //UART1�����жϱ�־λΪ1ʱ�˳�while
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
		while(!(UARTIF & rxif2)) //UART2�����жϱ�־λΪ1ʱ�˳�while
		{
			WDT_CLR();
		} 
		usart_temp[i++]= RXREG2;	
  }	
}



/****************************************************************/
/* ��������UartSendInfo									*/
/* ����˵���������ַ�������				*/
/* ��ڲ�����len,�������ݳ��ȣ�ptr,�������ݻ�������ַ     */
/* ���ڲ�����N/A											*/
/* ʱ��:  														*/
/* ����:  														*/
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

// 	RXSTA2 = rxen | pdsel_8_n;	//8λ���ݣ�����żУ��,����ʹ��
// 	TXSTA2 = txen;		//����ʹ�� txen
// 	
// 	SPBRGH2 = 0x03;	
//  	SPBRGL2 = 0x40;//9600,8M MCLK
// 	
// }

void Uart0_TX_DMA(INT8U near *ptr, INT8U len)
{

	DMA_Start(CH0,(INT16U)ptr,len,uart0_tx);
	while(!(UARTIF & txif0));
	//while(TXFIFOSTA0 & txff);  //���һ���ֽ�Ҫ�ȴ�UART���ͽ���
}

void Uart0_RX_DMA(INT8U near *ptr, INT8U len)
{
	DMA_Start(CH0,(INT16U)ptr,len,uart0_rx);
}

// void Uart2_TX_DMA(INT8U near *ptr, INT8U len)
// {
// 	DMA_Start(CH0,ptr,len,uart2_tx);
// 	while(!(UARTIF & txif2));
// 	//while(TXFIFOSTA2 & txff);  //���һ���ֽ�Ҫ�ȴ�UART���ͽ���
// }
