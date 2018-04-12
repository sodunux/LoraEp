#include "init.h"
#include "fm375_uart.h"



/****************************************************************/
/* ��������CalFrameBCC										*/
/* ����˵�������㷢�����ݻ�����Uart.TX_Buf��BCCУ����			*/
/* ��ڲ�����Uart.TX_Buf,Uart.TX_Buf[2]=len 					*/
/* ���ڲ�����У��� 									*/
/* ʱ��:														*/
/* ����:														*/
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
/* ��������Comm_Proc										*/
/* ����˵��������ͨ�Ž�֡������				*/
/* ��ڲ�����N/A											*/
/* ���ڲ�����N/A											*/
/* ʱ��:  														*/
/* ����:  														*/
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
	
//�������ͷ��

	TX_Buf[0] = 0x68;				//HEAD1
	TX_Buf[1] = UART_ADDRH;		//��ַ�ֽ�H  
	TX_Buf[2] = UART_ADDRL;		//��ַ�ֽ�L
	TX_Buf[3] = 0x97;		 		//HEAD2,��������ʼ��־
//	TX_Buf[4] = 0x00;				//����������
	TX_Buf[5] = Commandh;
	TX_Buf[6] = Commandl;
	
//�������β��	
	CalFrameBCC();
	
}

/****************************************************************/
/* ��������Uart_Proc											*/
/* ����˵����Uart����ͨ�Ž�֡����					*/
/* ��ڲ�����N/A											*/
/* ���ڲ�����N/A											*/
/* ʱ��:  														*/
/* ����:  														*/
/****************************************************************/
void Uart_Proc(void)
{	  

	Comm_Proc();						//ͨ�������

	switch(UART_ADDRL)
	{
		case 0:

			Uart3.SendStatus = 0x0;     //Ϊ��ֹSM3����ı�˱������˴��ٴζ����ʼ��
			Uart3.RecvOKFLAG = FALSE; 			//����������־״̬
			Uart3.pRecvPoint = 0;					//����ָ������
			Uart3.RecvStatus = 0;					//����״̬��λΪ��ʼ����װ̬
			if (Uart3.SendStatus == 0x00)
			{

				//������������
				Uart3.SendStatus = 0xFF;				//�����ͱ�־
				Uart3.pSendPoint = 1;
				Uart3.bSendOverFlag = FALSE;			//�巢�����ݽ�����־

				Uart3.pSendLenth = TX_Buf[4]+9;
//Uart3.pSendLenth = (TX_Buf[3]<<8)+TX_Buf[4]+9;
				ACC = 0x68;             //֡ͷ
				TXREG3 = ACC;						//Uart0���ڷ�
				
// 				while(!(UARTIF & txif3));
				UARTIE |= txie3;			//ENABLE TX INTERRUPT
			}			
		break;		
	}
}