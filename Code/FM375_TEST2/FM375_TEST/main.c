/************************************************************************/
/* 	ϵͳ������  									                */
/* 	��Ҫ����:															*/
/* 				                                                */
/*	Ӳ��ƽ̨��	 													  	*/
/*			FM375	                        	*/
/* 	����:xudayong													*/
/* 	����ʱ��:2016��03��30��												*/
/************************************************************************/

#include "init.h"
 struct  UartRunParam_struct near Uart3;		//����ͨ�����ݽṹ
 INT8U TX_Buf[TX_BUF_SIZE] _at_ 0x100;			//ͨ�Ž��շ��ͻ����� 100BYTES
 INT8U Commandh;
 INT8U Commandl;	
 INT8U Uart_test_int;//����UART��


 INT8U const FMSHINFO[] = {0x55, 0xAA, 0xFF, 0x00, 0x03, 0x07, 0x05};		//��Ȩ��Ϣ

//		ͨ����ʱ����
void delay(INT16U dlength) reentrant    //1ms
{
	INT16U i,j;
	for (i=0;i<dlength;i++)
	{
 		WDT_CLR();
		for (j=0;j<0x2a8;j++)
		{
			_nop_();
			WDT_CLR();
		}
	}
}

//		ͨ����ʱ����
void delay_ms(INT32U dlength) reentrant    //1ms
{
	INT16U m,n;
	unsigned long cnt;
	
	cnt = 33*dlength - 4;//���ϼ���Ŀ��ֵ��dlength����;����Ӧ�ü�1�������ڳ�����Ҳ������һ��ʱ�䣬���Լ�2��timer�ټ�3��32k����
	m = cnt/65535;
	n = cnt%65535;
	
	PERICLK_CTRL2 |= lptim_rcken;  //����ʱ��ʹ��
//	LSCLKSEL |= lptim_fcken;   //����Դѡ��ϵͳʱ��ʱ����ʹ�ܴ�λ
	LPTCTRL = 0;
	LPTCFG0 = 0;
	LPTCFG1 = 0;
	LPTCFG0 |= lptim_cnt_src_lsclk | cnt_clk_div1;  //����ʱ��LSCLK,1��Ƶ
	LPTCFG1 |= word_mode_wave_out_timer | cnt_mode_continue | pos_pol_out;
	LPTIMIF = 0;
	
	while(m)  //����65535�Ĳ���,ÿ��
	{
		m--;
		TARGETL = 0xff;
		TARGETH = 0xff;	
		LPTCTRL |= lpten;	//��������
		while(!(LPTIMIF&ovif)){WDT_CLR();};	//�ȴ�ʱ�䵽��
	}
	LPTCTRL = 0;//ֹͣ����
	
	
	TARGETL = (unsigned char)n;
	TARGETH = (unsigned char)(n>>8);
	LPTIMIF = 0;
	LPTCTRL |= lpten;		
	while(!(LPTIMIF&ovif)){WDT_CLR();};	  //�ȴ�ʱ�䵽��
	
	LPTCTRL = 0;//ֹͣ����
}

//		ͨ����ʱ����
void delay_us(INT16U dlength) reentrant    //1us
{
	INT16U i;
	for (i=0;i<dlength;i++)
	{
 		WDT_CLR();
	}
}

void main(void)
{
	
/*	
// 							#pragma asm
// 								EDATALEN2	EQU	0010H
// 								EDATASTART EQU 0000H
// 								
// 								IF EDATALEN2 <> 0

// 											MOV	WR8,#EDATALEN2 - 1 //��ַ����
// 											MOV WR14,#EDATASTART
// 									
// 											MOV A,#055H
// 	
// 									EDATALOOP2:	MOV	@WR8,R11   //R9:����ַ��λд���ַ��
// 											DEC	WR8,#1
// 											CMP WR8,WR14
// 											JNE	EDATALOOP2
// 											
// 											
// 									MOV WR2,#5000H
// 									DELAY10MS:	MOV      A,#05AH                        ; A=R11
// 															MOV      DPTR,#0C0H
// 															MOVX     @DPTR,A                        ; A=R11    //�幷
// 															DEC WR2,#1
// 															JNE	DELAY10MS
// 															
// 															LJMP READRAM
// 															
// 									READRAM:MOV	WR8,#EDATALEN2 - 1 //��ַ����
// 											MOV WR14,#EDATASTART
// 									EDATALOOP1:	MOV	R5,@WR8    //������Ӧ��ַ������
// 											CMP R5,R9   //�Ƚ϶���������
// 											JNE ERROR_PRO  //�������ȣ�������������
// 											DEC	WR8,#1
// 											CMP WR8,WR14
// 											JNE	EDATALOOP1
// 											
// 											LJMP NOMAL
// 	               
// 											ERROR_PRO:MOV      A,#01H                        ; A=R11
// 												MOV      DPTR,#0100H
// 												MOVX     @DPTR,A                        ; A=R11
// 												MOV      DPTR,#0112H
// 												MOVX     @DPTR,A                        ; A=R11
// 												MOV      DPTR,#0122H
// 												MOVX     @DPTR,A                         ; A=R11   //�ø�
// 												MOV      A,#00H                        ; A=R11
// 												MOV      DPTR,#0122H
// 												MOVX     @DPTR,A                         ; A=R11  //�õ�
// 												MOV      A,#01H                        ; A=R11
// 												MOV      DPTR,#0122H
// 												MOVX     @DPTR,A                         ; A=R11   //�ø�
// 												
// 												LJMP ENDASM
// 												
// 											NOMAL:MOV      A,#01H                        ; A=R11
// 												MOV      DPTR,#0100H
// 												MOVX     @DPTR,A                        ; A=R11
// 												MOV      DPTR,#0112H
// 												MOVX     @DPTR,A                        ; A=R11   
// 												MOV      DPTR,#0122H
// 												MOVX     @DPTR,A                         ; A=R11  //�ø�
// 												MOV      A,#00H                        ; A=R11
// 												MOV      DPTR,#0122H
// 												MOVX     @DPTR,A                         ; A=R11   //�õ�
// 												
// 											ENDASM: NOP
// 								ENDIF	
// 								
// 							#pragma endasm	
	
	
// 	INT16U volatile mtd,mrd;
//  	INT16U volatile *p,tmp;
// 	INT16U volatile buffer[2];
// 	buffer[0] = 0x55aa;
// 	buffer[1] = 0x1122;
// 	mtd = &buffer[0];
// 	p = &buffer[1];
// 	mrd  = p;
// 	*p = 0x3344;
// 	mrd = p - mtd;
*/

	INIT_CLK();
	delay(10);
	Init_SYS();   //���ÿ��Ź����жϵ�

  Init_IO_PAD();
		
	Init_UART_PAD();
	Init_UART();


//	PCFCR1 &= ~BIT7;
//	PCFCR2 |= BIT7;//{PCFCR2[7]:PCFCR1[7]}=2'b10,alternate function 
//	AFSELC |= afselc7_t3in0;//t3in0����	
	
// 	UartTx_3(PCFCR2);
// 	UartTx_3(PCFCR1);
// 	UartTx_3(PCPPEN);

//PF7��������
// 	PFPUEN |= BIT7;
// 		UartTx_3(PFFCR2);
// 		UartTx_3(PFFCR1);
// 		UartTx_3(PFPPEN);
// 		UartTx_3(PFPUEN);


// 	LSCLKSEL &= ~BIT1;//�ر� SVD ����ʱ�ӣ� SVD_CLKEN=0;
// 	
// 	UartTx_3(LSCLKSEL);
	UartTx_3(AUTH);//��ȡоƬȨ��ģʽ�Ĵ���ֵ������Ƿ�Ϊ�޾���ģʽ
	
	
	
	
	/*******************************\
		BOR���ԣ�BOR�µ絵λ����
	\*******************************/
	//PDRCTL = 0X00;
	//BORCTL=0x0006;      //11��,  1.75V
	//BORCTL=0x0002;    //01��   1.6V
	//BORCTL=0x0004;    //10��   1.65V
	//BORCTL=0x0006;      //11��,  1.75V
    
 	/*******************************\
		PDR���ԣ�PDR�µ絵λ����
	\*******************************/
	//BORCTL |= off_bor;
	//PDRCTL=0x0001;	    //00��   1.5V
	//PDRCTL=0x0003;    //01��   1.25V
	//PDRCTL=0x0005;    //10��   1.35V
	//PDRCTL=0x0007;    //11��,  1.4V

 	//***********************  clk output ***************************//		
			//FOUT0_SEL = fout0_coreclk_div64;  //���ʱ���ź�
			FOUT0_SEL = fout0_lsclk;
			PGFCR1 &= ~BIT4;
			PGFCR2 |= BIT4;//{PGFCR2[4]:PGFCR1[4]}=2'b10				
			AFSELG &= ~BIT4; //0: FOUT0
				
			FOUT1_SEL = fout1_rchf;
			PCFCR1 &= ~BIT3;
			PCFCR2 |= BIT3;//{PCFCR2[3]:PCFCR1[3]}=2'b10				
			AFSELC |= BIT3; //1: FOUT1							
				


	UartTx_3(RCHFADJ);
	UartTx_3(FDETIF);
	UartTx_3(RSTFLAG);
	UartTx_3(MDCSTA);

 	UartSendInfo(sizeof(FMSHINFO)/sizeof(*FMSHINFO),FMSHINFO);	//��λ�󴮿ڷ�����ʼ��Ϣ
	delay(1);

	while(1)
	{
  	WDT_CLR();
		
		if((Uart3.RecvOKFLAG) == true)
		{
			Uart_Proc();			        //Uartͨ�Ŵ���
		}
	}
	
}