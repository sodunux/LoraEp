/************************************************************************/
/* 	系统主函数  									                */
/* 	主要功能:															*/
/* 				                                                */
/*	硬件平台：	 													  	*/
/*			FM375	                        	*/
/* 	编制:xudayong													*/
/* 	编制时间:2016年03月30日												*/
/************************************************************************/

#include "init.h"
 struct  UartRunParam_struct near Uart3;		//串行通信数据结构
 INT8U TX_Buf[TX_BUF_SIZE] _at_ 0x100;			//通信接收发送缓冲区 100BYTES
 INT8U Commandh;
 INT8U Commandl;	
 INT8U Uart_test_int;//测试UART用


 INT8U const FMSHINFO[] = {0x55, 0xAA, 0xFF, 0x00, 0x03, 0x07, 0x05};		//版权信息

//		通用延时程序
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

//		通用延时程序
void delay_ms(INT32U dlength) reentrant    //1ms
{
	INT16U m,n;
	unsigned long cnt;
	
	cnt = 33*dlength - 4;//向上计数目标值，dlength毫秒;本来应该减1，但由于程序处理也会消耗一点时间，所以减2，timer少计3个32k周期
	m = cnt/65535;
	n = cnt%65535;
	
	PERICLK_CTRL2 |= lptim_rcken;  //总线时钟使能
//	LSCLKSEL |= lptim_fcken;   //计数源选择系统时钟时，需使能此位
	LPTCTRL = 0;
	LPTCFG0 = 0;
	LPTCFG1 = 0;
	LPTCFG0 |= lptim_cnt_src_lsclk | cnt_clk_div1;  //计数时钟LSCLK,1分频
	LPTCFG1 |= word_mode_wave_out_timer | cnt_mode_continue | pos_pol_out;
	LPTIMIF = 0;
	
	while(m)  //超过65535的部分,每次
	{
		m--;
		TARGETL = 0xff;
		TARGETH = 0xff;	
		LPTCTRL |= lpten;	//启动计数
		while(!(LPTIMIF&ovif)){WDT_CLR();};	//等待时间到达
	}
	LPTCTRL = 0;//停止计数
	
	
	TARGETL = (unsigned char)n;
	TARGETH = (unsigned char)(n>>8);
	LPTIMIF = 0;
	LPTCTRL |= lpten;		
	while(!(LPTIMIF&ovif)){WDT_CLR();};	  //等待时间到达
	
	LPTCTRL = 0;//停止计数
}

//		通用延时程序
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

// 											MOV	WR8,#EDATALEN2 - 1 //地址数据
// 											MOV WR14,#EDATASTART
// 									
// 											MOV A,#055H
// 	
// 									EDATALOOP2:	MOV	@WR8,R11   //R9:将地址低位写入地址中
// 											DEC	WR8,#1
// 											CMP WR8,WR14
// 											JNE	EDATALOOP2
// 											
// 											
// 									MOV WR2,#5000H
// 									DELAY10MS:	MOV      A,#05AH                        ; A=R11
// 															MOV      DPTR,#0C0H
// 															MOVX     @DPTR,A                        ; A=R11    //清狗
// 															DEC WR2,#1
// 															JNE	DELAY10MS
// 															
// 															LJMP READRAM
// 															
// 									READRAM:MOV	WR8,#EDATALEN2 - 1 //地址数据
// 											MOV WR14,#EDATASTART
// 									EDATALOOP1:	MOV	R5,@WR8    //读出相应地址的数据
// 											CMP R5,R9   //比较读出的数据
// 											JNE ERROR_PRO  //如果不相等，进入错误处理程序
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
// 												MOVX     @DPTR,A                         ; A=R11   //置高
// 												MOV      A,#00H                        ; A=R11
// 												MOV      DPTR,#0122H
// 												MOVX     @DPTR,A                         ; A=R11  //置低
// 												MOV      A,#01H                        ; A=R11
// 												MOV      DPTR,#0122H
// 												MOVX     @DPTR,A                         ; A=R11   //置高
// 												
// 												LJMP ENDASM
// 												
// 											NOMAL:MOV      A,#01H                        ; A=R11
// 												MOV      DPTR,#0100H
// 												MOVX     @DPTR,A                        ; A=R11
// 												MOV      DPTR,#0112H
// 												MOVX     @DPTR,A                        ; A=R11   
// 												MOV      DPTR,#0122H
// 												MOVX     @DPTR,A                         ; A=R11  //置高
// 												MOV      A,#00H                        ; A=R11
// 												MOV      DPTR,#0122H
// 												MOVX     @DPTR,A                         ; A=R11   //置低
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
	Init_SYS();   //设置看门狗、中断等

  Init_IO_PAD();
		
	Init_UART_PAD();
	Init_UART();


//	PCFCR1 &= ~BIT7;
//	PCFCR2 |= BIT7;//{PCFCR2[7]:PCFCR1[7]}=2'b10,alternate function 
//	AFSELC |= afselc7_t3in0;//t3in0功能	
	
// 	UartTx_3(PCFCR2);
// 	UartTx_3(PCFCR1);
// 	UartTx_3(PCPPEN);

//PF7进行上拉
// 	PFPUEN |= BIT7;
// 		UartTx_3(PFFCR2);
// 		UartTx_3(PFFCR1);
// 		UartTx_3(PFPPEN);
// 		UartTx_3(PFPUEN);


// 	LSCLKSEL &= ~BIT1;//关闭 SVD 工作时钟， SVD_CLKEN=0;
// 	
// 	UartTx_3(LSCLKSEL);
	UartTx_3(AUTH);//读取芯片权限模式寄存器值，检查是否为无晶体模式
	
	
	
	
	/*******************************\
		BOR测试，BOR下电档位设置
	\*******************************/
	//PDRCTL = 0X00;
	//BORCTL=0x0006;      //11档,  1.75V
	//BORCTL=0x0002;    //01档   1.6V
	//BORCTL=0x0004;    //10档   1.65V
	//BORCTL=0x0006;      //11档,  1.75V
    
 	/*******************************\
		PDR测试，PDR下电档位设置
	\*******************************/
	//BORCTL |= off_bor;
	//PDRCTL=0x0001;	    //00档   1.5V
	//PDRCTL=0x0003;    //01档   1.25V
	//PDRCTL=0x0005;    //10档   1.35V
	//PDRCTL=0x0007;    //11档,  1.4V

 	//***********************  clk output ***************************//		
			//FOUT0_SEL = fout0_coreclk_div64;  //输出时钟信号
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

 	UartSendInfo(sizeof(FMSHINFO)/sizeof(*FMSHINFO),FMSHINFO);	//复位后串口发出初始信息
	delay(1);

	while(1)
	{
  	WDT_CLR();
		
		if((Uart3.RecvOKFLAG) == true)
		{
			Uart_Proc();			        //Uart通信处理
		}
	}
	
}