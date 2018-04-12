/************************************************************************/
/* 	寄存器位定义头文件									                */
/* 	主要功能:															*/
/* 		1.中断处理程序	                            */
/*	硬件平台：	 													  	*/
/*		FM340			                        	*/
/* 	编制:xudayong														*/
/* 	编制时间:2014年10月30日												*/
/************************************************************************/

#include "init.h"

INT8U near flag;
INT8U near dma_end;
INT8U near dma_half;
INT8U near Timer0_Test_Cnt;
INT8U near Timer0_Test_Statues;
INT8U near Timer1_Test_Cnt;
INT8U near Timer1_Test_Statues;
INT8U near io_if;
INT8U near flash_op_end;
INT8U near ram_par_err;
/************************************************************************/
/* 函数名：Int0_ISR													    */
/* 功能说明：外部中断处理程序   						                */
/* 入口参数：N/A													    */
/* 出口参数：N/A													    */
/* 时间:  																*/
/* 作者:  	         													*/
/************************************************************************/
void Int0_ISR (void) interrupt 0 using 1
{

		TCON &= ~BIT1;
 	//IE0 = 0;//边沿触发IE0自动清零，电平触发软件清零
	
	//------------------- LVDINT ----------------------//
	INIT_CLK();
	Init_IO_PAD();
	Init_UART_PAD();
	Init_UART();
	
	if(LVDSTAT & lvdpu_if)
	{
 		UartTx_3(LVDSTAT);
		LVDSTAT &= ~lvdpu_if;

	}
	if(LVDSTAT & lvdpd_if)
	{
 		UartTx_3(LVDSTAT);
		LVDSTAT &= ~lvdpd_if;

	}		
	
	//------------------- lptim ----------------------//	

		if((LPTIMIF & trig_if) && (LPTIMIE & trig_ie))
		{			
			LPTIMIF &= ~trig_if;
		}
		if((LPTIMIF & ovif) && (LPTIMIE & ovie))
		{			
			LP_tim_cnt++;
			LPTIMIF &= ~ovif;
		}	
		if((LPTIMIF & compif) && (LPTIMIE & compie))
		{			
			LP_tim_cnt++;
			LPTIMIF &= ~compif;
		}
	
	//------------------- rtc ----------------------//	
	if((RTCIF2 & 0xff)&&(RTCIE2 & 0xff)) 
	{		
  		
		
		if((RTCIF2 & secif)&&(RTCIE2 & secie))
		{
			UartTx_3(RTCIF2);
			RTCIF2 &=(~secif); 	
 			rtc_sec_if=1;

		}
		if((RTCIF2 & minif)&&(RTCIE2 & minie))
		{
			
			RTCIF2 &= (~minif); 

		}	
		if((RTCIF2 & hourif)&&(RTCIE2 & hourie))
		{
			RTCIF2 &= (~hourif);	

		}	
		if((RTCIF2 & dayif)&&(RTCIE2 & dayie))	
		{
			RTCIF2 &= (~dayif);  

		}	
		if((RTCIF2 & hz2_if)&&(RTCIE2 & hz2_ie))
		{		
			RTCIF2 &= (~hz2_if); 

		}	
		if((RTCIF2 & hz4_if)&&(RTCIE2 & hz4_ie))
		{		
			RTCIF2 &= (~hz4_if); 

		}	
		if((RTCIF2 & hz8_if)&&(RTCIE2 & hz8_ie))
		{		
			RTCIF2 &= (~hz8_if); 		

				
		}	
		if((RTCIF2 & hz16_if)&&(RTCIE2 & hz16_ie))
		{		
			RTCIF2 &= (~hz16_if);

		}	
	}
	
	if((RTCIF1 & 0x1f)&&(RTCIE1 & 0x1f)) 
	{		
		if((RTCIF1 & adj128_if)&&(RTCIE1 & adj128_ie))	
		{
			RTCIF1 &= (~adj128_if);  

		}	
		if((RTCIF1 & alarm_if)&&(RTCIE1 & alarm_ie))
		{		
			RTCIF1 &= (~alarm_if); 

		}	
		if((RTCIF1 & hz1k_if)&&(RTCIE1 & hz1k_ie))
		{		
			RTCIF1 &= (~hz1k_if); 

		}	
		if((RTCIF1 & hz256_if)&&(RTCIE1 & hz256_ie))
		{		
			RTCIF1 &= (~hz256_if); 		

				
		}	
		if((RTCIF1 & hz64_if)&&(RTCIE1 & hz64_ie))
		{		
			RTCIF1 &= (~hz64_if);

		}			
	}

	
}

/************************************************************************/
/* 函数名：Timer0_ISR													*/
/* 功能说明：Timer0中断处理程序						                    */
/* 入口参数：N/A													    */
/* 出口参数：N/A													    */
/* 时间:  																*/
/* 作者: 													            */
/************************************************************************/
void Timer0_ISR (void) interrupt 1 using 1
{
	TF0 = 0;
//====定时器初值设定====================================	
	TR0 = 0;					//关闭定时器
	TH0 = 0xF2;				//定时寄存器赋值
	TL0 = 0xFA;				//定时寄存器赋值    8M主时钟下5ms
	TR0 = 1;					//开启定时器


	//======各定时器测试延时，计数因子1ms==================

	if(Timer0_Test_Cnt!=0)
	{
		Timer0_Test_Cnt--;
		if(Timer0_Test_Cnt==0)
		{
			Timer0_Test_Statues=1;
		}
	}
}

/************************************************************************/
/* 函数名：Int1_ISR													    */
/* 功能说明：外部中断1处理程序						                    */
/* 入口参数：N/A													    */
/* 出口参数：N/A													    */
/* 时间:  																*/
/* 作者: 													            */
/************************************************************************/
void Int1_ISR (void) interrupt 2 using 1
{

	TCON &= ~BIT3;
 	//IE1 = 0;//边沿触发IE1自动清零，电平触发软件清零

	if(EXTILIF & BIT7)
	{
		INIT_CLK();
		Init_IO_PAD();
		Init_UART_PAD();
		Init_UART();				
		
		UartTx_3(EXTILIF);
		EXTILIF &= ~BIT7;	
		EX1 = 0;
		io_if = 1;			

	}

	if(EXTIHIF & BIT6)
	{
		INIT_CLK();
		Init_IO_PAD();
		Init_UART_PAD();
		Init_UART();				
		
		UartTx_3(EXTIHIF);
		EXTIHIF &= ~BIT6;	
		EX1 = 0;
		io_if = 1;			

	}	
	
	//===== 中断嵌套验证用 ========//
	
	if(EXTIHIF & BIT1)
	{
		EXTIHIF &= ~BIT1;	
		
		RTCIF2 = 0;
		RTCIE2 |= secie;  //使能秒中断	
		
		rtc_sec_if = 0;
		while(!(rtc_sec_if)){WDT_CLR();}  //秒中断产生
		RTCIE2 = 0;

		delay(1);
		UartTx_3(0x55);
		UartTx_3(0xaa);
		io_if = 1;
	}	
	
	
//====== ADC静态测试用 =======//
	if(EXTIHIF & BIT0)
	{
		EXTIHIF &= ~BIT0;	
		io_if = 1;
	}		
	
		
	TCON &= ~BIT3;
}

/************************************************************************/
/* 函数名：Timer1_ISR													*/
/* 功能说明：Timer1中断处理程序						                    */
/* 入口参数：N/A													    */
/* 出口参数：N/A													    */
/* 时间:  																*/
/* 作者: 													            */
/************************************************************************/
void Timer1_ISR (void) interrupt 3 using 1
{
	TF1 = 0;
//====定时器初值设定====================================	

 	TR1 = 0;					//关闭定时器
	TH1 = 0xF2;				//定时寄存器赋值
	TL1 = 0xFA;				//定时寄存器赋值
	TR1 = 1;					//开启定时器


	//======各定时器测试延时，计数因子5ms==================

	if(Timer1_Test_Cnt!=0)
	{
		Timer1_Test_Cnt--;
		if(Timer1_Test_Cnt==0)
		{
			Timer1_Test_Statues=1;
		}
	}

}

/************************************************************************/
/* 函数名：es_ISR													*/
/* 功能说明：es中断处理程序						*/
/* 入口参数：N/A													*/
/* 出口参数：N/A													*/
/* 时间:  																*/
/* 作者: 													*/
/************************************************************************/
void es_ISR (void) interrupt 4 using 1
{
	
}

/************************************************************************/
/* 函数名：Timer2_ISR													*/
/* 功能说明：Timer2中断处理程序						                    */
/* 入口参数：N/A													    */
/* 出口参数：N/A													    */
/* 时间:  																*/
/* 作者: 													            */
/************************************************************************/
void Timer2_ISR (void) interrupt 5 using 1
{
	TF2 = 0;
	if(EXF2==1)
	{
		T2_Cap_Buf[T2_Cap_Cnt++] = RCAP2H;
		T2_Cap_Buf[T2_Cap_Cnt++] = RCAP2L;			
	}
	EXF2 = 0;
	
//	RCAP2H = 0xff;
//	RCAP2L = 0xfe;

}
/************************************************************************/
/* 函数名：PCA_ISR													*/
/* 功能说明：Timer2中断处理程序						                    */
/* 入口参数：N/A													    */
/* 出口参数：N/A													    */
/* 时间:  																*/
/* 作者: 													            */
/************************************************************************/
void PCA_ISR (void) interrupt 6 using 1
{
/*	INT8U i;
	if(CF ==1)									//计数中断							
	{
		CCON &=B1011_1111;				//关闭PCA Time Module
		CH= 0xF8;
		CL= 0xF0;
	//	GPIO8 ^= B0000_0001;
		PCA_Cmp_Cnt++;
		CF = 0;	
		CCON |=B0100_0000;				//启动PCA Module
				for(i=0;i<9;i++)
			IO1DATA ^= BIT0;
	}
	


	if(CCF0 ==1)								//module0 比较捕中断
	{
	//	GPIO8 ^=B0000_0010;
		
		PCA_Cap_Buf[PCA_Cap_Cnt++] = CCAP0H;
		PCA_Cap_Buf[PCA_Cap_Cnt++] = CCAP0L;	
	//	PCA_Cap_Cnt++;
	//	TXREG0 = CCAP0H;
	//	TXREG0 = CCAP0L;
		
		CCF0 = 0;
	
	}


	if(CCF1 ==1)							//module1 比较捕中断
	{
	//	GPIO8 ^=B0000_0100;
		PCA_Cap_Buf[PCA_Cap_Cnt++] = CCAP1H;
		PCA_Cap_Buf[PCA_Cap_Cnt++] = CCAP1L;	
		CCF1 = 0;	
	}


	
	if(CCF2 ==1)								//module2 比较捕中断
	{
	//	GPIO8 ^=B0000_1000;
		PCA_Cap_Buf[PCA_Cap_Cnt++] = CCAP2H;
		PCA_Cap_Buf[PCA_Cap_Cnt++] = CCAP2L;		
		CCF2 = 0;	
	}


	if(CCF3 ==1)								//module3 比较捕中断
	{
	//	GPIO8 ^=B0001_0000;
		PCA_Cap_Buf[PCA_Cap_Cnt++] = CCAP3H;
		PCA_Cap_Buf[PCA_Cap_Cnt++] = CCAP3L;		
		CCF3 = 0;	
	}*/
}
/************************************************************************/
/* 函数名：nmi_ISR													*/
/* 功能说明：nmi中断处理程序						*/
/* 入口参数：N/A													*/
/* 出口参数：N/A													*/
/* 时间:  																*/
/* 作者: 													*/
/************************************************************************/
void nmi_ISR (void) interrupt 7 using 1
{
	unsigned char i;
	//assign nmisrc_dout = {8{nmisrc_cs}} & {5'b0,ramparity_eif,lfdet_int,pmu_int};	
// 	PERICLK_CTRL2 = 0xff;
// 			PAFCR2 &= 0Xfe;
// 			PAPPEN |= BIT0;//使能PA0的推挽功能
// 			PAFCR1 |= BIT0;//{PAFCR2[0]:PAFCR1[0]}=2'b01,使能PA0输出
			PADATA |= BIT0;	
	
// 	INIT_CLK();
// 	Init_IO_PAD();
// 	Init_UART_PAD();
// 	Init_UART();
// 	UartTx_3(NMIFLAG);
// 	UartTx_3(LPRUNERR);
// 	UartTx_3(PMU_WKSRC);

 	if(NMIFLAG & lfdet_int)
 	{ 	
	 	if(FDETIF & lfdet_if)
		{
			FDETIE = 0;
			for(i=0;i<3;i++)
			{
				while(!(FDETIF&lfdet_out_b))   //软件清零前应查询LFDET_OUT_B，确保其为1，否则将无法清零LFDET_IF。且确保3次都为1
				{}
				delay(10);
			}
			
			FDETIF &= ~lfdet_if;
		}	

 	}
 	if(NMIFLAG & pmu_int)
 	{ 	
		if(LPRUNERR & lp_run_err)
		{
 			SET_MCLK(LSCLK,0,0);   //进入LPRUN模式时不是LSCLK会产生此中断，设置为LSCLK后，中断清零
			LPRUNERR = 0;
		}
			
		if(PMU_WKSRC & B0000_1111)
		{
			PMU_WKSRC = 0;
		}
			//PMU_WKSRC &= ~wkflag_clr_n;

 	}
 	
	NMIFLAG = 0;
// PADATA ^= BIT1;
}

/************************************************************************/
/* 函数名：Eint0_ISR													*/
/* 功能说明：intextra_n0中断处理程序						*/
/* 入口参数：N/A													*/
/* 出口参数：N/A													*/
/* 时间:  																*/ 
/* 作者: 													*/
/************************************************************************/
void Eint0_ISR (void) interrupt 8 using 1
{

	AIF0 = 0;

	if(RAMPARITY & ramparity_if)
	{
		UartTx_3(0X99);
		UartTx_3(RAMPARITY);
		RAMPARITY &= ~ramparity_if;
		ram_par_err = 1;
	}	

}
/************************************************************************/
/* 函数名：Eint1_ISR													*/
/* 功能说明：intextra_n1中断处理程序						*/
/* 入口参数：N/A													*/
/* 出口参数：N/A													*/
/* 时间:  																*/
/* 作者: 													*/
/************************************************************************/
void Eint1_ISR (void) interrupt 9 using 1
{
	
	ADCIF = 0;
	AIF1 = 0;
	adc_done = 1;
}


/************************************************************************/
/* 函数名：Eint2_ISR													*/
/* 功能说明：intextra_n2中断处理程序						*/
/* 入口参数：N/A													*/
/* 出口参数：N/A													*/
/* 时间:  																*/
/* 作者: 													*/
/************************************************************************/
void Eint2_ISR (void) interrupt 10 using 1
{
//	uchar volatile rdata;
	INT8U comm_buff;
	AIF2 = 0;
//  PADATA ^= BIT2;
	//Uart3
	if(RXSTA3 & oerr)   //oerr == 1
	{
		RXSTA3 &= (~oerr);//clear oerr
		UartTx_3(0x90);
	}
	else if(RXSTA3 & ferr)   //ferr == 1
	{	
		RXSTA3 &= (~ferr);//clear ferr
		UartTx_3(0x91);
	}
	else if(RXSTA3 & perr)   //perr == 1
	{
		RXSTA3 &= (~perr);//clear perr
		UartTx_3(0x92);
	}
	
	if(UARTIF & rxif3)
	{			
		comm_buff = RXREG3;					   	//从UART数据寄存器中取接收数据字节
		if(Uart_test_int == 0x10)
		{	
			ACC = comm_buff;
			TXREG3 = ACC;
			while(!(UARTIF & txif3));
		}
		else
		{
			switch (Uart3.RecvStatus)
			{
					case 0:								//包头标准0X68
							if( comm_buff ==0x68)
							{
									Uart3.RecvStatus = 1;
							} 						
						break;
		
					case 1:								//地址字节,用于多机通讯
							if( comm_buff == UART_ADDRH)
							{
									Uart3.RecvStatus = 2;
							}
							else
							{
								Uart3.RecvStatus = 0;
							}
					break;

					case 2:								//地址字节,用于多机通讯
							if( comm_buff == 0x00)
							{
									//UART_ADDRL =0x00;
									Uart3.RecvStatus = 3;
							 }
							else
							{
								Uart3.RecvStatus = 0;
							}
						break;

					case 3:								//包头标准0X97
							if( comm_buff ==0x97)
							{
								Uart3.RecvStatus = 4;
								Uart3.pRecvPoint = 0;
							}
							else
							{
								Uart3.RecvStatus = 0;
							}
						 break;
			
					 case 4:								//数据包长度
								Uart3.pRecvLenth = comm_buff ;
								Uart3.RecvStatus = 5;
							break;

					case 5:
							Commandh = comm_buff;
							Uart3.RecvStatus = 6;
						break;

					case 6:
						Commandl = comm_buff;
						if(Uart3.pRecvLenth == 0)
						{
							Uart3.RecvStatus = 8;
						}
						else 
						{
							Uart3.RecvStatus = 7;
						}
					break;
														
					case 7:										//数据
						TX_Buf[Uart3.pRecvPoint]=comm_buff;
						Uart3.pRecvPoint++;							
						if(Uart3.pRecvLenth  == Uart3.pRecvPoint)
						{
							Uart3.RecvStatus = 8;
						}
						break;

					case 8:										//校验和
						Uart3.bCRC = comm_buff;
						Uart3.RecvStatus = 9;
						break;

					case 9:	 									//包尾标志，接受结束
						if(comm_buff == 0x16)
						{
							Uart3.RecvOKFLAG = true;
						}			       
						Uart3.RecvStatus = 0;
						break;

					default:
							Uart3.RecvStatus = 0;
					 break;   
			}
	}

	}
	else if((UARTIF & txif3)&&(UARTIE & txie3))
	{
		
		if(Uart3.pSendPoint < Uart3.pSendLenth)		//发送长度检查
		{
			ACC = TX_Buf[Uart3.pSendPoint++];
			TXREG3 = ACC;							//发送数据,发送指针加1
			while(!(UARTIF & txif3));
		}
		else
		{
			Uart3.bSendOverFlag = TRUE;			//树发送数据完毕标志
			Uart3.SendStatus = 0;					//已经发送完,清发送数据标志
			UARTIE &= ~txie3;					//DISABLE TX INTERRUPT
		}
	}
}

/************************************************************************/
/* 函数名：Eint3_ISR													*/
/* 功能说明：intextra_n3中断处理程序						*/
/* 入口参数：N/A													*/
/* 出口参数：N/A													*/
/* 时间:  																*/
/* 作者: 													*/
/************************************************************************/
void Eint3_ISR (void) interrupt 11 using 1
{
// 	uchar tmp,tmp1;
	AIF3 = 0;
	
	if((SPSR & txbuf_empty)&&(SPIIE & tx_e_ie))  //如果发送缓冲区空，且发送缓冲区空中断使能了
	{
// 		UartTx_3(1);
// 		UartTx_3(SPSR);
		clr_if = 1;
		SPIIE &= ~tx_e_ie;//关闭发送空中断
	}
	if(((SPSR & rxbuf_full)==0)&&(SPIIE & rx_ne_ie))  //如果接收缓冲区空，且发送缓冲区空中断使能了
	{
// 		UartTx_3(2);
// 		UartTx_3(SPSR);
		clr_if = 1;
		SPIIE &= ~rx_ne_ie;
	}		
	if((SPSR & 0x78)&&(SPIIE & error_ie))  //如果错误
	{
// 		UartTx_3(SPSR);
		flag = SPSR;
		if(SPSR & m_error)
			SPCR4 &= ~clr_m_err;
		if(SPSR & s_error)
			SPCR4 &= ~clr_s_err;	
		if(SPSR & rxf_wcol)
			SPSR &= ~rxf_wcol;	
		if(SPSR & txf_wcol)
			SPSR &= ~txf_wcol;	
// 		UartTx_3(0X99);
// 		UartTx_3(flag);
	}			


	
	//================= I2C中断 =====================//
	if(SSPIR & i2cie)
	{
			clr_if = 1;		
		if(SSPIR & i2cif)
		{
				
			flag = SSPSTAT;
			if(SSPSTAT & i2c_wcol)
				SSPIR = 0;
			SSPIR &= (~i2cif);
			SSPSTAT = 0;
// 			UartTx_3(flag);
		}
		if(SSPERR & oierr)
		{
			TX_Buf[8] = SSPERR;
			TX_Buf[9] = SSPFSM; 		
			SSPERR &= ~oierr;
		}
		else if(SSPERR & sderr)
		{
			TX_Buf[8] = SSPERR;
			TX_Buf[9] = SSPFSM; 	
			SSPERR &= ~sderr;
		}
		else if(SSPERR & ierr)
		{
			TX_Buf[8] = SSPERR;
			TX_Buf[9] = SSPFSM; 	
			SSPERR &= ~ierr;
		}
	}

	
}

/************************************************************************/
/* 函数名：Eint4_ISR													*/
/* 功能说明：intextra_n4中断处理程序						*/
/* 入口参数：N/A													*/
/* 出口参数：N/A													*/
/* 时间:  																*/
/* 作者: 													*/
/************************************************************************/
void Eint4_ISR (void) interrupt 12 using 1
{
	
	PGDATA ^= BIT7;
	if(DISPIF & disp_donif)
		DISPIF &= ~disp_donif;
	if(DISPIF & disp_doffif)
		DISPIF &= ~disp_doffif;
	AIF4 = 0;
	
}

/************************************************************************/
/* 函数名：Eint5_ISR													*/
/* 功能说明：intextra_n5中断处理程序						*/
/* 入口参数：N/A													*/
/* 出口参数：N/A													*/
/* 时间:  																*/
/* 作者: 													*/
/************************************************************************/
void Eint5_ISR (void) interrupt 13 using 1
{
	AIF5 = 0;
	
	if(EPFLAG & fls_if)
	{
		flash_op_end = 1;
		EPFLAG &= ~fls_if;
	}
	
	if(EPFLAG & Auth_err)
	{
		UartTx_3(EPFLAG);
 		EPFLAG &= ~Auth_err;

	}
	if(EPFLAG & Key_err)
	{
		UartTx_3(EPFLAG);
 		delay(1);
 		EPFLAG &= ~Key_err;
// 		SOFTRST = 0x5C;   //如果key_err出错，软复位
	}
	if(EPFLAG & clk_err)
	{
		UartTx_3(EPFLAG);
 		EPFLAG &= ~clk_err;

	}	

	
	AIF5 = 0;
}

/************************************************************************/
/* 函数名：Eint6_ISR													*/
/* 功能说明：intextra_n6中断处理程序						*/
/* 入口参数：N/A													*/
/* 出口参数：N/A													*/
/* 时间:  																*/
/* 作者: 													*/
/************************************************************************/
void Eint6_ISR (void) interrupt 14 using 1
{
	AIF6 = 0;
	if(CH0STA & ch_half)
	{
		CH0STA &= ~ch_half;
		dma_half = 1;
	}
	if(CH1STA & ch_half)
	{
		CH1STA &= ~ch_half;
		dma_half = 1;
	}	
	if(CH2STA & ch_half)
	{
		CH2STA &= ~ch_half;
		dma_half = 1;
	}
	
	if(CH0STA & ch_end)
	{
		CH0STA &= ~ch_end;
		dma_end = 1;
	}
	if(CH1STA & ch_end)
	{
		CH1STA &= ~ch_end;
		dma_end = 1;
	}	
	if(CH2STA & ch_end)
	{
		CH2STA &= ~ch_end;
		dma_end = 1;
	}		
	
	AIF6 = 0;
}