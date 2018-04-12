#include "init.h"
#include "fm375_uart.h"
INT8U near p_mode;
INT8U near RTCTRIM_copy;
INT8U code MEASURE[] = {0x3a,0x4d,0x45,0x41,0x53,0x75,0x72,0x65,0x3f,0x0A};	//:MEASure?
INT8U code RUN[] = {0x53,0x59,0x53,0x54,0x3A,0x4B,0x45,0x59,0x20,0x39,0x0A};	//SYST:KEY 9
// INT8U code IDN[]={0x2A,0x49,0x44,0x4E,0x3F,0x5C,0x0A};  //"*IDN?\n"
// INT8U code F_RST[]={0x2A,0x52,0x53,0x54,0x0A};   //"*RST\n"
// INT8U code F_SET1[]={0x3A,0x53,0x45,0x4E,0x53,0x65,0x3A,0x45,0x56,0x45,0x4E,0x74,0x3A,0x4C,0x45,0x56,0x65,0x6C,0x31,0x56,0x0A};          //":SENSe:EVENt:LEVel 1V\n"
// INT8U code F_SET2[]={0x3A,0x53,0x45,0x4E,0x53,0x65,0x3A,0x45,0x56,0x45,0x4E,0x74,0x3A,0x53,0x4C,0x4F,0x50,0x65,0x50,0x4F,0x53,0x69,0x74,0x69,0x76,0x65,0x0A};    //":SENSe:EVENt:SLOPe POSitive\n"
// INT8U code F_SET3[]={0x3A,0x53,0x45,0x4E,0x53,0x65,0x3A,0x46,0x52,0x45,0x51,0x75,0x65,0x6E,0x63,0x79,0x3A,0x41,0x52,0x4D,0x31,0x30,0x55,0x53,0x0A};      //":SENSe:FREQuency:ARM 10US\n"

	INT8U const Leave_RC_mode[8]={"RMPC 0\r\n"};
	INT8U const RAMP_rate[10]={"RAMP 0.0\r\n"};
	INT8U const Hot_mode[8]={"SETN 0\r\n"};  //25-225°
	INT8U const Ambient_mode[8]={"SETN 1\r\n"};  //20-30°
	INT8U const Cold_mode[8]={"SETN 2\r\n"};  //-99.9-25°
	
INT8U const  Agilent34401A_Int[15]={"SYSTem:REMote\r\n"};
INT8U const  Agilent34401A_Mea[21]={"MEASure:VOLTage:DC?\r\n"};


INT8U near rtc_sec_if;	
INT8U near clr_if;
INT8U near adc_done;
//char usart_temp[20];
//Typ_word_bits		ADC_Convert_Data;


void ENABLE_BUF4TST_VREF08(void)
{
	PERICLK_CTRL2 |= pdc_clken | anac_clken;
	
	PFFCR1 = 0;
	PFFCR2 = 0;
	PFFCR1 |= BIT6;
	PFFCR2 |= BIT6;//{PFFCR2[6]:PFFCR1[6]}=2'b11,antst0_an3_en=1
	
	ANATESTSEL = 0;
	ANATESTSEL |= vref08 | buf4tst_en;
	
}


void ENABLE_ADC(void)
{
	PERICLK_CTRL2 |= adc_clken | adc_clk_512k;
	ADCCTL = 0x00;
	ADCCTL &= ~adc_vana_en;   //0：ADC用作温度传感器
	ADCTRIM1 = 0x3f;  //ADC计数调校高3位
	ADCTRIM2 = 0xff;  //ADC计数调校低8位
	ADCCTL |= adc_en;  //ADC使能
}

void close_IO_PAD(void)
{

	PERICLK_CTRL2 |= pdc_clken;
	
	PAPPEN = 0;//关闭PA0的推挽功能
	PBPPEN = 0;
	PCPPEN = 0;
	PDPPEN = 0;
	PEPPEN = 0;
	PFPPEN = 0;
	PGPPEN = 0;
	PH0PPEN = 0;

	PAFCR2 = 0;
	PBFCR2 = 0;
	PCFCR2 = 0;
	PDFCR2 = 0;
	PEFCR2 = 0;
	PFFCR2 = 0;
	PGFCR2 = 0;
	PHFCR2 = 0;
	
	PAFCR1 = 0XFF;
	PBFCR1 = 0XFF;
	PCFCR1 = 0XFF;
	PDFCR1 = 0XFF;
	PEFCR1 = 0XFF;
	PFFCR1 = 0XFF;
	PGFCR1 = 0XFF;
	PHFCR1 = 0XFF;		 //设置为输出使能
	
	
}

void close_all_module(void)
{
	PERICLK_CTRL2 |= pdc_clken;
// 	LSCLKSEL = 0x01;  //关闭LPTIM工作时钟;关闭SVD工作时钟;LCD和SVD工作时钟选为XTLF;如果关闭SVD_CLK_EN，LPCLK也会被门控，TESTEN复位失去作用
	RXSTA0 = 0;
	TXSTA0 = 0;
	RXSTA1 = 0;
	TXSTA1 = 0;
	RXSTA2 = 0;
	TXSTA2 = 0;
	RXSTA3 = 0;
	TXSTA3 = 0;	
	
	ANATESTSEL = 0;
	ADCCTL = 0x00;
	CKSRC_CTRL &= ~pll_en;
//	CKSRC_CTRL &= ~rchf_en;
//	LVDCTRL = 0;
	
}

void close_PERCLK(void)
{
	PERICLK_CTRL0 = 0;
	PERICLK_CTRL1 = 0;
//	PERICLK_CTRL2 = 0;	
// 	LSCLKSEL = 0x01;  //关闭LPTIM工作时钟;关闭SVD工作时钟;LCD和SVD工作时钟选为XTLF;如果关闭SVD_CLK_EN，LPCLK也会被门控，TESTEN复位失去作用

}

/************************************************************************
	函数名称:Pro_SYS
	函数功能:SYS功能验证
	入口参数:无
	出口参数:无
************************************************************************/
void Pro_SYS(void)
{
	INT32U  temp_addr;
	INT8U volatile temp_data;
 	INT8U *p;

	INT8U TEM_VALUE[10]={"SETP "};

	switch(Commandl)
	{
		case WRITE_EXBYTE:		//写固定地址寄存器
			
			temp_addr = ((INT32U)(TX_Buf[0]) << 16)|((INT32U)(TX_Buf[1]) << 8) | TX_Buf[2];
		
			temp_data = TX_Buf[3];
			p = (INT8U *)(temp_addr);
			*p = temp_data;

			TX_Buf[4] = 5;			
			TX_Buf[7] = (INT8U)(temp_addr >> 16);
			TX_Buf[8] = (INT8U)(temp_addr >> 8);
			TX_Buf[9] = (INT8U)(temp_addr);
			TX_Buf[10] = temp_data;
			TX_Buf[11] = *p;
				
		break;

		case READ_EXBYTE:		//读固定地址寄存器
			
			temp_addr = ((INT32U)(TX_Buf[0]) << 16)|((INT32U)(TX_Buf[1]) << 8) | TX_Buf[2];
			p = (INT8U *)(temp_addr);
			TX_Buf[4] = 4;			
			TX_Buf[7] = (INT8U)(temp_addr >> 16);
			TX_Buf[8] = (INT8U)(temp_addr >> 8);
			TX_Buf[9] = (INT8U)(temp_addr);
			TX_Buf[10] = *p;

		break;
		
		case UART_SELFTEST:		//串口测试指令

			TX_Buf[4] = 1;
			TX_Buf[7] = 0xAA;

		break;

		case SOFT_RST:   //软复位
	
			UartTx_3(0xaa);	
			SOFTRST = 0x5c;
			_nop_();
			_nop_();

		break;
		
		case INTC_NEST: 	  //中断嵌套
				
				io_if = 0;
		//=======  配置GPIOd1中断为最低优先级 ============//
				GPIO_EXTI_SEL3 = 0;
				GPIO_EXTI_SEL3 |= io_exti_sel_pd1;  //选择pd1作为引脚中断源
				GPIO_EXTIH_ES0 |= extih1_disable;//关闭所有中断
				
				
				PDPPEN = 0;  //关闭推挽
				PDPUEN = 0;  //关闭上拉
				PDFCR1 = 0;  
				PDFCR2 = 0; //{PDFCR2[1]:PDFCR1[1]}=2'b00,PD1为输入功能
				PDPUEN |= BIT1;//使能PD1的上拉
				
				EXTIHIF = 0;

				IPH = 0;
				IPL = 0; //{IPHX1:IPLX1}=00 :　设置INT1_N中断优先级最低
				IT1 = 0;//外部中断1低电平触发		
				EX1 = 1; //外部中断1使能
				
				GPIO_EXTIH_ES0 &= extih1_fall_edge;  //开启低位PD/PE/PF/PG的bit1下降沿中断		
		//=======  配置RTC秒中断为最高优先级 ============//		
				IPH = 0;
				IPL = 0;
				IPH |= BIT0;
				IPLX0 = 1;     //{IPHX0:IPLX0}=11 :　设置INT0_N中断优先级最高
				IT0 = 0;//外部中断0低电平触发		
				EX0 = 1; //外部中断0使能

			while(!(io_if)){WDT_CLR();}   //等待IO47中断返回
			TX_Buf[4] = 0;
			GPIO_EXTIH_ES0 |= extih1_disable;//关闭所有中断
		
		break;

		case IO_INT:   
			

			io_if = 0;

			GPIO_EXTI_SEL0 = 0;
			GPIO_EXTI_SEL0 |= io_exti_sel_pb7;  //选择PB7作为引脚中断源
			GPIO_EXTIL_ES1 = 0xff;//关闭所有中断
			
			
			PBPPEN = 0;  //关闭推挽
			PBPUEN = 0;  //关闭上拉
			PBFCR1 &= ~BIT7;  
			PBFCR2 &= ~BIT7; //{PBFCR2[7]:PBFCR1[7]}=2'b00,PB7为输入功能
			//PBPUEN |= BIT7;//使能PB7的上拉
			
			EXTILIF = 0;

			temp_data = PBDIN;
			
			IPH = 0;
			IPL = 0;
			IPH |= BIT2;
			IPLX1 = 1;     //{IPHX1:IPLX1}=11 :　设置INT1_N中断优先级最高	
				
			if(TX_Buf[0]==0x01)
			{
				IT1 = 1;//外部中断1下降沿触发	
				EX1 = 1;//外部中断1使能
			}
			else if(TX_Buf[0]==0)
			{
				IT1 = 0;//外部中断1低电平触发		
				EX1 = 1; //外部中断1使能				
			}
			
			if(TX_Buf[1]==1)
				GPIO_EXTIL_ES1 &= extil7_fall_edge;  //开启低位PA/PB/PC的bit7下降沿中断
			else if(TX_Buf[1]==2)
				GPIO_EXTIL_ES1 &= extil7_rise_edge;  //开启低位PA/PB/PC的bit7上升沿中断
			else if(TX_Buf[1]==3)
				GPIO_EXTIL_ES1 &= extil7_both_edge;  //开启低位PA/PB/PC的bit7双沿
			
 			while(!(io_if)){WDT_CLR();}   //等待中断返回
			TX_Buf[4] = 1;
			TX_Buf[7] = temp_data;
			GPIO_EXTIL_ES1 = 0xff;//关闭所有中断
			
		break;
			
			
// 		case TEMPER_SET:
// 			
// 			for(i=0;i<Uart3.pRecvLenth;i++)
// 			{
// 				TEM_VALUE[5+i] = TX_Buf[i];
// 			}
// 			TEM_VALUE[5+Uart3.pRecvLenth] = '\r';     //0x0d
// 			TEM_VALUE[5+Uart3.pRecvLenth] = '\n';     //0x0a

// 			Uart3_SendStr(8,Leave_RC_mode);
// 			Uart3_SendStr(10,RAMP_rate);
//  			if(TX_Buf[0]==0x2d)
// 			{
// 				Uart0_SendStr(8,Cold_mode);//-99.9-25°	
// 			}
// 			else 
// 			{
// 				Uart0_SendStr(8,Hot_mode); //25-225°
// 			}
// 			temp_data = 6+Uart3.pRecvLenth;
// 			Uart0_SendStr(temp_data,TEM_VALUE);//20-30°
// 			TX_Buf[4] = 0;
// 			
// 		break;

		
		default:
		break;		
	}
}

/************************************************************************
	函数名称:Pro_I2C
	函数功能:I2C模块功能验证
	入口参数:无
	出口参数:无
************************************************************************/
void Pro_I2C()
{
	INT8U  funcreturn,rw_num,i,j,k,times;
	INT16U  rw_addr;
	INT16U  baud_reg;
	INT8U  write_buf[100];
	INT8U  read_buf[100];

	INIT_I2C_PAD();
	
	switch(Commandl)
	{
		case	READ_ALL_I2CREG:

			TX_Buf[4] = 8;
			TX_Buf[7] = SSPCON;		//SSP控制寄存器 
			TX_Buf[8] = SSPSTAT;		//SSP状态寄存器
			TX_Buf[9] = SSPBRG;		//波特率设置寄存器
			TX_Buf[10] = SSPBUF;		//收发缓冲寄存器
			TX_Buf[11] = SSPIR;		//中断寄存器
			TX_Buf[12] = SSPBRGH;		
			TX_Buf[13] = SSPFSM;	
			TX_Buf[14] = SSPERR;		
			
		break;
		
		case	WRITE_ALL_I2CREG:

			SSPCON = TX_Buf[0];
			SSPSTAT = TX_Buf[1];
			SSPBRG = TX_Buf[2];
			SSPBUF = TX_Buf[3];			
			SSPIR = TX_Buf[4];
			SSPBRGH = TX_Buf[5];
			SSPFSM = TX_Buf[6];			
			SSPERR = TX_Buf[7];
		
			TX_Buf[4] = 8;
			TX_Buf[7] = SSPCON;		//SSP控制寄存器 
			TX_Buf[8] = SSPSTAT;		//SSP状态寄存器
			TX_Buf[9] = SSPBRG;		//波特率设置寄存器
			TX_Buf[10] = SSPBUF;		//收发缓冲寄存器
			TX_Buf[11] = SSPIR;		//中断寄存器
			TX_Buf[12] = SSPBRGH;		
			TX_Buf[13] = SSPFSM;	
			TX_Buf[14] = SSPERR;		
			
		break;

		case	WRITE_ALL_I2CREG_SAME:

			for(i=0; i<8; i++)
			{
				XBYTE[I2C_ADDR + i] = TX_Buf[0];
			}

			TX_Buf[4] = 8;
			TX_Buf[7] = SSPCON;		//SSP控制寄存器 
			TX_Buf[8] = SSPSTAT;		//SSP状态寄存器
			TX_Buf[9] = SSPBRG;		//波特率设置寄存器
			TX_Buf[10] = SSPBUF;		//收发缓冲寄存器
			TX_Buf[11] = SSPIR;		//中断寄存器
			TX_Buf[12] = SSPBRGH;		
			TX_Buf[13] = SSPFSM;	
			TX_Buf[14] = SSPERR;		
			
		break;		

		case 	I2C_BAUD_CTRL:

		//设置波特率:Baud=fclk/(2(SSPBGR[8:0]+1))==>SSPBGR[8:0]=(fclk/2Baud)-1                 

			baud_reg = 75/(TX_Buf[0])-1;  //验证时主频3M，以10K为单位，则3000000/(10000*4)=75
			if(baud_reg>255) SSPBRGH = sspbrgh;
			SSPBRG = (INT8U)baud_reg;
			TX_Buf[4] = 2;
			TX_Buf[7] = SSPBRG;
			TX_Buf[8] = SSPBRGH;
			
		break;			

		case 	I2C_START:
			
			if(TX_Buf[0] == 0x55)
			{				
				i2c_init();
				i2c_start();   //查询是否产生中断标志
				funcreturn = YES_I2C_IF;
			}
			else if(TX_Buf[0] == 0xaa)
			{				
				i2c_init();
				SSPCON |= sen;
				while((SSPSTAT & i2c_s) != i2c_s); //查询是否产生start标志
				funcreturn = YES_I2C_IF;
			}
			
			TX_Buf[4] = 1;		
			TX_Buf[7] = funcreturn;
				
			
		break;

		case 	I2C_STOP:
			
			if(TX_Buf[0] == 0x55)
			{				
				i2c_init();
				i2c_stop();
				funcreturn = YES_I2C_IF;
			}
			else if(TX_Buf[0] == 0xaa)
			{				
				i2c_init();
				SSPCON |= pen;
				while((SSPSTAT & i2c_p) != i2c_p);
				funcreturn = YES_I2C_IF;
			}
			else if(TX_Buf[0] == 0x88)
			{				
				i2c_init();
				for(i=0; i<TX_Buf[1]; i++)
					i2c_stop();
				funcreturn = YES_I2C_IF;
			}
			
			TX_Buf[4] = 1;		
			TX_Buf[7] = funcreturn;
			
		break;		

		case 	I2C_RESTART:

			if(TX_Buf[0] == 0x55)
			{				
				i2c_init();
				i2c_restart();
				funcreturn = YES_I2C_IF;
			}
			else if(TX_Buf[0] == 0xaa)
			{				
				i2c_init();
				i2c_start();
				SSPBUF = 0x55;
				i2c_int();
				i2c_restart();
				funcreturn = YES_I2C_IF;
			}

			TX_Buf[4] = 1;		
			TX_Buf[7] = funcreturn;

		break;

		case 	I2C_WRITE_NBYTE:
			
			rw_num = TX_Buf[0];
			rw_addr = TX_Buf[1];
			rw_addr = (rw_addr << 8) + TX_Buf[2];

	//		funcreturn = i2c_rw24xx(&TX_Buf[3], rw_num, rw_addr, 0xa0, 0, FM24C32);
	//		funcreturn = i2c_write(0xA0, rw_addr, &TX_Buf[3], rw_num, FM24C04);
			funcreturn = i2c_write1(0xA0, rw_addr, &TX_Buf[3], rw_num);

			TX_Buf[4] = 2;
			TX_Buf[7] = funcreturn;
			TX_Buf[8] = SSPERR;
			
		break;

		case 	I2C_WRITE_NBYTE_DMA:
			
			rw_num = TX_Buf[0];
			rw_addr = TX_Buf[1];
			rw_addr = (rw_addr << 8) + TX_Buf[2];

			funcreturn = i2c_write_DMA(0xA0, rw_addr, &TX_Buf[3], rw_num);

			TX_Buf[4] = 2;
			TX_Buf[7] = funcreturn;
			TX_Buf[8] = SSPERR;
			
		break;

		case 	I2C_RAND_READ_NBYTE_DMA:
			
			rw_num = TX_Buf[0];
			rw_addr = TX_Buf[1];
			rw_addr = (rw_addr << 8) + TX_Buf[2];

			funcreturn = i2c_random_read_DMA(0xA0, rw_addr, read_buf, rw_num);

			TX_Buf[4] = rw_num+1;
			TX_Buf[7] = funcreturn;
			for(i=0; i<rw_num; i++)
				TX_Buf[8+i] = read_buf[i];

		break;
				
		
		case 	I2C_WRITE_CONF:
			
			if(TX_Buf[0] == 0x55)
				funcreturn = i2c_write_conflict();
			else if(TX_Buf[0] == 0xaa)
				funcreturn = i2c_write_conflict2();

			TX_Buf[4] = 3;
			TX_Buf[7] = funcreturn;
			TX_Buf[8] = SSPERR; 

		break;

		case 	I2C_ERR:
			
			if(TX_Buf[0] == 0x01)
				funcreturn = i2c_oierr();
			else if(TX_Buf[0] == 0x02)
				funcreturn = i2c_sderr();
			else if(TX_Buf[0] == 0x03)
				funcreturn = i2c_ierr();
			
			SSPERR = 0x00;
			SSPIR = 0;  
			SSPERR = 0;
			SSPCON = 0;
			SSPSTAT = 0x00;
			SSPFSM = 0x00;
			
			TX_Buf[4] = 3;
			TX_Buf[7] = funcreturn;

		break;
			
		case 	I2C_WRITE_WR:
			
			rw_num = TX_Buf[0];
			rw_addr = TX_Buf[1];
			rw_addr = (rw_addr << 8) + TX_Buf[2];

			funcreturn = i2c_write(0x55, rw_addr, &TX_Buf[3], rw_num, FM24C128);//写指令为0x55，为错误指令不能写入

			TX_Buf[4] = 1;
			TX_Buf[7] = funcreturn;

		break;

		case 	I2C_RW_ALL:
			
			i2c_init();

			times = TX_Buf[0];
			for(k=0; k<times; k++)
			{
				//24c128
				for(i=0; i<100; i++)	//写读256页  ALL
				{
					WDT_CLR();					
					for(j=0; j<PAGE; j++)  //每页64byte
					{
						srand(j);
						write_buf[j] = rand();
// 						UartTx_3(write_buf[j]);
					}
					i2c_write(0xa0, i*PAGE, write_buf, PAGE, FM24C128);//32);
					i2c_random_read(0xa0, i*PAGE, read_buf, PAGE, FM24C128);//32);
					
// 					for(j=0; j<PAGE; j++)
// 					{
// 						UartTx_3(read_buf[j]);
// 				
// 					}
					for(j=0; j<PAGE; j++)
					{
						if(read_buf[j] != write_buf[j])
						{
							funcreturn = I2C_ERROR;
							goto back;
						}
					}
				}
			}
			funcreturn = I2C_SUCCESS;

			back:
			TX_Buf[4] = 2;
			TX_Buf[7] = funcreturn;
			TX_Buf[8] = times;

		break;	

		case 	I2C_RAND_READ_NBYTE:
			
			rw_num = TX_Buf[0];
			rw_addr = TX_Buf[1];
			rw_addr = (rw_addr << 8) + TX_Buf[2];

//			funcreturn = i2c_rw24xx(read_buf, rw_num, rw_addr, 0xa1, 0, FM24C32);
			funcreturn = i2c_random_read(0xA0, rw_addr, read_buf, rw_num, FM24C128);

			TX_Buf[4] = rw_num+1;
			TX_Buf[7] = funcreturn;
			for(i=0; i<rw_num; i++)
				TX_Buf[8+i] = read_buf[i];

		break;

		case 	I2C_CURR_READ_NBYTE:
			
			rw_num = TX_Buf[0];

			funcreturn =  i2c_current_read(0xa1, read_buf, rw_num);
	//		funcreturn = i2c_rw24xx(read_buf, rw_num, 0, 0xa1, 1, FM24C32);
	    	
			TX_Buf[4] = rw_num+1;
			TX_Buf[7] = funcreturn;
			for(i=0; i<rw_num; i++)
				TX_Buf[8+i] = read_buf[i];

		break;
		
		case 	I2C_READ_WITHOUTACK:
			
			rw_num = TX_Buf[0];
			rw_addr = TX_Buf[1];
			rw_addr = (rw_addr << 8) + TX_Buf[2];

			funcreturn = i2c_random_readwrong(0xa0, rw_addr, read_buf, rw_num, FM24C128);

			TX_Buf[4] = rw_num+1;
			TX_Buf[7] = funcreturn;
			for(i=0; i<rw_num; i++)
				TX_Buf[8+i] = read_buf[i];

		break;
		
		case 	I2C_SCLHL:

			SSPCON = 0x00;
			for(i=0; i<TX_Buf[0]; i++)
			{
				SSPCON &= ~sclhl;
				_nop_();_nop_();
				SSPCON |= sclhl;
				_nop_();_nop_();
			}

			TX_Buf[4] = 1;
			TX_Buf[7] = SSPCON;

		break;

		case 	I2C_SDAHL:
			
			SSPCON = 0x00;
			for(i=0; i<TX_Buf[0]; i++)
			{
				SSPCON &= ~sdahl;
				_nop_();_nop_();
				SSPCON |= sdahl;
				_nop_();_nop_();
			}
			
			TX_Buf[4] = 1;
			TX_Buf[7] = SSPCON;

		break;
		
		default:
		break;
	}
}


/************************************************************************
	函数名称:Pro_SPI
	函数功能:SPI模块功能验证
	入口参数:无
	出口参数:无
************************************************************************/
void Pro_SPI()
{
	INT8U near funcreturn,rw_num,i;//,k
	INT8U j;
	INT8U volatile tmp;
//	INT8U near rw_addr;
	INT8U near write_buf[200];
	INT8U near read_buf[200];

	INIT_SPI_PAD();
	switch(Commandl)
	{
		case	READ_ALL_SPIREG:

			TX_Buf[4] = 8;				//数据长度			
			for(i=0;i<8;i++)
			{
				TX_Buf[7+i] = XBYTE[SPI_ADDR+i];
			}		
			
		break;
		
		case WRITE_ALL_SPIREG:
			for(i=0; i<8; i++)
			{
				XBYTE[SPI_ADDR + i] = TX_Buf[i];
			}
				
			TX_Buf[4] = 8;				//数据长度			
			for(i=0;i<8;i++)
			{
				TX_Buf[7+i] = XBYTE[SPI_ADDR+i];
			}		
	
/*			if(SPSR & modf)
				SPCR = 0xc9;*/
		break;

		case WRITE_ALL_SPIREG_SAME:

			for(i=0; i<8; i++)
				XBYTE[SPI_ADDR + i] = TX_Buf[0];
			
			TX_Buf[4] = 8;				//数据长度			
			for(i=0;i<8;i++)
			{
				TX_Buf[7+i] = XBYTE[SPI_ADDR+i];
			}		

/*			if(SPSR & modf)
				SPDR = 0;*/
		break;		
		
		case	WRITE_25F04_ENABLE:

			spi_init();
			clr_if = 0;
			if(TX_Buf[0]==0x55)
			{
				AIE3 = 1;
				SPI_write_enable_int();	
			}
			else if(TX_Buf[0]==0xaa)
			{
				AIE3 = 0;
				SPIIE = 0;
				SPI_write_enable();
			}
			
			TX_Buf[4] = 0;				//数据长度			
			
		break;

		case	WRITE_25F04_DISABLE:

			spi_init();
			clr_if = 0;
		
			if(TX_Buf[0]==0x55)
			{
				AIE3 = 1;
				SPI_write_disable_int();
			}
			else if(TX_Buf[0]==0xaa)
			{
				AIE3 = 0;
				SPIIE = 0;
				SPI_write_disable();
				
			}
			TX_Buf[4] = 0;				//数据长度
			
		break;		
		
		case	CHIP_ERASE:

			spi_init();                         
			spi_chip_erase();				

			TX_Buf[4] = 0;				//数据长度
			
		break;

 		case	WRITE_25F04:
 		
 			rw_num = TX_Buf[0];
 			spi_init();
// 			AIE3 = 1;
// 			SPCR |= spie; 使能中断
 			write_spi_nbyte(TX_Buf[1],TX_Buf[2],TX_Buf[3],&TX_Buf[4],rw_num);

 			TX_Buf[4] = 0;				//数据长度

 		break;	
 			
 		case	READ_25F04:
 			
 			rw_num = TX_Buf[0];
 			spi_init();
 
// 			AIE3 = 1;
// 			SPCR |= spie; //使能中断
 			read_spi_nbyte(TX_Buf[1],TX_Buf[2],TX_Buf[3],read_buf,rw_num);

 			TX_Buf[4] = rw_num;				//数据长度

 			for(i=0;i<rw_num;i++)
 				TX_Buf[7+i] = read_buf[i];
 				
 		break;
 
 		case	WRITE_25F04_DMA:
 		
 			rw_num = TX_Buf[0];
 			spi_init();
			SPI_DMA_Config();
			
			write_spi_nbyte_dma(TX_Buf[1],TX_Buf[2],TX_Buf[3],(INT16U)&TX_Buf[4],rw_num);
			
 			TX_Buf[4] = 0;				//数据长度

 		break;	

		case	READ_25F04_DMA:
 		
 			rw_num = TX_Buf[0];
 			spi_init();
			SPI_DMA_Config();
 			read_spi_nbyte_dma(TX_Buf[1],TX_Buf[2],TX_Buf[3],(INT16U)&read_buf[0],rw_num);

 			TX_Buf[4] = rw_num;				//数据长度

 			for(i=0;i<rw_num;i++)
 				TX_Buf[7+i] = read_buf[i];

 		break;	
				
		
 	case 	SPI_RW_ALL:
 			funcreturn = SPI_SUCCESS;
 			spi_init();
			
 			for(i=0; i<TX_Buf[0]; i++)
 			{
 				spi_chip_erase();	
				delay(5000);
 				delay(5000);//Chip Erase Time
				
				for(j=0; j<200; j++)
 				{
 					write_buf[j] = rand();
 				}
 				write_spi_nbyte(0x00,0x00,0x00,write_buf,200);
 				delay(10);
 				read_spi_nbyte(0x00,0x00,0x00,read_buf,200);
				


				for(j=0; j<200; j++)
				{
					WDT_CLR();
					if(read_buf[j] != write_buf[j])
					{
						funcreturn = SPI_ERROR;
						UartTx_3(j);
						UartTx_3(read_buf[j]);
						UartTx_3(write_buf[j]);
						break;
					}
					else funcreturn = SPI_SUCCESS;		
				}
				if(funcreturn == SPI_ERROR)
					break;
			}
 															
 			TX_Buf[4] = 1;				//数据长度
 			TX_Buf[7] = funcreturn;

 		break;	
			
 	case 	SPI_RW_ALL_DMA:
 			funcreturn = SPI_SUCCESS;
 			spi_init();
			SPI_DMA_Config();
 			for(i=0; i<TX_Buf[0]; i++)
 			{
 				spi_chip_erase_dma();	
				delay(5000);
 				delay(5000);//Chip Erase Time
				
				for(j=0; j<10; j++)
 				{
 					write_buf[j] = rand();
 				}
				
				write_spi_nbyte_dma(0x00,0x00,0x00,(INT16U)&write_buf[0],10);		
 				delay(10);
				read_spi_nbyte_dma(0x00,0x00,0x00,(INT16U)&read_buf[0],10);


				for(j=0; j<10; j++)
				{
					WDT_CLR();
					if(read_buf[j] != write_buf[j])
					{
						funcreturn = SPI_ERROR;
						UartTx_3(j);
						UartTx_3(read_buf[j]);
						UartTx_3(write_buf[j]);
						break;
					}
					else funcreturn = SPI_SUCCESS;		
				}
				if(funcreturn == SPI_ERROR)
					break;
			}
 															
 			TX_Buf[4] = 1;				//数据长度
 			TX_Buf[7] = funcreturn;

 		break;	

		case	SPI_BAUD_SET:
			tmp = (SPCR1&0x1f);
			switch(TX_Buf[0])
			{
				case	0:  //  FCLK/2
					tmp += 0;           
				break;
				case	1:  //  FCLK/4
					tmp += 0x20;
				break;
				case	2:  //  FCLK/8
					tmp += 0x40;
				break;
				case	3:  //  FCLK/16
					tmp += 0x60;
				break;
				case	4:  //  FCLK/32
					tmp += 0x80;
				break;
				case	5:  //  FCLK/64
					tmp += 0xa0;
				break;
				case	6:  //  FCLK/128
					tmp += 0xc0;
				break;
				case	7:  //  FCLK/256
					tmp += 0xe0;
				break;
				
				default:
				break;
			}
			SPCR1 = tmp;
			TX_Buf[4] = 1;				//数据长度			
			TX_Buf[7] = SPCR1;			//SPI控制寄存器1
				
		break;
			
			
		case SPI_MASTER_ERR:
			spi_init();
			funcreturn = spi_master_error();
			TX_Buf[4] = 1;
			TX_Buf[7] = funcreturn;
		
		break;

		case SPI_TX_WCOL:
			spi_init();
			AIE3 = 1;
			funcreturn = spi_txcol_error();
			TX_Buf[4] = 1;
			TX_Buf[7] = funcreturn;
// 		UartTx_3(funcreturn);
// 		UartTx_3(flag);			
		
		break;
				
				
		default:
		break;
	}
}

/************************************************************************
	函数名称:Pro_RAM
	函数功能:RAM功能验证
	入口参数:无
	出口参数:无
************************************************************************/
void Pro_RAM(void)
{
	INT16U near temp_addr,length,j;
	INT8U near i,result;
	
 	INT8U *p = (INT8U *)(RAM_ADDR);
	ram_par_err = 0;
	switch(Commandl)
	{

		case READ_RAMBIST:
		
			TX_Buf[4] = 3;
		
			TX_Buf[7] = RAMBISTCTL;
			TX_Buf[8] = RAMBISTSTA;
			TX_Buf[9] = RAMPARITY;
			
		break;
			
		case READ_RAM_BYTE:
			
			temp_addr = TX_Buf[0];
			temp_addr = (temp_addr << 8)|TX_Buf[1];		
			p = (INT8U *)(RAM_ADDR + temp_addr);
			TX_Buf[4] = 4;
			TX_Buf[7] = *p;
			TX_Buf[8] = (uchar)(temp_addr>>8);
			TX_Buf[9] = (uchar)(temp_addr);
			TX_Buf[10] = ram_par_err;
			
		break;

		case WRITE_RAM:
			temp_addr = TX_Buf[0];
			temp_addr = (temp_addr << 8)|TX_Buf[1];		
			length = TX_Buf[2];
		
			p = (INT8U *)(RAM_ADDR + temp_addr);	
			for(i=0;i<length;i++)
			{
				*p = TX_Buf[3];
				p++;
			}
			
			p = (INT8U *)(RAM_ADDR + temp_addr);
			TX_Buf[4] = 3;
			TX_Buf[7] = *p;
			TX_Buf[8] = (uchar)(temp_addr>>8);
			TX_Buf[9] = (uchar)(temp_addr);
		
		break;

// 		case STOP_RAM_TEST:
// 			temp_addr = TX_Buf[0];
// 			temp_addr = (temp_addr << 8)|TX_Buf[1];
// 			p = (INT8U *)(RAM_ADDR + temp_addr);
// 		
// 			erase_FLASH(0xa000);
// 			write_FLASH(0xa000,*p);//-----将RAM中的数据先备份到DATA FLASH中-----//
// 			*p = TX_Buf[2];    //将需要校验的数据放入相应RAM中
// 		
// 			close_all_module();	
// 			close_IO_PAD();							
// 			goto_stop();  //IO43 中断唤醒
// 		
// 			_nop_();
// 			_nop_();
// 			_nop_();
// 			_nop_();
// 			_nop_();
// 			_nop_();
// 			_nop_();
// //================= 唤醒后进入相应中断然后退回到此处 =======================//
// 			result = *p;   //唤醒后再读出之前放入RAM中的数据

// 			*p = read_FLASH(0xa000);  //将备份的数据放入RAM中

// 		
// 			INIT_CLK();
// 			IO4OUTEN |= BIT5;  //GPIO45 OUT CLK
// 			TESTCON |= clko_coreclk | clko_en; //remap0 | gpio_outen
// 			Init_UART_PAD();
// 			Init_UART();			
// 			INIT_INT();	
// 			
// 			Commandh = 0x60;
// 			Commandl = 0x07;

// 			TX_Buf[4] = 1;	
// 			TX_Buf[7] = result;
// 		
// 		break;

		case WR_RAM_CHECK:
			result = xram_wr_check();
		
			TX_Buf[4] = 2;
			TX_Buf[7] = result;
			TX_Buf[8] = ram_par_err;
		
		break;		

		case RAMBIST_ON:

			RAMBISTSTA = 0;
			RAMPARITYIE = ramparity_ie;
			AIE0 = 1;

			
//-----将RAM中的数据先备份到DATA FLASH中-----//

			erase_FLASH(0xFFE000);
			erase_FLASH(0xFFE200);
			erase_FLASH(0xFFE400);
			erase_FLASH(0xFFE600);
			erase_FLASH(0xFFE800);
			erase_FLASH(0xFFEA00);
			erase_FLASH(0xFFEC00);
			erase_FLASH(0xFFEE00);

			p = (INT8U *)(RAM_ADDR);
			for(j=0;j<4096;j++)     
			{			
				write_FLASH(j+0xFFE000,*p);
				p++;
				delay(1);
			}
//-----------------------------------------//			
			PGDATA ^= BIT7;
			RAMBISTCTL |= rambist_start;
			while(!(RAMBISTSTA & rambist_done))
			{WDT_CLR();}
			PGDATA ^= BIT7;
			
//------------将RAM中的数据恢复------------//			
			p = (INT8U *)(RAM_ADDR);
			for(j=0;j<4096;j++)     
			{
				*p = read_FLASH(j+0xFFE000);
				p++;
			}		
//-----------------------------------------//			

			if(RAMBISTSTA & rambist_error)
				result = 1;
			else result = 0;
			

			RAMBISTSTA = 0;
			RAMBISTCTL = 0;
			TX_Buf[4] = 2;
			TX_Buf[7] = result;
			TX_Buf[8] = ram_par_err;
			WDT_CLR();
		break;
		
		default:
		break;		
	}
}

/************************************************************************
	函数名称:Pro_RTC
	函数功能:RTC模块功能验证
	入口参数:无
	出口参数:无
************************************************************************/
void Pro_RTC(void)
{
	INT8U near i,j,k;
	INT8U near rdata1[7],rdata2[7];
// 	INT8U addatah;
// 	INT8U addatal;
// 	INT8U tmp1,times,funcreturn;
// 	INT8U tmp2;
// 	INT8U near write_buf[256];
// 	INT8U near read_buf[256];
// 	INT16U addr,nvdata;
// 	INT8U k,i;
// 	INT16U j;
// 	INT16U *p;
// 	INT8U *q;
//	INIT_RTC_PAD();
	
	EX0 = 1;
//  	IT0 = 1;//外部中断0下降沿触发
	IT0 = 0;//外部中断0低电平触发	
	switch(Commandl)
	{
		case READ_ALL_RTCREG:

			TX_Buf[4] = 21;
			for(i=0;i<21;i++)
			{
				TX_Buf[7+i] = XBYTE[RTC_ADDR+i];
			}

		break;
			
		case WRITE_ALL_RTCREG:		
			
			for(i=0;i<21;i++)
			{
				 XBYTE[RTC_ADDR+i] = TX_Buf[i];
			}
			
		
			TX_Buf[4] = 21;
			for(i=0;i<21;i++)
			{
				TX_Buf[7+i] = XBYTE[RTC_ADDR+i];
			}
		break;

		case WRITE_ALL_RTCREG_SAME:		
			for(i=0;i<21;i++)
			{
				 XBYTE[RTC_ADDR+i] = TX_Buf[0];
			}
			TX_Buf[4] = 21;
			for(i=0;i<21;i++)
			{
				TX_Buf[7+i] = XBYTE[RTC_ADDR+i];
			}
		break;
			
		case TIME_SET:

			RTCWE = 0xAC; //使能RTC写
			rtc_sec_if=0;

			IT0 = 1;//外部中断0下降沿触发	
			EX0 = 1;
			RTCIE2 |= secie;

			while(!(rtc_sec_if))
			{}
			
			BCDSEC = TX_Buf[0];
			BCDMIN = TX_Buf[1];
			BCDHOUR = TX_Buf[2];
			BCDDATE = TX_Buf[3];
			BCDWEEK = TX_Buf[4];
			BCDMONTH = TX_Buf[5];
			BCDYEAR = TX_Buf[6];

			TX_Buf[4] = 7;
			TX_Buf[7] = BCDSEC;
			TX_Buf[8] = BCDMIN;
			TX_Buf[9] = BCDHOUR;
			TX_Buf[10] = BCDDATE;
			TX_Buf[11] = BCDWEEK;
			TX_Buf[12] = BCDMONTH;
			TX_Buf[13] = BCDYEAR;
				
			EX0 = 0;
			RTCIE2 = 0;				
		break;			
			
		case TIME_READ1:
		
		
			j=0;
			
			do{
				k=0xaa;
				
				rdata1[0] = BCDSEC  ;
				rdata1[1] = BCDMIN  ;
				rdata1[2] = BCDHOUR ;
				rdata1[3] = BCDDATE;
				rdata1[4] = BCDWEEK;
				rdata1[5] = BCDMONTH;
				rdata1[6] = BCDYEAR;
			
				rdata2[0] = BCDSEC  ;
				rdata2[1] = BCDMIN  ;
				rdata2[2] = BCDHOUR ;
				rdata2[3] = BCDDATE;
				rdata2[4] = BCDWEEK;
				rdata2[5] = BCDMONTH;
				rdata2[6] = BCDYEAR;

				for(i=0;i<7;i++)
				{
					if(rdata1[i]!=rdata2[i])
						k=0x55;
				}	
				j++;
			}
			while((j<3)&&(k==0x55));  //若两次读出数据不相等，读2遍


			TX_Buf[4] = 8;
			TX_Buf[7] = k;  
			for(i=0;i<7;i++)
			{
				TX_Buf[8+i] = rdata2[i];
			}	
				
		break;					
			
		case TIME_READ2:
			rtc_sec_if=0;
			EX0 = 1;
			RTCIE2 |= secie;
			while(!(rtc_sec_if))
			{}
				rdata2[0] = BCDSEC  ;
				rdata2[1] = BCDMIN  ;
				rdata2[2] = BCDHOUR ;
				rdata2[3] = BCDDATE;
				rdata2[4] = BCDWEEK;
				rdata2[5] = BCDMONTH;
				rdata2[6] = BCDYEAR;
				
			TX_Buf[4] = 7;
			for(i=0;i<7;i++)
			{
				TX_Buf[7+i] = rdata2[i];
			}					
				
		break;				
					

 		case RTC_ENABLE:
 			
 			RTCWE = 0xac;     //使能RTC写
 			delay(1);

 				
 			TX_Buf[4] = 1;
 			TX_Buf[7] = RTCWE;

 		break;					

 		case RTC_DISABLE:
 			
 			RTCWE = 0xca;     //禁止RTC写
 			delay(1);
 				
 			TX_Buf[4] = 1;
 			TX_Buf[7] = RTCWE;

 		break;	
		
		
 		case RTCADJ_SET:  //
 			
			ADSIGN = TX_Buf[0];  //调校方向,0为加，1为减
 			ADJUST1 = (TX_Buf[1]>>1); //调校值高位,TX_Buf[1] = C(Common Value)
 			ADJUST = ((TX_Buf[1]&0x01)<<7)+(TX_Buf[2]&0X7F);  //调校值低位,TX_Buf[2] = D(Differential Value )
 			TX_Buf[4] = 0;

 		break;	

 		case RTCADJ_PLL:   //精确秒时标调校，pll产生的虚拟秒时标
 			
 			CKSRC_CTRL |= pll_en;//使能PLL
 			delay(1);//软件等待PLL锁定时间
			PRLSEN = pr1sen;//置位pr1sen寄存器，使能每秒调校秒时标产生
				
			ADSIGN = TX_Buf[0];  //调校方向,0为加，1为减
 			ADJUST1 = TX_Buf[1]; //调校值高3位
 			ADJUST = TX_Buf[2];  //调校值低8位
 			TX_Buf[4] = 0;

 		break;	
	
		
		case RTC_OUT:
			
			FSEL = TX_Buf[0]; //输出RTC内部时标信号
			if((TX_Buf[0]==0)||(TX_Buf[0]==1)||(TX_Buf[0]==8)||(TX_Buf[0]==0x0E))
			{
				CKSRC_CTRL |= pll_en;//使能PLL
				PRLSEN = pr1sen;//使能虚拟调校功能，pll才能产生虚拟秒时标
			}
				
				
			FOUT0_SEL = fout0_rtctm;  //输出rtc时标
// 			PGFCR1 = 0;
// 			PGFCR2 = 0;
// 			PGFCR2 |= BIT4;//{PGFCR2[4]:PGFCR1[4]}=2'b10				
// 			AFSELG &= ~BIT4; //0: FOUT0
			
			
			PFFCR1 &= ~BIT5;
			PFFCR2 |= BIT5;//{PFFCR2[5]:PFFCR1[5]}=2'b10				
			AFSELF &= ~BIT5; //0: FOUT0

			TX_Buf[4] = 0;

		break;	
			
		default:
		break;
	}
}

/************************************************************************
	函数名称:Pro_PMU
	函数功能:PMU模块功能验证
	入口参数:无
	出口参数:无
************************************************************************/
void Pro_PMU(void)
{
	INT8U near i,tmp;
	INT8U near mclk,wake_source;
	
	switch(Commandl)
	{
		case READ_ALL_PMUREG:

			TX_Buf[4] = 6;
			for(i=0;i<5;i++)
			{
				TX_Buf[7+i] = XBYTE[PMU_ADDR+i];
			}
			
			TX_Buf[12] = PMU_PDCTRL;

		break;
			
		case WRITE_ALL_PMUREG:		
			
			for(i=0;i<5;i++)
			{
				 XBYTE[PMU_ADDR+i] = TX_Buf[i];
			}
			PMU_PDCTRL = TX_Buf[5];
		
			TX_Buf[4] = 6;
			for(i=0;i<5;i++)
			{
				TX_Buf[7+i] = XBYTE[PMU_ADDR+i];
			}
			TX_Buf[12] = PMU_PDCTRL;
		break;

		case WRITE_ALL_RPMUREG_SAME:		
			for(i=0;i<5;i++)
			{
				 XBYTE[PMU_ADDR+i] = TX_Buf[0];
			}
			PMU_PDCTRL = TX_Buf[0];
			
			TX_Buf[4] = 6;
			for(i=0;i<5;i++)
			{
				TX_Buf[7+i] = XBYTE[PMU_ADDR+i];
			}
			TX_Buf[12] = PMU_PDCTRL;
		break;
			
		case ACT_LPRUN:
				
			close_all_module();	//关闭所有外设的总线时钟
			close_IO_PAD();	
			PMU_CFG = 0;
		
			ULPCTL = TX_Buf[0];  //固件配置ULPCTRL寄存器，校准ULPLDO输出1.5V
			mclk = TX_Buf[1];
		
			XTLF_IPWRB = TX_Buf[2];		

			SET_MCLK(mclk,(TX_Buf[3]<<1),TX_Buf[4]);//配置进入SLEEP模式前的主时钟
			_nop_();
			_nop_();

			if(TX_Buf[5]==1)  //使能lprun_err中断
				LPRUNERRIE |= lprun_ie;
			else  
				LPRUNERRIE &= ~lprun_ie;
				
			if(TX_Buf[6]==1)  //开启LPTIM工作时钟
				LSCLKSEL |= lptim_fcken;
			else  
				LSCLKSEL &= ~lptim_fcken;				
				
			if(TX_Buf[7]==1)  //开启svd工作时钟
				LSCLKSEL |= svd_clken;
			else  
				LSCLKSEL &= ~svd_clken;						
			
			if(TX_Buf[8]==1)  //LCD和LVD工作时钟选xtlf
				LSCLKSEL |= svd_lcd_clk_xtlf;
			else  
				LSCLKSEL &= ~BIT0;							

			if(TX_Buf[9]==1)  //关闭BOR
				BORCTL |= off_bor;
			else  
				BORCTL &= ~off_bor;	
				
			if(TX_Buf[10]==1)  //关闭PDR
				PDRCTL &= ~pdr_en;   //关闭PDR
			else 
				PDRCTL |= pdr_en; 
				
			if(TX_Buf[11]==1)  //关闭RCLP
				CKSRC_CTRL &= ~lp_rclpctrl;   //关闭RCLP
			else 
				CKSRC_CTRL |= lp_rclpctrl; 
								
//			CKSRC_CTRL |= pll_en;
//			FOUT1_SEL = fout1_lsclk;
//			PCFCR1 &= ~BIT3;
//			PCFCR2 |= BIT3;//{PCFCR2[3]:PCFCR1[3]}=2'b10				
//			AFSELC |= BIT3; //1: FOUT1			
			
			close_PERCLK();			
			
			
//			PBFCR2 &= 0xfe;
//			PBPPEN |= BIT0;//使能PA0的推挽功能
//			PBFCR1 |= BIT0;//{PAFCR2[0]:PAFCR1[0]}=2'b01,使能PA0输出
	
			PMU_CFG |= lprun_mode;  //进入lprun
			_nop_();
		  while(1)
			{
 				WDT_CLR();//清狗
// 				PBDATA ^= BIT0;
			}
			TX_Buf[4] = 0;
		

		break;
	
		case ACT_LPRUN_ACT:   //ACTIVE模式进入LPRUN，然后退出LPRUN
		
			close_IO_PAD();	
			CKSRC_CTRL |= pll_en;//使能PLL
			FOUT1_SEL = fout1_pllo_div4;
			PCFCR1 &= ~BIT3;
			PCFCR2 |= BIT3;//{PCFCR2[3]:PCFCR1[3]}=2'b10				
			AFSELC |= BIT3; //1: FOUT1					
			
			ENABLE_ADC();  //使能ADC
			delay(2000);
			
			
			PMU_CFG = 0;
			ULPCTL = TX_Buf[0];  //固件配置ULPCTRL寄存器，校准ULPLDO输出1.5V
			mclk = TX_Buf[1];
			XTLF_IPWRB = TX_Buf[2];	
			SET_MCLK(mclk,(TX_Buf[3]<<1),TX_Buf[4]);//配置进入SLEEP模式前的主时钟
			_nop_();
			_nop_();

			if(TX_Buf[5]==1)  //使能lprun_err中断
				LPRUNERRIE |= lprun_ie;
			else  
				LPRUNERRIE &= ~lprun_ie;
				
			if(TX_Buf[6]==1)  //开启LPTIM工作时钟
				LSCLKSEL |= lptim_fcken;
			else  
				LSCLKSEL &= ~lptim_fcken;				
				
			if(TX_Buf[7]==1)  //开启svd工作时钟
				LSCLKSEL |= svd_clken;
			else  
				LSCLKSEL &= ~svd_clken;						
			
			if(TX_Buf[8]==1)  //LCD和LVD工作时钟选xtlf
				LSCLKSEL |= svd_lcd_clk_xtlf;
			else  
				LSCLKSEL &= ~BIT0;		
				
			if(TX_Buf[9]==1)  //关闭BOR
				BORCTL |= off_bor;
			else  
				BORCTL &= ~off_bor;	
				
			if(TX_Buf[10]==1)  //关闭PDR
				PDRCTL &= ~pdr_en;   //关闭PDR
			else 
				PDRCTL |= pdr_en; 
				
			if(TX_Buf[11]==1)  //关闭RCLP
				CKSRC_CTRL &= ~lp_rclpctrl;   //关闭RCLP
			else 
				CKSRC_CTRL |= lp_rclpctrl; 				
						
				
			PMU_CFG |= lprun_mode; //进入lprun
			_nop_();
			delay(100);
			 
			PMU_CFG = 0;    //退出LPRUN模式
			while(!((PMU_CFG & active_mode)==0));  //查询到确实退出STB模式
			_nop_();
			
			INIT_CLK();
			Init_UART_PAD();
			Init_UART();
			Init_SYS();   //设置看门狗、中断等
			TX_Buf[4] = 0;
		break;
		
		case ACT_SL:     
			close_IO_PAD();	
			close_all_module();	
		
			PAFCR2 &= 0Xfe;
			PAPPEN |= BIT0;//使能PA0的推挽功能
			PAFCR1 |= BIT0;//{PAFCR2[0]:PAFCR1[0]}=2'b01,使能PA0输出
			PADATA = 0;	
		
// 	UartTx_3(0x99);
// 	UartTx_3(TX_Buf[12]);			
			PMU_CFG = 0;
			ULPCTL = TX_Buf[0];  //固件配置ULPCTRL寄存器，校准ULPLDO输出1.5V
			mclk = TX_Buf[1];
			XTLF_IPWRB = TX_Buf[3];		
				
			wake_source = TX_Buf[2];   //唤醒源
			p_mode = SLEEP;  //根据不同的模式配置中断类型
			wake_event_set(wake_source,p_mode);//开启相应唤醒源中断
		
			Low_power_Ctrl(TX_Buf[4],TX_Buf[5],TX_Buf[6],TX_Buf[7],TX_Buf[8],TX_Buf[13],0,0);//低功耗制

			if(TX_Buf[9]==1)  //关闭BOR
				BORCTL |= off_bor;
			else  
				BORCTL &= ~off_bor;		
			
//			if(TX_Buf[10]==1)  //使能lprun_err中断
//				LPRUNERRIE |= lprun_ie;
//			else  
//				LPRUNERRIE &= ~lprun_ie;			
			
			if(TX_Buf[14]==1)  //关闭PDR
				PDRCTL &= ~pdr_en;   //关闭PDR
			else 
				PDRCTL |= pdr_en; 

      //==== 唤醒等待配置时间控制   ===== //
			WTCFG = (TX_Buf[15]<<2) | TX_Buf[16];
			
			if(TX_Buf[17]==1)  //开启LPTIM工作时钟
				LSCLKSEL |= lptim_fcken;
			else  
				LSCLKSEL &= ~lptim_fcken;				
				
			if(TX_Buf[18]==1)  //开启svd工作时钟
				LSCLKSEL |= svd_clken;
			else  
				LSCLKSEL &= ~svd_clken;						
			
			if(TX_Buf[19]==1)  //LCD和LVD工作时钟选xtlf
				LSCLKSEL |= svd_lcd_clk_xtlf;
			else  
				LSCLKSEL &= ~BIT0;			

			SET_MCLK(mclk,(TX_Buf[11]<<1),TX_Buf[12]);//配置进入SLEEP模式前的主时钟

			Set_SYSWDT(wdtov_time_4096S);			//设置wdt周期
			
			close_PERCLK();	
			
			PMU_CFG |= sleep_mode;
			PCON |= idl;     //进入sleep
			_nop_();

	INIT_CLK();
	
	Init_UART_PAD();
	Init_UART();
	Init_SYS();   //设置看门狗、中断等
			TX_Buf[4] = 0;

		break;
	
		case ACT_LPRUN_SL:  
		 
			close_all_module();	
			close_IO_PAD();	
			CKSRC_CTRL |= pll_en;//使能PLL
			ENABLE_ADC();
		
						
			PMU_CFG = 0;
		
			ULPCTL = TX_Buf[0];  //固件配置ULPCTRL寄存器，校准ULPLDO输出1.5V
			mclk = TX_Buf[1];
		
			XTLF_IPWRB = TX_Buf[3];		

			if(TX_Buf[10]==1)  //使能lprun_err中断
				LPRUNERRIE |= lprun_ie;
			else  
				LPRUNERRIE &= ~lprun_ie;					

			SET_MCLK(mclk,(TX_Buf[11]<<1),TX_Buf[12]);//配置进入LPRUN模式前的主时钟
 
			_nop_();
			_nop_();
			PMU_CFG |= lprun_mode;  //进入lprun
			_nop_();
		
		
			delay(200);
			
			wake_source = TX_Buf[2];   //唤醒源
			p_mode = SLEEP;  //根据不同的模式配置中断类型
			wake_event_set(wake_source,p_mode);//开启相应唤醒源中断
			
			Low_power_Ctrl(TX_Buf[4],TX_Buf[5],TX_Buf[6],TX_Buf[7],TX_Buf[8],TX_Buf[13],0,0);//低功耗控制		

			if(TX_Buf[9]==1)  //关闭BOR
				BORCTL |= off_bor;
			else  
				BORCTL &= ~off_bor;			

			if(TX_Buf[14]==1)  //关闭PDR
				PDRCTL &= ~pdr_en;   //关闭PDR
			else 
				PDRCTL |= pdr_en;   

      //==== 唤醒等待配置时间控制   ===== //
			WTCFG = (TX_Buf[15]<<2) | TX_Buf[16];
			
			if(TX_Buf[17]==1)  //开启LPTIM工作时钟
				LSCLKSEL |= lptim_fcken;
			else  
				LSCLKSEL &= ~lptim_fcken;				
				
			if(TX_Buf[18]==1)  //开启svd工作时钟
				LSCLKSEL |= svd_clken;
			else  
				LSCLKSEL &= ~svd_clken;						
			
			if(TX_Buf[19]==1)  //LCD和LVD工作时钟选xtlf
				LSCLKSEL |= svd_lcd_clk_xtlf;
			else  
				LSCLKSEL &= ~BIT0;					
			
			Set_SYSWDT(wdtov_time_4096S);			//设置wdt周期
			close_PERCLK();	
			
			
			_nop_();
			tmp = PMU_CFG;
			tmp = ((tmp&0xBF)|sleep_mode);
			PMU_CFG = tmp;	
			PCON |= idl;     //进入sleep
			_nop_();


	INIT_CLK();
//  	IO4OUTEN |= BIT5;  //GPIO45 OUT CLK
//   TESTCON |= clko_coreclk | clko_en; //remap0 | gpio_outen
	Init_UART_PAD();
 	Init_UART();	
	Init_SYS();   //设置看门狗、中断等
			TX_Buf[4] = 0;
		break;		

		case ACT_ST:     
			close_all_module();	
			close_IO_PAD();	
			PMU_CFG = 0;
			ULPCTL = TX_Buf[0];  //固件配置ULPCTRL寄存器，校准ULPLDO输出1.5V
			mclk = TX_Buf[1];
			XTLF_IPWRB = TX_Buf[3];		
				
			wake_source = TX_Buf[2];   //唤醒源
			p_mode = STOP;  //根据不同的模式配置中断类型
			wake_event_set(wake_source,p_mode);//开启相应唤醒源中断,配置模块寄存器时，要打开相应模块的总线时钟使能
			
			Low_power_Ctrl(TX_Buf[4],TX_Buf[5],TX_Buf[6],TX_Buf[7],TX_Buf[8],TX_Buf[15],TX_Buf[11],TX_Buf[12]);//低功耗控制

			if(TX_Buf[9]==1)  //关闭BOR
				BORCTL |= off_bor;
			else  
				BORCTL &= ~off_bor;		

			if(TX_Buf[16]==1)  //关闭PDR
				PDRCTL &= ~pdr_en;   //关闭PDR
			else 
				PDRCTL |= pdr_en;   

      //==== 唤醒等待配置时间控制   ===== //
			WTCFG = (TX_Buf[17]<<2) | TX_Buf[18];			
			
			SET_MCLK(mclk,(TX_Buf[13]<<1),TX_Buf[14]);//配置进入STOP模式前的主时钟
			
			if(TX_Buf[19]==1)  //开启LPTIM工作时钟
				LSCLKSEL |= lptim_fcken;
			else  
				LSCLKSEL &= ~lptim_fcken;				
				
			if(TX_Buf[20]==1)  //开启svd工作时钟
				LSCLKSEL |= svd_clken;
			else  
				LSCLKSEL &= ~svd_clken;						
			
			if(TX_Buf[21]==1)  //LCD和LVD工作时钟选xtlf
				LSCLKSEL |= svd_lcd_clk_xtlf;
			else  
				LSCLKSEL &= ~BIT0;					
			
			Set_SYSWDT(wdtov_time_4096S);			//设置wdt周期
			close_PERCLK();	
 			
			
			PMU_CFG |= stop_mode;
			PCON |= pwd;     //进入STOP
			_nop_();
// 	EA = 0;		
// 			delay(100);
// SOFTRST = 0X5C;
			FOUT0_SEL = fout0_coreclk_div64;  //输出时钟信号
			PGFCR1 = 0;
			PGFCR2 = 0;
			PGFCR2 |= BIT4;//{PGFCR2[4]:PGFCR1[4]}=2'b10				
			AFSELG &= ~BIT4; //0: FOUT0


	INIT_CLK();
	Init_UART_PAD();
	Init_UART();
	Init_SYS();   //设置看门狗、中断等
			TX_Buf[4] = 0;
			
		break;
	
		case ACT_LPRUN_ST: 
			close_all_module();  
			close_IO_PAD();	
			CKSRC_CTRL |= pll_en;//使能PLL
			ENABLE_ADC();
		
						
			PMU_CFG = 0;
		
			ULPCTL = TX_Buf[0];  //固件配置ULPCTRL寄存器，校准ULPLDO输出1.5V
			mclk = TX_Buf[1];
		
			XTLF_IPWRB = TX_Buf[3];		

			if(TX_Buf[10]==1)  //使能lprun_err中断
				LPRUNERRIE |= lprun_ie;
			else  
				LPRUNERRIE &= ~lprun_ie;				
				
			SET_MCLK(mclk,(TX_Buf[13]<<1),TX_Buf[14]);//配置进入LPRUN模式前的主时钟
		
			_nop_();
			_nop_();
			PMU_CFG |= lprun_mode;  //进入lprun
			_nop_();
		
			delay(200);
			
			ULPCTL = TX_Buf[0];  //固件配置ULPCTRL寄存器，校准ULPLDO输出1.5V
			mclk = TX_Buf[1];


			wake_source = TX_Buf[2];   //唤醒源
			p_mode = STOP;  //根据不同的模式配置中断类型
			wake_event_set(wake_source,p_mode);//开启相应唤醒源中断
	
			Low_power_Ctrl(TX_Buf[4],TX_Buf[5],TX_Buf[6],TX_Buf[7],TX_Buf[8],TX_Buf[15],TX_Buf[11],TX_Buf[12]);//低功耗控制

			if(TX_Buf[9]==1)  //关闭BOR
				BORCTL |= off_bor;
			else  
				BORCTL &= ~off_bor;		
				
			if(TX_Buf[16]==1)  //关闭PDR
				PDRCTL &= ~pdr_en;   //关闭PDR
			else 
				PDRCTL |= pdr_en;  
			
      //==== 唤醒等待配置时间控制   ===== //
			WTCFG = (TX_Buf[17]<<2) | TX_Buf[18];					

			if(TX_Buf[19]==1)  //开启LPTIM工作时钟
				LSCLKSEL |= lptim_fcken;
			else  
				LSCLKSEL &= ~lptim_fcken;				
				
			if(TX_Buf[20]==1)  //开启svd工作时钟
				LSCLKSEL |= svd_clken;
			else  
				LSCLKSEL &= ~svd_clken;						
			
			if(TX_Buf[21]==1)  //LCD和LVD工作时钟选xtlf
				LSCLKSEL |= svd_lcd_clk_xtlf;
			else  
				LSCLKSEL &= ~BIT0;			
							
			Set_SYSWDT(wdtov_time_4096S);			//设置wdt周期
			close_PERCLK();	
			
			
			_nop_();
			tmp = PMU_CFG;
			tmp = ((tmp&0xBF)|stop_mode);
			PMU_CFG = tmp;	
			PCON |= pwd;     //进入STOP
			_nop_();


	INIT_CLK();
//  	IO4OUTEN |= BIT5;  //GPIO45 OUT CLK
//   TESTCON |= clko_coreclk | clko_en; //remap0 | gpio_outen
	Init_UART_PAD();
 	Init_UART();	
	Init_SYS();   //设置看门狗、中断等
			TX_Buf[4] = 0;
		break;				
		

		case XTLF_CLOSE_ALL:  
			
			close_IO_PAD();	
			SET_MCLK(LSCLK,0,0);
			close_all_module();	
			close_PERCLK();	
			EA = 0;
			while(1)
			{WDT_CLR();}
		break;	
		
		case XTLF_OPEN_ADC:  
			
			close_IO_PAD();			
			SET_MCLK(LSCLK,0,0);
			close_all_module();
			close_PERCLK();		
			EA = 0;
		
			ENABLE_ADC();

			while(1)
			{WDT_CLR();}		
		break;	

		case XTLF_OPEN_DISP:  
		
			close_IO_PAD();	
			SET_MCLK(LSCLK,0,0);
			close_all_module();	
			EA = 0;
			close_PERCLK();	
			
			INIT_DISP_COUT_PAD();
			DISP_TEST_EN(1);	//rin
			chip_poweron_display();
				
			while(1)
			{WDT_CLR();}		
		break;	

		case XTLF_OPEN_PLL:  
		
			close_IO_PAD();	
			SET_MCLK(LSCLK,0,0);
			close_all_module();	
			EA = 0;
			
			CKSRC_CTRL |= pll_en;
			PLLDBH = TX_Buf[0];
			PLLDBL = TX_Buf[1];
			
			close_PERCLK();			
			while(1)
			{WDT_CLR();}		
		break;	

		case XTLF_OPEN_RCHF:  
			close_IO_PAD();	
			SET_MCLK(LSCLK,0,0);
			close_all_module();	
			EA = 0;
			
			CKSRC_CTRL |= (TX_Buf[0]<<6) | rchf_en;  //RCHF 中心频率
			RCHFADJ = TX_Buf[1];
			
			close_PERCLK();			
			while(1)
			{WDT_CLR();}		
		break;		

		case XTLF_OPEN_BUF4TST:  
			close_IO_PAD();	
			SET_MCLK(LSCLK,0,0);
			close_all_module();	
			EA = 0;
		
			ENABLE_BUF4TST_VREF08();
					
			close_PERCLK();	
			while(1)
			{WDT_CLR();}		
		break;	

//		case RAM_STANDBY_TEST:
//			close_all_module();	
//			close_IO_PAD();	
//			
//			FDETCTRL = 0x00;  //open fdet
//			CLKGATE = 0xff;
//			IO1OUTEN |= BIT3;  //LED V4
//			IO1DATA = BIT3;  //LED V4 OFF
//		
//			IO4OUTEN |= BIT5;  //GPIO45 OUT CLK
//			TESTCON = clko_coreclk | clko_en; //gpio_outen
//		
//			PWMODE = 0;
//		
//			ULPCTRL = TX_Buf[0];  //固件配置ULPCTRL寄存器，校准ULPLDO输出1.5V
//			mclk = TX_Buf[1];
//		
//			XTCTRL1 = (TX_Buf[2]<<3);
//			XTCTRL2 = TX_Buf[3];
//			IPWRCTRL = (TX_Buf[4]<<4) | TX_Buf[5];		
//		
//			SET_MCLK(mclk);   //配置进入STANDBY模式前的主时钟
//			_nop_();
//			_nop_();
//			PWMODE |= standby;  //进入standby
//			_nop_();
//			tmp = xram_wr_check();//STANDBY模式下对RAM进行写读校验
//			if(tmp==0x55)
//				IO1DATA = BIT3;  //LED V4 OFF
//			else IO1DATA = 0;  //LED V4 ON if no error
//			
//			delay_us(5);
//			
//			PWMODE = 0;    //退出STB模式
//			while(!(PWMODE==0));  //查询到确实退出STB模式
//			_nop_();
//			
//	INIT_CLK();
// 	IO4OUTEN |= BIT5;  //GPIO45 OUT CLK
//  TESTCON |= clko_coreclk | clko_en; //remap0 | gpio_outen
//	Init_UART_PAD();
// 	Init_UART();						
//			TX_Buf[4] = 0;
//		
//
//		break;
			
		default:
			break;
		
	}
	
}

/************************************************************************
	函数名称:Pro_WDT
	函数功能:复位功能验证
	入口参数:无
	出口参数:无
************************************************************************/
void Pro_WDT(void)
{
	INT8U i;
	switch(Commandl)
	{
		case	READ_ALL_RSTREG:

			TX_Buf[4] = 5;				//数据长度
			for(i=0; i<5; i++)
			{
				TX_Buf[7+i] = XBYTE[WDT_ADDR+i]; 
			}	
		
		break;

		case	WRITE_ALL_RSTREG:
			
			for(i=0; i<5; i++)
			{
				XBYTE[WDT_ADDR+i] = TX_Buf[i]; 
			}					
			
			TX_Buf[4] = 5;				//数据长度
			for(i=0; i<5; i++)
			{
				TX_Buf[7+i] = XBYTE[WDT_ADDR+i]; 
			}	
		
		break;
		
		case	WRITE_ALL_RSTREG_SAME:
			
			for(i=0; i<5; i++)
			{
				XBYTE[WDT_ADDR+i] = TX_Buf[0]; 
			}				
			
			TX_Buf[4] = 5;				//数据长度
			for(i=0; i<5; i++)
			{
				TX_Buf[7+i] = XBYTE[WDT_ADDR+i]; 
			}	
		
		break;


		case SOFT_RST:   //软复位
				
			SOFTRST = 0x5c;
			_nop_();
			_nop_();

		break;
		
		case SYSWDT_OVFLOW:   //系统看门狗
				
			Set_SYSWDT(wdtov_time_8S);
			while(1){}//死循环，不喂狗

		break;		

		case SYSWDT_CLEAR:  
			while(1){WDT_CLR(); }//死循环，喂狗,死机
		break;	
	
			
		default:
		break;		
	}
}

/************************************************************************
	函数名称:Pro_DMA
	函数功能:DMA功能验证
	入口参数:无
	出口参数:无
************************************************************************/
void Pro_DMA(void)
{
	uchar i;
	INT8U wr_num;
	INT16U mov_num;
	INT16U src_addr;
	INT16U dst_addr;

	switch(Commandl)
	{
		case	READ_ALL_DMAREG:

			TX_Buf[4] = 21;				//数据长度
			for(i=0; i<21; i++)
			{
				TX_Buf[7+i] = XBYTE[DMA_ADDR+i]; 
			}	
		
		break;

		case	WRITE_ALL_DMAREG:
			
			for(i=0; i<21; i++)
			{
				XBYTE[DMA_ADDR+i] = TX_Buf[i]; 
			}			
			
			TX_Buf[4] = 21;				//数据长度
			for(i=0; i<21; i++)
			{
				TX_Buf[7+i] = XBYTE[DMA_ADDR+i]; 
			}	
		
		break;
		
		case	WRITE_ALL_DMAREG_SAME:
			
			for(i=0; i<21; i++)
			{
				XBYTE[DMA_ADDR+i] = TX_Buf[0]; 
			}			
			
			TX_Buf[4] = 21;				//数据长度
			for(i=0; i<21; i++)
			{
				TX_Buf[7+i] = XBYTE[DMA_ADDR+i]; 
			}	
		
		break;

		case	DMA_UART_TX:
			wr_num = TX_Buf[0];
			Uart0_TX_DMA(&TX_Buf[1], wr_num); 
			TX_Buf[4] = 0;				//数据长度
		break;		

		case	DMA_UART0_RXTX:
			wr_num = TX_Buf[0];
			Uart0_RX_DMA(&TX_Buf[0], wr_num); 
			Uart0_TX_DMA(&TX_Buf[0], wr_num); 
			TX_Buf[4] = 0;				//数据长度
		break;	

		case	DMA2_FLASH_RAM:
	
			mov_num = TX_Buf[0];
			mov_num = (mov_num << 8) + TX_Buf[1];
			src_addr = TX_Buf[2];
			src_addr = (src_addr << 8) + TX_Buf[3];
			dst_addr = TX_Buf[4];
			dst_addr = (dst_addr << 8) + TX_Buf[5];

			DMA2_Start(src_addr,dst_addr,mov_num);
			TX_Buf[4] = 0;				//数据长度			
		
		break;				
		
		default:
			break;
		
	}
	
}


void Pro_U7816()
{
	INT8U near write_num, i,j, func_return;//
	INT8U near times,timesinfo,error_flag;
	INT8U near U7816_buf[U7816BUFLEN]; 
	INT8U near write[80];//={0x00,0xd6,0x87,0x00};//,0x0f}; 
	INT8U near read[80];//={0x00,0xb0,0x87,0x00};//,0x0f};
	INT8U near readrandom[5];//={0x00,0x84,0x00,0x00,0x08};
	INT8U near selef[7];
// 	INT8U near write[80]={0x00,0xd0,0x81,0x00};//,0x0f}; 
// 	INT8U near read[80]={0x00,0xb0,0x81,0x00};//,0x0f};
// 	INT8U near readrandom[5]={0x00,0x84,0x00,0x00,0x08};	
	 write[0]=0x00;
	 write[1]=0xd6;
 	 write[2]=0x87;
	 write[3]=0x00;
// 	 write[4]=0x02;
// 	 write[5]=0x55;
// 	 write[6]=0xaa;

		read[0]=0x00;
		read[1]=0xb0;
		read[2]=0x87;
		read[3]=0x00;
// 		read[4]=0x02;
	
		readrandom[0]=0x00;
		readrandom[1]=0x84;
		readrandom[2]=0x00;
		readrandom[3]=0x00;
		readrandom[4]=0x08;
		
		selef[0]=0x00;
		selef[1]=0xa4;
		selef[2]=0x00;
		selef[3]=0x00;
		selef[4]=0x02;		
		selef[5]=0x3f;
		selef[6]=0x01;			
		
		
	INIT_7816_PAD();

	switch(Commandl)
	{
	
		case	READ_ALL_7816REG:
			
			TX_Buf[4] = 14;
			TX_Buf[7] = U7816CTRL0;		//U7816通道0端口寄存器 
			TX_Buf[8] = U7816CTRL1;		//U7816通道1端口寄存器  
			TX_Buf[9] = U7816FRMCTRL0;	//U7816帧格式控制寄存器0 
			TX_Buf[10] = U7816FRMCTRL1;	//U7816帧格式控制寄存器1
			TX_Buf[11] = U7816EGTCTRL;	//U7816 EGT配置寄存器 
			TX_Buf[12] = CLK_DIV;			//U7816工作时钟分配控制寄存器 
			TX_Buf[13] = PRE_DIVH;			//U7816预分频控制寄存器 
			TX_Buf[14] = PRE_DIVL;			//U7816预分频控制寄存器 
			TX_Buf[15] = U7816RXBUF;		//U7816通道0数据接收缓冲寄存器 
			TX_Buf[16] = U7816TXBUF;		//U7816通道0数据发送缓冲寄存器 
			TX_Buf[17] = U7816INTEN;		//U7816中断标志寄存器 
			TX_Buf[18] = U7816PRIMARYSTATUS;	//U7816主状态寄存器
			TX_Buf[19] = U7816ERRSTATUS;		//U7816错误信息寄存器 
			TX_Buf[20] = U7816SECONDSTATUS;	//U7816次状态寄存器
			
		break;

		case	WRITE_ALL_7816REG:

			for(i=0; i<14; i++)
			{
				XBYTE[U7816_ADDR + i] = TX_Buf[i];
			}

			TX_Buf[4] = 14;
			TX_Buf[7] = U7816CTRL0;		//U7816通道0端口寄存器 
			TX_Buf[8] = U7816CTRL1;		//U7816通道1端口寄存器  
			TX_Buf[9] = U7816FRMCTRL0;	//U7816帧格式控制寄存器0 
			TX_Buf[10] = U7816FRMCTRL1;	//U7816帧格式控制寄存器1
			TX_Buf[11] = U7816EGTCTRL;	//U7816 EGT配置寄存器 
			TX_Buf[12] = CLK_DIV;			//U7816工作时钟分配控制寄存器 
			TX_Buf[13] = PRE_DIVH;			//U7816预分频控制寄存器 
			TX_Buf[14] = PRE_DIVL;			//U7816预分频控制寄存器 
			TX_Buf[15] = U7816RXBUF;		//U7816通道0数据接收缓冲寄存器 
			TX_Buf[16] = U7816TXBUF;		//U7816通道0数据发送缓冲寄存器 
			TX_Buf[17] = U7816INTEN;		//U7816中断标志寄存器 
			TX_Buf[18] = U7816PRIMARYSTATUS;	//U7816主状态寄存器
			TX_Buf[19] = U7816ERRSTATUS;		//U7816错误信息寄存器 
			TX_Buf[20] = U7816SECONDSTATUS;	//U7816次状态寄存器
			
		break;
		
		case	WRITE_ALL_7816REG_SAME:

			for(i=0; i<14; i++)
			{
				XBYTE[U7816_ADDR + i] = TX_Buf[0];
			}

			TX_Buf[4] = 14;
			TX_Buf[7] = U7816CTRL0;		//U7816通道0端口寄存器 
			TX_Buf[8] = U7816CTRL1;		//U7816通道1端口寄存器  
			TX_Buf[9] = U7816FRMCTRL0;	//U7816帧格式控制寄存器0 
			TX_Buf[10] = U7816FRMCTRL1;	//U7816帧格式控制寄存器1
			TX_Buf[11] = U7816EGTCTRL;	//U7816 EGT配置寄存器 
			TX_Buf[12] = CLK_DIV;			//U7816工作时钟分配控制寄存器 
			TX_Buf[13] = PRE_DIVH;			//U7816预分频控制寄存器 
			TX_Buf[14] = PRE_DIVL;			//U7816预分频控制寄存器 
			TX_Buf[15] = U7816RXBUF;		//U7816通道0数据接收缓冲寄存器 
			TX_Buf[16] = U7816TXBUF;		//U7816通道0数据发送缓冲寄存器 
			TX_Buf[17] = U7816INTEN;		//U7816中断标志寄存器 
			TX_Buf[18] = U7816PRIMARYSTATUS;	//U7816主状态寄存器
			TX_Buf[19] = U7816ERRSTATUS;		//U7816错误信息寄存器 
			TX_Buf[20] = U7816SECONDSTATUS;	//U7816次状态寄存器
			
		break;
			
		case	WRITE_U7816TXBUF:

			U7816TXBUF = 0x15;//TX_Buf[1];
			while((U7816PRIMARYSTATUS & 0x02) == 0x00);
			U7816TXBUF = 0xA0;//TX_Buf[2];
			while((U7816PRIMARYSTATUS & 0x02) == 0x00);
			U7816TXBUF = 0x22;//TX_Buf[2];
			while((U7816PRIMARYSTATUS & 0x02) == 0x00);
			U7816TXBUF = 0x34;//TX_Buf[2];
			while((U7816PRIMARYSTATUS & 0x02) == 0x00);
			
			TX_Buf[4] = 1;		//U7816错误信息寄存器 
			TX_Buf[7] = U7816TXBUF;	//U7816次状态寄存器
			
		break;			
			
			
		case U7816_RESET:                   //冷复位及复位值读回

			memset( U7816_buf, 0x00, U7816BUFLEN );
			
			INIT_7816();		//

			func_return = ColdReset(U7816_buf);    //返回值定义见i2c_7816_adc.c

	    TX_Buf[4] = 34;
			TX_Buf[7] = func_return;
			for(i=0; i<33; i++)                      //ATR最多33字节
				TX_Buf[8+i] = U7816_buf[i];
			
		break;
		
		
		case U7816_READRAN:              //发送命令,read EF
			//read data
			memset( U7816_buf, 0x00, U7816BUFLEN ); 
			func_return = U7816_comm2(U7816_buf, readrandom);

			TX_Buf[4] = 11;	
			TX_Buf[7] = func_return;
			for(i=0; i<10; i++)                      //ATR最多33字节
				TX_Buf[8+i] = U7816_buf[i];
		 
		break;

		case U7816_WRITE:                //发送命令select EF,wirte EF	
			memset( U7816_buf, 0x00, U7816BUFLEN ); 
			write[4] = TX_Buf[0];
			write_num = TX_Buf[0];
			for(i=0; i<write_num; i++)
			{	
				write[5+i] = TX_Buf[i+1]; 	
			}
			func_return = U7816_comm1(U7816_buf, write);
//			delay(10);

			TX_Buf[4] = 3;	
			TX_Buf[7] = func_return;
			for(i=0; i<2; i++)                      //ATR最多33字节
				TX_Buf[8+i] = U7816_buf[i];

		break;

		case U7816_READ:              //发送命令,read EF

			//read data
			memset( U7816_buf, 0x00, U7816BUFLEN ); 
			read[4] = TX_Buf[0];
			j = TX_Buf[0];
			func_return = U7816_comm2(U7816_buf, read);

			TX_Buf[4] = j+3;	
			TX_Buf[7] = func_return;
			for(i=0; i<j+2; i++)                      //ATR最多33字节
				TX_Buf[8+i] = U7816_buf[i];
	
		break;
		
		case U7816_WRITE_DMA:                //发送命令select EF,wirte EF	
			memset( U7816_buf, 0x00, U7816BUFLEN ); 
			write[4] = TX_Buf[0];
			write_num = TX_Buf[0];
			for(i=0; i<write_num; i++)
			{	
				write[5+i] = TX_Buf[i+1]; 	
			}
			func_return = U7816_comm1_DMA(U7816_buf, write);
//			delay(10);

			TX_Buf[4] = 3;	
			TX_Buf[7] = func_return;
			for(i=0; i<2; i++)                      //ATR最多33字节
				TX_Buf[8+i] = U7816_buf[i];

		break;

		case U7816_READ_DMA:              //发送命令,read EF

			//read data
			memset( U7816_buf, 0x00, U7816BUFLEN ); 
			read[4] = TX_Buf[0];
			j = TX_Buf[0];
			func_return = U7816_comm2_DMA(U7816_buf, read);

			TX_Buf[4] = j+3;	
			TX_Buf[7] = func_return;
			for(i=0; i<j+2; i++)                      //ATR最多33字节
				TX_Buf[8+i] = U7816_buf[i];
	
		break;

		case U7816_TESTRW:		//大数据量写读校验		
			times = TX_Buf[0];
			timesinfo = times;
			error_flag = 0;
			while(times--)
			{		
					func_return = ColdReset(U7816_buf);				//cold reset
					if (func_return != 0xcc)
					{	
						error_flag = 1;
						goto	error;
					}
				
					write[4]= 0x20;			//lc
					for(j=0; j<write[4]; j++)
						write[5+j] = rand();
					func_return = U7816_comm1(U7816_buf, write);		//write file
					if(func_return != 0xff)
					{
						error_flag =3;
						goto 	error;
					}
					
					read[4] = 0x20;                   //Le
					func_return = U7816_comm2(U7816_buf, read);
					if(func_return != 0xff)
					{
						error_flag =4;
						goto 	error;
					}
					for(j=0; j<0x20; j++)
					{
						if(U7816_buf[j] != write[5+j])				//rw check
						{
							error_flag = 5;
							goto		error;
						}
						else
							error_flag = 0x55;
					}
			}

		error:
			TX_Buf[4] = 3;	     //PASS:55 FF times
			TX_Buf[7] = error_flag;    
			TX_Buf[8] = func_return;
			TX_Buf[9] = timesinfo;
			
		break;

	}
}

/************************************************************************
	函数名称:Pro_FLIP
	函数功能:FPLI251功能验证
	入口参数:无
	出口参数:无
************************************************************************/
void Pro_FLIP(void)
{
	volatile INT8U et1=0;
	volatile INT8U et2=0;
	volatile INT8U et3=0;

	switch(Commandl)
	{
		case READ_ALL_SFRREG:

			TX_Buf[4] = 128;
			read_all_sfr();
			
		break;

// 		case PCA_TEST:
// 				et1=PCA_Timer(0xC0,PCA_Buf,0x7D,0x5A);  //计算420ms内PCA计数个数=420*1000/(Tclk*4*(ffff-f800+1))
// 				//et2=PCA_Capture(0xD0,PCA_Buf,0x00,0x02,0x1A,0x0A);
// 			TX_Buf[4] = 2;
// 			if(et1==1) TX_Buf[7] = 0x55;// && (et2==1)
// 			else TX_Buf[7] = 0xAA;
// 			TX_Buf[8] = et1;		//     1：pass             
// //   		TX_Buf[9] = et2;		//       1：pass  
// 		break;
		
		case TIMER2_TEST:
// 				et3=TIMER2_Capture(T2_Buf,0x09,0x00,0x0210,0x01E0);  //内部计数源，即主时钟2M，捕捉源为32K的8分频。周期捕捉值=(2000000/32768)*8=0x01E8   (验证时32K为31250，故捕捉值为0200)
// 			TX_Buf[4] = 2;
// 			if(et3==1) TX_Buf[7] = 0x55;
// 			else TX_Buf[7] = 0xAA;
// 			TX_Buf[8] = et3;		//TIMER2 捕捉功能测试      1：pass             
		
		break;

		case TIMER0_TEST:
// 				et1=timer0_Cnt(T2_Buf,0x05,0xc0,0x90);	//16BIT定时器 	计数模式，计数源32k,5ms内计数值=5000/30.5= 164 = 0xa4

// 			TX_Buf[4] = 2;
// 			if(et1==1) TX_Buf[7] = 0x55;
// 			else TX_Buf[7] = 0xAA;
// 			TX_Buf[8] = et1;		//TIMER1 计数功能测试      1：pass   
// 		
// 			IO4OUTEN |= BIT5;  //GPIO45 OUT CLK
// 			TESTCON = clko_coreclk | clko_en; //gpio45输出32k至GPIO45，令timer1计数32K的下降沿
		break;
				
		case TIMER1_TEST:
// 				et2=timer1_Cnt(T2_Buf,0x50,0xc0,0x90);	//16BIT定时器 	计数模式，计数源32k,5ms内计数值=5000/30.5= 164 = 0xa4
// 			
// 			TX_Buf[4] = 2;
// 			if(et2==1) TX_Buf[7] = 0x55;
// 			else TX_Buf[7] = 0xAA;
// 			TX_Buf[8] = et2;		//TIMER1计数功能测试      1：pass      
// 		
// 			IO4OUTEN |= BIT5;  //GPIO45 OUT CLK
// 			TESTCON = clko_coreclk | clko_en; //gpio45输出32k至GPIO45，令timer1计数32K的下降沿		
		break;
		
		default:
			break;
		
	}
	
}

/************************************************************************
	函数名称:Pro_CLKGEN
	函数功能:CLKGEN模块功能验证
	入口参数:无
	出口参数:无
************************************************************************/
void Pro_CLKGEN()
{
	
	INT8U near i;
	INT8U *p = (INT8U *)(0x1000);
	
	switch(Commandl)
	{
		case	READ_ALL_CLKREG:

			TX_Buf[4] = 15;				//数据长度			
			for(i=0;i<15;i++)
			{
				TX_Buf[7+i] = XBYTE[RCC_ADDR+i];
			}		

		break;

		case	WRITE_ALL_CLKREG:
			for(i=0;i<15;i++)
			{
				XBYTE[RCC_ADDR+i] = TX_Buf[i];
			}	

			TX_Buf[4] = 15;				//数据长度			
			for(i=0;i<15;i++)
			{
				TX_Buf[7+i] = XBYTE[RCC_ADDR+i];
			}	

		break;

		case	WRITE_ALL_CLKREG_SAME:
		
			for(i=0;i<15;i++)
			{
				XBYTE[RCC_ADDR+i] = TX_Buf[0];
			}	

			TX_Buf[4] = 15;				//数据长度			
			for(i=0;i<15;i++)
			{
				TX_Buf[7+i] = XBYTE[RCC_ADDR+i];
			}	

		break;		

		case	CLK_OUT:
				
			FOUT0_SEL = TX_Buf[0];  //输出时钟信号
			XTLF_IPWRB = TX_Buf[1];
			CKSRC_CTRL = (TX_Buf[2]<<6) | rchf_en;  //RCHF 中心频率
			RCHFADJ = TX_Buf[3];
			RCLP_TRIM = TX_Buf[4];
			PLLDBH = TX_Buf[5];
			PLLDBL = TX_Buf[6];		
		
			
			if((TX_Buf[0]==2)||(TX_Buf[0]==4))
 				CKSRC_CTRL |= rchf_en;//使能RCHF
			if(TX_Buf[0]==6)
 				CKSRC_CTRL |= pll_en;//使能PLL
			
			if(TX_Buf[7]==1)
			{
				PGFCR1 &= ~BIT4;
				PGFCR2 |= BIT4;//{PGFCR2[4]:PGFCR1[4]}=2'b10				
				AFSELG &= ~BIT4; //0: FOUT0
			}
			else if(TX_Buf[7]==0)
			{	
				PFFCR1 &= ~BIT5;
				PFFCR2 |= BIT5;//{PFFCR2[5]:PFFCR1[5]}=2'b10				
				AFSELF &= ~BIT5; //0: FOUT0
			}
			
			TX_Buf[4] = 0;	

		break;	
			
		case	CLK_OUT1:
				
			FOUT1_SEL = TX_Buf[0];  //输出时钟信号
			XTLF_IPWRB = TX_Buf[1];
			CKSRC_CTRL = (TX_Buf[2]<<6) | rchf_en;  //RCHF 中心频率
			RCHFADJ = TX_Buf[3];
			RCLP_TRIM = TX_Buf[4];
			PLLDBH = TX_Buf[5];
			PLLDBL = TX_Buf[6];		  //fpll = 32k*DB[9:0]
		
			if((TX_Buf[0]==1)||(TX_Buf[0]==2))
 				CKSRC_CTRL |= pll_en;//使能PLL
			if(TX_Buf[0]==3)
				CKSRC_CTRL |= rchf_en;//使能RCHF	
		
			PCFCR1 &= ~BIT3;
			PCFCR2 |= BIT3;//{PCFCR2[3]:PCFCR1[3]}=2'b10				
			AFSELC |= BIT3; //0: FOUT1

			TX_Buf[4] = 0;	

		break;				
		
		case	RCHF_TEST:
			PLLDBH = 0;  	  //fpll = 32k*(DB[9:0]+1)
			PLLDBL = 0xf3;  //fpll = 32768*244 = 8M
			SET_MCLK(PLL,0,0);  //主时钟设置为PLL=8M
		
			CKSRC_CTRL |= rchf_en;
			delay(1);
			FOUT1_SEL = fout1_rchf;
			PCFCR1 &= ~BIT3;
			PCFCR2 |= BIT3;//{PCFCR2[3]:PCFCR1[3]}=2'b10			
			AFSELC |= BIT3; //1: FOUT1		
		
			delay(1);

// 				Uart1_SendInfo(sizeof(IDN)/sizeof(*IDN),IDN);  //控制频率计运行
//// 				UART1_receiveStr(49);
//// 				UartSendInfo(49,usart_temp);
// 				Uart1_SendInfo(sizeof(F_RST)/sizeof(*F_RST),F_RST);  控制频率计运行
// 				Uart1_SendInfo(sizeof(F_SET1)/sizeof(*F_SET1),F_SET1);  控制频率计运行
// 				Uart1_SendInfo(sizeof(F_SET2)/sizeof(*F_SET2),F_SET2);  控制频率计运行
// 				Uart1_SendInfo(sizeof(F_SET3)/sizeof(*F_SET3),F_SET3);  控制频率计运行
 						
			for(i=0;i<0x81;i++)
			{
				RCHFADJ = i;
				Uart1_SendInfo(sizeof(RUN)/sizeof(*RUN),RUN);
				delay(500);  //等待稳定
				Uart1_SendInfo(sizeof(MEASURE)/sizeof(*MEASURE),MEASURE);
				UART1_receiveStr(12);
				UartSendInfo(12,usart_temp);//sizeof(usart_temp)/sizeof(*usart_temp)
				delay(100);

			}	
			TX_Buf[4] = 0;
		break;	


		case	XTLF_FDET:

			FDETIE = 0;
			//FDETIE |= lfdet_ie;
			while(!(FDETIF&lfdet_if))   //检测到停振
			{WDT_CLR();}
		
			TX_Buf[4] = 1;	
			TX_Buf[7] = FDETIF&0x01;				
			FDETIF = 0;
		break;	
		
		case ADDR_OVF:
			RSTCFG |= addr_rst_en;
			*p = 0x88;
			p++;
			p++;
			p++;
			*p = 0x99;
			TX_Buf[4] = 0;
		break;
		
		
 		case	RCLP_TEST:

			PLLDBH = 0;  	  //fpll = 32k*(DB[9:0]+1)
			PLLDBL = 0xf3;  //fpll = 32768*244 = 8M
			SET_MCLK(PLL,0,0);  //主时钟设置为PLL=8M
			 		
			CKSRC_CTRL |= lp_rclpctrl;
			delay(1);
			FOUT0_SEL = fout0_rclp;
			PGFCR1 &= ~BIT4;
			PGFCR2 |= BIT4;//{PGFCR2[4]:PGFCR1[4]}=2'b10				
			AFSELG &= ~BIT4; //0: FOUT0	
		
			delay(1);

// 				Uart1_SendInfo(sizeof(IDN)/sizeof(*IDN),IDN);  //控制频率计运行
//// 				UART1_receiveStr(49);
//// 				UartSendInfo(49,usart_temp);
// 				Uart1_SendInfo(sizeof(F_RST)/sizeof(*F_RST),F_RST);  //控制频率计运行
// 				Uart1_SendInfo(sizeof(F_SET1)/sizeof(*F_SET1),F_SET1);  //控制频率计运行
// 				Uart1_SendInfo(sizeof(F_SET2)/sizeof(*F_SET2),F_SET2); // 控制频率计运行
// 				Uart1_SendInfo(sizeof(F_SET3)/sizeof(*F_SET3),F_SET3);  //控制频率计运行 
 						
			for(i=0;i<0x10;i++)
			{
				RCLP_TRIM = i;
//				Uart1_SendInfo(sizeof(RUN)/sizeof(*RUN),RUN);
				delay(500);  //等待稳定
				Uart1_SendInfo(sizeof(MEASURE)/sizeof(*MEASURE),MEASURE);
				UART1_receiveStr(12);
				UartSendInfo(12,usart_temp);//sizeof(usart_temp)/sizeof(*usart_temp)
				delay(100);

			}	
			TX_Buf[4] = 0;	

 		break;	

		case	LPRUN_RCHF:
			close_all_module();	//关闭所有外设
			close_IO_PAD();	
			PMU_CFG = 0;
		
			ULPCTL = TX_Buf[2];  //固件配置ULPCTRL寄存器，校准ULPLDO输出1.5V
			XTLF_IPWRB = TX_Buf[3];		
		
			SET_MCLK(LSCLK,0,0);//配置进入SLEEP模式前的主时钟
			_nop_();
			_nop_();

			LPRUNERRIE |= lprun_ie;
			LSCLKSEL = 0X03;//关闭LPTIM工作时钟，开启svd工作时钟，LCD和LVD工作时钟选xtlf
					
			BORCTL &= ~off_bor;	  //开BOR
				
			PDRCTL |= pdr_en;  //开PDR
				
			CKSRC_CTRL = (TX_Buf[0]<<6) | rchf_en;  //RCHF 中心频率
			RCHFADJ = TX_Buf[1];
			
			CKSRC_CTRL &= ~lp_rclpctrl;   //关闭RCLP
											
			close_PERCLK();	
			
			PMU_CFG |= lprun_mode;  //进入lprun
			_nop_();
		  while(1)
			{
 				WDT_CLR();//清狗
			}
			TX_Buf[4] = 0;		

		break;			

		case	SAME_CLK_LPRUN:
			close_all_module();	//关闭所有外设
			close_IO_PAD();	
			PMU_CFG = 0;
		
			ULPCTL = TX_Buf[0];  //固件配置ULPCTRL寄存器，校准ULPLDO输出1.5V

			XTLF_IPWRB = TX_Buf[1];		
		
			SET_MCLK(LSCLK,0,0);//配置进入SLEEP模式前的主时钟
			_nop_();
			_nop_();

			LPRUNERRIE |= lprun_ie;
			LSCLKSEL = 0X03;//关闭LPTIM工作时钟，开启svd工作时钟，LCD和LVD工作时钟选xtlf
					
			BORCTL &= ~off_bor;	  //开BOR
				
			PDRCTL |= pdr_en;  //开PDR
				
			CKSRC_CTRL &= ~rchf_en;  //关RCHF 
			CKSRC_CTRL &= ~lp_rclpctrl;   //关闭RCLP

			close_PERCLK();	
			PMU_CFG |= lprun_mode;  //进入lprun
			_nop_();
		  while(1)
			{
 				WDT_CLR();//清狗
			}
			TX_Buf[4] = 0;		
		
		break;	
		
		default:
		break;
	}
}

unsigned int CRC_TEST_EN(unsigned int crc_init_data,unsigned char datain_reflect,unsigned char dataout_reflect,unsigned char dataout_xor,unsigned int crc_xor_data,unsigned char crc_mode,unsigned int num,unsigned char *p)
{
	unsigned int i;
	unsigned int result;		
			CRC_CR = 0;
			CRC_FLSCRC = 0;
			CRC_CAL1 = (unsigned char)(crc_init_data>>8) ;    //初值高8位
			CRC_CAL0 = (unsigned char)(crc_init_data);    //初值低8位

			CRC_CR |= datain_reflect;  //输入数据是否反转
			CRC_CR |= dataout_reflect;  //输出数据是否反转
			CRC_CR |= dataout_xor;  //输出结果是否异或
			
			CRC_XOR1 = (unsigned char)(crc_xor_data>>8) ;    //异或数据高8位     
			CRC_XOR0 = (unsigned char)(crc_xor_data);      //异或数据低8位           
			
			CRC_CR |= crc_mode;  
			
			for(i=0;i<num;i++)
			{
				CRC_DRL = *p;  //写入计算值，启动CRC
				while(CRC_CR&crc_busy){}  //crc_busy=0：计算结束
				p++;
			}
			
			result = CRC_DRH;
			result = (result<<8)+CRC_DRL;
			
			return result;
						
}

unsigned int CRC_Flash_TEST(unsigned int crc_init_data,unsigned char datain_reflect,unsigned char dataout_reflect,unsigned char dataout_xor,unsigned int crc_xor_data,unsigned char crc_mode,unsigned char crc_flash_con)
{

	unsigned int result;
			
			CRC_CR = 0;
			CRC_FLSCRC = 0;
			CRC_CAL1 = (unsigned char)(crc_init_data>>8) ;    //初值高8位
			CRC_CAL0 = (unsigned char)(crc_init_data);    //初值低8位
			
			CRC_CR |= datain_reflect;  //输入数据是否反转
			CRC_CR |= dataout_reflect;  //输出数据是否反转
			CRC_CR |= dataout_xor;  //输出结果是否异或
			
			CRC_XOR1 = (unsigned char)(crc_xor_data>>8) ;    //异或数据高8位     
			CRC_XOR0 = (unsigned char)(crc_xor_data);      //异或数据低8位           
			
			CRC_CR |= crc_mode;  
			
			CRC_FLSCRC |= crc_flash_con;
			CRC_FLSCRC |= FLSCRCEN;  //启动

			while(CRC_CR&crc_busy){}  //crc_busy=0：计算结束
			
			result = CRC_DRH;
			result = (result<<8)+CRC_DRL;
			
			return result;
			
}

/************************************************************************
	函数名称:Pro_CRC
	函数功能:CRC模块功能验证
	入口参数:无
	出口参数:无
************************************************************************/
void Pro_CRC()
{
	
	INT8U near i;
	INT16U near crc_ini_d,crc_xor_d,crc_result;
	
	switch(Commandl)
	{
		case	READ_ALL_CRCREG:

			TX_Buf[4] = 8;				//数据长度			
			for(i=0;i<8;i++)
			{
				TX_Buf[7+i] = XBYTE[CRC_ADDR+i];
			}		

		break;

		case	WRITE_ALL_CRCREG:
			for(i=0;i<8;i++)
			{
				XBYTE[CRC_ADDR+i] = TX_Buf[i];
			}	

			TX_Buf[4] = 8;				//数据长度			
			for(i=0;i<8;i++)
			{
				TX_Buf[7+i] = XBYTE[CRC_ADDR+i];
			}	

		break;

		case	WRITE_ALL_CRCREG_SAME:
		
			for(i=0;i<8;i++)
			{
				XBYTE[CRC_ADDR+i] = TX_Buf[0];
			}	

			TX_Buf[4] = 8;				//数据长度			
			for(i=0;i<8;i++)
			{
				TX_Buf[7+i] = XBYTE[CRC_ADDR+i];
			}	

		break;		

		case	CRC_TEST:

			crc_ini_d = TX_Buf[0] ;
			crc_ini_d = (crc_ini_d<<8)+TX_Buf[1] ;//初值
		
			crc_xor_d = TX_Buf[5];
			crc_xor_d = (crc_xor_d<<8)+TX_Buf[6] ;//异或值
	
			crc_result = CRC_TEST_EN(crc_ini_d,TX_Buf[2],TX_Buf[3],TX_Buf[4],crc_xor_d,TX_Buf[7],TX_Buf[8],&TX_Buf[9]);
			
			TX_Buf[4] = 2;							
			TX_Buf[7] = (unsigned char)(crc_result>>8);
			TX_Buf[8] = (unsigned char)(crc_result);
			
		break;		

		case	CRC_5A5A:

			crc_ini_d = TX_Buf[0] ;
			crc_ini_d = (crc_ini_d<<8)+TX_Buf[1] ;//初值
		
			crc_xor_d = TX_Buf[5];
			crc_xor_d = (crc_xor_d<<8)+TX_Buf[6] ;//异或值
			
			crc_result = CRC_TEST_EN(crc_ini_d,TX_Buf[2],TX_Buf[3],TX_Buf[4],crc_xor_d,TX_Buf[7],2,&TX_Buf[9]);
			
			TX_Buf[4] = 2;							
			TX_Buf[7] = (unsigned char)(crc_result>>8);
			TX_Buf[8] = (unsigned char)(crc_result);
			
		break;	

		case	CRC_11223344:

			crc_ini_d = TX_Buf[0] ;
			crc_ini_d = (crc_ini_d<<8)+TX_Buf[1] ;//初值
		
			crc_xor_d = TX_Buf[5];
			crc_xor_d = (crc_xor_d<<8)+TX_Buf[6] ;//异或值
			
			crc_result = CRC_TEST_EN(crc_ini_d,TX_Buf[2],TX_Buf[3],TX_Buf[4],crc_xor_d,TX_Buf[7],4,&TX_Buf[9]);
			
			TX_Buf[4] = 2;							
			TX_Buf[7] = (unsigned char)(crc_result>>8);
			TX_Buf[8] = (unsigned char)(crc_result);
			
		break;


		case	CRC_FLASH:

			crc_ini_d = TX_Buf[0] ;
			crc_ini_d = (crc_ini_d<<8)+TX_Buf[1] ;//初值
		
			crc_xor_d = TX_Buf[5];
			crc_xor_d = (crc_xor_d<<8)+TX_Buf[6] ;//异或值
			
			crc_result = CRC_Flash_TEST(crc_ini_d,TX_Buf[2],TX_Buf[3],TX_Buf[4],crc_xor_d,TX_Buf[7],TX_Buf[8]);
			
			TX_Buf[4] = 2;							
			TX_Buf[7] = (unsigned char)(crc_result>>8);
			TX_Buf[8] = (unsigned char)(crc_result);
			
		break;


		
		default:
		break;
	}
}

/************************************************************************
	函数名称:Pro_LPTIM
	函数功能:CRC模块功能验证
	入口参数:无
	出口参数:无
************************************************************************/
void Pro_LPTIM()
{
	
	INT8U near i,et1;
	
	switch(Commandl)
	{
		case	READ_ALL_LPTIMREG:

			TX_Buf[4] = 11;				//数据长度			
			for(i=0;i<11;i++)
			{
				TX_Buf[7+i] = XBYTE[LPTIM_ADDR+i];
			}		

		break;

		case	WRITE_ALL_LPTIMREG:
			for(i=0;i<11;i++)
			{
				XBYTE[LPTIM_ADDR+i] = TX_Buf[i];
			}	

			TX_Buf[4] = 11;				//数据长度			
			for(i=0;i<11;i++)
			{
				TX_Buf[7+i] = XBYTE[LPTIM_ADDR+i];
			}	

		break;

		case	WRITE_ALL_LPTIMREG_SAME:
		
			for(i=0;i<11;i++)
			{
				XBYTE[LPTIM_ADDR+i] = TX_Buf[0];
			}	

			TX_Buf[4] = 11;				//数据长度			
			for(i=0;i<11;i++)
			{
				TX_Buf[7+i] = XBYTE[LPTIM_ADDR+i];
			}	

		break;		
			
		case LPTIM_CNT_N:
	// 1：选择时钟源，设置分频值，设置工作模式和计数模式。
// 2：设置高低位比较寄存器的值。
// 3：设置高低位目标寄存器的值
// 4：打开中断标志使能。
// 5：打开LPTEN使能位，启动计数器。
//Lptim_Cnt(lpcfg0,lpcfg1,lpcompl,lpcomph,lptargtl,lptargth,ie_con,Cnt_Max,Cnt_Min)		
	//普通计数模式：cfg0 = 0：计数源lsclk，1分频
	//------------- cfg1 = 0:tomde=0:带波形输出的普通定时器模式,连续计数模式	
  //------------- tagrth:tagrtl = 000f,16个clk产生溢出
  //=== 5ms内计数值=5000US/(30.5US*16)=0X0A		
			et1 = Lptim_Cnt(TX_Buf[0],TX_Buf[1],TX_Buf[2],TX_Buf[3],TX_Buf[4],TX_Buf[5],TX_Buf[6],0x0F,0x05);//普通
			TX_Buf[4] = 2;
			if(et1==1) TX_Buf[7] = 0x55;
			else TX_Buf[7] = 0xAA;
			TX_Buf[8] = et1;	
		break;
		
		
		default:
		break;
	}
}

/************************************************************************
	函数名称:Pro_FLSCTRL
	函数功能:FLASH控制模块功能验证
	入口参数:无
	出口参数:无
************************************************************************/
void Pro_FLSCTRL()
{
	INT16U near wr_result,i;
	INT32U near fl_addr;
	INT8U near fl_data;
	INT8U RX_Buff[512] _at_ 0xc00;
//   INT8U const RX_Buff[] = {0x55, 0xAA, 0xFF, 0x00, 0x03, 0x07, 0x05};		//版权信息	
	
	
	switch(Commandl)
	{
		case	READ_ALL_FLSCTRLREG:

			TX_Buf[4] = 7;	
			TX_Buf[7] = ERCSR;	
			TX_Buf[8] = PRCSR;                          
			TX_Buf[9] = 0x00;                        
			TX_Buf[10] = FLSKEY; 
			TX_Buf[11] = FLSIE;
			TX_Buf[12] = 0x00;                        
			TX_Buf[13] = EPFLAG; 
		

		break;

		case	WRITE_ALL_TFLSCTRLREG:
			ERCSR = TX_Buf[0];
			PRCSR = TX_Buf[1];
			FLSKEY = TX_Buf[3];
			FLSIE = TX_Buf[4];
			EPFLAG = TX_Buf[6];

			TX_Buf[4] = 7;	
			TX_Buf[7] = ERCSR;	
			TX_Buf[8] = PRCSR;                          
			TX_Buf[9] = 0x00;                        
			TX_Buf[10] = FLSKEY; 
			TX_Buf[11] = FLSIE;
			TX_Buf[12] = 0x00;                        
			TX_Buf[13] = EPFLAG; 

		break;

		case	WRITE_ALL_FLSCTRLREG_SAME:
			ERCSR = TX_Buf[0];
			PRCSR = TX_Buf[0];
			FLSKEY = TX_Buf[0];
			FLSIE = TX_Buf[0];
			EPFLAG = TX_Buf[0];

			TX_Buf[4] = 7;	
			TX_Buf[7] = ERCSR;	
			TX_Buf[8] = PRCSR;                          
			TX_Buf[9] = 0x00;                        
			TX_Buf[10] = FLSKEY; 
			TX_Buf[11] = FLSIE;
			TX_Buf[12] = 0x00;                        
			TX_Buf[13] = EPFLAG; 

		break;		

		case WRITE_DF_BYTE:
// 		RCCTRL2 = 0x58;   
// 		delay(1);
// 		SET_MCLK(RCHF_8M);
// 		delay(1);
// 	SPBRGH0 = 0x01;	
//  	SPBRGL0 = 0xA0;//高速，300,2M MCLK
			fl_addr = TX_Buf[0];
			fl_addr = (fl_addr << 8) + TX_Buf[1];
			fl_addr = (fl_addr << 8) + TX_Buf[2];
		
			fl_data = TX_Buf[3];
			write_FLASH(fl_addr,fl_data);
			TX_Buf[4] = 0;
		break;
		
		case READ_DF_BYTE:
			fl_addr = TX_Buf[0];
			fl_addr = (fl_addr << 8) + TX_Buf[1];
			fl_addr = (fl_addr << 8) + TX_Buf[2];
			fl_data = read_FLASH(fl_addr);
			TX_Buf[4] = 1;
			TX_Buf[7] = fl_data;
		break;
		
		case ERASE_SECTOR:

			fl_addr = TX_Buf[0];
			fl_addr = (fl_addr << 8) + TX_Buf[1];
			fl_addr = (fl_addr << 8) + TX_Buf[2];
			erase_FLASH(fl_addr);
			delay(1);
			TX_Buf[4] = 0;
		break;
		
		case FLS_WR_CHECK:
			fl_addr = TX_Buf[0];
			fl_addr = (fl_addr << 8) + TX_Buf[1];
			fl_addr = (fl_addr << 8) + TX_Buf[2];
			fl_data = TX_Buf[3];
			
			wr_result = flash_wr_check(fl_addr,fl_data);
			
			TX_Buf[4] = 2;
			if(wr_result==0x55aa) 
			{
				TX_Buf[7] = 0x55;
				TX_Buf[8] = 0xaa;
			}
			else 
			{
				TX_Buf[7] = (INT8U)(wr_result>>8);
				TX_Buf[8] = (INT8U)wr_result;		
			}
			
		break;

		case ERASE_CHIP:

			erase_FLASH_CHIP();
			delay(1);
			TX_Buf[4] = 0;
		break;	

		case WRITE_FLS_BUFFER:

			fl_addr = TX_Buf[0];
			fl_addr = (fl_addr << 8) + TX_Buf[1];
			fl_addr = (fl_addr << 8) + TX_Buf[2];
		
			for(i=0;i<512;i++)
			{
				RX_Buff[i] = TX_Buf[3];
			}

			write_FLASH_Buffer(fl_addr,&RX_Buff[0]);
			TX_Buf[4] = 0;
		break;		
		
		default:
		break;
	}
}


/************************************************************************
	函数名称:Pro_ANA
	函数功能:模拟模块功能验证
	入口参数:无
	出口参数:无
************************************************************************/
void Pro_ANA(void)
{
	uchar near i,ADC_Same_Num;
	unsigned int ADC_Ideal_value;
	Typ_word_bits		ADC_Convert_Data;
	
	switch(Commandl)
	{
		case	READ_ALL_ANAREG:

			TX_Buf[4] = 21;				//数据长度
			for(i=0; i<21; i++)
			{
				TX_Buf[7+i] = XBYTE[ANA_ADDR+i]; 
			}	
		
		break;
			
		case	WRITE_ALL_ANAREG:
			
			for(i=0; i<21; i++)
			{
				XBYTE[ANA_ADDR+i] = TX_Buf[i]; 
			}	
			
			
			TX_Buf[4] = 21;				//数据长度
			for(i=0; i<21; i++)
			{
				TX_Buf[7+i] = XBYTE[ANA_ADDR+i]; 
			}	
		
		break;

		case	WRITE_ALL_ANAREG_SAME:

			for(i=0; i<21; i++)
			{
				XBYTE[ANA_ADDR+i] = TX_Buf[0]; 
			}	

			TX_Buf[4] = 21;				//数据长度
			for(i=0; i<21; i++)
			{
				TX_Buf[7+i] = XBYTE[ANA_ADDR+i]; 
			}	
		
		break;


		case	BUF4TST_OUT:
			
			PFFCR1 |= BIT6;
			PFFCR2 |= BIT6;//{PFFCR2[6]:PFFCR1[6]}=2'b11,antst0_an3_en=1
			
			ANATESTSEL = 0;
			ANATESTSEL |= buf4tst_bypass;  //直接输出:使用ADC测量电源电压（主电源或LDO）时，必须将buf4tst_bypass置1
			ANATESTSEL |= TX_Buf[0];
		
			if((TX_Buf[0]==vbe1_adc)||(TX_Buf[0]==vbe2_adc)||(TX_Buf[0]==vbe3_adc)||(TX_Buf[0]==vcm_adc_buf))
			{
				ADCCTL = 0x00;
				ADCCTL &= ~adc_vana_en;   //0：ADC用作温度传感器
				
				ADCTRIM1 = 0x03;  //ADC计数调校高3位
				ADCTRIM2 = 0xff;  //ADC计数调校低8位
		  
				ADCCTL |= adc_en;  //ADC使能				
			}
			else
				ADCCTL = 0x00;
			
			ANATESTSEL |= buf4tst_en;
			
			TX_Buf[4] = 1;			
			TX_Buf[7] = ANATESTSEL;
		
		break;

		case	BUF4TST_BUF_OUT:
			

			PFFCR1 |= BIT6;
			PFFCR2 |= BIT6;//{PFFCR2[6]:PFFCR1[6]}=2'b11,antst0_an3_en=1
			
			ANATESTSEL = 0;
			ANATESTSEL |= TX_Buf[0];
			if((TX_Buf[0]==vbe1_adc)||(TX_Buf[0]==vbe2_adc)||(TX_Buf[0]==vbe3_adc)||(TX_Buf[0]==vcm_adc_buf))
			{
				ADCCTL = 0x00;
				ADCCTL &= ~adc_vana_en;   //0：ADC用作温度传感器
				
				ADCTRIM1 = 0x03;  //ADC计数调校高3位
				ADCTRIM2 = 0xff;  //ADC计数调校低8位
		  
				ADCCTL |= adc_en;  //ADC使能				
			}
			else
				ADCCTL = 0x00;
			
			ANATESTSEL |= buf4tst_en;
			
			TX_Buf[4] = 1;			
			TX_Buf[7] = ANATESTSEL;
		
		break;

		case SLEEP_BUF4TST:  
		
			close_IO_PAD();	
			close_all_module();	//关闭外设时钟
			PMU_CFG = 0;
			ULPCTL = TX_Buf[0];  //固件配置ULPCTRL寄存器，校准ULPLDO输出1.5V
			XTLF_IPWRB = TX_Buf[1];		
						
			wake_event_set(WKUP,SLEEP);//开启相应唤醒源中断,wkup（PB7）唤醒sleep
			
			RAMLPCFG = 0x03;//RAM进入不保持模式
			BORCTL |= off_bor;//关BOR
			PMU_PDCTRL = 0; //PD掉电电荷泄放使能
			CKSRC_CTRL &= ~lp_rclpctrl;//sleep/stop下关闭RCLP
			PDRCTL &= ~pdr_en;   //关闭PDR
			LSCLKSEL = 0x03;  //开启svd工作时钟,LCD和LVD工作时钟选xtlf
						
			SET_MCLK(RCHF_8M,0,0);//配置进入SLEEP模式前的主时钟

			Set_SYSWDT(wdtov_time_4096S);			//设置wdt周期

			
			LVDCTRL = 0;
			
			ENABLE_BUF4TST_VREF08();
			
			close_PERCLK();	
			PMU_CFG |= sleep_mode;
			PCON |= idl;     //进入sleep
			_nop_();

		INIT_CLK();
		Init_SYS();   //设置看门狗、中断等
		Init_UART_PAD();
		Init_UART();
		
			TX_Buf[4] = 0;
		
			

		break;	
						
		case	LVD_TEST:
			
			PERICLK_CTRL2 |= anac_clken;
			
			if(TX_Buf[5]==0x01)   //工作时钟XTLF
			{
				LSCLKSEL = svd_clken | svd_lcd_clk_xtlf;
			}
			else if(TX_Buf[5]==0)   //RCLP
			{
				LSCLKSEL = svd_clken | svd_lcd_clk_rclp;
			}	
			
			EX0 = 0; //外部中断0禁止
				
			LVDCTRL = 0;
			LVDLPC = 0;
			LVDCTRL |= TX_Buf[1];    //阈值电压配置
			LVDLPC |= TX_Buf[2]; //启动间隔配置


					
			if(TX_Buf[3]==0x01)   //欠压中断使能
			{
				LVDCTRL |= lvdpd_ie;
			}
			else if(TX_Buf[3]==0)   //欠压中断禁止
			{
				LVDCTRL &= ~lvdpd_ie;
			}			

			if(TX_Buf[4]==0x01)   //欠压恢复中断使能
			{
				LVDCTRL |= lvdpu_ie;
			}
			else if(TX_Buf[4]==0)   //欠压恢复中断禁止
			{
				LVDCTRL &= ~lvdpu_ie;
			}	
			
			if(TX_Buf[0]==0x02)   //常使能
			{
				LVDCTRL &= B1100_1111;
				LVDCTRL |= lvd_enable;
			}
			else if(TX_Buf[0]==0x01)   //间歇使能
			{
				LVDCTRL &= B1100_1111;
				LVDCTRL |= lvd_en_batch;
			}
			
			IPH = 0;
			IPL = 0;
			IPH |= BIT0;
			IPLX0 = 1;     //{IPHX0:IPLX0}=11 :　设置INT0_N中断优先级最高
			IT0 = 0;//外部中断0低电平触发		
			EX0 = 1; //外部中断0使能

			TX_Buf[4] = 1;		
			TX_Buf[7] = LVDSTAT;				
		
		break;

		case	SLEEP_LVD:
			
			close_all_module();	//关闭外设时钟
			close_IO_PAD();	
			
			PERICLK_CTRL2 |= anac_clken;
			LSCLKSEL = svd_clken | svd_lcd_clk_xtlf;
			
			EX0 = 0; //外部中断0禁止
				
			LVDCTRL = 0;
			LVDLPC = 0;
			LVDCTRL |= TX_Buf[1];    //阈值电压配置
			LVDLPC |= TX_Buf[2]; //启动间隔配置
			
			if(TX_Buf[0]==0x02)   //常使能
			{
				LVDCTRL &= B1100_1111;
				LVDCTRL |= lvd_enable;
			}
			else if(TX_Buf[0]==0x01)   //间歇使能
			{
				LVDCTRL &= B1100_1111;
				LVDCTRL |= lvd_en_batch;
			}
			
			PMU_CFG = 0;
			ULPCTL = TX_Buf[3];  //固件配置ULPCTRL寄存器，校准ULPLDO输出1.5V
			XTLF_IPWRB = TX_Buf[4];	
						
			wake_event_set(WKUP,SLEEP);//开启相应唤醒源中断,wkup（PB7）唤醒sleep
			
			RAMLPCFG = 0x03;//RAM进入不保持模式
			BORCTL |= off_bor;//关BOR
			PMU_PDCTRL = 0; //PD掉电电荷泄放不使能

			CKSRC_CTRL &= ~lp_rclpctrl;//关闭RCLP		
	
			PDRCTL &= ~pdr_en;   //关闭PDR
						
			SET_MCLK(RCHF_8M,0,0);//配置进入SLEEP模式前的主时钟


			Set_SYSWDT(wdtov_time_4096S);			//设置wdt周期

			close_PERCLK();	
			PMU_CFG |= sleep_mode;
			PCON |= idl;     //进入sleep
			_nop_();

		INIT_CLK();
		Init_SYS();   //设置看门狗、中断等
		Init_UART_PAD();
		Init_UART();
		
			TX_Buf[4] = 0;		
			
		
		break;
		
		case	PDR_EN:
			
			PDRCTL = 0;
			PDRCTL |= (TX_Buf[0]<<1);
			PDRCTL |= pdr_en;
			TX_Buf[4] = 0;			
		
		break;

		case	BOR_EN:
			
			BORCTL = 1;
			BORCTL |= (TX_Buf[0]<<1);
			BORCTL &= ~off_bor;  //打开bor
			TX_Buf[4] = 0;			
		
		break;

		case	ADC_SDM:
		
			PERICLK_CTRL2 |= adc_clken;
			
			PERICLK_CTRL2 |= TX_Buf[2]; //ADC转换时钟选择
						
			ADCCTL = 0x00;
			ADCCTL &= ~adc_vana_en;   //0：ADC用作温度传感器
			
			ADCTRIM1 = TX_Buf[0];  //ADC计数调校高3位
			ADCTRIM2 = TX_Buf[1];  //ADC计数调校低8位
		  
			ADCCTL |= adc_ie;  //中断使能
			AIE1 = 1;
		
			TX_Buf[4] = 32;				//数据长度	
			for(i=0;i<16;i++)
			{
				adc_done = 0;
				ADCCTL |= adc_en;  //ADC使能
				while(!(adc_done)){}  //adc转换完成

				TX_Buf[7+i*2] = ADCDATA1;      //取出转换结果
				TX_Buf[8+i*2] = ADCDATA2;
				ADCCTL &= ~adc_en;
				delay(1);
			}
		
		break;

		case	TEST_34401:
			Uart2_SendStr(15,Agilent34401A_Int);//初始化34401
			delay(1);
			Uart2_SendStr(21,Agilent34401A_Mea);   //UART2向万用表发送测试命令
			UART2_receiveStr(17);
			UartSendInfo(17,usart_temp); //UART3将电压数据发送到PC机
		
			TX_Buf[4] = 0;				//数据长度	

		
		break;					
		
		case	ADC_STATIC_TEST:
		
			PERICLK_CTRL2 |= adc_clken | pdc_clken;
			PERICLK_CTRL2 |= TX_Buf[2]; //ADC转换时钟选择
	//===== 初始化ADC输入引脚AN1：PF7 =======//		
			PFPPEN &= ~BIT7;
			PFPUEN &= ~BIT7;
			PFFCR1 |= BIT7;
			PFFCR2 |= BIT7;   //AN1：关闭对应引脚的数字通道控制寄存器（关闭输入使能、输出使能、上拉使能）
	//===== 初始化ADC =======//					
			ADCCTL = 0x00;
			ADCCTL |= adc_vana_en;   //ADC用于测量外部电压
			
			ADCTRIM1 = TX_Buf[0];  //ADC计数调校高3位
			ADCTRIM2 = TX_Buf[1];  //ADC计数调校低8位
			
			ANATESTSEL = 0;
			ANATESTSEL |= buf4tst_en;    //模拟测试buffer使能

			ANATESTSEL |= an1;   //Buf4tst输入信号选择AN1：PF7
			ANATESTSEL |= buf4tst_bypass;

			ADCCTL |= adc_ie;  //禁止ADC中断
			AIE1 = 1;
		
/*			
//====== 开启PE1的引脚输入功能，用于ADC测试结束标志 =======//			
			PEPPEN &= ~BIT1;  //关闭推挽
			PEPUEN &= ~BIT1;  //关闭上拉
			PEFCR1 &= ~BIT1;  
			PEFCR2 &= ~BIT1; //{PEFCR2[1]:PEFCR1[1]}=2'b00,PE1为输入功能
			//PEPUEN |= BIT1;//使能PE1的上拉
			
//====== 开启PE0的上升沿中断，用于每次ADC启动的标志 =======//			
			GPIO_EXTI_SEL3 = 0;
			GPIO_EXTI_SEL3 |= io_exti_sel_pe0;  //选择PE0作为引脚中断源
			GPIO_EXTIH_ES0 = 0xff;//关闭所有中断
			
			PEPPEN &= ~BIT0;  //关闭推挽
			PEPUEN &= ~BIT0;  //关闭上拉
			PEFCR1 &= ~BIT0;  
			PEFCR2 &= ~BIT0; //{PEFCR2[0]:PEFCR1[0]}=2'b00,PE0为输入功能
			//PEPUEN |= BIT0;//使能PE0的上拉
			
			EXTIHIF = 0;

			EX1 = 1;//外部中断1使能
			IT1 = 1;//外部中断1下降沿触发
				
			GPIO_EXTIH_ES0 &= extih0_rise_edge;  //开启PD/PE/PF/PG的bit0上升沿中断


//===== ADC循环 ======//
			while(PEDATA | BIT1)             //PE1为0表示ADC全部测试结束，接FM318的GPIO20
			{
		// 		用PE0上升沿中断,接FM318的GPIO21
				if(io_if==1)   //启动ADC转换    
				{
					io_if = 0;

					ADCCTL |= adc_en;  //ADC使能
					while(!(ADCCTL&adc_if)){}  //adc转换完成
		
					tmph = ADCDATA1;      //取出转换结果
					tmpl = ADCDATA2;      //取出转换结果
					
					ADCCTL &= ~adc_en;     

		 			UartTx_0(tmph);
					UartTx_0(tmpl);
				}
			}
			

		  while(1)
			{
				ADCCTL &= ~adc_en;     //全部测试结束
			}		
			
*/	
			Uart2_SendStr(15,Agilent34401A_Int);//初始化34401
			
			ADC_Ideal_value = 50;
			while(ADC_Ideal_value <= 2000)   //理想的ADC转换值从50到1500后结束测试
			{
//				ADCCTL |= adc_en;  //ADC使能
//				while(!(ADCCTL&adc_if)){}  //adc转换完成
//				delay(10);
				
				adc_done = 0;
				ADCCTL |= adc_en;  //ADC使能
				while(!(adc_done)){}  //adc转换完成
				
				ADC_Convert_Data.byte.HB = ADCDATA1;
				ADC_Convert_Data.byte.LB = ADCDATA2;
						
//			UartTx_3(ADCDATA1);
//			UartTx_3(ADCDATA2);
//			
//			UartTx_3(ADC_Convert_Data.byte.HB);
//			UartTx_3(ADC_Convert_Data.byte.LB);
//			UartTx_3((ADC_Convert_Data.word_val>>8));
//			UartTx_3(ADC_Convert_Data.word_val);
//			
//			
//			UartTx_3((ADC_Ideal_value>>8));
//			UartTx_3(ADC_Ideal_value);
				
				ADCCTL &= ~adc_en;			
					
//			UartTx_3(ADC_Convert_Data.byte.HB);
//			UartTx_3(ADC_Convert_Data.byte.LB);
//			UartTx_3((ADC_Convert_Data.word_val>>8));
//			UartTx_3(ADC_Convert_Data.word_val);

			delay(10);			
							
				if(ADC_Convert_Data.word_val == ADC_Ideal_value)
				{
					ADC_Same_Num++;
//					CLOSE_SWTICH();//关闭充放电开关
					V3_LED_TOG();
				}
				else if(ADC_Convert_Data.word_val > ADC_Ideal_value)
				{
					DISCHARGE(); //将PE1置为低电平，对积分器进行放电，积分器输出降低。
					V1_LED_TOG();
				}
				else if(ADC_Convert_Data.word_val < ADC_Ideal_value)
				{
					CHARGE();	        //将PE0置为低电平，对积分器进行充电，积分器输出升高。
					V2_LED_TOG();
				}	
				
				if(ADC_Same_Num==20)
				{
					V4_LED_TOG();
					ADC_Same_Num = 0;
					CLOSE_SWTICH();//关闭充放电开关
					Uart2_SendStr(21,Agilent34401A_Mea);   //UART2向万用表发送测试命令
					UART2_receiveStr(17);
					UartSendInfo(17,usart_temp); //UART3将电压数据发送到PC机
					ADC_Ideal_value++;
				}		
				
				WDT_CLR();			
				
			} 
			
			ADCCTL &= ~adc_en;
			CLOSE_SWTICH();//关闭充放电开关
			TX_Buf[4] = 0;

		break;

		case	ADC_MEA_BUF4TST_IN:    //转换buf4tst输入信号
			
			PERICLK_CTRL2 |= adc_clken | pdc_clken;
			
			PERICLK_CTRL2 |= TX_Buf[2]; //ADC转换时钟选择
			
			
			if(TX_Buf[3]==0x0b)    //ADC转换引脚电压时:AN0(PC1)
			{
				PCPPEN &= ~BIT1;
				PCPUEN &= ~BIT1;
				PCFCR1 |= BIT1;
				PCFCR2 |= BIT1;   //1、	关闭对应引脚的数字通道控制寄存器（关闭输入使能、输出使能、上拉使能）			
			}
			
			if(TX_Buf[3]==0x0C)    //ADC转换引脚电压时:AN1(PF7)
			{
				PFPPEN &= ~BIT7;
				PFPUEN &= ~BIT7;
				PFFCR1 |= BIT7;
				PFFCR2 |= BIT7;   //1、	关闭对应引脚的数字通道控制寄存器（关闭输入使能、输出使能、上拉使能）			
			}
			
			if(TX_Buf[3]==0x0D)    //ADC转换引脚电压时:AN2(PF5)
			{
				PFPPEN &= ~BIT5;
				PFPUEN &= ~BIT5;
				PFFCR1 |= BIT5;
				PFFCR2 |= BIT5;   //1、	关闭对应引脚的数字通道控制寄存器（关闭输入使能、输出使能、上拉使能）			
			}
			
			
			ADCTRIM1 = TX_Buf[0];  //ADC计数调校高3位
			ADCTRIM2 = TX_Buf[1];  //ADC计数调校低8位
		
			ANATESTSEL = 0;
			ANATESTSEL |= buf4tst_en;    //模拟测试buffer使能

			ANATESTSEL |= TX_Buf[3];   //Buf4tst输入信号选择
			ANATESTSEL |= buf4tst_bypass;
			ADCCTL = 0x00;
			ADCCTL |= adc_vana_en;  //1：ADC用于测量外部电压
		
			ADCCTL |= adc_ie;  //中断使能
			AIE1 = 1;
		
			TX_Buf[4] = 32;				//数据长度	
			for(i=0;i<16;i++)
			{
				adc_done = 0;
				ADCCTL |= adc_en;  //ADC使能
				while(!(adc_done)){}  //adc转换完成

				TX_Buf[7+i*2] = ADCDATA1;      //取出转换结果
				TX_Buf[8+i*2] = ADCDATA2;
				ADCCTL &= ~adc_en;
				delay(1);
			}

		break;
			
		case	ADC_TIME_TEST:
		
			PERICLK_CTRL2 |= adc_clken;
			
			PERICLK_CTRL2 |= TX_Buf[2]; //ADC转换时钟选择
		
			ADCCTL = 0x00;
			ADCCTL &= ~adc_vana_en;   //0：ADC用作温度传感器
			
			ADCTRIM1 = TX_Buf[0];  //ADC计数调校高3位
			ADCTRIM2 = TX_Buf[1];  //ADC计数调校低8位
		  
			ADCCTL &= ~adc_ie;  //中断使能禁止
			AIE1 = 0;
		
			TX_Buf[4] = 32;				//数据长度	
			for(i=0;i<16;i++)
			{
				adc_done = 0;
				
				PGDATA = 0;
				ADCCTL |= adc_en;  //ADC使能
				while(!(ADCIF & adc_if)){}  //adc转换完成
				PGDATA = 1;
					
				TX_Buf[7+i*2] = ADCDATA1;      //取出转换结果
				TX_Buf[8+i*2] = ADCDATA2;
				ADCCTL &= ~adc_en;
				delay(1);
			}
		
		break;		
	
		case ANA_SAME_SLEEP:
		
			close_IO_PAD();	
			close_all_module();	//关闭外设时钟
			
			PMU_CFG = 0;
			ULPCTL = TX_Buf[0];  //固件配置ULPCTRL寄存器，校准ULPLDO输出1.5V
			XTLF_IPWRB = TX_Buf[1];		
						
			wake_event_set(WKUP,SLEEP);//开启相应唤醒源中断,wkup（PB7）唤醒sleep
			
			RAMLPCFG = 0x00;//RAM进入不保持模式
			BORCTL |= off_bor;//关BOR
			PMU_PDCTRL = 0; //PD掉电电荷泄放使能
			CKSRC_CTRL &= ~lp_rclpctrl;//sleep/stop下关闭RCLP
			PDRCTL &= ~pdr_en;   //关闭PDR
			//LSCLKSEL = 0x03;  //开启svd工作时钟,LCD和LVD工作时钟选xtlf
		LSCLKSEL = 0;
						
			SET_MCLK(RCHF_8M,0,0);//配置进入SLEEP模式前的主时钟

			Set_SYSWDT(wdtov_time_4096S);			//设置wdt周期

			LVDCTRL = 0;
			close_PERCLK();	
			PMU_CFG |= sleep_mode | xtlf_off ;
			PCON |= idl;     //进入sleep
			_nop_();

		INIT_CLK();
		Init_SYS();   //设置看门狗、中断等
		Init_UART_PAD();
		Init_UART();
		
			TX_Buf[4] = 0;
			
		break;		
		
		default:
		break;
		
	}
	
}



// void adc_test(INT8U *ptr)
// {
// 		
// 	ADCCTL |= adc_ie;  //中断使能
// 	AIE0 = 1;
// 	adc_done = 0;
// 	ADCCTL |= adc_en;  //ADC使能
// 	while(!(adc_done)){}  //adc转换完成
// 	*ptr++ = (ADCDATA1&0x3f);
// 	*ptr++ = ADCDATA2;
// 	ADCCTL &= ~adc_en;

// }

// void adc_voltage(void)
// {
// 	uchar i;
// 	/*线性调整率测试程序*/
// //	E3631A_Switch(1);
// 	
// 	IO5OUTEN = 0x0f;  //IO50~IO53输出使能，用于输出指令
// 	IO1INEN |= BIT3;  //IO13用于接收状态信号
// // 	IO1PUEN |= BIT3;  
// 	
// 	
// 	UartTx_3(0x01);	
// 	IO5DATA = 0x01;   
// 	while(!(IO1DATA & BIT3))   //等待FM318设置好电源电压
// 	{WDT_CLR();}
// 	IO5DATA = 0;
// 	for(i=0;i<8;i++)
// 	{
// 		adc_test(&TX_Buf[7+i*2]);  //进行ADC测试
// 	}		
// 	UartTx_3(0x02);	
// 	
// 	
// 	
// 	IO5DATA = 0x02;   
// 	while(!(IO1DATA & BIT3))   //等待FM318设置好电源电压
// 	{WDT_CLR();}
// 	IO5DATA = 0;
// 	for(i=0;i<8;i++)
// 	{
// 		adc_test(&TX_Buf[23+i*2]); //进行ADC测试
// 	}		

// 	IO5DATA = 0x03;   
// 	while(!(IO1DATA & BIT3))   //等待FM318设置好电源电压
// 	{WDT_CLR();}
// 	IO5DATA = 0;
// 	for(i=0;i<8;i++)
// 	{
// 		adc_test(&TX_Buf[39+i*2]);  //进行ADC测试
// 	}		
// 	
// 	IO5DATA = 0x04;   
// 	while(!(IO1DATA & BIT3))   //等待FM318设置好电源电压
// 	{WDT_CLR();}
// 	IO5DATA = 0;
// 	for(i=0;i<8;i++)
// 	{
// 		adc_test(&TX_Buf[55+i*2]);  //进行ADC测试
// 	}			
// 	
// 	IO5DATA = 0x05;   
// 	while(!(IO1DATA & BIT3))   //等待FM318设置好电源电压
// 	{WDT_CLR();}
// 	IO5DATA = 0;
// 	for(i=0;i<8;i++)
// 	{
// 		adc_test(&TX_Buf[71+i*2]);  //进行ADC测试
// 	}			
// 	
// 	IO5DATA = 0x06;   
// 	while(!(IO1DATA & BIT3))   //等待FM318设置好电源电压
// 	{WDT_CLR();}
// 	IO5DATA = 0;
// 	for(i=0;i<8;i++)
// 	{
// 		adc_test(&TX_Buf[87+i*2]);  //进行ADC测试
// 	}			
// 	
// 	IO5DATA = 0x07;   
// 	while(!(IO1DATA & BIT3))   //等待FM318设置好电源电压
// 	{WDT_CLR();}
// 	IO5DATA = 0;
// 	for(i=0;i<8;i++)
// 	{
// 		adc_test(&TX_Buf[103+i*2]);  //进行ADC测试
// 	}			
// 	
// 	IO5DATA = 0x08;   
// 	while(!(IO1DATA & BIT3))   //等待FM318设置好电源电压
// 	{WDT_CLR();}
// 	IO5DATA = 0;
// 	for(i=0;i<8;i++)
// 	{
// 		adc_test(&TX_Buf[119+i*2]);  //进行ADC测试
// 	}			
// 	
// 	IO5DATA = 0x09;   
// 	while(!(IO1DATA & BIT3))   //等待FM318设置好电源电压
// 	{WDT_CLR();}
// 	IO5DATA = 0;
// 	for(i=0;i<8;i++)
// 	{
// 		adc_test(&TX_Buf[135+i*2]);  //进行ADC测试
// 	}			
// 	
// 	IO5DATA = 0x0a;   
// 	while(!(IO1DATA & BIT3))   //等待FM318设置好电源电压
// 	{WDT_CLR();}
// 	IO5DATA = 0;
// 	for(i=0;i<8;i++)
// 	{
// 		adc_test(&TX_Buf[151+i*2]);  //进行ADC测试
// 	}			

// //	E3631A_Switch(0);
// }

/************************************************************************
	函数名称:Pro_MOD
	函数功能:MOD模块功能验证
	入口参数:无
	出口参数:无
************************************************************************/
void Pro_MOD()
{
	INT8U near i;
	switch(Commandl)
	{
		case	READ_ALL_MODREG:

			TX_Buf[4] = 5;				//数据长度
			for(i=0; i<5; i++)
			{
				TX_Buf[7+i] = XBYTE[MOD_ADDR+i]; 
			}	
		
		break;
			
		case	WRITE_ALL_MODREG:
			
			for(i=0; i<5; i++)
			{
				XBYTE[MOD_ADDR+i] = TX_Buf[i]; 
			}	
			
			TX_Buf[4] = 5;				//数据长度
			for(i=0; i<5; i++)
			{
				TX_Buf[7+i] = XBYTE[MOD_ADDR+i]; 
			}	
		
		break;

		case	WRITE_ALL_MODREG_SAME:

			for(i=0; i<5; i++)
			{
				XBYTE[MOD_ADDR+i] = TX_Buf[0]; 
			}	
			
			TX_Buf[4] = 5;				//数据长度
			for(i=0; i<5; i++)
			{
				TX_Buf[7+i] = XBYTE[MOD_ADDR+i]; 
			}	
		
		break;

		default:
		break;
	}
}

/************************************************************************
	函数名称:Pro_DISP
	函数功能:DISP模块功能验证
	入口参数:无
	出口参数:无
************************************************************************/
void Pro_DISP()
{
	INT8U near i,disp_mode;//,wake_source,p_mode,mclk;

	switch(Commandl)
	{
		case	READ_ALL_DISPREG:

			TX_Buf[4] = 32;
			for(i=0; i<32; i++)
			{
				TX_Buf[7+i] = XBYTE[DISP_ADDR+i]; 
			}	
					
		break;

		case	WRITE_ALL_DISPREG:
			
			for(i=0; i<32; i++)
			{
				XBYTE[DISP_ADDR+i] = TX_Buf[i]; 
			}	
			
			TX_Buf[4] = 32;				//数据长度
			for(i=0; i<32; i++)
			{
				TX_Buf[7+i] = XBYTE[DISP_ADDR+i]; 
			}	
		
		break;

		case	WRITE_ALL_DISPREG_SAME:

			for(i=0; i<32; i++)
			{
				XBYTE[DISP_ADDR+i] = TX_Buf[0]; 
			}	
			
			TX_Buf[4] = 32;				//数据长度
			for(i=0; i<32; i++)
			{
				TX_Buf[7+i] = XBYTE[DISP_ADDR+i]; 
			}	
		
		break;
			
		case	DISP_TEST_RIN:	
			
			disp_mode = 1;  //rin
			INIT_DISP_RIN_PAD();
			DISP_TEST_EN(disp_mode);
			chip_poweron_display();
			TX_Buf[4] = 0;	
		
		
		//Added By ZRH,YANGYU_Test
		
// 			DISPDATA0 = 0xff;
// 			DISPDATA1 = 0xff;
// 			DISPDATA2 = 0xff;
// 			DISPDATA3 = 0xff;
// 			DISPDATA4 = 0xff;
// 			DISPDATA5 = 0xff;
// 			DISPDATA6 = 0xff;
// 			DISPDATA7 = 0xff;
// 			DISPDATA8 = 0xff;
// 			DISPDATA9 = 0xff;
// 			DISPDATA10 = 0xff;
// 			DISPDATA11 = 0xff;
// 			DISPDATA12 = 0xff;
// 			DISPDATA13 = 0xff;
// 			DISPDATA14 = 0xff;
// 			DISPDATA15 = 0xff;
// 			DISPDATA16 = 0xff;
// 			DISPDATA17 = 0xff;
		
		
		
			
		break;
		
		case	DISP_TEST_COUT:	
		
			disp_mode = 0;  //cout
			INIT_DISP_COUT_PAD();
			DISP_TEST_EN(disp_mode);
			chip_poweron_display();
			TX_Buf[4] = 0;	

		break;		

		case	SLEEP_DISP_RIN:	

			close_IO_PAD();	
			close_all_module();	//关闭外设时钟
			
			PMU_CFG = 0;
			ULPCTL = TX_Buf[11];  //固件配置ULPCTRL寄存器，校准ULPLDO输出1.5V
			XTLF_IPWRB = TX_Buf[12];	
						
			wake_event_set(WKUP,SLEEP);//开启相应唤醒源中断,wkup（PB7）唤醒sleep
			
			RAMLPCFG = 0x03;//RAM进入不保持模式
			BORCTL |= off_bor;//关BOR
			PMU_PDCTRL = 0; //PD掉电电荷泄放不使能
			CKSRC_CTRL &= ~lp_rclpctrl;//sleep/stop下关闭RCLP
			PDRCTL &= ~pdr_en;   //关闭PDR
			LSCLKSEL = 0x03;  //开启svd工作时钟,LCD和LVD工作时钟选xtlf
						
			SET_MCLK(RCHF_8M,0,0);//配置进入SLEEP模式前的主时钟

			if(TX_Buf[13]==1) INIT_DISP_RIN_PAD();    //外部带载
				
			disp_mode = 1;  //rin		
			DISP_TEST_EN(disp_mode);
			chip_poweron_display();			

			Set_SYSWDT(wdtov_time_4096S);			//设置wdt周期
	
			close_PERCLK();	
			PMU_CFG |= sleep_mode;
			PCON |= idl;     //进入sleep
			_nop_();

		INIT_CLK();
		Init_SYS();   //设置看门狗、中断等
		Init_UART_PAD();
		Init_UART();
		
			TX_Buf[4] = 0;

		break;		

		case	SLEEP_DISP_COUT:	
		
			close_IO_PAD();	
			close_all_module();	//关闭外设时钟
			
			PMU_CFG = 0;
			ULPCTL = TX_Buf[11];  //固件配置ULPCTRL寄存器，校准ULPLDO输出1.5V
			XTLF_IPWRB = TX_Buf[12];	
						
			wake_event_set(WKUP,SLEEP);//开启相应唤醒源中断,wkup（PB7）唤醒sleep
			
			RAMLPCFG = 0x03;//RAM进入不保持模式
			BORCTL |= off_bor;//关BOR
			PMU_PDCTRL = 0; //PD掉电电荷泄放使能
			CKSRC_CTRL &= ~lp_rclpctrl;//sleep/stop下关闭RCLP
			PDRCTL &= ~pdr_en;   //关闭PDR
			LSCLKSEL = 0x03;  //开启svd工作时钟,LCD和LVD工作时钟选xtlf
						
			SET_MCLK(RCHF_8M,0,0);//配置进入SLEEP模式前的主时钟

			if(TX_Buf[13]==1) INIT_DISP_COUT_PAD();    //外部带载
			disp_mode = 0;  //COUT
			DISP_TEST_EN(disp_mode);
			chip_poweron_display();			

			Set_SYSWDT(wdtov_time_4096S);			//设置wdt周期

			close_PERCLK();	
			PMU_CFG |= sleep_mode;
			PCON |= idl;     //进入sleep
			_nop_();

		INIT_CLK();
		Init_SYS();   //设置看门狗、中断等
		Init_UART_PAD();
		Init_UART();
		
			TX_Buf[4] = 0;

		break;		

		case	SLEEP_SAME_con:	
			close_all_module();	//关闭外设时钟
			close_IO_PAD();	
			DISPCTRL = 0;
			LCDSET = 0;
			LCDTEST = 0;
			
			PMU_CFG = 0;
			ULPCTL = TX_Buf[0];  //固件配置ULPCTRL寄存器，校准ULPLDO输出1.5V
			XTLF_IPWRB = TX_Buf[1];		
						
			wake_event_set(WKUP,SLEEP);//开启相应唤醒源中断,wkup（PB7）唤醒sleep
			
			RAMLPCFG = 0x03;//RAM进入不保持模式
			BORCTL |= off_bor;//关BOR
			PMU_PDCTRL = 0; //PD掉电电荷泄放使能
			CKSRC_CTRL &= ~lp_rclpctrl;//sleep/stop下关闭RCLP
			PDRCTL &= ~pdr_en;   //关闭PDR
			LSCLKSEL = 0x03;  //开启svd工作时钟,LCD和LVD工作时钟选xtlf
						
			SET_MCLK(RCHF_8M,0,0);//配置进入SLEEP模式前的主时钟

			Set_SYSWDT(wdtov_time_4096S);			//设置wdt周期

			
			close_PERCLK();	
			PMU_CFG |= sleep_mode;
			PCON |= idl;     //进入sleep
			_nop_();

		INIT_CLK();
		Init_SYS();   //设置看门狗、中断等
		Init_UART_PAD();
		Init_UART();
		
			TX_Buf[4] = 0;
			
		break;		
		
		case	LCD_TEST_MODE:	
		
			INIT_DISP_COUT_PAD();
			disp_mode = 0;  //COUT
			DISP_TEST_EN(disp_mode);

			if(TX_Buf[11]==0)  
			{
					LCDTEST &= ~disp_lcdtest_lcctrl;  
			}
			else if(TX_Buf[11]==1) 
			{
					LCDTEST |= disp_lcdtest_lcctrl;  
			}				
		
			DISPDATA0 = TX_Buf[12];
			DISPDATA1 = TX_Buf[13];
			DISPDATA2 = TX_Buf[14];
			DISPDATA3 = TX_Buf[15];
			DISPDATA4 = TX_Buf[16];
		
			LCDTEST |= disp_lcdtesten;  //LCD测试模式使能

			TX_Buf[4] = 0;	
			
		break;

		case LCD_DISP_MODE:
		
			INIT_DISP_COUT_PAD();
			disp_mode = 1;  //rin
			DISP_TEST_EN(disp_mode);	

			if(TX_Buf[11]==0)  
			{
					DISPCTRL &= ~disp_test;  //显示全灭
			}
			else if(TX_Buf[11]==1)  
			{
					DISPCTRL |= disp_test;  //显示全亮
			}		
			
			DISPCTRL |= disp_dispmod;  //显示测试模式

			TX_Buf[4] = 0;	
		break;


		
		default:
		break;
	}
}

/************************************************************************
	函数名称:Pro_PAD
	函数功能:PAD模块功能验证
	入口参数:无
	出口参数:无
************************************************************************/
void Pro_PAD(void)
{
	INT8U near i;
	
	switch(Commandl)
	{
		case READ_ALL_PADREG:

			TX_Buf[4] = 77;
			for(i=0; i<77; i++)
			{
				TX_Buf[7+i] = XBYTE[PAD_ADDR+i]; 
			}	
			
		break;

		case	WRITE_ALL_PADREG:
			
			for(i=0; i<77; i++)
			{
				XBYTE[PAD_ADDR+i] = TX_Buf[i]; 
			}	
			
			TX_Buf[4] = 77;				//数据长度
			for(i=0; i<77; i++)
			{
				TX_Buf[7+i] = XBYTE[PAD_ADDR+i]; 
			}	
		
		break;

		case	WRITE_ALL_PADREG_SAME:

			for(i=0; i<77; i++)
			{
				XBYTE[PAD_ADDR+i] = TX_Buf[0]; 
			}	
			
			TX_Buf[4] = 77;				//数据长度
			for(i=0; i<77; i++)
			{
				TX_Buf[7+i] = XBYTE[PAD_ADDR+i]; 
			}	
		
		break;
			
		default:
		break;
	}
}

/************************************************************************
	函数名称:Pro_IDD
	函数功能:功耗测试
	入口参数:无
	出口参数:无
************************************************************************/
void Pro_IDD(void)
{
	INT8U m_clk;
	
	switch(Commandl)
	{
		case WORK_MIN_IDD:
			m_clk = TX_Buf[0];
			SET_MCLK(m_clk,(TX_Buf[1]<<1),TX_Buf[2]);//配置进入SLEEP模式前的主时钟
			
			RCHFADJ = TX_Buf[3];
			XTLF_IPWRB = TX_Buf[4];
			RCLP_TRIM = TX_Buf[5];
			PLLDBH = TX_Buf[6];
			PLLDBL = TX_Buf[7];		

			close_IO_PAD();	
			
			if(TX_Buf[8]==1)
			{
				FOUT0_SEL = fout0_coreclk_div64;  //输出时钟信号
				PGFCR1 &= ~BIT4;
				PGFCR2 |= BIT4;//{PGFCR2[4]:PGFCR1[4]}=2'b10				
				AFSELG &= ~BIT4; //0: FOUT0
			}
		
		
			PDRCTL = pdr_en;  //打开PDR
			BORCTL = 0;  //打开BOR
			while(1)
			{WDT_CLR();}			
		
		break;
			
			
			
		default:
		break;
	}	
}