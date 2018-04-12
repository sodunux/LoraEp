#include "LoraEp.h"
#include "Init.h"
INT8U LoraStatus;
INT16U LP_tim_cnt;
INT8U LowVolt;
INT8U Rtc_workday_flag;

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


void RtcConfig()
{
	#ifdef RTC_ON 
	RTCWE = 0xAC; //使能RTC写
	RTCIE2 |= hourie;//小时中断
	BCDSEC = 0;
	BCDMIN = 0;
	BCDHOUR = 9;//复位初始值为9,建议换电池时间为早上9点。
	BCDDATE = 0;
	BCDWEEK = 0;
	BCDMONTH = 0;
	BCDYEAR = 0;
	RTCWE = 0xCA;
	#endif
}


void RtcHander()
{
	#ifdef RTC_ON
	//BCDHOUR
	if((BCDHOUR>=8)&&(BCDHOUR<=20))
		Rtc_workday_flag=1;
	else
		Rtc_workday_flag=0;
	#endif
	
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

/************************************************************************/
/* 函数名：INIT_CLK												*/
/* 功能说明：CLK初始化子程序      					    		*/
/* 入口参数：N/A													*/
/* 出口参数：N/A													*/
/************************************************************************/
void INIT_CLK(void)
{
	CKSRC_CTRL = rchf_8M | rchf_en;
	RCHFADJ = 0X40;
		
	//RCHF=时钟trim
	HSDIVSEL = hsclk_rchf | hsclk_div2;
	MCLKSEL = mclk_hsclk;
	PERICLK_CTRL0 = 0;
	PERICLK_CTRL1 = B1000_0011;
	PERICLK_CTRL2 = B1001_1100;
	LSCLKSEL = B0000_0111;

	WDT_CLR();//先进行一次清狗
	WDTCFG = wdtov_time_4096S;//设置周期
	
	PDRCTL = pdr_en;  //打开PDR	
	BORCTL |= off_bor;  //关闭BOR
	EA = 1;			//打开中断
	IPH = 0;
	IPL = 0;
	IPH |= BIT0;
	IPLX0 = 1; //{IPHX0:IPLX0}=11 :　设置INT0_N中断优先级最高
	IT0 = 1;//外部中断0低电平触发		
	EX0 = 1; //外部中断0使能			
	
	
	XTLF_IPWRB = 0x03;	//XTLF I=300 nA
	
// PBPUEN |=BIT7;
// PBFCR1 &= ~BIT7;
// PBFCR2 |= BIT7;
// AFSELB |= BIT7;//1: WKUP1	

	GPIO_EXTI_SEL0 = 0;
	GPIO_EXTI_SEL0 |= io_exti_sel_pb7;  //选择PB7作为引脚中断源
// 	GPIO_EXTIH_ES1 = 0xff;//关闭所有中断
	GPIO_EXTIL_ES1 = 0xff;//关闭所有中断
	
	
	PBPPEN = 0;  //关闭推挽
	PBPUEN = 0;  //关闭上拉
	PBFCR1 &= ~BIT7;  
	PBFCR2 &= ~BIT7; //{PBFCR2[7]:PBFCR1[7]}=2'b00,Pb7为输入功能
	//PBPUEN |= BIT7;//使能PB7的上拉
	
	EXTILIF = 0;


	EX1 = 1;//外部中断1使能
	IT1 = 1;//外部中断1下降沿触发
	//Q
// 	GPIO_EXTIH_ES1 &= extih7_fall_edge;  //开启低位PD/PE/PF/PG的bit6上升沿中断	
	GPIO_EXTIL_ES1 &= extil7_fall_edge;  //开启低位Pa/Pb/Pc的bit7下降沿中断	
	
	
// 			FOUT0_SEL = fout0_rchf_div64;
// 			PFFCR1 &= ~BIT5;
// 			PFFCR2 |= BIT5;//{PFFCR2[5]:PFFCR1[5]}=2'b10				
// 			AFSELF &= ~BIT5; //0: FOUT0		
}


/************************************************************************/
/* 函数名：INIT_SYSWDT													*/
/* 功能说明：使能看门狗							*/
/* 入口参数：N/A													*/
/* 出口参数：N/A													*/
/************************************************************************/

void WDT_CLR()
{
	PERICLK_CTRL2 |= wdt_clken;
	WDTSERV = 0x5A;
}


void close_IO_PAD(void)
{

	PERICLK_CTRL2 |= pdc_clken;
	PADATA = 0xff;	
	PAPPEN = 1;//关闭PA0的推挽功能
	//PBPPEN = 0;
	PCPPEN = 0;
	PDPPEN = 0;
	PEPPEN = 0;
	PFPPEN = 0;
	PGPPEN = 0;
	PH0PPEN = 0;

	PAFCR2 = 0;
	//PBFCR2 = 0;
	PCFCR2 = 0;
	PDFCR2 = 0;
	PEFCR2 = 0;
	PFFCR2 = 0;
	PGFCR2 = 0;
	PHFCR2 = 0;
	
	PAFCR1 = 0XFF;
	//PBFCR1 = 0XFF;
	PCFCR1 = 0XFF;
	PDFCR1 = 0XFF;
	PEFCR1 = 0XFF;
	PFFCR1 = 0XFF;
	PGFCR1 = 0XFF;
	PHFCR1 = 0XFF;		 //设置为输出使能
	
	
}

void Led_LoraON()
{

	LedCtrl(GLED,LED_ON);
	delay(20);
}

void Led_LowVolt()
{
	LedCtrl(RLED,LED_ON);
	delay(20);
}

void LedSignal()
{
	INT16U i,j;
	static INT8U s=0;
	
	for(i=0;i<1000;i++)
	{
		for(j=0;j<100;j++)
			_nop_();
	}
	if(s==0)
	{
		LedCtrl(RLED,LED_OFF);
		LedCtrl(GLED,LED_OFF);
		s=1;	
	}
	else
	{
		LedCtrl(RLED,LED_ON);
		LedCtrl(GLED,LED_ON);
		s=0;					
	}
}

//进入SleepMode
void EnterSleepMode()
{
			close_IO_PAD();	
			PMU_CFG = 0;
			PMU_CFG |= sleep_mode;
			PCON |= idl;     //进入Sleep
			_nop_();
}



//LpTim设置

void LpTimConfig()
{
	// 1：选择时钟源，设置分频值，设置工作模式和计数模式。
	// 2：设置高低位比较寄存器的值。
	// 3：设置高低位目标寄存器的值
	// 4：打开中断标志使能。
	// 5：打开LPTEN使能位，启动计数器。
	//Lptim_Cnt(lpcfg0,lpcfg1,lpcompl,lpcomph,lptargtl,lptargth,ie_con,Cnt_Max,Cnt_Min)		
	LPTIMIF = 0;
	//LPTCFG0 = B0001_0000; 
	LPTCFG0 = B0000_0000;	//2s（1分频）
	//LPTCFG0 = B0000_0111; //系统时钟，128分频
	LPTCFG1 = B0000_0000;			//普通定时模式，连续计数模式
	
	LPTCMPL = 0;
	LPTCMPH = 0;
	TARGETL = 0xFF;
	TARGETH = 0xFF;
	LPTIMIE = ovie;
	LP_tim_cnt = 0;		
	LPTIMIF=0;
	LPTCTRL |= lpten;		//启动LPTIM
}

//发送数据到Lora模块
void LoraSend(INT8U *datbuf,INT8U len)
{
	#ifndef RTC_ON
	INT8U i;
	for(i=0;i<len;i++)
	{
		UartTx_0(datbuf[i]);
		delay(5);
	}
	#else 
	INT8U i;
	if(Rtc_workday_flag==1)
	{
		for(i=0;i<len;i++)
		{
			UartTx_0(datbuf[i]);
			delay(5);
		}		
	}
	#endif
}



//Uart设置
void Uart_Config()
{
	PFFCR1 &= ~BIT4;
	PFFCR2 |= BIT4;//{PFFCR2[4]:PFFCR1[4]}=2'b10,alternate function 
	AFSELF |= afself4_txd0;//TXD0功能	

	PFFCR1 &= ~BIT3;
	PFFCR2 |= BIT3;//{PFFCR2[3]:PFFCR1[3]}=2'b10,alternate function 
	AFSELF |= afself3_rxd0;//RXD0功能	
		
	
 	PFFCR1 &= ~BIT2;
 	PFFCR2 |= BIT2;//{PAFCR2[1]:PAFCR1[1]}=2'b10,alternate function 
 	AFSELF |= afself2_txd1;//TXD1功能	

 	PFFCR1 &= ~BIT1;
 	PFFCR2 |= BIT1;//{PAFCR2[0]:PAFCR1[0]}=2'b10,alternate function 
 	AFSELF |= afself1_rxd1;//RXD1功能		
	
	RXSTA0 = rxen | pdsel_8_n;	//8位数据，无奇偶校验,接收使能
	TXSTA0 = txen;		//发送使能 txen
	
	RXSTA1 = rxen | pdsel_8_n;	//8位数据，无奇偶校验,接收使能
	TXSTA1 = txen;		//发送使能 txen	
	
	SPBRGH0 = 0x00;	
 	SPBRGL0 = 0x47;//115200,8M MCLK
	
 	SPBRGH1 = 0x03;	
  SPBRGL1 = 0x40;//9600,8M MCLK	
	
}

void LedConfig()
{
	PADATA = 0xff;	
	PAFCR2 &= 0X0F;
	PAPPEN |= BIT4 | BIT5;//使能PA0的推挽功能
	PAFCR1 |= BIT4 | BIT5 | BIT6 | BIT7;//{PAFCR2[0]:PAFCR1[0]}=2'b01,使能PA0输出
	
//	PADATA = 0x00;	
}

void LedCtrl(INT8U Ledn,INT8U LED_Status)
{
	
	if(Ledn==GLED)
	{
		if(LED_Status==LED_OFF)
			PADATA |=GLED;
		else
			PADATA&=~GLED;
	}
	if(Ledn==RLED)
	{
		if(LED_Status==LED_OFF)
			PADATA |=RLED;
		else
			PADATA&=~RLED;
	}
	
}

//SVD设置
void  LoraSVDConfig()
{
	LowVolt=0;
	LVDCTRL = B0110_0100 ; //
	LVDLPC	= B0000_0000 ; //4s
}


void LpTimHander()
{
	INT8U buff1[2]={LORA_ID,LORA_IDLE};
	WDT_CLR();	Uart_Config();
	LedConfig();
	LP_tim_cnt ++;
	if(LoraStatus==Lora_ON)
	{
		buff1[1]=LORA_BUSY;
	}
	if(LP_tim_cnt==900) //30min=30*60=2*900
	{
		LoraSend(buff1,2);
		LP_tim_cnt=0;
	}
}

void SVDHander()
{
	WDT_CLR();
	LowVolt=1;
}

void WkupHander()
{
	INT8U buff2[2]={LORA_ID,LORA_IDLE};
	INT8U buff3[2]={LORA_ID,LORA_BUSY};
	WDT_CLR();
	Uart_Config();
	LedConfig();
	if(LoraStatus==Lora_ON)
		{
			LoraStatus=Lora_OFF;
			LoraSend(buff2,2);
		}
		
	else
	{
		LoraStatus=Lora_ON;	
		LoraSend(buff3,2);
	}
			
}


void LoraInit()
{
	INIT_CLK();
	Uart_Config();
	LedConfig();	
	LpTimConfig();
	LoraSVDConfig();
	RtcConfig();
	LoraStatus=Lora_OFF; //会议室初始态为空闲
	Rtc_workday_flag=1; 
}






