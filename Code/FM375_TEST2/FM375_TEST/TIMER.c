#include "init.h"

// uchar near PCA_Buf[20];
// uchar PCA_Cmp_Cnt;
// uchar PCA_Cap_Cnt;
// uchar near PCA_Cap_Buf[12];
// uchar PCA_PWM_cnt;

uchar near T2_Buf[10];
uchar T2_Cap_Cnt;
uchar near T2_Cap_Buf[12];

INT16U LP_tim_cnt;

void INIT_T0(void)
{
	TR0 = 0;
	TMOD |= 0x01;		//T1挂起，T0为定时器方式，16位定时器，
	ET0	= 1;				//T0中断使能
	TH0 = 0xF2;				//定时寄存器赋值
	TL0 = 0xFA;				//定时寄存器赋值    8M主时钟下5ms
}
// void INIT_T1(void)
// {
// 	TR1 = 0;
// 	TMOD |= 0x10;		//T1挂起，T1为定时器方式，16位定时器，
// 	ET1	= 1;				//T1中断使能
// 	TH0 = 0xF2;				//定时寄存器赋值
// 	TL0 = 0xFA;				//定时寄存器赋值    8M主时钟下5ms
// }

/*
uchar PCA_Timer(uchar Num,uchar *PCA_Buf2, uchar Cnt_Max,uchar Cnt_Min)
{
	*PCA_Buf2++ = Num;
	//====中断配置==================
	
	//====寄存器配置，使用clk/4作为时钟===============

	CH= 0xF8;
	CL= 0x00;
							// 	 7 		 6 		 5	 	 4 		 3 		 2 		 1 		 0
	CCON = 0x00;			//	CF 		CR 		-- 		CCF4 	CCF3 	CCF2 	CCF1 	CCF0	
	CMOD = 0x03;			//	CIDL 	WDTE 	UF2 	UF1 	UF0 	CPS1 	CPS0 	ECF
																	//  0      0 :clk/12
																	//  0      1 :clk/4
																	//  1      0 :time0 overflow
																	//  1      1 :ECI

	//	PCA捕捉控制寄存器
	CCAPM0    = 0;			//	-- 		ECOMx	CAPPx 	CAPNx 	MATx 	TOGx 	PWMx 	ECCFx	
	CCAPM1    = 0;	
	CCAPM2    = 0;	
	CCAPM3    = 0;	

	CCAP0L    = 0;			
	CCAP0H    = 0;
	
	PCA_Cmp_Cnt = 0;				//pca定时中断计数
	
	INIT_T1();						//timer1参数配置
	//启动timer1						
	Timer1_Test_Cnt = 84;				//420ms计时延时
	Timer1_Test_Statues = 0;			//定时标志
	TR1=1;							//启动cpu_Time1
	
	EC = 1;						//PCA中断使能开启
	CCON |=B0100_0000;				//启动PCA Module
	 while (Timer1_Test_Statues==0)		//T1定时器计时等待 420ms后，Timer_Test_Statues为1
	{
		WDT_CLR();
	}
	 
	 TR1=0;
	 TMOD &= 0x0F;	
	 CCON &=B1011_1111;				//关闭PCA Time Module
	 EC = 0;
	
   *PCA_Buf2++ = PCA_Cmp_Cnt;
 UartTx_0(PCA_Cmp_Cnt);
 UartTx_0(0x88);
   if ((PCA_Cmp_Cnt > Cnt_Max ) ||(PCA_Cmp_Cnt < Cnt_Min))
   {
		*PCA_Buf2++ = 0x02;
//         	return (PCA_Buf2);	
		 return(0x02);	
   }
	 else
	{
		*PCA_Buf2++ = 0x01;	
//         	return (PCA_Buf2);		
		return(0x01);			
	}

}



uchar PCA_Capture(uchar Num,uchar *PCA_buf2,uchar CON,uchar MOD,uchar Cnt_Max,uchar Cnt_Min)
{
	uchar i,temp;
	union B16_B08 CAP_Value[3];
	union B16_B08 a,b;

	*PCA_buf2++ = Num;
	//====中断配置==================

	//====寄存器配置===============
	CCON        = CON;			//	CF 		CR 		-- 		CCF4 	CCF3 	CCF2 	CCF1 	CCF0
	CMOD        = MOD;			//	CIDL 	WDTE 	UF2 	UF1 	UF0 	CPS1 	CPS0 	ECF
																		//  0      0 :clk/12
																		//  0      1 :clk/4
																		//  1      0 :time0 overflow
																		//  1      1 :ECI
//	PCA捕捉控制寄存器
	CCAPM0 = 0;
	CCAPM1 = 0;
	CCAPM2 = 0;
	CCAPM3 = 0;
	
	switch(Num)
	{
		case 0xD0:   //	module0
				CCAPM0    = B0010_0001;//使能中断，捕获上沿			//	-- 		ECOMx	CAPPx 	CAPNx 	MATx 	TOGx 	PWMx 	ECCFx	
				CCAP0L    = 0x00;			
				CCAP0H    = 0x00;//捕捉中断有效时，保存计数CH/CL中的计数值
				break;
				
		case	 0xD1:
				CCAPM1    = B0010_0001;			//	-- 		ECOMx	CAPPx 	CAPNx 	MATx 	TOGx 	PWMx 	ECCFx	
				CCAP1L    = 0x00;			
				CCAP1H    = 0x00;
				break;

		case	 0xD2:
				CCAPM2    = B0010_0001;			//	-- 		ECOMx	CAPPx 	CAPNx 	MATx 	TOGx 	PWMx 	ECCFx	
				CCAP2L    = 0x00;			
				CCAP2H    = 0x00;
				break;

		case	 0xD3:
				CCAPM3   = B0010_0001;			//	-- 		ECOMx	CAPPx 	CAPNx 	MATx 	TOGx 	PWMx 	ECCFx	
				CCAP3L    = 0x00;			
				CCAP3H    = 0x00;
				break;

		default:
				break;
		


	}


	PCA_Cap_Cnt = 0;
	
	for(i=0;i<12;i++)
	{
		PCA_Cap_Buf[i]=0;
	}
	
	
	
	//===增加定时控制,防止捕捉启动失败,导致死循环========
	INIT_T1();							//timer1参数配置					
	Timer1_Test_Cnt = 1;					//5ms计时延时
	Timer1_Test_Statues = 0;
	TR1=1;	
	
	EC = 1;
	CCON |=B0100_0000;		//启动PCA Module
	while(PCA_Cap_Cnt<12)
	{

		if(Timer1_Test_Statues==1)			//在5ms内无法完成捕捉则直接退出循环
		{	
			break;
		}
	}
	
	CCON =0;		//关闭PCA Module
	EC = 0;
	TR1=0;								//关闭定时器
	TMOD &= 0x0F;	

	
	
	*PCA_buf2++ = PCA_Cap_Cnt;
	
//  UartTx_0(PCA_Cmp_Cnt);
//  UartTx_0(0x88);

	CAP_Value[0].B08[0] = PCA_Cap_Buf[6];
	CAP_Value[0].B08[1] = PCA_Cap_Buf[7];
	CAP_Value[1].B08[0] = PCA_Cap_Buf[8];
	CAP_Value[1].B08[1] = PCA_Cap_Buf[9];
	CAP_Value[2].B08[0] = PCA_Cap_Buf[10];
	CAP_Value[2].B08[1] = PCA_Cap_Buf[11];

	a.B16= CAP_Value[2].B16 - CAP_Value[1].B16;
	b.B16= CAP_Value[1].B16 - CAP_Value[0].B16;
	temp = (a.B16+b.B16)/2; 
//   UartTx_0(PCA_Cap_Buf[0]);
// 	UartTx_0(PCA_Cap_Buf[1]);
//   UartTx_0(PCA_Cap_Buf[2]);
// 	UartTx_0(PCA_Cap_Buf[3]);		
//   UartTx_0(PCA_Cap_Buf[4]);
// 	UartTx_0(PCA_Cap_Buf[5]);		
//   UartTx_0(PCA_Cap_Buf[6]);
// 	UartTx_0(PCA_Cap_Buf[7]);
//   UartTx_0(PCA_Cap_Buf[8]);
// 	UartTx_0(PCA_Cap_Buf[9]);	
//   UartTx_0(PCA_Cap_Buf[10]);
// 	UartTx_0(PCA_Cap_Buf[11]);		
//   UartTx_0(0x66);
 
	*PCA_buf2++ = temp;
// UartTx_0(temp);
  if ((temp> Cnt_Max ) ||(temp < Cnt_Min))
  {
			*PCA_buf2++ = 0x02;
        	
//       return (PCA_buf2);	
		return(0x02);	
  }
	else
	{
		*PCA_buf2++ = 0x01;
        			
//         	return (PCA_buf2);			
		return(0x01);	
	}
	
}*/


/*
INT8U timer0_Cnt(INT8U *T2_buf2,INT8U MOD,INT8U Cnt_Max,INT8U Cnt_Min)
{
	INT8U cnt_value;

	TR0 = 0;
	TMOD = MOD;		//0x05:T1挂起，T0为定时器方式，16位定时器;CT0=1,timer0 counts negative transition on timer0 input pin(GPIO44)
	ET0	= 0;				//T0中断进制
	TH0 = 0;				//定时寄存器赋值
 	TL0= 0;				  //定时寄存器赋值  
	TR0 = 1;  //启动TIMER0计数
	
	
	INIT_T1();
	Timer1_Test_Cnt = 1;					//5ms计时延时
	Timer1_Test_Statues = 0;
	TR1=1;		
	
	
	while(Timer1_Test_Statues==0){}  //5ms后退出循环
	cnt_value = TL0;//读取timer0此时计数值，理论值0xac
		
	UartTx_0(cnt_value);
		
  if ((cnt_value> Cnt_Max ) ||(cnt_value < Cnt_Min))
  {
			*T2_buf2++ = 0x02;
		return(0x02);	
  }
	else
	{
		*T2_buf2++ = 0x01;		
		return(0x01);	
	}	

}

INT8U timer1_Cnt(INT8U *T2_buf2,INT8U MOD,INT8U Cnt_Max,INT8U Cnt_Min)
{
	INT8U cnt_value;

	TR1 = 0;
	TMOD = MOD;		//0x50:T0挂起，T1为定时器方式，16位定时器;CT1=1,timer1 counts negative transition on timer1 input pin(GPIO45)
	ET1	= 0;				//T1中断进制
	TH1 = 0;				//定时寄存器赋值
 	TL1 = 0;				  //定时寄存器赋值  
	
	INIT_T0();
	Timer0_Test_Cnt = 1;					//5ms计时延时
	Timer0_Test_Statues = 0;
	TR0 = 1;		
	
	TR1 = 1;  //启动TIMER1计数
	while(Timer0_Test_Statues==0){}  //5ms后退出循环
	cnt_value = TL1;//读取timer1此时计数值，理论值0xac
		
	UartTx_0(cnt_value);
		
  if ((cnt_value> Cnt_Max ) ||(cnt_value < Cnt_Min))
  {
			*T2_buf2++ = 0x02;
		return(0x02);	
  }
	else
	{
		*T2_buf2++ = 0x01;		
		return(0x01);	
	}	

}

uchar TIMER2_Capture(uchar *T2_buf2,uchar CON,uchar MOD,INT16U Cnt_Max,INT16U Cnt_Min)
{
	INT16U i,temp;
	union B16_B08 CAP_Value[3];
	union B16_B08 a,b;

	
	T2CON        = CON;			
	T2MOD        = MOD;		


	T2_Cap_Cnt = 0;
	
	for(i=0;i<12;i++)
	{
		T2_Cap_Buf[i]=0;
	}
	
	
	
	//===增加定时控制,防止捕捉启动失败,导致死循环========
	INIT_T1();							//timer1参数配置					
	Timer1_Test_Cnt = 1;					//5ms计时延时
	Timer1_Test_Statues = 0;
	TR1=1;	
	
	ET2 = 1;   //打开T2中断
	TR2 = 1;  //启动T2 Module
	while(T2_Cap_Cnt<12)
	{
		
		if(Timer1_Test_Statues==1)			//在5ms内无法完成捕捉则直接退出循环
		{	
			break;
		}
	}
	
	T2CON =0;		//关闭T2 Module
	ET2 = 0;
	TR1=0;								//关闭定时器
	TMOD &= 0x0F;	
	TR2 = 0; 
	
	
	*T2_buf2++ = T2_Cap_Cnt;
	
//  UartTx_0(T2_Cap_Cnt);
//  UartTx_0(0x88);

	CAP_Value[0].B08[0] = T2_Cap_Buf[6];
	CAP_Value[0].B08[1] = T2_Cap_Buf[7];
	CAP_Value[1].B08[0] = T2_Cap_Buf[8];
	CAP_Value[1].B08[1] = T2_Cap_Buf[9];
	CAP_Value[2].B08[0] = T2_Cap_Buf[10];
	CAP_Value[2].B08[1] = T2_Cap_Buf[11];

	a.B16= CAP_Value[2].B16 - CAP_Value[1].B16;
	b.B16= CAP_Value[1].B16 - CAP_Value[0].B16;
	temp = (a.B16+b.B16)/2; 
  UartTx_0(T2_Cap_Buf[0]);
	UartTx_0(T2_Cap_Buf[1]);
  UartTx_0(T2_Cap_Buf[2]);
	UartTx_0(T2_Cap_Buf[3]);		
  UartTx_0(T2_Cap_Buf[4]);
	UartTx_0(T2_Cap_Buf[5]);		
  UartTx_0(T2_Cap_Buf[6]);
	UartTx_0(T2_Cap_Buf[7]);
  UartTx_0(T2_Cap_Buf[8]);
	UartTx_0(T2_Cap_Buf[9]);	
  UartTx_0(T2_Cap_Buf[10]);
	UartTx_0(T2_Cap_Buf[11]);		
//   UartTx_0(0x66);


// 	UartTx_0((temp>>8));	
// 	UartTx_0(temp);

 
	*T2_buf2++ = temp;
  
  if ((temp> Cnt_Max ) ||(temp < Cnt_Min))
  {
			*T2_buf2++ = 0x02;
        	
//       return (PCA_buf2);	
		return(0x02);	
  }
	else
	{
		*T2_buf2++ = 0x01;
        			
//         	return (PCA_buf2);			
		return(0x01);	
	}
	
}
*/

//====================================================================
//NUM为PWM的输出模块，可以设定为0-4。
/*uchar * PCA_PWM(uchar *PCA_buf2,uchar Num,uchar MOD,uchar CAPMx,uchar CAPxL,uchar CAPxH)
{
	EC = 0;
	//====公共寄存器配置，使用clk/4作为时钟===============
	PCATCSEL = 0;         	//PCA定时/计数时钟控制寄存器	
	PCACTRL   = 0;          	//PCA控制寄存器
	CCON = 0x00;		//	CF 		CR 		-- 		CCF4 	CCF3 	CCF2 	CCF1 	CCF0

	CCAPM0 = 0; 	//	--		ECOMx	CAPPx	CAPNx	MATx	TOGx	PWMx	ECCFx	
	CCAPM1 = 0;
	CCAPM2 = 0;
	CCAPM3 = 0;
	CCAPM4 = 0;

	PCA_PWM_cnt = 10;		//倒计时计数50ms
	
	switch(Num)
	{
		case 0:
				GPIO22CFG = gpio_etmr | gpio_outen;  //pcacomp0
				CMOD = MOD;			//	CIDL 	WDTE 	UF2 	UF1 	UF0 	CPS1 	CPS0 	ECF
				CCAPM0 = CAPMx;		//	-- 		ECOMx	CAPPx 	CAPNx 	MATx 	TOGx 	PWMx 	ECCFx	
				CCAP0L  = CAPxL;			
				CCAP0H = CAPxH;
				break;
		case 1:	
				GPIO23CFG = gpio_etmr | gpio_outen;  //pcacomp1 
				CMOD = MOD;			//	CIDL 	WDTE 	UF2 	UF1 	UF0 	CPS1 	CPS0 	ECF
				CCAPM1 = CAPMx;		//	-- 		ECOMx	CAPPx 	CAPNx 	MATx 	TOGx 	PWMx 	ECCFx	
				CCAP1L  = CAPxL;			
				CCAP1H = CAPxH;
				break;

		case 2:
				GPIO33CFG = gpio_etmr | gpio_outen;  //pcacomp2
				CMOD = MOD;			//	CIDL 	WDTE 	UF2 	UF1 	UF0 	CPS1 	CPS0 	ECF
				CCAPM2 = CAPMx;		//	-- 		ECOMx	CAPPx 	CAPNx 	MATx 	TOGx 	PWMx 	ECCFx
				CCAP2L  = CAPxL;			
				CCAP2H = CAPxH;
				break;

		case 3:
				GPIO34CFG = gpio_etmr | gpio_outen;  //pcacomp3
				CMOD = MOD;			//	CIDL 	WDTE 	UF2 	UF1 	UF0 	CPS1 	CPS0 	ECF
				CCAPM3 = CAPMx;		//	-- 		ECOMx	CAPPx 	CAPNx 	MATx 	TOGx 	PWMx 	ECCFx
				CCAP3L  = CAPxL;			
				CCAP3H = CAPxH;
				break;

		case 4:
				GPIO35CFG = gpio_etmr | gpio_outen;  //pcacomp4
				CMOD = MOD;			//	CIDL 	WDTE 	UF2 	UF1 	UF0 	CPS1 	CPS0 	ECF
				CCAPM4 = CAPMx;		//	-- 		ECOMx	CAPPx 	CAPNx 	MATx 	TOGx 	PWMx 	ECCFx	
				CCAP4L  = CAPxL;			
				CCAP4H = CAPxH;
				break;
				
		default:
				break;

	}

	CCON |=B0100_0000;				//启动PCA Module
	
	*PCA_buf2++ = 0x01;	
        return (PCA_buf2);
		
}*/


INT8U Lptim_Cnt(INT8U lpcfg0,INT8U lpcfg1,INT8U lpcompl,INT8U lpcomph,INT8U lptargtl,INT8U lptargth,INT8U ie_con,INT8U Cnt_Max,INT8U Cnt_Min)
{
// 1：选择时钟源，设置分频值，设置工作模式和计数模式。
// 2：设置高低位比较寄存器的值。
// 3：设置高低位目标寄存器的值
// 4：打开中断标志使能。
// 5：打开LPTEN使能位，启动计数器。
	
	
	LPTIMIF = 0;
	LPTCFG0 = lpcfg0;
	LPTCFG1 = lpcfg1;
	LPTCMPL = lpcompl;
	LPTCMPH = lpcomph;
	TARGETL = lptargtl;
	TARGETH = lptargth;
	LPTIMIE = ie_con;
	IT0 = 1;//外部中断0下降沿触发	
	EX0 = 1; //外部中断0使能	
	LP_tim_cnt = 0;	
	
	INIT_T0();
	Timer0_Test_Cnt = 1;					//5ms计时延时
	Timer0_Test_Statues = 0;
	TR0 = 1;		//启动cpu timer0	
	LPTCTRL |= lpten;		//启动LPTIM
	while(Timer0_Test_Statues==0){}  //5ms后退出循环

	TR0=0;
	TMOD &= 0xF0;	
	LPTCTRL = 0;
		
// 	UartTx_0((INT8U)(LP_tim_cnt>>8));
// 	UartTx_0(LP_tim_cnt);
		
  if ((LP_tim_cnt> Cnt_Max ) ||(LP_tim_cnt < Cnt_Min))
  {
		return(0x02);	
  }
	else
	{	
		return(0x01);	
	}	

}








