/************************************************************************/
/* 	系统初始化函数  									                */
/* 	主要功能:															*/
/* 		1.完成系统初始化                                                */
/*	硬件平台：	 													  	*/
/*			FM340		                        	*/
/* 	编制:xudayong													*/
/* 	编制时间:2014年10月30日												*/
/************************************************************************/

#include "init.h"

//#define 	trim_signal_write_to_flash					//将校正的数据写到FLASH
unsigned int  	edata	min_dlt,ossfet_array[sweep_window*2];	//修改溢出错误
unsigned char 	edata trim_result,cap_overflow_cnt,over_flow_cnt,ossfet_point,trim_ideal,trim_min,trim_max,trim_set,trim_test;
unsigned int  	edata cap_00,cap_7F,cap_value,trim_value;
unsigned long  	edata cap_cal;
unsigned char 	edata trim_2M,trim_4M,trim_8M;
unsigned char 	trim_of_min_dlt;


UInt edata bt_temp;

/*************************************************
函数名称：
简要描述：使用ET3的捕捉操作
输入： 	  发送的数据
输出： 	  
*************************************************/	
unsigned int initial_cap_test_8(void)
{
unsigned int  capture_first,capture_second,cap_val;			//最大65536 支持
unsigned long val;
unsigned char i,stabel_cnt,flag;

	PERICLK_CTRL0 |= et34_clken;
	
//设定ET3工作模式
	ET3IE = 0;			//  -	-	-	-	-	-	capie	ovie
	ET3IF = 0;
//=== 捕捉模式,脉冲周期捕捉,捕捉清零，连续捕捉，周期捕捉模式时上沿捕捉 ====//
	ET3CTRL = 0x44;           //ETMR3控制寄存器
//=== 捕捉源group2,计数源group1；group2：xtlf；group1：mcu_clk ===//
	ET3INSEL= 0X05;           //0x05,ETMR3输入源选择寄存器
//=== 计数源不分频 ====//
	ET3PRESCALE1=  0x00;      //ETMR3预分频寄存器1
//==== 捕捉源32分频 ===//
	ET3PRESCALE2= 0x1F;       //ETMR3预分频寄存器2
	
	capture_first=0X0000;
	flag=0;
	ET3CTRL |= et34_cen;  //启动ET3计数
	val=0;			
	RTCIF2 &= ~hz16_if;									//清除62.5ms中断标志
	over_flow_cnt=0;
	stabel_cnt=2;										//采样丢弃的次数  用于稳定
	for(i=0;i<8;)										//4次已经很准  8次更好
	{

		if((ET3IF&et34_capif)==et34_capif)	
		{		
//	UartTx_3(ET3STARTH); 
//	UartTx_3(ET3STARTL);			
			if(flag==0)
			{
				capture_first=((ET3STARTH<<8)+ET3STARTL);
				RTCIF2 &= ~hz16_if;						//清除62.5ms中断标志
				flag=1;									//标志开始的一次采样完成
			}	
			else
			{
				
				RTCIF2&= ~hz16_if;						//清除62.5ms中断标志
				capture_second=((ET3STARTH<<8)+ET3STARTL);	
				cap_val = (unsigned int)capture_second;

				if(stabel_cnt==0)
				{
					val+=cap_val;
					i++;								//一次有效的捕捉
				}
				else
				{
					stabel_cnt--;						//前面的几次采样不计算在内
				}
			}
			EXF2 = 0;									//清除标志一次
		}
		
		if(RTCIF2 & hz16_if) 
		{	
			RTCIF2 &= ~hz16_if;				//清除溢出标志
			over_flow_cnt++;
			if(over_flow_cnt==2)  
			{	
				over_flow_cnt=0;
				cap_overflow_cnt++;
				return 0XFFFF;				//错误返回
			}	
		}
		
		WDT_CLR();
	}
	
	
	cap_val=(val>>3);
	return cap_val;	
	
}
/*************************************************
T2_capture_1ms_for_TRIM
简要描述：首先测试最小TRIM值的频率cap_00  和最大TRIM值的频率cap_7F   计算TRIM步进值
          trim_step=(cap_7F-cap_00)/128   （1）
          然后根据需要TRIM频率对应计数值 cap_ideal 计算大致的 trim_ideal 值 （前提是芯片的trim和频率线性 这已经被测试证实）
          trim_ideal=(cap_ideal-cap_00)/trim_step  （2）
          将(1)带入(2) 得到
          trim_ideal=(cap_ideal-cap_00)*128/(cap_7F-cap_00) (3)
          //所以只要测试得出  cap_00 cap_7F 就能获得  trim_ideal  然后在 trim_ideal附近筛选最佳值
输入： 	  FREQ_mode  	希望校正的中心频率 可选  rchf_8M rchf_16M rchf_24M rchf_32M
		  trim_counter  根据标准宽度31US计算出来的理论时钟的对应宽度		
输出： 	  0X00-7F是校正值   >0X80是错误值
修改日志：2016/6/16 16:33:35 在计算捕捉值和理论值求差的过程中 必须使用int 型数据存储差值 不能使用 unsigned char 
因为有一些芯片斜率较大  导致  在扫描范围的边界产生大于256的差值   而产生异常的 判断  
*************************************************/
unsigned char T2_capture_1ms_for_TRIM(unsigned char FREQ_mode,unsigned int trim_counter)
{
	unsigned char i,RCHFADJ_bakeup,CKSRC_CTRL_bakeup,EA_bakeup; 	
//保存环境
//unsigned long addr;
	trim_value=trim_counter;
	RCHFADJ_bakeup=RCHFADJ;
	CKSRC_CTRL_bakeup=CKSRC_CTRL;
	EA_bakeup=0;
	if(EA) EA_bakeup=1;
//设定需要TRIM的频段
	CKSRC_CTRL &= 0x3f;							
	CKSRC_CTRL |= FREQ_mode;	//频率模式
//设定最小TRIM测试频率
	trim_result=0X00;			//默认状态
	cap_overflow_cnt=0;							//记录总的测试过程中的捕捉溢出次数
//初始化TRIM内容
	for(i=0;i<(sweep_window*2);i++)
	{
		ossfet_array[i]=0X5555;
	}
	trim_ideal=0XEE;
	trim_min=0XEE;	
	trim_max=0XEE;
	RCHFADJ=0X00;
	
	cap_00=initial_cap_test_8();				//捕捉16次平均值  	带一秒超时控制 不会死机	 
//	UartTx_3((unsigned char)(cap_00>>8)); 
//	UartTx_3((unsigned char)(cap_00));
//设定最大TRIM测试频率
	RCHFADJ=0X7F;
	cap_7F=initial_cap_test_8();				//捕捉16次平均值	带一秒超时控制 不会死机
//	UartTx_3((unsigned char)(cap_7F>>8)); 
//	UartTx_3((unsigned char)(cap_7F));	

	
//判断是不是严重不线性
	if(cap_00>cap_7F)			
	{	
		trim_result|=0x01;					//设定标志  不退出
		trim_of_min_dlt=0XC0;				//设定不矫正  最高BIT=1表示TRIM失效了
	}
//就是最低频率还大于 需要TRIM的频率	
	if(cap_00>trim_counter)		
	{	
		trim_result|=0x02;					//设定标志  不退出
		trim_of_min_dlt=0X80;				//设定到最小  最高BIT=1表示TRIM失效了
	}
//就是最高频率还小于 需要TRIM的频率	
	if(cap_7F<trim_counter)		
	{	
		trim_result|=0x04;					//设定标志  不退出
		trim_of_min_dlt=0XFF;				//设定到最大  最高BIT=1表示TRIM失效了
	}
	if(cap_overflow_cnt)
	{
		trim_result|=0x08;					//设定标志  不退出
		trim_of_min_dlt=0XD0;				//设定到最大  最高BIT=1表示TRIM失效了
	}
	
	if(trim_result==0)			
	{
	
	//没有出现上述的失效  才进入理论TRIM值计算
	//计算trim_ideal
		cap_cal=trim_counter;						
		cap_cal=cap_cal-cap_00;						//cap_cal=trim_counter-cap_00
		cap_cal=cap_cal<<7;							//cap_cal=(trim_counter-cap_00)*128
		cap_value=cap_7F-cap_00;					//cap_value=cap_7F-cap_00;				

		i=0;								//捕捉理想值
		while(cap_cal>=cap_value)
		{
			cap_cal=cap_cal-cap_value;
			i++;
		}
		trim_ideal=i;							//计算出的理论值相对实际值偏低 需要进行+5的修正
		
		
		//根据设定的 sweep_window 产生实际扫描窗口
		//trim_min  扫描开始的值
		if(trim_ideal>=sweep_window)  
		{
			trim_min=trim_ideal-sweep_window;			
		}
		else						 
		{
			trim_min=0;						
		}	
		//trim_max  扫描结束的值
		trim_max=trim_ideal+sweep_window; 
		if(trim_max>=0X7F)			 
		{
		     trim_max=0X7F;					
		}
		//开始扫描的过程
	
		cap_overflow_cnt=0;

		ossfet_point=0;								//存储差值变量
		
		
		min_dlt=0XFFFF;								//存储最小差值对应的TRIM值
		trim_of_min_dlt=trim_min;					//存储最小差值对应的TRIM值
		
		i=trim_min;									//开始扫描值
		while(i<=trim_max)
		{
			RCHFADJ=i;							//杜绝 出现 0X28---直接写0X28引发出现的异常频率
			cap_value=initial_cap_test_8();		//测试频率   带一秒超时控制 不会死机
			//和期望TRIM值求差    ossfet_array[ossfet_point]  是INT型数据  确保捕捉值和采样值的差能大于255
			if(cap_value>=trim_counter)  	
			{
				ossfet_array[ossfet_point]=cap_value-trim_counter;
			}
			else							
			{
				ossfet_array[ossfet_point]=trim_counter-cap_value;
				
			}
			//保存差值
			if(ossfet_array[ossfet_point]<min_dlt)  		//去掉可能存在的符号位
			{
				min_dlt=ossfet_array[ossfet_point];		//去掉可能存在的符号位	
				trim_of_min_dlt=i;
			}
			ossfet_point++;
			i++;
		}
		//全空间扫描完成   最后测试一次最佳值
		
		RCHFADJ=trim_of_min_dlt;				//最佳值
		cap_value=initial_cap_test_8();		//测试频率   带一秒超时控制 不会死机
		if(cap_overflow_cnt)
		{
			trim_result|=0x10;			//设定标志  不退出
			trim_of_min_dlt=0XE0;				//设定到最大  最高BIT=1表示TRIM失效了
		}
	}
#if defined trim_signal_write_to_flash	
	//将TRIM的数据写入 flash中存储起来
	addr=trim_signal_flash_addr;				//存储TRIM信息到 指定地址
	erase_FLASH(addr);							//0XFF:FC00-0XFF:FDFF  512字节擦除
	write_FLASH(addr+0x00,trim_result);			//最后结果
	write_FLASH(addr+0x01,trim_ideal);			//理论值
	write_FLASH(addr+0x02,sweep_window);		//trim窗口
	write_FLASH(addr+0x03,trim_min);			//最小值
	write_FLASH(addr+0x04,trim_max);			//最大值
	write_FLASH(addr+0x05,trim_of_min_dlt);		//最后TRIM计算值	
	write_FLASH(addr+0x06,cap_overflow_cnt);	//捕捉过程累计溢出次数
	bt_temp.Int=cap_00;
	write_FLASH(addr+0x07,bt_temp.Char[0]);
	write_FLASH(addr+0x08,bt_temp.Char[1]);
	bt_temp.Int=cap_7F;
	write_FLASH(addr+0x09,bt_temp.Char[0]);
	write_FLASH(addr+0x0A,bt_temp.Char[1]);
	bt_temp.Int=cap_value;
	write_FLASH(addr+0x0B,bt_temp.Char[0]);
	write_FLASH(addr+0x0C,bt_temp.Char[1]);
	bt_temp.Int=trim_counter;
	write_FLASH(addr+0x0D,bt_temp.Char[0]);
	write_FLASH(addr+0x0E,bt_temp.Char[1]);
	//存储 捕捉差值表
	addr=trim_signal_flash_addr+0X10;
	ossfet_point=0;
	for(i=trim_min;i<=trim_max;i++)
	{
		bt_temp.Int=ossfet_array[ossfet_point];    			//unsigned int 类型数据
		write_FLASH(addr,bt_temp.Char[0]);   //高位
		addr++;
		write_FLASH(addr,bt_temp.Char[1]);	//低位	
		addr++;
		ossfet_point++;
	}
#endif
//	RCCTRL1=RCCTRL1_bakeup;
//	RCCTRL2=RCCTRL2_bakeup;
	if(EA_bakeup) EA=1;
	return trim_of_min_dlt;
}

//void INIT_T0(void)
//{
//	TR0 = 0;
//	TMOD |= 0x01;		//T1挂起，T0为定时器方式，16位定时器，
//	ET0	= 1;				//T0中断使能
//	TH0 = 0XFC;				//定时寄存器赋值
// 	TL0= 0XBE;				//定时寄存器赋值  //2M主时钟下 5ms
//}
//void INIT_T1(void)
//{
//	TR1 = 0;
//	TMOD |= 0x10;		//T1挂起，T1为定时器方式，16位定时器，
//	ET1	= 1;				//T1中断使能
//	TH1 = 0XFC;				//定时寄存器赋值
//	TL1= 0XBE;					//定时寄存器赋值  //2M主时钟下 5ms
//}

/************************************************************************/
/* 函数名：INIT_CLK												*/
/* 功能说明：CLK初始化子程序      					    		*/
/* 入口参数：N/A													*/
/* 出口参数：N/A													*/
/************************************************************************/
void INIT_CLK(void)
{
	unsigned  char trim_value_8M;
	unsigned  char volatile flash_data0,flash_data1,flash_data2,tmp_cnt;	

//	RCHFADJ = 0x3F;
	CKSRC_CTRL |= rchf_8M | rchf_en;
	HSDIVSEL = hsclk_rchf | hsclk_div2;
	MCLKSEL = mclk_hsclk;
	
// 	CKSRC_CTRL |= pll_en;
// 	HSDIVSEL = hsclk_pll | hsclk_div2;
// 	MCLKSEL = mclk_hsclk | hsclk_sel_div;
	
	PERICLK_CTRL0 = 0xff;
	PERICLK_CTRL1 = 0xff;
	PERICLK_CTRL2 = 0xff;
	
	LSCLKSEL = 0x07;
	
	
	
	
	//Revised By ZRH
	//用捕捉值作为RCHF的调校值	
	trim_value_8M = T2_capture_1ms_for_TRIM(rchf_8M,7812);	 	//1024HZ 捕捉源，8M计数源  .理论值：7812
	//在 trim_value 中 包含很多信息  如果小于0X7F 就是调校值  否则是错误信息
	
			if(trim_value_8M>0x7f)
			RCHFADJ = 0x40;
		else
			RCHFADJ = trim_value_8M; 			//会是最接近的时钟 bit7是无效的
}


/************************************************************************/
/* 函数名：SET_MCLK												*/
/* 功能说明：CLK配置程序				    		*/
/* 入口参数：N/A													*/
/* 出口参数：N/A													*/
/************************************************************************/
void SET_MCLK(INT8U mclk,INT8U en_hs_div,INT8U div_num)
{

	switch(mclk)
	{
		case LSCLK: //LSCLK只有在XTLF停振状态下才会切换为RCLP，不能由CPU控制

			_nop_();
			MCLKSEL = mclk_lsclk;
			CKSRC_CTRL &= ~rchf_en;
			CKSRC_CTRL &= ~pll_en;			
			_nop_();

		break;

		case RCHF_8M:
		
			_nop_();
			CKSRC_CTRL |= rchf_8M | rchf_en;
			HSDIVSEL = hsclk_rchf | div_num;
			MCLKSEL = mclk_hsclk | (en_hs_div);//<<1
			_nop_();

		break;
		
		case RCHF_16M:
			_nop_();
			CKSRC_CTRL |= rchf_16M | rchf_en;
			HSDIVSEL = hsclk_rchf | div_num;
			MCLKSEL = mclk_hsclk | (en_hs_div);//<<1
			_nop_();
	
		break;
		
		case RCHF_24M:
			_nop_();
			CKSRC_CTRL |= rchf_24M | rchf_en;
			HSDIVSEL = hsclk_rchf | div_num;
			MCLKSEL = mclk_hsclk | (en_hs_div);//<<1
			_nop_();
			
		break;
		
		case RCHF_32M:
			_nop_();
			CKSRC_CTRL |= rchf_32M | rchf_en;
			HSDIVSEL = hsclk_rchf | div_num;
			MCLKSEL = mclk_hsclk | (en_hs_div);//<<1
			_nop_();
		
		break;
		
		case PLL:

			_nop_();
			CKSRC_CTRL |= pll_en;
			delay(1);
			HSDIVSEL = hsclk_pll | div_num;
			MCLKSEL = mclk_hsclk | (en_hs_div<<1);
			_nop_();
			_nop_();
		break;

		
		default:			
		break;
	}

}

/************************************************************************/
/* 函数名：INIT_IO_PAD													*/
/* 功能说明：IO管脚初始化子程序 							*/
/* 入口参数：N/A													*/
/* 出口参数：N/A													*/
/************************************************************************/
void Init_IO_PAD(void)
{
	PBPPEN = 0;
	PBFCR1 = 0;
	PBFCR2 = 0;
	PBPPEN |= BIT0 | BIT1 | BIT2;//使能PA0的推挽功能
	PBFCR1 |= BIT0 | BIT1 | BIT2;//{PAFCR2[0]:PAFCR1[0]}=2'b01,使能PA0输出
	PBDATA = 0xff;
	
	
	PAFCR2 &= 0X0F;
	PAPPEN |= BIT4 | BIT5 | BIT6 | BIT7;//使能PA0的推挽功能
	PAFCR1 |= BIT4 | BIT5 | BIT6 | BIT7;//{PAFCR2[0]:PAFCR1[0]}=2'b01,使能PA0输出
	PADATA = 0xff;	
	
	
	
	
	PGFCR2 &= ~BIT7;
	PGPPEN |= BIT7;//使能PG7的推挽功能
	PGFCR1 |= BIT7;//{PGFCR2[7]:PGFCR1[7]}=2'b01,使能PG7输出
	
	PEFCR2 &= ~BIT0;
	PEPPEN |= BIT0;//使能PE0的推挽功能
	PEFCR1 |= BIT0;//{PEFCR2[0]:PEFCR1[0]}=2'b01,使能PE0输出:CHARGE
	
	PEFCR2 &= ~BIT1;
	PEPPEN |= BIT1;//使能PE1的推挽功能
	PEFCR1 |= BIT1;//{PEFCR2[1]:PEFCR1[1]}=2'b01,使能PE1输出:DISCHARGE		
	
	PEDATA |= 0X03;//关闭充放电模拟开关
}

/************************************************************************/
/* 函数名：IInit_INT												    */
/* 功能说明：中断相关初始化子程序      					    		*/
/* 入口参数：N/A													    */
/* 出口参数：N/A													    */
/************************************************************************/
void Init_INT(void)
{

	EA = 1;
	AIE = 0x04;
// 	ES = 0;	//
// 	AIE6 = 0; //
//  	AIE5 = 0; //
// 	AIE4 = 0; //
// 	AIE3 = 0; //
// 	AIE2 = 1; //
//  	AIE1 = 0; //
// 	AIE0 = 0; //

// 	FDETIE = 0;
// 	FDETIF = 0;		

}


/************************************************************************/
/* 函数名：INIT_SYSWDT													*/
/* 功能说明：使能看门狗							*/
/* 入口参数：N/A													*/
/* 出口参数：N/A													*/
/************************************************************************/
void Set_SYSWDT(INT8U wdt_Period)
{
	//SYSWDT
	WDT_CLR();//先进行一次清狗
	WDTCFG = wdt_Period;//设置周期
}

/************************************************************************/
/* 函数名：INIT_SYS													*/
/* 功能说明：系统初始化子程序       							    */
/* 入口参数：N/A													*/
/* 出口参数：N/A													*/
/************************************************************************/
void INIT_SYS(void)
{
	Set_SYSWDT(wdtov_time_4096S);			//设置wdt周期
	PDRCTL = pdr_en;  //打开PDR
	BORCTL = 0;  //打开BOR
	//BORCTL = 0x01;  //关闭BOR
	Init_INT();   //设置中断
}
