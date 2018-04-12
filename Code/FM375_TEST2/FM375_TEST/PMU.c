#include "init.h"

// void close_all_module(void)
// {
// 	RTCCTRL = 0;
// 	DISPCTRL = 0;
// 	LCDTEST = 0;
// 	LCDSET = 0;
// 	PDRCTRL = 0x01;  //close pdr
// 	FDETCTRL = 0x01;  //close fdet
// 	//FDETCTRL = 0x00;  //OPEN fdet
// 	TRNGCTRL = 0;		//close trng
// 	LSENCTRL = 0;
// 	LVDCTRL = 0;
// 	CLKGATE = 0;
// 	CTRLSTATUSREG = 0;//CLOSE SM3
// 	ANAEN = 0;
// }

void Low_power_Ctrl(INT8U off_vref,INT8U en_cvs,INT8U off_xtlf,INT8U lp_ram_h,INT8U lp_ram_l,INT8U rclp_off,INT8U stop_wake_time,INT8U en_disc)
{
			if(off_vref==1)  //关闭vref，进入dpsleep
				PMU_CFG |= vref_off;
			else    //进入sleep
				PMU_CFG &= ~vref_off;

			if(en_cvs==1)  //使能降低内核电压
				PMU_CFG |= cvs_en;
			else  
				PMU_CFG &= ~cvs_en;			

			if(off_xtlf==1)  //关闭XTLF
				PMU_CFG |= xtlf_off;
			else  
				PMU_CFG &= ~xtlf_off;		

			if(lp_ram_h==1)  //高2KB ram进入不保存模式
				RAMLPCFG |= ram1_lpmode;
			else  
				RAMLPCFG &= ~ram1_lpmode;			

			if(lp_ram_l==1)  //低2KB ram进入不保存模式
				RAMLPCFG |= ram0_lpmode;
			else  
				RAMLPCFG &= ~ram0_lpmode;						
			
			if(rclp_off==1)  //sleep/stop下关闭RCLP
				CKSRC_CTRL &= ~lp_rclpctrl;
			else
				CKSRC_CTRL |= lp_rclpctrl;				
			
			PMU_PDCTRL = 0;
			PMU_PDCTRL |= (stop_wake_time<<1);  //STOP后，唤醒事件屏蔽时间控制
			
			if(en_disc==1)  //PD掉电电荷泄放使能
				PMU_PDCTRL |= disc_en;
			else  
				PMU_PDCTRL &= ~disc_en;			
}

void wake_event_set(INT8U wake_source,INT8U power_mode)
{
	
	
	switch(wake_source)
	{
		case LFDET:   //NMI
			FDETIE = 0;
			while(!(FDETIF&lfdet_out_b))   //软件清零前应查询LFDET_OUT_B，确保其为1，否则将无法清零LFDET_IF。且确保3次都为1
			{}
			FDETIF = 0;
				
			FDETIE |= lfdet_ie;
			
		break;
		
		case LVD_PD:      //int0_n
			
			LSCLKSEL |= svd_clken | svd_lcd_clk_xtlf;
		
			if(power_mode==SLEEP)
			{
				IT0 = 1;//外部中断0下降沿触发	
				EX0 = 1; //外部中断0使能
				
				LVDCTRL = 0;
				LVDCTRL |= lvd_3_42v;
				LVDCTRL |= lvd_enable | lvdpd_ie;		
			}
			else if(power_mode==STOP)
			{
				IPH = 0;
				IPL = 0;
				IPH |= BIT0;
				IPLX0 = 1;     //{IPHX0:IPLX0}=11 :　设置INT0_N中断优先级最高
				IT0 = 0;//外部中断0低电平触发		
				EX0 = 1; //外部中断0使能
				
				LVDCTRL = 0;
				LVDCTRL |= lvd_3_42v;
				LVDCTRL |= lvd_enable | lvdpd_ie;			
			}
		break;
			
		case LVD_PU:      //int0_n
			
			LSCLKSEL |= svd_clken | svd_lcd_clk_xtlf;
		
			if(power_mode==SLEEP)
			{
				IT0 = 1;//外部中断0下降沿触发	
				EX0 = 1; //外部中断0使能
				
				LVDCTRL = 0;
				LVDCTRL |= lvd_3_42v;
				LVDCTRL |= lvd_enable | lvdpu_ie;		
			}
			else if(power_mode==STOP)
			{
				IPH = 0;
				IPL = 0;
				IPH |= BIT0;
				IPLX0 = 1;     //{IPHX0:IPLX0}=11 :　设置INT0_N中断优先级最高
				IT0 = 0;//外部中断0低电平触发		
				EX0 = 1; //外部中断0使能
				
				LVDCTRL = 0;
				LVDCTRL |= lvd_3_42v;
				LVDCTRL |= lvd_enable | lvdpu_ie;				
			}
		break;
			
		case RTCINT:     //INT0_N
			
			RTCWE = 0xAC;  //使能RTC写
			RTCIE2 = 0;
			RTCIF2 = 0;	
			BCDSEC = 0x56;     //令RTC 4s后产生分中断
			BCDMIN = 0x01;
			BCDHOUR = 0;

			if(power_mode==SLEEP)
			{
				IT0 = 1;//外部中断0下降沿触发	
				EX0 = 1; //外部中断0使能
					
				RTCIE2 |= minie;  //使能分钟中断	
			}
			else if(power_mode==STOP)
			{
				IPH = 0;
				IPL = 0;
				IPH |= BIT0;
				IPLX0 = 1;     //{IPHX0:IPLX0}=11 :　设置INT0_N中断优先级最高
				IT0 = 0;//外部中断0低电平触发		
				EX0 = 1; //外部中断0使能
				
				RTCIE2 |= minie;  //使能分钟中断		
			}				

		break;
		
		case IOINT:     //INT1_N
				
			GPIO_EXTI_SEL0 = 0;
			GPIO_EXTI_SEL0 |= io_exti_sel_pf6;  //选择PF6作为引脚中断源
			GPIO_EXTIH_ES1 = 0xff;//关闭所有中断
			
			
			PFPPEN = 0;  //关闭推挽
			PFPUEN = 0;  //关闭上拉
			PFFCR1 &= ~BIT6;  
			PFFCR2 &= ~BIT6; //{PBFCR2[7]:PBFCR1[7]}=2'b00,PF6为输入功能
			//PBPUEN |= BIT7;//使能PB7的上拉
			
			EXTILIF = 0;

			if(power_mode==SLEEP)
			{
				EX1 = 1;//外部中断1使能
				IT1 = 1;//外部中断1下降沿触发
					
				GPIO_EXTIH_ES1 &= extih6_fall_edge;  //开启低位PD/PE/PF/PG的bit6上升沿中断
			}	
		break;
				
		case WKUP:   //NWKUPx引脚中断 -> NMI,(测试PB7)
			PBFCR1 &= ~BIT7;
			PBFCR2 |= BIT7;//{PBFCR2[7]:PBFCR1[7]}=2'b10:alternate function (2 functions selected by AFSEL registers)
			AFSELB |= BIT7;//1: WKUP1
		
// 			PDFCR1 &= ~BIT1;
// 			PDFCR2 |= BIT1;//{PDFCR2[1]:PDFCR1[1]}=2'b10:alternate function (2 functions selected by AFSEL registers)
// 			AFSELD |= BIT1;//1: WKUP2	
			
// 			PFPUEN |= BIT6;
// 			PFFCR1 &= ~BIT6;
// 			PFFCR2 |= BIT6;//{PBFCR2[7]:PBFCR1[7]}=2'b10:alternate function (2 functions selected by AFSEL registers)
// 			AFSELF |= afself6_wkup3;//1: WKUP3

		
		break;		

		case IWDG:   //看门狗
			Set_SYSWDT(wdtov_time_8S);
		break;		

		case LPTIM_INT:   // INT0_N
// 1：选择时钟源，设置分频值，设置工作模式和计数模式。
// 2：设置高低位比较寄存器的值。
// 3：设置高低位目标寄存器的值
// 4：打开中断标志使能。
// 5：打开LPTEN使能位，启动计数器。
		
			LSCLKSEL |= lptim_fcken | svd_lcd_clk_xtlf;  
		
			LPTCFG0 = 0;
			LPTCFG1 = 0;
			LPTCFG0 |= lptim_cnt_src_lsclk | cnt_clk_div1;
			LPTCFG1 |= word_mode_wave_out_timer | cnt_mode_continue | pos_pol_out;
			TARGETL = 0xff;
			TARGETH = 0xff;
			LPTIMIE = ovie;
			LPTIMIF = 0;
			if(power_mode==SLEEP)
			{
				IT0 = 1;//外部中断0下降沿触发	
				EX0 = 1; //外部中断0使能
					
				LPTCTRL |= lpten;	
			}
			else if(power_mode==STOP)
			{
				IPH = 0;
				IPL = 0;
				IPH |= BIT0;
				IPLX0 = 1;     //{IPHX0:IPLX0}=11 :　设置INT0_N中断优先级最高
				IT0 = 0;//外部中断0低电平触发		
				EX0 = 1; //外部中断0使能
				
				LPTCTRL |= lpten;	
			}					
			
		break;	


		case RXD2_RX:   //NWKUPx引脚中断 -> NMI,(测试rxd2)
			PDFCR1 &= ~BIT2;
			PDFCR2 |= BIT2;//{PDFCR2[2]:PDFCR1[2]}=2'b10:alternate function (2 functions selected by AFSEL registers)
			AFSELD &= ~BIT2;//0: RXD2
// 			UartTx_0(0X19);
		break;	

		case NO_WKUP:   //没有唤醒源
			
		break;	
		
		default:			
		break;
	}
}

/************************************************************************/
/* 函数名：goto_stop													*/
/* 功能说明：进入stop，唤醒源为IO43中断							*/
/* 入口参数：N/A													*/
/* 出口参数：N/A													*/
/************************************************************************/
// void goto_stop(void)
// {

// 				IO4INEN |= BIT3;   
//  				IO4PUEN |= BIT3;
// 				IO4INF |= BIT3;//滤波使能
// 				IO4IFCFG1 |= io43_neg_int;
// 				IO4IF = 0;
// 				IPH = 0;
// 				IPL = 0;
// 				IPH |= BIT2;
// 				IPLX1 = 1;     //{IPHX1:IPLX1}=11 :　设置INT1_N中断优先级最高
// 				IT1 = 0;//外部中断1低电平触发		
// 				EX1 = 1; //外部中断1使能
// 				IO4IE |= BIT3;		

// 	
//  			PWMODE = 0;
//  			ULPCTRL = 0x74;  //固件配置ULPCTRL寄存器，校准ULPLDO输出1.5V
// 			
// 			PWMODE &= ~wkup_dest;   //唤醒目标为ACTIVE模式

// 			PWMODE |= dpswk_500u;    //唤醒时间500us
// 						
// 			PWMODE &= ~dpstop_en;   //常规休眠
// 			PWMODE |= stop;
// 			PCON |= pwd;     //进入STOP
// 			_nop_();
// }


// pmu_test(INT8U pmu_mode,INT8U ulpldo,INT8U mclk_sel,INT8U wk_source,INT8U wk_dest,INT8U wk_time)
// {
// 	INT8U near mclk,wake_source;
// 	switch(pmu_mode)
// 	{
// 		case SLEEP:
// 			
// 		break;
// 		
// 		case DPSLEEP:
// 			
// 		break;
// 		
// 		case STOP:
// 			
// 		break;
// 		
// 		case DPSTOP:
// 			
// 		break;		
// 		
// 		
// 		default:
// 		break;
// 	}
// }
