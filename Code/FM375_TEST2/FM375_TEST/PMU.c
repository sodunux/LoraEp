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
			if(off_vref==1)  //�ر�vref������dpsleep
				PMU_CFG |= vref_off;
			else    //����sleep
				PMU_CFG &= ~vref_off;

			if(en_cvs==1)  //ʹ�ܽ����ں˵�ѹ
				PMU_CFG |= cvs_en;
			else  
				PMU_CFG &= ~cvs_en;			

			if(off_xtlf==1)  //�ر�XTLF
				PMU_CFG |= xtlf_off;
			else  
				PMU_CFG &= ~xtlf_off;		

			if(lp_ram_h==1)  //��2KB ram���벻����ģʽ
				RAMLPCFG |= ram1_lpmode;
			else  
				RAMLPCFG &= ~ram1_lpmode;			

			if(lp_ram_l==1)  //��2KB ram���벻����ģʽ
				RAMLPCFG |= ram0_lpmode;
			else  
				RAMLPCFG &= ~ram0_lpmode;						
			
			if(rclp_off==1)  //sleep/stop�¹ر�RCLP
				CKSRC_CTRL &= ~lp_rclpctrl;
			else
				CKSRC_CTRL |= lp_rclpctrl;				
			
			PMU_PDCTRL = 0;
			PMU_PDCTRL |= (stop_wake_time<<1);  //STOP�󣬻����¼�����ʱ�����
			
			if(en_disc==1)  //PD������й��ʹ��
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
			while(!(FDETIF&lfdet_out_b))   //�������ǰӦ��ѯLFDET_OUT_B��ȷ����Ϊ1�������޷�����LFDET_IF����ȷ��3�ζ�Ϊ1
			{}
			FDETIF = 0;
				
			FDETIE |= lfdet_ie;
			
		break;
		
		case LVD_PD:      //int0_n
			
			LSCLKSEL |= svd_clken | svd_lcd_clk_xtlf;
		
			if(power_mode==SLEEP)
			{
				IT0 = 1;//�ⲿ�ж�0�½��ش���	
				EX0 = 1; //�ⲿ�ж�0ʹ��
				
				LVDCTRL = 0;
				LVDCTRL |= lvd_3_42v;
				LVDCTRL |= lvd_enable | lvdpd_ie;		
			}
			else if(power_mode==STOP)
			{
				IPH = 0;
				IPL = 0;
				IPH |= BIT0;
				IPLX0 = 1;     //{IPHX0:IPLX0}=11 :������INT0_N�ж����ȼ����
				IT0 = 0;//�ⲿ�ж�0�͵�ƽ����		
				EX0 = 1; //�ⲿ�ж�0ʹ��
				
				LVDCTRL = 0;
				LVDCTRL |= lvd_3_42v;
				LVDCTRL |= lvd_enable | lvdpd_ie;			
			}
		break;
			
		case LVD_PU:      //int0_n
			
			LSCLKSEL |= svd_clken | svd_lcd_clk_xtlf;
		
			if(power_mode==SLEEP)
			{
				IT0 = 1;//�ⲿ�ж�0�½��ش���	
				EX0 = 1; //�ⲿ�ж�0ʹ��
				
				LVDCTRL = 0;
				LVDCTRL |= lvd_3_42v;
				LVDCTRL |= lvd_enable | lvdpu_ie;		
			}
			else if(power_mode==STOP)
			{
				IPH = 0;
				IPL = 0;
				IPH |= BIT0;
				IPLX0 = 1;     //{IPHX0:IPLX0}=11 :������INT0_N�ж����ȼ����
				IT0 = 0;//�ⲿ�ж�0�͵�ƽ����		
				EX0 = 1; //�ⲿ�ж�0ʹ��
				
				LVDCTRL = 0;
				LVDCTRL |= lvd_3_42v;
				LVDCTRL |= lvd_enable | lvdpu_ie;				
			}
		break;
			
		case RTCINT:     //INT0_N
			
			RTCWE = 0xAC;  //ʹ��RTCд
			RTCIE2 = 0;
			RTCIF2 = 0;	
			BCDSEC = 0x56;     //��RTC 4s��������ж�
			BCDMIN = 0x01;
			BCDHOUR = 0;

			if(power_mode==SLEEP)
			{
				IT0 = 1;//�ⲿ�ж�0�½��ش���	
				EX0 = 1; //�ⲿ�ж�0ʹ��
					
				RTCIE2 |= minie;  //ʹ�ܷ����ж�	
			}
			else if(power_mode==STOP)
			{
				IPH = 0;
				IPL = 0;
				IPH |= BIT0;
				IPLX0 = 1;     //{IPHX0:IPLX0}=11 :������INT0_N�ж����ȼ����
				IT0 = 0;//�ⲿ�ж�0�͵�ƽ����		
				EX0 = 1; //�ⲿ�ж�0ʹ��
				
				RTCIE2 |= minie;  //ʹ�ܷ����ж�		
			}				

		break;
		
		case IOINT:     //INT1_N
				
			GPIO_EXTI_SEL0 = 0;
			GPIO_EXTI_SEL0 |= io_exti_sel_pf6;  //ѡ��PF6��Ϊ�����ж�Դ
			GPIO_EXTIH_ES1 = 0xff;//�ر������ж�
			
			
			PFPPEN = 0;  //�ر�����
			PFPUEN = 0;  //�ر�����
			PFFCR1 &= ~BIT6;  
			PFFCR2 &= ~BIT6; //{PBFCR2[7]:PBFCR1[7]}=2'b00,PF6Ϊ���빦��
			//PBPUEN |= BIT7;//ʹ��PB7������
			
			EXTILIF = 0;

			if(power_mode==SLEEP)
			{
				EX1 = 1;//�ⲿ�ж�1ʹ��
				IT1 = 1;//�ⲿ�ж�1�½��ش���
					
				GPIO_EXTIH_ES1 &= extih6_fall_edge;  //������λPD/PE/PF/PG��bit6�������ж�
			}	
		break;
				
		case WKUP:   //NWKUPx�����ж� -> NMI,(����PB7)
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

		case IWDG:   //���Ź�
			Set_SYSWDT(wdtov_time_8S);
		break;		

		case LPTIM_INT:   // INT0_N
// 1��ѡ��ʱ��Դ�����÷�Ƶֵ�����ù���ģʽ�ͼ���ģʽ��
// 2�����øߵ�λ�ȽϼĴ�����ֵ��
// 3�����øߵ�λĿ��Ĵ�����ֵ
// 4�����жϱ�־ʹ�ܡ�
// 5����LPTENʹ��λ��������������
		
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
				IT0 = 1;//�ⲿ�ж�0�½��ش���	
				EX0 = 1; //�ⲿ�ж�0ʹ��
					
				LPTCTRL |= lpten;	
			}
			else if(power_mode==STOP)
			{
				IPH = 0;
				IPL = 0;
				IPH |= BIT0;
				IPLX0 = 1;     //{IPHX0:IPLX0}=11 :������INT0_N�ж����ȼ����
				IT0 = 0;//�ⲿ�ж�0�͵�ƽ����		
				EX0 = 1; //�ⲿ�ж�0ʹ��
				
				LPTCTRL |= lpten;	
			}					
			
		break;	


		case RXD2_RX:   //NWKUPx�����ж� -> NMI,(����rxd2)
			PDFCR1 &= ~BIT2;
			PDFCR2 |= BIT2;//{PDFCR2[2]:PDFCR1[2]}=2'b10:alternate function (2 functions selected by AFSEL registers)
			AFSELD &= ~BIT2;//0: RXD2
// 			UartTx_0(0X19);
		break;	

		case NO_WKUP:   //û�л���Դ
			
		break;	
		
		default:			
		break;
	}
}

/************************************************************************/
/* ��������goto_stop													*/
/* ����˵��������stop������ԴΪIO43�ж�							*/
/* ��ڲ�����N/A													*/
/* ���ڲ�����N/A													*/
/************************************************************************/
// void goto_stop(void)
// {

// 				IO4INEN |= BIT3;   
//  				IO4PUEN |= BIT3;
// 				IO4INF |= BIT3;//�˲�ʹ��
// 				IO4IFCFG1 |= io43_neg_int;
// 				IO4IF = 0;
// 				IPH = 0;
// 				IPL = 0;
// 				IPH |= BIT2;
// 				IPLX1 = 1;     //{IPHX1:IPLX1}=11 :������INT1_N�ж����ȼ����
// 				IT1 = 0;//�ⲿ�ж�1�͵�ƽ����		
// 				EX1 = 1; //�ⲿ�ж�1ʹ��
// 				IO4IE |= BIT3;		

// 	
//  			PWMODE = 0;
//  			ULPCTRL = 0x74;  //�̼�����ULPCTRL�Ĵ�����У׼ULPLDO���1.5V
// 			
// 			PWMODE &= ~wkup_dest;   //����Ŀ��ΪACTIVEģʽ

// 			PWMODE |= dpswk_500u;    //����ʱ��500us
// 						
// 			PWMODE &= ~dpstop_en;   //��������
// 			PWMODE |= stop;
// 			PCON |= pwd;     //����STOP
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
