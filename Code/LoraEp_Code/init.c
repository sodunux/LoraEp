/************************************************************************/
/* 	ϵͳ��ʼ������  									                */
/* 	��Ҫ����:															*/
/* 		1.���ϵͳ��ʼ��                                                */
/*	Ӳ��ƽ̨��	 													  	*/
/*			FM340		                        	*/
/* 	����:xudayong													*/
/* 	����ʱ��:2014��10��30��												*/
/************************************************************************/

#include "init.h"

//#define 	trim_signal_write_to_flash					//��У��������д��FLASH
unsigned int  	edata	min_dlt,ossfet_array[sweep_window*2];	//�޸��������
unsigned char 	edata trim_result,cap_overflow_cnt,over_flow_cnt,ossfet_point,trim_ideal,trim_min,trim_max,trim_set,trim_test;
unsigned int  	edata cap_00,cap_7F,cap_value,trim_value;
unsigned long  	edata cap_cal;
unsigned char 	edata trim_2M,trim_4M,trim_8M;
unsigned char 	trim_of_min_dlt;


UInt edata bt_temp;

/*************************************************
�������ƣ�
��Ҫ������ʹ��ET3�Ĳ�׽����
���룺 	  ���͵�����
����� 	  
*************************************************/	
unsigned int initial_cap_test_8(void)
{
unsigned int  capture_first,capture_second,cap_val;			//���65536 ֧��
unsigned long val;
unsigned char i,stabel_cnt,flag;

	PERICLK_CTRL0 |= et34_clken;
	
//�趨ET3����ģʽ
	ET3IE = 0;			//  -	-	-	-	-	-	capie	ovie
	ET3IF = 0;
//=== ��׽ģʽ,�������ڲ�׽,��׽���㣬������׽�����ڲ�׽ģʽʱ���ز�׽ ====//
	ET3CTRL = 0x44;           //ETMR3���ƼĴ���
//=== ��׽Դgroup2,����Դgroup1��group2��xtlf��group1��mcu_clk ===//
	ET3INSEL= 0X05;           //0x05,ETMR3����Դѡ��Ĵ���
//=== ����Դ����Ƶ ====//
	ET3PRESCALE1=  0x00;      //ETMR3Ԥ��Ƶ�Ĵ���1
//==== ��׽Դ32��Ƶ ===//
	ET3PRESCALE2= 0x1F;       //ETMR3Ԥ��Ƶ�Ĵ���2
	
	capture_first=0X0000;
	flag=0;
	ET3CTRL |= et34_cen;  //����ET3����
	val=0;			
	RTCIF2 &= ~hz16_if;									//���62.5ms�жϱ�־
	over_flow_cnt=0;
	stabel_cnt=2;										//���������Ĵ���  �����ȶ�
	for(i=0;i<8;)										//4���Ѿ���׼  8�θ���
	{

		if((ET3IF&et34_capif)==et34_capif)	
		{		
//	UartTx_3(ET3STARTH); 
//	UartTx_3(ET3STARTL);			
			if(flag==0)
			{
				capture_first=((ET3STARTH<<8)+ET3STARTL);
				RTCIF2 &= ~hz16_if;						//���62.5ms�жϱ�־
				flag=1;									//��־��ʼ��һ�β������
			}	
			else
			{
				
				RTCIF2&= ~hz16_if;						//���62.5ms�жϱ�־
				capture_second=((ET3STARTH<<8)+ET3STARTL);	
				cap_val = (unsigned int)capture_second;

				if(stabel_cnt==0)
				{
					val+=cap_val;
					i++;								//һ����Ч�Ĳ�׽
				}
				else
				{
					stabel_cnt--;						//ǰ��ļ��β�������������
				}
			}
			EXF2 = 0;									//�����־һ��
		}
		
		if(RTCIF2 & hz16_if) 
		{	
			RTCIF2 &= ~hz16_if;				//��������־
			over_flow_cnt++;
			if(over_flow_cnt==2)  
			{	
				over_flow_cnt=0;
				cap_overflow_cnt++;
				return 0XFFFF;				//���󷵻�
			}	
		}
		
		WDT_CLR();
	}
	
	
	cap_val=(val>>3);
	return cap_val;	
	
}
/*************************************************
T2_capture_1ms_for_TRIM
��Ҫ���������Ȳ�����СTRIMֵ��Ƶ��cap_00  �����TRIMֵ��Ƶ��cap_7F   ����TRIM����ֵ
          trim_step=(cap_7F-cap_00)/128   ��1��
          Ȼ�������ҪTRIMƵ�ʶ�Ӧ����ֵ cap_ideal ������µ� trim_ideal ֵ ��ǰ����оƬ��trim��Ƶ������ ���Ѿ�������֤ʵ��
          trim_ideal=(cap_ideal-cap_00)/trim_step  ��2��
          ��(1)����(2) �õ�
          trim_ideal=(cap_ideal-cap_00)*128/(cap_7F-cap_00) (3)
          //����ֻҪ���Եó�  cap_00 cap_7F ���ܻ��  trim_ideal  Ȼ���� trim_ideal����ɸѡ���ֵ
���룺 	  FREQ_mode  	ϣ��У��������Ƶ�� ��ѡ  rchf_8M rchf_16M rchf_24M rchf_32M
		  trim_counter  ���ݱ�׼���31US�������������ʱ�ӵĶ�Ӧ���		
����� 	  0X00-7F��У��ֵ   >0X80�Ǵ���ֵ
�޸���־��2016/6/16 16:33:35 �ڼ��㲶׽ֵ������ֵ���Ĺ����� ����ʹ��int �����ݴ洢��ֵ ����ʹ�� unsigned char 
��Ϊ��һЩоƬб�ʽϴ�  ����  ��ɨ�跶Χ�ı߽��������256�Ĳ�ֵ   �������쳣�� �ж�  
*************************************************/
unsigned char T2_capture_1ms_for_TRIM(unsigned char FREQ_mode,unsigned int trim_counter)
{
	unsigned char i,RCHFADJ_bakeup,CKSRC_CTRL_bakeup,EA_bakeup; 	
//���滷��
//unsigned long addr;
	trim_value=trim_counter;
	RCHFADJ_bakeup=RCHFADJ;
	CKSRC_CTRL_bakeup=CKSRC_CTRL;
	EA_bakeup=0;
	if(EA) EA_bakeup=1;
//�趨��ҪTRIM��Ƶ��
	CKSRC_CTRL &= 0x3f;							
	CKSRC_CTRL |= FREQ_mode;	//Ƶ��ģʽ
//�趨��СTRIM����Ƶ��
	trim_result=0X00;			//Ĭ��״̬
	cap_overflow_cnt=0;							//��¼�ܵĲ��Թ����еĲ�׽�������
//��ʼ��TRIM����
	for(i=0;i<(sweep_window*2);i++)
	{
		ossfet_array[i]=0X5555;
	}
	trim_ideal=0XEE;
	trim_min=0XEE;	
	trim_max=0XEE;
	RCHFADJ=0X00;
	
	cap_00=initial_cap_test_8();				//��׽16��ƽ��ֵ  	��һ�볬ʱ���� ��������	 
//	UartTx_3((unsigned char)(cap_00>>8)); 
//	UartTx_3((unsigned char)(cap_00));
//�趨���TRIM����Ƶ��
	RCHFADJ=0X7F;
	cap_7F=initial_cap_test_8();				//��׽16��ƽ��ֵ	��һ�볬ʱ���� ��������
//	UartTx_3((unsigned char)(cap_7F>>8)); 
//	UartTx_3((unsigned char)(cap_7F));	

	
//�ж��ǲ������ز�����
	if(cap_00>cap_7F)			
	{	
		trim_result|=0x01;					//�趨��־  ���˳�
		trim_of_min_dlt=0XC0;				//�趨������  ���BIT=1��ʾTRIMʧЧ��
	}
//�������Ƶ�ʻ����� ��ҪTRIM��Ƶ��	
	if(cap_00>trim_counter)		
	{	
		trim_result|=0x02;					//�趨��־  ���˳�
		trim_of_min_dlt=0X80;				//�趨����С  ���BIT=1��ʾTRIMʧЧ��
	}
//�������Ƶ�ʻ�С�� ��ҪTRIM��Ƶ��	
	if(cap_7F<trim_counter)		
	{	
		trim_result|=0x04;					//�趨��־  ���˳�
		trim_of_min_dlt=0XFF;				//�趨�����  ���BIT=1��ʾTRIMʧЧ��
	}
	if(cap_overflow_cnt)
	{
		trim_result|=0x08;					//�趨��־  ���˳�
		trim_of_min_dlt=0XD0;				//�趨�����  ���BIT=1��ʾTRIMʧЧ��
	}
	
	if(trim_result==0)			
	{
	
	//û�г���������ʧЧ  �Ž�������TRIMֵ����
	//����trim_ideal
		cap_cal=trim_counter;						
		cap_cal=cap_cal-cap_00;						//cap_cal=trim_counter-cap_00
		cap_cal=cap_cal<<7;							//cap_cal=(trim_counter-cap_00)*128
		cap_value=cap_7F-cap_00;					//cap_value=cap_7F-cap_00;				

		i=0;								//��׽����ֵ
		while(cap_cal>=cap_value)
		{
			cap_cal=cap_cal-cap_value;
			i++;
		}
		trim_ideal=i;							//�����������ֵ���ʵ��ֵƫ�� ��Ҫ����+5������
		
		
		//�����趨�� sweep_window ����ʵ��ɨ�贰��
		//trim_min  ɨ�迪ʼ��ֵ
		if(trim_ideal>=sweep_window)  
		{
			trim_min=trim_ideal-sweep_window;			
		}
		else						 
		{
			trim_min=0;						
		}	
		//trim_max  ɨ�������ֵ
		trim_max=trim_ideal+sweep_window; 
		if(trim_max>=0X7F)			 
		{
		     trim_max=0X7F;					
		}
		//��ʼɨ��Ĺ���
	
		cap_overflow_cnt=0;

		ossfet_point=0;								//�洢��ֵ����
		
		
		min_dlt=0XFFFF;								//�洢��С��ֵ��Ӧ��TRIMֵ
		trim_of_min_dlt=trim_min;					//�洢��С��ֵ��Ӧ��TRIMֵ
		
		i=trim_min;									//��ʼɨ��ֵ
		while(i<=trim_max)
		{
			RCHFADJ=i;							//�ž� ���� 0X28---ֱ��д0X28�������ֵ��쳣Ƶ��
			cap_value=initial_cap_test_8();		//����Ƶ��   ��һ�볬ʱ���� ��������
			//������TRIMֵ���    ossfet_array[ossfet_point]  ��INT������  ȷ����׽ֵ�Ͳ���ֵ�Ĳ��ܴ���255
			if(cap_value>=trim_counter)  	
			{
				ossfet_array[ossfet_point]=cap_value-trim_counter;
			}
			else							
			{
				ossfet_array[ossfet_point]=trim_counter-cap_value;
				
			}
			//�����ֵ
			if(ossfet_array[ossfet_point]<min_dlt)  		//ȥ�����ܴ��ڵķ���λ
			{
				min_dlt=ossfet_array[ossfet_point];		//ȥ�����ܴ��ڵķ���λ	
				trim_of_min_dlt=i;
			}
			ossfet_point++;
			i++;
		}
		//ȫ�ռ�ɨ�����   ������һ�����ֵ
		
		RCHFADJ=trim_of_min_dlt;				//���ֵ
		cap_value=initial_cap_test_8();		//����Ƶ��   ��һ�볬ʱ���� ��������
		if(cap_overflow_cnt)
		{
			trim_result|=0x10;			//�趨��־  ���˳�
			trim_of_min_dlt=0XE0;				//�趨�����  ���BIT=1��ʾTRIMʧЧ��
		}
	}
#if defined trim_signal_write_to_flash	
	//��TRIM������д�� flash�д洢����
	addr=trim_signal_flash_addr;				//�洢TRIM��Ϣ�� ָ����ַ
	erase_FLASH(addr);							//0XFF:FC00-0XFF:FDFF  512�ֽڲ���
	write_FLASH(addr+0x00,trim_result);			//�����
	write_FLASH(addr+0x01,trim_ideal);			//����ֵ
	write_FLASH(addr+0x02,sweep_window);		//trim����
	write_FLASH(addr+0x03,trim_min);			//��Сֵ
	write_FLASH(addr+0x04,trim_max);			//���ֵ
	write_FLASH(addr+0x05,trim_of_min_dlt);		//���TRIM����ֵ	
	write_FLASH(addr+0x06,cap_overflow_cnt);	//��׽�����ۼ��������
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
	//�洢 ��׽��ֵ��
	addr=trim_signal_flash_addr+0X10;
	ossfet_point=0;
	for(i=trim_min;i<=trim_max;i++)
	{
		bt_temp.Int=ossfet_array[ossfet_point];    			//unsigned int ��������
		write_FLASH(addr,bt_temp.Char[0]);   //��λ
		addr++;
		write_FLASH(addr,bt_temp.Char[1]);	//��λ	
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
//	TMOD |= 0x01;		//T1����T0Ϊ��ʱ����ʽ��16λ��ʱ����
//	ET0	= 1;				//T0�ж�ʹ��
//	TH0 = 0XFC;				//��ʱ�Ĵ�����ֵ
// 	TL0= 0XBE;				//��ʱ�Ĵ�����ֵ  //2M��ʱ���� 5ms
//}
//void INIT_T1(void)
//{
//	TR1 = 0;
//	TMOD |= 0x10;		//T1����T1Ϊ��ʱ����ʽ��16λ��ʱ����
//	ET1	= 1;				//T1�ж�ʹ��
//	TH1 = 0XFC;				//��ʱ�Ĵ�����ֵ
//	TL1= 0XBE;					//��ʱ�Ĵ�����ֵ  //2M��ʱ���� 5ms
//}

/************************************************************************/
/* ��������INIT_CLK												*/
/* ����˵����CLK��ʼ���ӳ���      					    		*/
/* ��ڲ�����N/A													*/
/* ���ڲ�����N/A													*/
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
	//�ò�׽ֵ��ΪRCHF�ĵ�Уֵ	
	trim_value_8M = T2_capture_1ms_for_TRIM(rchf_8M,7812);	 	//1024HZ ��׽Դ��8M����Դ  .����ֵ��7812
	//�� trim_value �� �����ܶ���Ϣ  ���С��0X7F ���ǵ�Уֵ  �����Ǵ�����Ϣ
	
			if(trim_value_8M>0x7f)
			RCHFADJ = 0x40;
		else
			RCHFADJ = trim_value_8M; 			//������ӽ���ʱ�� bit7����Ч��
}


/************************************************************************/
/* ��������SET_MCLK												*/
/* ����˵����CLK���ó���				    		*/
/* ��ڲ�����N/A													*/
/* ���ڲ�����N/A													*/
/************************************************************************/
void SET_MCLK(INT8U mclk,INT8U en_hs_div,INT8U div_num)
{

	switch(mclk)
	{
		case LSCLK: //LSCLKֻ����XTLFͣ��״̬�²Ż��л�ΪRCLP��������CPU����

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
/* ��������INIT_IO_PAD													*/
/* ����˵����IO�ܽų�ʼ���ӳ��� 							*/
/* ��ڲ�����N/A													*/
/* ���ڲ�����N/A													*/
/************************************************************************/
void Init_IO_PAD(void)
{
	PBPPEN = 0;
	PBFCR1 = 0;
	PBFCR2 = 0;
	PBPPEN |= BIT0 | BIT1 | BIT2;//ʹ��PA0�����칦��
	PBFCR1 |= BIT0 | BIT1 | BIT2;//{PAFCR2[0]:PAFCR1[0]}=2'b01,ʹ��PA0���
	PBDATA = 0xff;
	
	
	PAFCR2 &= 0X0F;
	PAPPEN |= BIT4 | BIT5 | BIT6 | BIT7;//ʹ��PA0�����칦��
	PAFCR1 |= BIT4 | BIT5 | BIT6 | BIT7;//{PAFCR2[0]:PAFCR1[0]}=2'b01,ʹ��PA0���
	PADATA = 0xff;	
	
	
	
	
	PGFCR2 &= ~BIT7;
	PGPPEN |= BIT7;//ʹ��PG7�����칦��
	PGFCR1 |= BIT7;//{PGFCR2[7]:PGFCR1[7]}=2'b01,ʹ��PG7���
	
	PEFCR2 &= ~BIT0;
	PEPPEN |= BIT0;//ʹ��PE0�����칦��
	PEFCR1 |= BIT0;//{PEFCR2[0]:PEFCR1[0]}=2'b01,ʹ��PE0���:CHARGE
	
	PEFCR2 &= ~BIT1;
	PEPPEN |= BIT1;//ʹ��PE1�����칦��
	PEFCR1 |= BIT1;//{PEFCR2[1]:PEFCR1[1]}=2'b01,ʹ��PE1���:DISCHARGE		
	
	PEDATA |= 0X03;//�رճ�ŵ�ģ�⿪��
}

/************************************************************************/
/* ��������IInit_INT												    */
/* ����˵�����ж���س�ʼ���ӳ���      					    		*/
/* ��ڲ�����N/A													    */
/* ���ڲ�����N/A													    */
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
/* ��������INIT_SYSWDT													*/
/* ����˵����ʹ�ܿ��Ź�							*/
/* ��ڲ�����N/A													*/
/* ���ڲ�����N/A													*/
/************************************************************************/
void Set_SYSWDT(INT8U wdt_Period)
{
	//SYSWDT
	WDT_CLR();//�Ƚ���һ���幷
	WDTCFG = wdt_Period;//��������
}

/************************************************************************/
/* ��������INIT_SYS													*/
/* ����˵����ϵͳ��ʼ���ӳ���       							    */
/* ��ڲ�����N/A													*/
/* ���ڲ�����N/A													*/
/************************************************************************/
void INIT_SYS(void)
{
	Set_SYSWDT(wdtov_time_4096S);			//����wdt����
	PDRCTL = pdr_en;  //��PDR
	BORCTL = 0;  //��BOR
	//BORCTL = 0x01;  //�ر�BOR
	Init_INT();   //�����ж�
}
