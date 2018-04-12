#include "init.h"

unsigned char data f_i,f_j,f_k,f_l;				//���ٲ�����i j  k

LCD_char_flag DISP_char_flag; 
LCD_DISP_BUFF DISP_buff;    
  
  INT8U  Dispdata0Buf;
  INT8U  Dispdata1Buf;
  INT8U  Dispdata2Buf;
  INT8U  Dispdata3Buf;
  INT8U  Dispdata4Buf;
  INT8U  Dispdata5Buf;
  INT8U  Dispdata6Buf;
  INT8U  Dispdata7Buf;
  INT8U  Dispdata8Buf;
  INT8U  Dispdata9Buf;
  INT8U  Dispdata10Buf;
  INT8U  Dispdata11Buf;						
  INT8U  Dispdata12Buf;
  INT8U  Dispdata13Buf;
  INT8U  Dispdata14Buf;
  INT8U  Dispdata15Buf;
  INT8U  Dispdata16Buf;
	INT8U  Dispdata17Buf;
	
//---------global variables------------
//������빲���������һ��
const unsigned char  code   DispTab[]={
	0x3F,  //"0"
	0x06,  //"1"
	0x5B,  //"2"
	0x4F,  //"3"
	0x66,  //"4"
	0x6D,  //"5"
	0x7D,  //"6"
	0x07,  //"7"
	0x7F,  //"8"
	0x6F,  //"9"
	0x77,  //"A"
	0x7C,  //"b"
	0x58,  //"c"
	0x5E,  //"d"
	0x79,  //"E"
	0x71,  //"F"
	0x76,  //"H"  16
	0x74,  //"h"  17
	0x38,  //"L"	18
	0x54,  //"n"  19
	0x37,  //"N" 20
	0x5C,  //"o" 21   0
	0x73,  //"P" 22
	0x67,  //"q" 23
	0x50,  //"r" 24
	0x78,  //"t" 25
	0x3E,  //"U" 26
	0x6E,  //"y" 27
	0x40,  //"-" 28
	0x08,  //"_" 29	
	0x80,  //"." 
	0x30,  //"I"	
	0x00   //Ϩ��
};

void INIT_DISP_RIN_PAD(void)
{
	PERICLK_CTRL2 |= pdc_clken;
	PAFCR2 = 0xff;
	PAFCR1 = 0xff;//{PAFCR2[0~7]:PAFCR1[0~7]}=2'b11,ʹ��PA0~PA7��DISP���Ź���:COM0~COM3,COM4/SEG24,COM5/SEG25,SEG0,SEG1

	PBFCR2 |= 0x3f;
	PBFCR1 |= 0x3f;//{PBFCR2[0~5]:PBFCR1[0~5]}=2'b11,ʹ��PB0~PB5��DISP���Ź���:SEG2~SEG7

	PCFCR2 = 0xFF;
	PCFCR1 = 0xFF;//{PCFCR2[0~7]:PCFCR1[0~7]}=2'b11,ʹ��PC0~PC7��DISP���Ź���:SEG8~SEG15	
	
	PDFCR2 |= 0x03;
	PDFCR1 |= 0x03;//{PDFCR2[0~1]:PDFCR1[0~1]}=2'b11,ʹ��PD0~PD1��DISP���Ź���:SEG16~SEG17
	
	PEFCR2 |= 0xf8;
	PEFCR1 |= 0xf8;//{PEFCR2[3~7]:PEFCR1[3~7]}=2'b11,ʹ��PE3~PE7��DISP���Ź���:SEG18~SEG22
	
	PFFCR2 |= 0x01;
	PFFCR1 |= 0x01;//{PFFCR2[0~5]:PFFCR1[0~5]}=2'b11,ʹ��PF0~PF5��DISP���Ź���:SEG23,VDISP1,VDISP2,VDISP3,VCIN1,VCIN2	
		
	COMSEG_EN1 = 0xff;
	COMSEG_EN2 = 0xff;
	COMSEG_EN3 = 0xff;
	COMSEG_EN4 = 0xff;
}


void INIT_DISP_COUT_PAD(void)
{
	PERICLK_CTRL2 |= pdc_clken;
	
	PAFCR2 = 0xff;
	PAFCR1 = 0xff;//{PAFCR2[0~7]:PAFCR1[0~7]}=2'b11,ʹ��PA0~PA7��DISP���Ź���:COM0~COM3,COM4/SEG24,COM5/SEG25,SEG0,SEG1

	PBFCR2 |= 0x3f;
	PBFCR1 |= 0x3f;//{PBFCR2[0~5]:PBFCR1[0~5]}=2'b11,ʹ��PB0~PB5��DISP���Ź���:SEG2~SEG7

	PCFCR2 = 0xFF;
	PCFCR1 = 0xFF;//{PCFCR2[0~7]:PCFCR1[0~7]}=2'b11,ʹ��PC0~PC7��DISP���Ź���:SEG8~SEG15	
	
	PDFCR2 |= 0x03;
	PDFCR1 |= 0x03;//{PDFCR2[0~1]:PDFCR1[0~1]}=2'b11,ʹ��PD0~PD1��DISP���Ź���:SEG16~SEG17
	
	PEFCR2 |= 0xf8;
	PEFCR1 |= 0xf8;//{PEFCR2[3~7]:PEFCR1[3~7]}=2'b11,ʹ��PE3~PE7��DISP���Ź���:SEG18~SEG22
	
	PFFCR2 |= 0x3f;
	PFFCR1 |= 0x3f;//{PFFCR2[0~5]:PFFCR1[0~5]}=2'b11,ʹ��PF0~PF5��DISP���Ź���:SEG23,VDISP1,VDISP2,VDISP3,VCIN1,VCIN2	
		
	COMSEG_EN1 = 0xff;
	COMSEG_EN2 = 0xff;
	COMSEG_EN3 = 0xff;
	COMSEG_EN4 = 0xff;
}

void DISP_TEST_EN(INT8U disp_mode)
{
	PERICLK_CTRL0 |= lcd_clken;

	DISPCTRL = 0;
	LCDSET = 0;
	LCDTEST = 0;
	DISPCTRL |= antipolar;    // LCD��COM��SEG����LCD�رյ�����½ӵ�
	
	LCDSET |= TX_Buf[0];//com��
	LCDDF = TX_Buf[1];
	
	LCDSET |= (TX_Buf[2]<<1);   //��������
	LCDSET |= (TX_Buf[3]<<2);   //�ⲿVLCD�������
	
	DISPCTRL |= (TX_Buf[4]<<2);   //��˸ʹ�ܿ���λ
	TON = TX_Buf[5];
	TOFF = TX_Buf[6];
	
	DISPIE = disp_donie | disp_doffie;
	AIE4 = 1;
	
	ENMODE = disp_mode | (TX_Buf[9]<<5) | (TX_Buf[8]<<1) | (TX_Buf[7]<<3);
	
	LCDBIAS = TX_Buf[10];
	
	DISPCTRL |= disp_lcd_en;	
		
}


/*************************************************
�������ƣ�chip_poweron_display
��Ҫ������оƬ��λ��ʹ��LCD��ʾ������Ϣ�Ĳ���  �����������ʾ�����쳣��Ϣ  �����н���ʹ������ʾ��λ��Ϣ
		  LCD����ʾ�ؼ���3������  
		  clr_LCD_ram_segment_symbol   ���RAM�е���ʾ������  ����8�ַ���������־    ��������Һ���ı����--��Ӧ  �����Ӧ��Һ���Լ�Ӳ�����Ӷ��й�
		  set_LCD_ram_by_symbol_flag   �����û���Һ����ʾ��־��  ��ʾ������
		  
		  set_LCD_segment_by_loc_and_value				�ı�RAM��������8����
		  set_LCD_segment_by_point_num_byte_loc			�ı�RAM��������8����
		  set_LCD_segment_by_point_num_compbyte_loc		�ı�RAM��������8����
		  
		  �������������� RAM��ʾ�������������Ѿ����޸��� �����Ҫ�޸ĵ��� ��ʾ�Ĵ�����
		  
		  write_LCD_ram_to_LCD_register();				//��RAM��ֵд���Ĵ����� һ��Ҫ����������ܸ�����ʾ
		  
���룺 	  Ŀǰ֧�����ֱ�־   
����� 	  �޸ı�־  
*************************************************/
void chip_poweron_display(void)
{

unsigned char buff[8];

#if 0
unsigned char i,RSTFLAG_bakeup,RCHF_trim_value;


RSTFLAG_bakeup = RSTFLAG;
RCHF_trim_value = RCHFADJ;


//������ʵ��һ��LCD��ʾ�ĸ��ֺ��� �û�ȥʹ��

//1 �趨���е�LCD��ʾ�Ĵ���=0XFF 
	LCD_all_reg_0xFF();
	LCD_display_delay(LCD_disp_delay);		//�ȴ���ʾЧ��
	
//2 �趨���е���ʾ�Ĵ���=0X00
	LCD_all_reg_0x00();	
	LCD_display_delay(LCD_disp_delay);		//�ȴ���ʾЧ��


//3 ������е���ʾ ��������оƬ��  RAM  �����ⲿ�Ĵ�����
	clr_LCD_ram_segment_symbol();
	//�趨����Ҫ��ʾ������  ����Ķ���ֵ  1-10�� 8�����λ��   DispTab[X] ��X��Ӧ�Ķ����
	set_LCD_segment_by_loc_and_value(1,0x0a);   //��1��λ�õ�8��ʾA
	set_LCD_segment_by_loc_and_value(2,0x0b);
	set_LCD_segment_by_loc_and_value(3,0x0c);
	set_LCD_segment_by_loc_and_value(4,0x0d);
	set_LCD_segment_by_loc_and_value(5,1);
	set_LCD_segment_by_loc_and_value(6,2);
	set_LCD_segment_by_loc_and_value(7,3);
	set_LCD_segment_by_loc_and_value(8,4);
	set_LCD_segment_by_loc_and_value(9,5);
	set_LCD_segment_by_loc_and_value(10,6);		//��10��λ�õ�8��ʾ6	
	write_LCD_ram_to_LCD_register();			//��RAM��ֵд���Ĵ�����
	LCD_display_delay(LCD_disp_delay);			//�ȴ���ʾЧ��

//4 ��ʾRAM���������������� ��֧���ֽ���ʾ  ��ʾ�ĸ��� ��LCD�ĸ�������
	clr_LCD_ram_segment_symbol();
	buff[0]=0X12;
	buff[1]=0X34;
	buff[2]=0X56;
	buff[3]=0X78;
	buff[4]=0X9A;
	set_LCD_segment_by_point_num_compbyte_loc(buff,5,1);  //compbyte �����ֽڸߵ�4bit��ʾ  buff ���鿪ʼ�� 5���ֽڵ�����  ��ʾ��ʼλ���ڵ�1���ַ�
	write_LCD_ram_to_LCD_register();			//��RAM��ֵд���Ĵ�����
	RTCIF2&=~hz2_if;
	LCD_display_delay(LCD_disp_delay);			//�ȴ���ʾЧ��
	
//5 �������Ҫ��ʾ�������ǰ��ֽ�ģʽ  ����һ��Ҫ��ʾһЩ  0-F��� 5bit�������ַ�  ͬ�����Բ���
	clr_LCD_ram_segment_symbol();
	buff[0]=char_H;   //��ʾTOEN1830
	buff[1]=char_e;
	buff[2]=char_L;
	buff[3]=char_L;	
	buff[4]=char_O;
	buff[5]=3;
	buff[6]=7;
	buff[7]=5;
	set_LCD_segment_by_point_num_byte_loc(buff,8,3);  //byte�����ֽ�8bit��ʾ   SM3_key ���鿪ʼ�� 8���ֽڵ�����  ��ʾ��ʼλ���ڵ�3���ַ�
	write_LCD_ram_to_LCD_register();			//��RAM��ֵд���Ĵ�����

	LCD_display_delay(LCD_disp_delay);			//�ȴ���ʾЧ��


//6 ��ʾDEMOҺ���ϵ�  ���汾��
	clr_LCD_ram_segment_symbol();
	DISP_char_flag.bits.s_buchang=1;			//��ʾ����
	DISP_char_flag.bits.s_tiaojiao=1;			//��ʾ��У
	DISP_char_flag.bits.s_moshi	=1;    
	DISP_char_flag.bits.s_banben=1;    
	write_LCD_symbol_to_LCD_register();			//���±�־����ʾ�Ĵ�����ʾ

	LCD_display_delay(LCD_disp_delay);			//�ȴ���ʾЧ��
	
	
//7 ����ָ��λ�����LCD�ַ�  ��ʾ  
	for(i=4;i<11;i++)
	{
		set_LCD_ram_by_loc_segment(i,0x00);    	//�����ֱ����ʾ��LCD�Ĵ��� ���ǽ���Ӧ��ʾ�����Ч
		write_LCD_ram_to_LCD_register();		//��RAM��ֵд���Ĵ�����
		LCD_display_delay(LCD_disp_delay);		//�ȴ���ʾЧ��
	}

//8 �������
	DISP_char_flag.bits.s_T2=0;
	write_LCD_symbol_to_LCD_register();			//���±�־����ʾ�Ĵ�����ʾ
	LCD_display_delay(LCD_disp_delay);			//�ȴ���ʾЧ��
	DISP_char_flag.bits.s_P8=0;
	write_LCD_symbol_to_LCD_register();			//���±�־����ʾ�Ĵ�����ʾ
	LCD_display_delay(LCD_disp_delay);			//�ȴ���ʾЧ��


//9 ��λ��ʾ����
	for(i=3;i<11;i++)
	{
		set_LCD_segment_by_loc_and_value(i,0x8); //��ʾ0X08�Ķ�����Ϣ  ע��� set_LCD_ram_by_loc_segment������
		write_LCD_ram_to_LCD_register();		 //��RAM��ֵд���Ĵ�����
		LCD_display_delay(LCD_disp_delay);		//�ȴ���ʾЧ��
	}

//10 �����ʾ��λ����Ϣ   RST:XX YY 		XX�Ǹ�λ��־  YY��RCHF��TRIMֵ
	clr_LCD_ram_segment_symbol();
	buff[0]=char_r;   //��ʾrst
	buff[1]=char_S;
	buff[2]=char_t;
	buff[3]=((RSTFLAG_bakeup>>4)&0X0F);
	buff[4]=(RSTFLAG_bakeup&0X0F);
	buff[5]=((RCHF_trim_value>>4)&0X0F);
	buff[6]=(RCHF_trim_value&0X0F);	
	set_LCD_segment_by_point_num_byte_loc(buff,7,4);  //byte�����ֽ�8bit��ʾ   SM3_key ���鿪ʼ�� 8���ֽڵ�����  ��ʾ��ʼλ���ڵ�3���ַ�
	write_LCD_ram_to_LCD_register();			//��RAM��ֵд���Ĵ�����

	//���� T1��־  P8��־
	DISP_char_flag.bits.s_T2=1;
	DISP_char_flag.bits.s_P8=1;
	write_LCD_symbol_to_LCD_register();			//���±�־����ʾ�Ĵ�����ʾ
	
	LCD_display_delay(2*LCD_disp_delay);			//�ȴ���ʾЧ��
//��ʾ����
#endif
	
//5 �������Ҫ��ʾ�������ǰ��ֽ�ģʽ  ����һ��Ҫ��ʾһЩ  0-F��� 5bit�������ַ�  ͬ�����Բ���
	clr_LCD_ram_segment_symbol();
	buff[0]=char_H;   //��ʾTOEN1830
	buff[1]=char_E;
	buff[2]=char_L;
	buff[3]=char_L;	
	buff[4]=0;
	buff[5]=3;
	buff[6]=7;
	buff[7]=5;
	set_LCD_segment_by_point_num_byte_loc(buff,8,3);  //byte�����ֽ�8bit��ʾ   SM3_key ���鿪ʼ�� 8���ֽڵ�����  ��ʾ��ʼλ���ڵ�3���ַ�
	write_LCD_ram_to_LCD_register();			//��RAM��ֵд���Ĵ�����

	DISP_char_flag.bits.s_P7=1;
	write_LCD_symbol_to_LCD_register();			//���±�־����ʾ�Ĵ�����ʾ
	
//	LCD_display_delay(LCD_disp_delay);			//�ȴ���ʾЧ��	
}

/***************************************************************/
// Function    : LcdDisp
// Input       : lcdbit-which bit display,from left 1~8
//               show-display data
// Output      : none
// Description : display on
/***************************************************************/
void LcdDisp(INT8U lcdbit, INT8U show)
{
 	INT8U Bitcode;

     Bitcode = DispTab[show];

	
	switch( lcdbit )	//������һλ���
	{	
        //------------ ��1��"8"�ַ� ----------	��� com lcdseg mcuseg
		case 0x01:
			if( (Bitcode & 0x01) == 0x01 ) 		//1A,com3,seg4 
	            SET(Dispdata3Buf,SEG4);    
			if( (Bitcode & 0x02) == 0x02 )		//1B,com2,seg4 
	            SET(Dispdata2Buf,SEG4); 
			if( (Bitcode & 0x04) == 0x04 )   	//1C,com1,seg4 
	            SET(Dispdata1Buf,SEG4); 
			if( (Bitcode & 0x08) == 0x08 )    	//1D,com0,seg4
		        SET(Dispdata0Buf,SEG4); 
			if( (Bitcode & 0x10) == 0x10 )		//1E,com1,seg3 
		        SET(Dispdata1Buf,SEG3);  
			if( (Bitcode & 0x20) == 0x20 )    	//1F,com3,seg3 
		        SET(Dispdata3Buf,SEG3); 
			if( (Bitcode & 0x40) == 0x40 )   	//1G,com2,seg3 
	            SET(Dispdata2Buf,SEG3);
		break;
	
		//------------ ��2��"8"�ַ� ----------
		case 0x02:
			if( (Bitcode & 0x01) == 0x01 ) 		//2A,com3,seg6
	            SET(Dispdata3Buf,SEG6);    
			if( (Bitcode & 0x02) == 0x02 )		//2B,com2,seg6 
	            SET(Dispdata2Buf,SEG6); 
			if( (Bitcode & 0x04) == 0x04 )   	//2C,com1,seg6 
	            SET(Dispdata1Buf,SEG6); 
			if( (Bitcode & 0x08) == 0x08 )    	//2D,com0,seg6 
		        SET(Dispdata0Buf,SEG6); 
			if( (Bitcode & 0x10) == 0x10 )		//2E,com1,seg5 
		        SET(Dispdata1Buf,SEG21);  
			if( (Bitcode & 0x20) == 0x20 )    	//2F,com3,seg5 
		        SET(Dispdata3Buf,SEG21); 
			if( (Bitcode & 0x40) == 0x40 )   	//2G,com2,seg5
	            SET(Dispdata2Buf,SEG21);	  
		break;
	
		//------------ ��3��"8"�ַ� ----------
		case 0x03:
			if( (Bitcode & 0x01) == 0x01 ) 		//3A,com0,seg11 
	            SET(Dispdata4Buf,SEG11);    
			if( (Bitcode & 0x02) == 0x02 )		//3B,com1,seg11  
	            SET(Dispdata5Buf,SEG11); 
			if( (Bitcode & 0x04) == 0x04 )   	//3C,com2,seg11  
	            SET(Dispdata6Buf,SEG11); 
			if( (Bitcode & 0x08) == 0x08 )    	//3D,com3,seg10
		        SET(Dispdata7Buf,SEG10); 
			if( (Bitcode & 0x10) == 0x10 )		//3E,com2,seg10 
		        SET(Dispdata6Buf,SEG10);  
			if( (Bitcode & 0x20) == 0x20 )    	//3F,com0,seg10 
		        SET(Dispdata4Buf,SEG10); 
			if( (Bitcode & 0x40) == 0x40 )   	//3G,com1,seg10  
	            SET(Dispdata5Buf,SEG10);
		break;
	
		//------------ ��4��"8"�ַ� ----------
		case 0x04:
			if( (Bitcode & 0x01) == 0x01 ) 		//4A,com0,seg13
	            SET(Dispdata4Buf,SEG13);    
			if( (Bitcode & 0x02) == 0x02 )		//4B,com1,seg13 
	            SET(Dispdata5Buf,SEG13); 
			if( (Bitcode & 0x04) == 0x04 )   	//4C,com2,seg13 
	            SET(Dispdata6Buf,SEG13); 
			if( (Bitcode & 0x08) == 0x08 )    	//4D,com3,seg12 
		        SET(Dispdata7Buf,SEG12); 
			if( (Bitcode & 0x10) == 0x10 )		//4E,com2,seg12 
		        SET(Dispdata6Buf,SEG12);  
			if( (Bitcode & 0x20) == 0x20 )    	//4F,com0,seg12
		        SET(Dispdata4Buf,SEG12); 
			if( (Bitcode & 0x40) == 0x40 )   	//4G,com1,seg12  
	            SET(Dispdata5Buf,SEG12);
		break;
	
		//------------ ��5��"8"�ַ� ----------
		case 0x05:
			if( (Bitcode & 0x01) == 0x01 ) 		//5A,com0,seg15
	            SET(Dispdata4Buf,SEG15);    
			if( (Bitcode & 0x02) == 0x02 )		//5B,com1,seg15 
	            SET(Dispdata5Buf,SEG15); 
			if( (Bitcode & 0x04) == 0x04 )   	//5C,com2,seg15 
	            SET(Dispdata6Buf,SEG15); 
			if( (Bitcode & 0x08) == 0x08 )    	//5D,com3,seg14
		        SET(Dispdata7Buf,SEG14); 
			if( (Bitcode & 0x10) == 0x10 )		//5E,com2,seg14 
		        SET(Dispdata8Buf,SEG14);  
			if( (Bitcode & 0x20) == 0x20 )    	//5F,com0,seg14 
		        SET(Dispdata4Buf,SEG14); 
			if( (Bitcode & 0x40) == 0x40 )   	//5G,com1,seg14  
	            SET(Dispdata5Buf,SEG14);
		break;
	
		//------------ ��6��"8"�ַ� ----------
		case 0x06:
			if( (Bitcode & 0x01) == 0x01 ) 		//6A,com0,seg17
	            SET(Dispdata8Buf,SEG17);    
			if( (Bitcode & 0x02) == 0x02 )		//6B,com1,seg17
	            SET(Dispdata9Buf,SEG17); 
			if( (Bitcode & 0x04) == 0x04 )   	//6C,com2,seg17 
	            SET(Dispdata10Buf,SEG17); 
			if( (Bitcode & 0x08) == 0x08 )    	//6D,com3,seg16
		        SET(Dispdata11Buf,SEG16); 
			if( (Bitcode & 0x10) == 0x10 )		//6E,com2,seg16 
		        SET(Dispdata10Buf,SEG16);  
			if( (Bitcode & 0x20) == 0x20 )    	//6F,com0,seg16 
		        SET(Dispdata8Buf,SEG16); 
			if( (Bitcode & 0x40) == 0x40 )   	//6G,com1,seg16 
	            SET(Dispdata9Buf,SEG16);
		break;

		//------------ ��7��"8"�ַ� ----------
		case 0x07:
			if( (Bitcode & 0x01) == 0x01 ) 		//7A,com0,seg19
	            SET(Dispdata8Buf,SEG19);    
			if( (Bitcode & 0x02) == 0x02 )		//7B,com1,seg19 
	            SET(Dispdata9Buf,SEG19); 
			if( (Bitcode & 0x04) == 0x04 )   	//7C,com2,seg19 
	            SET(Dispdata10Buf,SEG19); 
			if( (Bitcode & 0x08) == 0x08 )    	//7D,com3,seg18
		        SET(Dispdata11Buf,SEG18); 
			if( (Bitcode & 0x10) == 0x10 )		//7E,com2,seg18 
		        SET(Dispdata10Buf,SEG18);  
			if( (Bitcode & 0x20) == 0x20 )    	//7F,com0,seg18 
		        SET(Dispdata8Buf,SEG18); 
			if( (Bitcode & 0x40) == 0x40 )   	//7G,com1,seg18  
	            SET(Dispdata9Buf,SEG18);
		break;
	
		//------------ ��8��"8"�ַ� ----------
		case 0x08:
			if( (Bitcode & 0x01) == 0x01 ) 		//8A,com0,seg21
	            SET(Dispdata8Buf,SEG21);    
			if( (Bitcode & 0x02) == 0x02 )		//8B,com1,seg21 
	            SET(Dispdata9Buf,SEG21); 
			if( (Bitcode & 0x04) == 0x04 )   	//8C,com2,seg21 
	            SET(Dispdata10Buf,SEG21); 
			if( (Bitcode & 0x08) == 0x08 )    	//8D,com3,seg20 
		        SET(Dispdata11Buf,SEG20); 
			if( (Bitcode & 0x10) == 0x10 )		//8E,com2,seg20 
		        SET(Dispdata10Buf,SEG20);  
			if( (Bitcode & 0x20) == 0x20 )    	//8F,com0,seg20 
		        SET(Dispdata8Buf,SEG20); 
			if( (Bitcode & 0x40) == 0x40 )   	//8G,com1,seg20 
	            SET(Dispdata9Buf,SEG20);
		break;

		//------------ ��9��"9"�ַ� ----------
		case 0x09:
			if( (Bitcode & 0x01) == 0x01 ) 		//9A,com0,seg23
	            SET(Dispdata8Buf,SEG23);    
			if( (Bitcode & 0x02) == 0x02 )		//9B,com1,seg23 
	            SET(Dispdata9Buf,SEG23); 
			if( (Bitcode & 0x04) == 0x04 )   	//9C,com2,seg23
	            SET(Dispdata10Buf,SEG23); 
			if( (Bitcode & 0x08) == 0x08 )    	//9D,com3,seg22
		        SET(Dispdata11Buf,SEG22); 
			if( (Bitcode & 0x10) == 0x10 )		//9E,com2,seg22
		        SET(Dispdata10Buf,SEG22);  
			if( (Bitcode & 0x20) == 0x20 )    	//9F,com0,seg22 
		        SET(Dispdata8Buf,SEG22); 
			if( (Bitcode & 0x40) == 0x40 )   	//9G,com1,seg22
	            SET(Dispdata9Buf,SEG22);
		break;

		//------------ ��10��"a"�ַ� ----------
		case 0x0a:
			if( (Bitcode & 0x01) == 0x01 ) 		//10A,com0,seg25
	            SET(Dispdata12Buf,SEG25);    
			if( (Bitcode & 0x02) == 0x02 )		//10B,com1,seg25 
	            SET(Dispdata13Buf,SEG25); 
			if( (Bitcode & 0x04) == 0x04 )   	//10C,com2,seg25
	            SET(Dispdata14Buf,SEG25); 
			if( (Bitcode & 0x08) == 0x08 )    	//10D,com3,seg24
		        SET(Dispdata15Buf,SEG24); 
			if( (Bitcode & 0x10) == 0x10 )		//10E,com2,seg24
		        SET(Dispdata14Buf,SEG24);  
			if( (Bitcode & 0x20) == 0x20 )    	//10F,com0,seg24 
		        SET(Dispdata12Buf,SEG24); 
			if( (Bitcode & 0x40) == 0x40 )   	//10G,com1,seg24
	            SET(Dispdata13Buf,SEG24);
		break;
			
		default:
		break;
	}
}

/*************************************************
�������ƣ�LCD_all_reg_0xFF
��Ҫ�������趨ȫ����ʾ�Ĵ���=0XFF ���ڲ���LCD
���룺 	  ��
����� 	  �޸���ʾ�Ĵ���
�޸���־��
*************************************************/
void  LCD_all_reg_0xFF(void)
{
unsigned char i;
	PERICLK_CTRL0|=lcd_clken;	
	for(i=0;i<18;i++)	XBYTE[0x0069+i]=0XFF;			//LCD����ģʽ ȫ������
}

/*************************************************
�������ƣ�LCD_all_reg_0x00
��Ҫ�������趨ȫ����ʾ�Ĵ���=0X00 ���ڲ���LCD
���룺 	  ��
����� 	  �޸���ʾ�Ĵ���
�޸���־��
*************************************************/
void  LCD_all_reg_0x00(void)
{
unsigned char i;
	PERICLK_CTRL0|=lcd_clken;	
	for(i=0;i<18;i++)	XBYTE[0x0069+i]=0X00;			//LCD����ģʽ ȫ������
}

/*************************************************
�������ƣ�clr_LCD_ram_segment_symbol
��Ҫ�������趨ȫ����ʾ����������ʾ���Ż�����=0X00
���룺 	  ��
����� 	  8�������ʾRAM����0   ��ʾ���źͱ�־ȫ����00
�޸���־��
*************************************************/
void clr_LCD_ram_segment_symbol(void)
{
unsigned char i;
	for(i=0;i<sizeof(DISP_buff.buff);i++)	DISP_buff.buff[i]=0X00;
	DISP_char_flag.Flag=0X00000000;			//������еı�־
}

/*************************************************
�������ƣ�write_LCD_symbol_to_LCD_register
��Ҫ�����������û���LCD���ű���ϢDISP_char_flag.bits.xxxx  �趨����ʾ������DISP_buff�ı�־��Ϣ  ����LCD�Ĵ���
���룺 	  ��
����� 	  ��ʾ�������ķ��ű�־���޸�
�޸���־��
*************************************************/
void write_LCD_symbol_to_LCD_register(void)
{
	if(DISP_char_flag.bits.s_buchang 		)		{buchang		=1;}  
	else																		{buchang		=0;}
	if(DISP_char_flag.bits.s_tiaojiao		)		{tiaojiao		=1;}
	else																		{tiaojiao		=0;}    
	if(DISP_char_flag.bits.s_S1				)			{S1				=1;}   
	else 																		{S1				=0;}
	if(DISP_char_flag.bits.s_T2				)			{T2				=1;}   
	else 																		{T2				=0;}
	if(DISP_char_flag.bits.s_T3				)			{T3				=1;}
	else 																		{T3				=0;}
	if(DISP_char_flag.bits.s_moshi			)		{moshi			=1;}   
	else 																		{moshi			=0;}
	if(DISP_char_flag.bits.s_pinglv			)		{pinglv			=1;}
	else 																		{pinglv			=0;}		   
	if(DISP_char_flag.bits.s_wendu			)		{wendu			=1;}
	else 																		{wendu			=0;}		   
	if(DISP_char_flag.bits.s_banben			)		{banben			=1;}
	else 																		{banben			=0;}		   
	if(DISP_char_flag.bits.s_tiaojiaozhi	)	{tiaojiaozhi	=1;}
	else 																		{tiaojiaozhi	=0;}		   
	if(DISP_char_flag.bits.s_qiangdu		)		{qiangdu		=1;}
	else 																		{qiangdu		=0;}		   
	if(DISP_char_flag.bits.s_bianshuju		)	{bianshuju		=1;}
	else 																		{bianshuju		=0;}		   
	if(DISP_char_flag.bits.s_biancheng		)	{biancheng		=1;}
	else 																		{biancheng		=0;}		   
	if(DISP_char_flag.bits.s_qian		    )		{qian			=1;}
	else 																		{qian			=0;}		   
	if(DISP_char_flag.bits.s_hou			)			{hou			=1;}
	else 																		{hou			=0;}		   
	if(DISP_char_flag.bits.s_hezhi			)		{hezhi			=1;}
	else 																		{hezhi			=0;}		   
	if(DISP_char_flag.bits.s_sizhao			)		{sizhao			=1;}
	else 																		{sizhao			=0;}		   
	if(DISP_char_flag.bits.s_miao		    )		{miao			=1;}
	else 																		{miao			=0;}		   
	if(DISP_char_flag.bits.s_S2				)			{S2				=1;}
	else 																		{S2				=0;}		   
	if(DISP_char_flag.bits.s_shi			)			{shi			=1;}
	else 																		{shi			=0;}		   
	if(DISP_char_flag.bits.s_adc			)			{adc			=1;}
	else 																		{adc			=0;}		   
	if(DISP_char_flag.bits.s_P3				)			{P3				=1;}
	else 																		{P3				=0;}		   
	if(DISP_char_flag.bits.s_P4				)			{P4				=1;}
	else 																		{P4				=0;}		   
	if(DISP_char_flag.bits.s_P5				)			{P5				=1;}
	else 																		{P5				=0;}		   
	if(DISP_char_flag.bits.s_P6				)			{P6				=1;}
	else 																		{P6				=0;}		   
	if(DISP_char_flag.bits.s_P7				)			{P7				=1;}
	else 																		{P7				=0;}		   
	if(DISP_char_flag.bits.s_P8				)			{P8				=1;} 
	else 																		{P8				=0;}		   
	if(DISP_char_flag.bits.s_P9				)			{P9				=1;}
	else 																		{P9				=0;}		   
	if(DISP_char_flag.bits.s_S3				)			{S3				=1;}
	else 																		{S3				=0;}		
	if(DISP_char_flag.bits.s_T1				)			{T1				=1;}
	else 																		{T1				=0;}	
	write_LCD_ram_to_LCD_register();			//���µ��Ĵ���	    
}

/*************************************************
�������ƣ�write_LCD_ram_to_LCD_register
��Ҫ������������ʾ������  RAM ��оƬ���ⲿ�Ĵ���   
���룺 	  ��
����� 	  ��ʾ���ݱ�����
�޸���־��
*************************************************/
void write_LCD_ram_to_LCD_register(void)
{
	PERICLK_CTRL0|=lcd_clken;
	DISPDATA0=DISP_buff.byte_mode.buf_DISPDATA0.byte_val;
	DISPDATA1=DISP_buff.byte_mode.buf_DISPDATA1.byte_val;
	DISPDATA2=DISP_buff.byte_mode.buf_DISPDATA2.byte_val;
	DISPDATA3=DISP_buff.byte_mode.buf_DISPDATA3.byte_val;
	DISPDATA4=DISP_buff.byte_mode.buf_DISPDATA4.byte_val;
	DISPDATA5=DISP_buff.byte_mode.buf_DISPDATA5.byte_val;
	DISPDATA6=DISP_buff.byte_mode.buf_DISPDATA6.byte_val;
	DISPDATA7=DISP_buff.byte_mode.buf_DISPDATA7.byte_val;
	DISPDATA8=DISP_buff.byte_mode.buf_DISPDATA8.byte_val;
	DISPDATA9=DISP_buff.byte_mode.buf_DISPDATA9.byte_val;
	DISPDATA10=DISP_buff.byte_mode.buf_DISPDATA10.byte_val;
	DISPDATA11=DISP_buff.byte_mode.buf_DISPDATA11.byte_val;
	DISPDATA12=DISP_buff.byte_mode.buf_DISPDATA12.byte_val;

}

/*************************************************
�������ƣ�set_LCD_ram_by_loc_segment
��Ҫ���������� �趨��λ��   ����ʾ��8������Ϣ 
���룺 	  ��
����� 	  8�������ʾRAM����0   ��ʾ���źͱ�־ȫ����00
�޸���־��
*************************************************/
void set_LCD_ram_by_loc_segment(unsigned char loc,unsigned char segment_data)
{
	switch(loc)
	{
		case 0x01:
			//����ȫ����0  �û�ֻҪ���¾ͻ��Զ�����ʾλ�ø������봦��
			A1=0;
			B1=0;
			C1=0;
			D1=0;
			E1=0;
			F1=0;
			G1=0;
			
			if(segment_data & 0x01)	A1=1;
			if(segment_data & 0x02)	B1=1;
			if(segment_data & 0x04)	C1=1;
			if(segment_data & 0x08)	D1=1;
			if(segment_data & 0x10)	E1=1;
			if(segment_data & 0x20)	F1=1;
			if(segment_data & 0x40)	G1=1;
			break;

		case 0x02:
			A2=0;
			B2=0;
			C2=0;
			D2=0;
			E2=0;
			F2=0;
			G2=0;
			
			if(segment_data & 0x01)	A2=1;
			if(segment_data & 0x02)	B2=1;
			if(segment_data & 0x04)	C2=1;
			if(segment_data & 0x08)	D2=1;
			if(segment_data & 0x10)	E2=1;
			if(segment_data & 0x20)	F2=1;
			if(segment_data & 0x40)	G2=1;
			break;
	
		case 0x03:
			A3=0;
			B3=0;
			C3=0;
			D3=0;
			E3=0;
			F3=0;
			G3=0;
	
			
			
			if(segment_data & 0x01)	A3=1;
			if(segment_data & 0x02)	B3=1;
			if(segment_data & 0x04)	C3=1;
			if(segment_data & 0x08)	D3=1;
			if(segment_data & 0x10)	E3=1;
			if(segment_data & 0x20)	F3=1;
			if(segment_data & 0x40)	G3=1;
			break;
		
		case 0x04:
			A4=0;
			B4=0;
			C4=0;
			D4=0;
			E4=0;
			F4=0;
			G4=0;
			
			if(segment_data & 0x01)	A4=1;
			if(segment_data & 0x02)	B4=1;
			if(segment_data & 0x04)	C4=1;
			if(segment_data & 0x08)	D4=1;
			if(segment_data & 0x10)	E4=1;
			if(segment_data & 0x20)	F4=1;
			if(segment_data & 0x40)	G4=1;
			break;
		
		case 0x05:
			A5=0;
			B5=0;
			C5=0;
			D5=0;
			E5=0;
			F5=0;
			G5=0;
		
			if(segment_data & 0x01)	A5=1;
			if(segment_data & 0x02)	B5=1;
			if(segment_data & 0x04)	C5=1;
			if(segment_data & 0x08)	D5=1;
			if(segment_data & 0x10)	E5=1;
			if(segment_data & 0x20)	F5=1;
			if(segment_data & 0x40)	G5=1;
			break;
		
		case 0x06:
			A6=0;
			B6=0;
			C6=0;
			D6=0;
			E6=0;
			F6=0;
			G6=0;
			
			if(segment_data & 0x01)	A6=1;
			if(segment_data & 0x02)	B6=1;
			if(segment_data & 0x04)	C6=1;
			if(segment_data & 0x08)	D6=1;
			if(segment_data & 0x10)	E6=1;
			if(segment_data & 0x20)	F6=1;
			if(segment_data & 0x40)	G6=1;
			break;
		
		case 0x07:
			A7=0;
			B7=0;
			C7=0;
			D7=0;
			E7=0;
			F7=0;
			G7=0;
			if(segment_data & 0x01)	A7=1;
			if(segment_data & 0x02)	B7=1;
			if(segment_data & 0x04)	C7=1;
			if(segment_data & 0x08)	D7=1;
			if(segment_data & 0x10)	E7=1;
			if(segment_data & 0x20)	F7=1;
			if(segment_data & 0x40)	G7=1;
			break;
		
		case 0x08:
			A8=0;
			B8=0;
			C8=0;
			D8=0;
			E8=0;
			F8=0;
			G8=0;
			if(segment_data & 0x01)	A8=1;
			if(segment_data & 0x02)	B8=1;
			if(segment_data & 0x04)	C8=1;
			if(segment_data & 0x08)	D8=1;
			if(segment_data & 0x10)	E8=1;
			if(segment_data & 0x20)	F8=1;
			if(segment_data & 0x40)	G8=1;
			break;
			
			
		case 0x09:
			A9=0;
			B9=0;
			C9=0;
			D9=0;
			E9=0;
			F9=0;
			G9=0;
			if(segment_data & 0x01)	A9=1;
			if(segment_data & 0x02)	B9=1;
			if(segment_data & 0x04)	C9=1;
			if(segment_data & 0x08)	D9=1;
			if(segment_data & 0x10)	E9=1;
			if(segment_data & 0x20)	F9=1;
			if(segment_data & 0x40)	G9=1;
			break;
		case 0x0a:
			A10=0;
			B10=0;
			C10=0;
			D10=0;
			E10=0;
			F10=0;
			G10=0;
			if(segment_data & 0x01)	A10=1;
			if(segment_data & 0x02)	B10=1;
			if(segment_data & 0x04)	C10=1;
			if(segment_data & 0x08)	D10=1;
			if(segment_data & 0x10)	E10=1;
			if(segment_data & 0x20)	F10=1;
			if(segment_data & 0x40)	G10=1;
			break;
			
		default:
			break;	
	}
}

/*************************************************
�������ƣ�set_LCD_segment_by_point_num_byte_loc
��Ҫ���������ݸ�����point  ����num������  ���β�LCDҺ����  �ٸ�����ʾ��ʼ��λ�� ����Ӧ��ʾλ��LCDRAM��1
���룺 	  pointָ����Ҫ��ʾ�ĵ�ַ  num����ʾ���ֽڸ���  lcd_location�ǿ�ʼ��λ�ã�֧��1-10��	 	
����� 	  �޸�LCD��ʾ������
�޸���־��
*************************************************/
void set_LCD_segment_by_loc_and_value(unsigned char loc,unsigned char disp_char)
{				
	
	if(disp_char<sizeof(DispTab))	f_k=DispTab[disp_char];	//�ڷ�Χ�ڲŲ����򲻲��
	else 							f_k=0X00;				//����ȫ��	
	set_LCD_ram_by_loc_segment(loc,f_k);  					//���ݲ��Ķ���ֵ  ����Ҫ��ʾ��bit ���õ�1					
}	

/*************************************************
�������ƣ�set_LCD_segment_by_point_num_byte_loc
��Ҫ���������ݸ�����point  ����num������  ���β�LCDҺ����  �ٸ�����ʾ��ʼ��λ�� ����Ӧ��ʾλ��LCDRAM��1
���룺 	  pointָ����Ҫ��ʾ�ĵ�ַ  num����ʾ���ֽڸ���  lcd_location�ǿ�ʼ��λ�ã�֧��1-10��	 	
����� 	  �޸�LCD��ʾ������
�޸���־��
*************************************************/
void set_LCD_segment_by_point_num_byte_loc(unsigned char *point,unsigned char num,unsigned char loc)
{				
												//��һ��λ��
	for(f_i=0;f_i<num;f_i++)
	{
		f_j=*(point+f_i);								//�������θ�4λ������	�е��������ַ� �� "-"
		if(f_j<sizeof(DispTab))	f_k=DispTab[f_j];		//�ڷ�Χ�ڲŲ����򲻲��
		else 					f_k=0X00;				//����ȫ��	
		f_l=loc+f_i;
		set_LCD_ram_by_loc_segment(f_l,f_k);  		//���ݲ��Ķ���ֵ  ����Ҫ��ʾ��bit ���õ�1					
	}
}	

/*************************************************
�������ƣ�set_LCD_segment_by_point_num_compbyte_loc      compbyte ѹ�����ֽ���Ϣ  ����һ���ֽ�ǰ���ֽںͺ���ֽڶ�Ҫ��ʾ
��Ҫ���������ݸ�����point  ����num������  ���β�LCDҺ����  �ٸ�����ʾ��ʼ��λ�� �趨��LCD��ʾRAM������
���룺 	  pointָ����Ҫ��ʾ�ĵ�ַ  num����ʾ���ֽڸ���  lcd_location�ǿ�ʼ��λ�ã�֧��1-10��	 	
����� 	  �޸�LCD��ʾ������
�޸���־��
*************************************************/
void set_LCD_segment_by_point_num_compbyte_loc(unsigned char *point,unsigned char num,unsigned char loc)
{				
unsigned char temp;
	f_l=loc;
	for(f_i=0;f_i<num;f_i++)
	{
		f_j=*(point+f_i);								
		f_k=(f_j>>4)&0X0F;								//����λ��ǰ
		if(f_k<sizeof(DispTab))	temp=DispTab[f_k];		//�ڷ�Χ�ڲŲ����򲻲��
		else 					temp=0X00;				//����ȫ��	
		set_LCD_ram_by_loc_segment(f_l,temp);  	//����оƬ�趨����ʾλ�ø�����ʾ����
		f_l++;											//��ʾλ������	
		f_k=f_j&0X0F;									//��4λ
		if(f_k<sizeof(DispTab))	temp=DispTab[f_k];		//�ڷ�Χ�ڲŲ����򲻲��
		else 					temp=0X00;				//����ȫ��	
		set_LCD_ram_by_loc_segment(f_l,temp);  	//����оƬ�趨����ʾλ�ø�����ʾ����
		f_l++;
	}
}	

/*************************************************
�������ƣ�LCD_display_delay
��Ҫ������LCD���ֲ�����ʾ����ʱ����
���룺 	  ��
����� 	  �޸ı�־RTC500ms��ʱ��־  
*************************************************/
void LCD_display_delay(unsigned char del_val)
{
unsigned char i;	
	RTCIF2&=~hz2_if;
	for(i=0;i<del_val;i++)
	{
		while((RTCIF2&hz2_if)==0X00)
		{	WDT_CLR();}
		RTCIF2&=~hz2_if;
	}
}
