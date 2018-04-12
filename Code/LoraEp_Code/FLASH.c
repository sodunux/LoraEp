#include "init.h"

//data flash:ffa000~ffbfff; bootflash:ff1000~ff9fff
//fl_addr:1000~bfff
void erase_FLASH_CHIP(void)     
{
	uchar *DB_FLASH = (INT8U *)(0xff0000);    
	flash_op_end = 0;
	
	EPFLAG = 0;
	EA = 1;					//1.	���浱ǰ�ж�״̬���ر��޹��ж�
	AIE5 = 1;
	FLSIE = err_ie | fls_ie;
// 	EA = 0;					//1.	���浱ǰ�ж�״̬���ر��޹��ж�
// 	AIE5 = 0;
// 	FLSIE = 0;
	
	ERCSR = Era_Type_c | Era_Req;		//2.	����ERCSR�Ĵ�����Erase Request λ
	PRCSR = 0x00;		//3.	���PRSCR�Ĵ�����Program Requestλ
	FLSKEY = 0x96;		//4.	��Flash Key��ַд��8'h96
	FLSKEY = 0x7D;		//5.	��Flash Key��ַд��8'h7d
	*DB_FLASH = 0xc9;		//6.	��Ŀ�������������ַд8��hC9
	_nop_();_nop_();_nop_();_nop_();_nop_();
	_nop_();_nop_();_nop_();_nop_();_nop_();
	
	while(!flash_op_end){}
	
// 	while(!(EPFLAG & fls_if))   //��д��ɺ��˳�
// 	{
// 		if(EPFLAG & B1110_0000)
// 		{
// 				UartTx_0(EPFLAG);
// 	// 			SOFTRST = 0x5C;   //���������λ
// 		}
// 	}

	EA = 1;
	
}

//data flash:ffa000~ffbfff; bootflash:ff1000~ff9fff
//fl_addr:1000~bfff
void erase_FLASH(INT32U fl_addr)      //SECTOR 512B/SECTOR
{
	uchar *DB_FLASH = (INT8U *)(fl_addr);    
	flash_op_end = 0;
	
	EPFLAG = 0;
	EA = 1;					//1.	���浱ǰ�ж�״̬���ر��޹��ж�
	AIE5 = 1;
	FLSIE = err_ie | fls_ie;

// 	EA = 0;					//1.	���浱ǰ�ж�״̬���ر��޹��ж�
// 	AIE5 = 0;
// 	FLSIE = 0;
	
	ERCSR = Era_Type_s | Era_Req;		//2.	����ERCSR�Ĵ�����Erase Request λ
	PRCSR = 0x00;		//3.	���PRSCR�Ĵ�����Program Requestλ
	FLSKEY = 0x96;		//4.	��Flash Key��ַд��8'h96
	FLSKEY = 0x7D;		//5.	��Flash Key��ַд��8'h7d
	*DB_FLASH = 0xc9;		//6.	��Ŀ�������������ַд8��hC9
	_nop_();_nop_();_nop_();_nop_();_nop_();
	_nop_();_nop_();_nop_();_nop_();_nop_();
	
	while(!flash_op_end){}
		
// 	while(!(EPFLAG & fls_if))   //��д��ɺ��˳�
// 	{
// 		if(EPFLAG & B1110_0000)
// 		{
// 				UartTx_0(EPFLAG);
// 	// 			SOFTRST = 0x5C;   //���������λ
// 		}
// 	}

	EA = 1;
	
}

//data flash:ffa000~ffbfff; bootflash:ff1000~ff9fff
//fl_addr:1000~bfff
void write_FLASH(INT32U fl_addr,INT8U fl_data)
{
	
	uchar *DB_FLASH = (INT8U *)(fl_addr);    
	flash_op_end = 0;
	
	EPFLAG = 0;
	EA = 1;					//1.	���浱ǰ�ж�״̬���ر��޹��ж�
	AIE5 = 1;
	FLSIE = err_ie | fls_ie;
	
	
	ERCSR = 0x00;		//2.	���ERCSR�Ĵ�����Erase Request λ
	PRCSR = Pro_Req;		//3.	����PRSCR�Ĵ�����Program Requestλ
	FLSKEY = 0xA5;		//4.	��Flash Key��ַд��8'hA5
	FLSKEY = 0xF1;		//5.	��Flash Key��ַд��8'hF1
	*DB_FLASH = fl_data;  //6.	��Flashָ���ĵ�ַд�������ݣ�д����֮ǰ���Ȳ�������
	//*NVR_FLASH = 0x81;
	//UartTx_0(EPFLAG);
	
	_nop_();_nop_();_nop_();_nop_();_nop_();
	
	//while(!flash_op_end){}
		
// 	if(EPFLAG & B1110_0000)
// 		UartTx_0(EPFLAG);
// 		//SOFTRST = 0x5C;   //���������λ
	EA = 1;
}

//data flash:ffa000~ffbfff; bootflash:ff1000~ff9fff
//fl_addr:1000~bfff
void write_FLASH_Buffer(INT32U fl_addr,INT8U *point_data)
{
	INT16U i;
	INT8U *DB_RAM = (INT8U *)RAM_LAST_SECTOR;   
	uchar *DB_FLASH = (INT8U *)(fl_addr);    
	flash_op_end = 0;
	
	EPFLAG = 0;
	EA = 1;					//1.	���浱ǰ�ж�״̬���ر��޹��ж�
	AIE5 = 1;
	FLSIE = err_ie | fls_ie;
	
	
	ERCSR = 0x00;		//2.	���ERCSR�Ĵ�����Erase Request λ
	PRCSR = Pro_Req;		//3.	����PRSCR�Ĵ�����Program Requestλ
	
	
	for(i=0;i<512;i++)	//4. �������ݻ���,��RAM���512�ֽ������������
	{
		*DB_RAM = *point_data;
		point_data++;
		DB_RAM++;
	}		
	
	
	FLSKEY = 0xA5;		//5.	��Flash Key��ַд��8'hA5
	FLSKEY = 0xC4;		//6.	��Flash Key��ַд��8'hC4
	
	*DB_FLASH = 0;  //7.	��Flashָ���ĵ�ַд���κ����ݣ��������
	
	//UartTx_0(EPFLAG);
	
	_nop_();_nop_();_nop_();_nop_();_nop_();
	
	while(!flash_op_end){}
		
// 	if(EPFLAG & B1110_0000)
// 		UartTx_0(EPFLAG);
// 		//SOFTRST = 0x5C;   //���������λ
	EA = 1;
}


//data flash:ffa000~ffbfff; bootflash:ff1000~ff9fff
//fl_addr:1000~bfff
INT8U read_FLASH(INT32U fl_addr)
{
	uchar fl_data;
	uchar *DB_FLASH = (INT8U *)(fl_addr);   

	fl_data = *DB_FLASH;
	_nop_();_nop_();

	return fl_data;
}

INT16U flash_wr_check(INT32U fl_addr,INT8U fl_data)
{
	INT16U i;
	INT8U temp_data;
	
	for(i=0;i<0x200;i++)
	{
		write_FLASH(i+fl_addr,fl_data); //fl_addr��ʼ������д��fl_data
	}
	
	for(i=0;i<0x200;i++)
	{
		temp_data = read_FLASH(i+fl_addr);  //fl_addr��ʼ�����������ж��Ƿ�Ϊд�������fl_data
		if(temp_data!=fl_data) return i;  //��У������ص�ǰ��ƫ�Ƶ�ַ
	}	
	return 0x55aa;
}
	