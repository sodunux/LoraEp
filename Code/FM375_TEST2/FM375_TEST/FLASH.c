#include "init.h"

//data flash:ffa000~ffbfff; bootflash:ff1000~ff9fff
//fl_addr:1000~bfff
void erase_FLASH_CHIP(void)     
{
	uchar *DB_FLASH = (INT8U *)(0xff0000);    
	flash_op_end = 0;
	
	EPFLAG = 0;
	EA = 1;					//1.	保存当前中断状态并关闭无关中断
	AIE5 = 1;
	FLSIE = err_ie | fls_ie;
// 	EA = 0;					//1.	保存当前中断状态并关闭无关中断
// 	AIE5 = 0;
// 	FLSIE = 0;
	
	ERCSR = Era_Type_c | Era_Req;		//2.	设置ERCSR寄存器的Erase Request 位
	PRCSR = 0x00;		//3.	清除PRSCR寄存器的Program Request位
	FLSKEY = 0x96;		//4.	向Flash Key地址写入8'h96
	FLSKEY = 0x7D;		//5.	向Flash Key地址写入8'h7d
	*DB_FLASH = 0xc9;		//6.	向目标扇区内任意地址写8’hC9
	_nop_();_nop_();_nop_();_nop_();_nop_();
	_nop_();_nop_();_nop_();_nop_();_nop_();
	
	while(!flash_op_end){}
	
// 	while(!(EPFLAG & fls_if))   //擦写完成后退出
// 	{
// 		if(EPFLAG & B1110_0000)
// 		{
// 				UartTx_0(EPFLAG);
// 	// 			SOFTRST = 0x5C;   //如果出错，软复位
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
	EA = 1;					//1.	保存当前中断状态并关闭无关中断
	AIE5 = 1;
	FLSIE = err_ie | fls_ie;

// 	EA = 0;					//1.	保存当前中断状态并关闭无关中断
// 	AIE5 = 0;
// 	FLSIE = 0;
	
	ERCSR = Era_Type_s | Era_Req;		//2.	设置ERCSR寄存器的Erase Request 位
	PRCSR = 0x00;		//3.	清除PRSCR寄存器的Program Request位
	FLSKEY = 0x96;		//4.	向Flash Key地址写入8'h96
	FLSKEY = 0x7D;		//5.	向Flash Key地址写入8'h7d
	*DB_FLASH = 0xc9;		//6.	向目标扇区内任意地址写8’hC9
	_nop_();_nop_();_nop_();_nop_();_nop_();
	_nop_();_nop_();_nop_();_nop_();_nop_();
	
	while(!flash_op_end){}
		
// 	while(!(EPFLAG & fls_if))   //擦写完成后退出
// 	{
// 		if(EPFLAG & B1110_0000)
// 		{
// 				UartTx_0(EPFLAG);
// 	// 			SOFTRST = 0x5C;   //如果出错，软复位
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
	EA = 1;					//1.	保存当前中断状态并关闭无关中断
	AIE5 = 1;
	FLSIE = err_ie | fls_ie;
	
	
	ERCSR = 0x00;		//2.	清除ERCSR寄存器的Erase Request 位
	PRCSR = Pro_Req;		//3.	设置PRSCR寄存器的Program Request位
	FLSKEY = 0xA5;		//4.	向Flash Key地址写入8'hA5
	FLSKEY = 0xF1;		//5.	向Flash Key地址写入8'hF1
	*DB_FLASH = fl_data;  //6.	向Flash指定的地址写入编程数据，写操作之前需先擦除数据
	//*NVR_FLASH = 0x81;
	//UartTx_0(EPFLAG);
	
	_nop_();_nop_();_nop_();_nop_();_nop_();
	
	//while(!flash_op_end){}
		
// 	if(EPFLAG & B1110_0000)
// 		UartTx_0(EPFLAG);
// 		//SOFTRST = 0x5C;   //如果出错，软复位
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
	EA = 1;					//1.	保存当前中断状态并关闭无关中断
	AIE5 = 1;
	FLSIE = err_ie | fls_ie;
	
	
	ERCSR = 0x00;		//2.	清除ERCSR寄存器的Erase Request 位
	PRCSR = Pro_Req;		//3.	设置PRSCR寄存器的Program Request位
	
	
	for(i=0;i<512;i++)	//4. 进行数据缓冲,向RAM最高512字节填充待编程数据
	{
		*DB_RAM = *point_data;
		point_data++;
		DB_RAM++;
	}		
	
	
	FLSKEY = 0xA5;		//5.	向Flash Key地址写入8'hA5
	FLSKEY = 0xC4;		//6.	向Flash Key地址写入8'hC4
	
	*DB_FLASH = 0;  //7.	向Flash指定的地址写入任何数据，启动编程
	
	//UartTx_0(EPFLAG);
	
	_nop_();_nop_();_nop_();_nop_();_nop_();
	
	while(!flash_op_end){}
		
// 	if(EPFLAG & B1110_0000)
// 		UartTx_0(EPFLAG);
// 		//SOFTRST = 0x5C;   //如果出错，软复位
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
		write_FLASH(i+fl_addr,fl_data); //fl_addr开始的扇区写入fl_data
	}
	
	for(i=0;i<0x200;i++)
	{
		temp_data = read_FLASH(i+fl_addr);  //fl_addr开始的扇区读出判断是否为写入的数据fl_data
		if(temp_data!=fl_data) return i;  //若校验错，返回当前的偏移地址
	}	
	return 0x55aa;
}
	