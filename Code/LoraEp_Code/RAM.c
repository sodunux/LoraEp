#include "init.h"

uchar xram_wr_check(void)
{
	INT16U i;
	INT16U n;
	INT8U m,k;
	INT8U bakeup_data,TEMPD;
	INT8U *p = (INT8U *)(RAM_ADDR);
	
	
	TEMPD = 0xAA;					//AA是正常的
// UartTx_0(0x88);
//地址累加测试	
	i=0;
	
	for(m=0;m<=0x0F;m++)
	{

		k=m;

		for(n=0;n<=0xff;n++)
		{
			bakeup_data=*p;
			*p=k;
// 		UartTx_0(EBYTE[i]);
// 		UartTx_0(k);			
			
			_nop_();
			_nop_();
			_nop_();
			_nop_();
			_nop_();
			_nop_();
			_nop_();
			_nop_();
			_nop_();
			_nop_();
			_nop_();
			_nop_();
			_nop_();
			_nop_();
			_nop_();
			_nop_();
			_nop_();
			_nop_();
			_nop_();
			_nop_();
			

			if(*p!=k) 
			{
				return 0x55;
			}
			*p=bakeup_data;
			k++;
			i++;
		}
	}
	
	
	TEMPD = 0xAA;					//AA是正常的
	
	for(i=0;i<0x1000;i++)
	{
		WDT_CLR();
		p = (INT8U *)(RAM_ADDR + i);
		bakeup_data=*p;
		*p = 0xff;
		_nop_();
		_nop_();
		_nop_();
		_nop_();
		_nop_();
		_nop_();
		_nop_();
		_nop_();
		_nop_();
		_nop_();
		
		if(*p!=0xff) TEMPD=0x55;
		
		*p=0x00;
		_nop_();
		_nop_();
		_nop_();
		_nop_();
		_nop_();
		_nop_();
		_nop_();
		_nop_();
		_nop_();
		_nop_();
		
		if(*p!=0x00) TEMPD=0x55;
		
		*p=0x55;
		_nop_();
		_nop_();
		_nop_();
		_nop_();
		_nop_();
		_nop_();
		_nop_();
		_nop_();
		_nop_();
		_nop_();
		
		if(*p!=0x55) TEMPD=0x55;
		
		*p=0xaa;
		_nop_();
		_nop_();
		_nop_();
		_nop_();
		_nop_();
		_nop_();
		_nop_();
		_nop_();
		_nop_();
		_nop_();

		if(*p!=0xaa) TEMPD=0x55;
		
		
		*p=bakeup_data;
// 		UartTx_0(TEMPD);
		if(TEMPD==0x55) break;		//测试错误  停止测试
		delay_us(1);
	}

	return TEMPD;	

}


// void Write_RAM(INT16U ram_offset_addr,INT8U ram_data,INT8U length)
// {

// 	INT8U *p = (INT8U *)(RAM_ADDR+ram_offset_addr);
// 	INT8U i;
// 	
// 	for(i=0;i<length;i++)
// 	{
// 		*p = ram_data;
// 		p++;
// 	}

// }


// INT8U Read_RAM(INT16U ram_offset_addr)
// {
// 	INT8U *p = (INT8U *)(RAM_ADDR+ram_offset_addr);
// 	INT8U ram_data;
// 	
// 	ram_data = *p;

// 	return ram_data;

// }