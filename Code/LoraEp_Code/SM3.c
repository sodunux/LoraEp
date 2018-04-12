#include "init.h"

// void set_total_len_keydata(unsigned char *pxram)
// {
// 	unsigned char tempbyte;
// 	tempbyte = total_len.lenlow.u8data[0]>>5;	
// 	
// 	total_len.lenlow.u32data = total_len.lenlow.u32data<<3;  //���ݵĳ���*8Ϊbit��
// 	total_len.lenhigh.u32data = total_len.lenhigh.u32data<<3;
// 	total_len.lenhigh.u8data[3]|=tempbyte;

// 	for(tempbyte =0;tempbyte<4;tempbyte++)
// 	{
// 		*pxram=total_len.lenhigh.u8data[tempbyte];
// 		pxram++;
// 	}
// 	
// 	for(tempbyte =0;tempbyte<4;tempbyte++){     //pxramָ��ĵ�ַ���ó���l��16�������ݣ�����key[]����3���ֽڣ���24��bit����˴���������24
// 		*pxram=total_len.lenlow.u8data[tempbyte];
// 		pxram++;
// 	}
// 	
// 	return;
// }

void sm3_start(void)
{
	sm3_done = 0;
	AIE1 = 1; //
	CTRLSTATUSREG |= hash_done_ie; //ʹ���ж�
	CTRLSTATUSREG |= start;
// 	while(!(CTRLSTATUSREG & hash_done))    //SM3 END
// 	{WDT_CLR();}
	while(!sm3_done)    //SM3 END
	{WDT_CLR();}
	sm3_done = 0;
	AIE1 = 0; //
// 	CTRLSTATUSREG &= ~hash_done;     //clear if	
}

void sm3_calc(INT8U round,INT16U offset_addr)
{
  INT8U i;

	INT8U *p = (INT8U *)(RAM_ADDR);
	INT32U *q = (INT32U *)(RAM_ADDR);	
	

	q = (INT32U *)(RAM_ADDR+offset_addr+0x60);
	*q = 0x7380166f;q++;     //����ʼ����д��RAM(ram_result)��Ե�ַ7��h60~7��h7f
	*q = 0x4914b2b9;q++;
	*q = 0x172442d7;q++;
	*q = 0xda8a0600;q++;
	*q = 0xa96f30bc;q++;
	*q = 0x163138aa;q++;
	*q = 0xe38dee4d;q++;
	*q = 0xb0fb0e4e;q++;	
			
	switch(round)
	{
		case	1:
			p = (INT8U *)(RAM_ADDR+offset_addr);
			for(i=0;i<64;i++)       //����Ϣд��RAM(ram_message)��Ե�ַ7��h00~7��h3f
			{
				*p = TX_Buf[3+i];
				p++;
			}			
		
			sm3_start();
			
			TX_Buf[4] = 32;
			p = (INT8U *)(RAM_ADDR+offset_addr+0x60);    //READ RESULT
			for(i=0;i<32;i++)
			{
				TX_Buf[7+i] = *p;
				p++;
			}
			WDT_CLR();
							
		break;
			
		case	2:
			p = (INT8U *)(RAM_ADDR+offset_addr);
			for(i=0;i<64;i++)       //����Ϣд��RAM(ram_message)��Ե�ַ7��h00~7��h3f
			{
				*p = TX_Buf[3+i];
				p++;
			}		

			sm3_start();
			
			p = (INT8U *)(RAM_ADDR+offset_addr);
			for(i=64;i<128;i++)       //���ڶ�����Ϣд��RAM(ram_message)��Ե�ַ7��h00~7��h3f
			{
				*p = TX_Buf[3+i];
				p++;
			}			
			WDT_CLR();
			sm3_start();
			
			TX_Buf[4] = 32;
			p = (INT8U *)(RAM_ADDR+offset_addr+0x60);    //READ RESULT
			for(i=0;i<32;i++)
			{
				TX_Buf[7+i] = *p;
				p++;
			}				
			WDT_CLR();			
		break;
		
		case	3:
			p = (INT8U *)(RAM_ADDR+offset_addr);
			for(i=0;i<64;i++)       //����Ϣд��RAM(ram_message)��Ե�ַ7��h00~7��h3f
			{
				*p = TX_Buf[3+i];
				p++;
			}		

			sm3_start();
			
			p = (INT8U *)(RAM_ADDR+offset_addr);
			for(i=64;i<128;i++)       //���ڶ�����Ϣд��RAM(ram_message)��Ե�ַ7��h00~7��h3f
			{
				*p = TX_Buf[3+i];
				p++;
			}			
			WDT_CLR();
			sm3_start();

			p = (INT8U *)(RAM_ADDR+offset_addr);
			for(i=128;i<192;i++)       //���ڶ�����Ϣд��RAM(ram_message)��Ե�ַ7��h00~7��h3f
			{
				*p = TX_Buf[3+i];
				p++;
			}			
			WDT_CLR();
			sm3_start();
			
			TX_Buf[4] = 32;
			p = (INT8U *)(RAM_ADDR+offset_addr+0x60);    //READ RESULT
			for(i=0;i<32;i++)
			{
				TX_Buf[7+i] = *p;
				p++;
			}				
			WDT_CLR();					
		break;
		
		default:
		break;
	}	
}


uchar sm3_self_test(INT16U offset_addr)
{
  INT8U i,sm3_result[32];
	
	INT8U *p = (INT8U *)(RAM_ADDR);
	INT32U *q = (INT32U *)(RAM_ADDR);	
	
	sm3_result[0] = 0x66;
	sm3_result[1] = 0xc7;
	sm3_result[2] = 0xf0;
	sm3_result[3] = 0xf4;
	sm3_result[4] = 0x62;
	sm3_result[5] = 0xee;
	sm3_result[6] = 0xed;
	sm3_result[7] = 0xd9;
	sm3_result[8] = 0xd1;
	sm3_result[9] = 0xf2;
	sm3_result[10] = 0xd4;
	sm3_result[11] = 0x6b;
	sm3_result[12] = 0xdc;
	sm3_result[13] = 0x10;
	sm3_result[14] = 0xe4;
	sm3_result[15] = 0xe2;
	sm3_result[16] = 0x41;
	sm3_result[17] = 0x67;
	sm3_result[18] = 0xc4;
	sm3_result[19] = 0x87;
	sm3_result[20] = 0x5c;
	sm3_result[21] = 0xf2;
	sm3_result[22] = 0xf7;
	sm3_result[23] = 0xa2;
	sm3_result[24] = 0x29;
	sm3_result[25] = 0x7d;
	sm3_result[26] = 0xa0;
	sm3_result[27] = 0x2b;
	sm3_result[28] = 0x8f;
	sm3_result[29] = 0x4b;
	sm3_result[30] = 0xa8;
	sm3_result[31] = 0xe0;
	
	
	
	q = (INT32U *)(RAM_ADDR+offset_addr+0x60);
	*q = 0x7380166f;q++;     //����ʼ����д��RAM(ram_result)��Ե�ַ7��h60~7��h7f
	*q = 0x4914b2b9;q++;
	*q = 0x172442d7;q++;
	*q = 0xda8a0600;q++;
	*q = 0xa96f30bc;q++;
	*q = 0x163138aa;q++;
	*q = 0xe38dee4d;q++;
	*q = 0xb0fb0e4e;q++;	

	p = (INT8U *)(RAM_ADDR+offset_addr+0x00);

	*p = 0x61;p++; 					//����Ϣд��RAM(ram_message)��Ե�ַ7��h00~7��h3f
	*p = 0x62;p++;
	*p = 0x63;p++;
	*p = 0x80;p++;
	for(i=4;i<63;i++)      
	{	
		*p = 0x00;p++;
	}			
	*p = 0x18;

IO1DATA = BIT4;	//���ڲ���SM3 64byte����ʱ��
	sm3_start();
IO1DATA = 0;
	
	p = (INT8U *)(RAM_ADDR+offset_addr+0x60);    //READ RESULT
	for(i=0;i<32;i++)
	{
		if(*p != sm3_result[i])
			return 0;
		p++;
		WDT_CLR();			
	}
	return 1;

}