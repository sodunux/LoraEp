#include "init.h"

/************************************************************************/
/* 函数名：INIT_SPI_PAD													*/
/* 功能说明：SPI管脚初始化子程序 							*/
/* 入口参数：N/A													*/
/* 出口参数：N/A													*/
/************************************************************************/
void INIT_SPI_PAD(void)
{
// 	PDFCR1 = 0;
// 	PDFCR2 = 0;
// 	PDFCR2 |= BIT4 | BIT5 | BIT6 | BIT7;//{PDFCR2[4]:PDFCR1[4]}=2'b10
// 	AFSELD &= (~BIT4)&(~BIT5)&(~BIT6)&(~BIT7);//0：MOSI，MISO,SCK,SSN  
// 	
// 	PDPPEN = 0;
// 	PDPPEN |= BIT0;//使能PD0的推挽功能
// 	PDFCR1 |= BIT0;//{PDFCR2[0]:PDFCR1[0]}=2'b01,使能PD0输出,SPI_CS
// 	CS_SET();//	关闭SPI传输
// 	
// // 	PDPUEN |= BIT7;
	
	
	PAFCR1 &= 0XF0;
	PAFCR2 |= BIT0 | BIT1 | BIT2 | BIT3;//{PAFCR2[0]:PAFCR1[0]}=2'b10
	AFSELA &= 0XF0; //0：MOSI，MISO,SCK,SSN  
	
//	PAPUEN |= BIT2;
	
	PAPPEN |= BIT4;//使能PA4的推挽功能    
	PAFCR2 &= ~BIT4;
	PAFCR1 |= BIT4;//{PDFCR2[0]:PDFCR1[0]}=2'b01,使能PA4输出,SPI_CS
	CS_SET();//	关闭SPI传输
	
// 	PDPUEN |= BIT7; 	
	
	
	
}


/********************************************************
*	函数名：spi_init                               
*	函数功能：spi寄存器初始化 
*   入口参数：无
*   出口参数：无             
********************************************************/
void spi_init(void)
{
// 	SPCR1 = 0;
// 	SPCR2 = 0;
// 	SPCR3 = 0;
// 	SPIIE = 0;
//  	SPCR1 |= mstr;  //MASTER 
//  	SPCR1 |= cpha;  //SPI模块在串行时钟的第二个跳变沿采样数据
//  	SPCR1 |= cpol;  //串行同步时钟的空闲状态为高电平
// 	SPCR1 &= ~ssn_mode;
// 	SPCR1 &= ~lsb_first;//MSB FIRST
// 	SPCR2 |= spi_en | no_wait;	// ssn_mcu_en | //硬件控制ssn    | sample_p
// // 	SPCR2 &= ~txonly_auo_clr;
// // 	SPCR3 |= ssn_mcu;  //SSN被软件写成1
// //  	SPCR3 |= tx_only;
// 	
// // 	SPCR3 |= s_filter;
	
	
	SPCR1 = 0;
	SPCR2 = 0;
	SPCR3 = 0;
	SPCR4 = 0x0f;
	SPIIE = 0;
	SPCR1 |= BIT7;  //100： fPCLK/32 
	
	SPCR1 |= ssn_mode;   //1'b 1： 每个8Bit会被拉高，拉高的时间为起码1个SCK Cycle，如果WAIT_CNT不为0，那么拉高的时间为1+WAIT_CNT个SCK Cycle
 	SPCR1 &= ~lsb_first;//MSB FIRST
 	SPCR1 |= mstr;  //MASTER 
 	SPCR1 &= ~cpha;  //SPI模块在串行时钟的第一个跳变沿采样数据
 	SPCR1 &= ~cpol;  //串行同步时钟的空闲状态为低电平
//	SPCR1 &= ~ssn_mode;  //1'b 0： 在Txbuf为非空，且已经发送完毕8Bit，SSN保持低
	


	SPCR2 |= spi_en | no_wait;	//    | ssn_mcu_en
	
// 	SPCR2 |= sample_p;
	
// 	SPCR2 &= ~txonly_auo_clr;


//  	SPCR3 |= ssn_mcu;  //SSN被软件写成1
//  	SPCR3 |= tx_only;
 	SPCR3 |= s_filter;
	
	
	
}

/*******************************************************
*   函数名：wait_no_trans_data                                
*   函数功能：等中断标志清除
*   入口参数：无
*   出口参数：无                          
*******************************************************/
// void wait_no_trans_data(void)
// {
// 	uchar tmp,tmp1;
// 	while(SPSR & busy){}  //查询到busy为0：表示TXBUF为空，且SPI没有在传输数据
// 	//while(!(SPSR & txbuf_empty)){}  //查询到txbuf_empty为1，TXBUF中无数据，可以再次写入
// }
/*******************************************************
*   函数名：wait_if_clear                                
*   函数功能：等中断标志清除
*   入口参数：无
*   出口参数：无                          
*******************************************************/
// void wait_if_clear()
// {
// 	while(!clr_if)
// 	{
// // 		WDT_CLR();
// 	}
// 	clr_if = 0;
// }

/********************************************************
*	函数名：SPI_wr_byte                               
*	函数功能：spi发送数据
*   入口参数：无
*   出口参数：无             
********************************************************/
INT8U SPI_RW_byte( INT8U dat)
{
	INT8U rx_data;
	delay_us(1);
	while(!(SPSR & txbuf_empty)){}//等待发送缓冲区空，查询到txbuf_empty==1时，即TXBUF中无数据时再写入新的数据
	TXBUF = dat;
	while(!(SPSR & rxbuf_full)){} //等待接收缓冲区满，1: RXBUF中有数据，接收完一个BYTE
	rx_data = RXBUF;
	delay_us(1);

	return rx_data;
}

/********************************************************
*	函数名：SPI_wr_byte_int                               
*	函数功能：spi发送数据
*   入口参数：无
*   出口参数：无             
********************************************************/
INT8U SPI_RW_byte_int( INT8U dat)
{
	INT8U rx_data;
	delay_us(1);
	clr_if = 0;
	while(!(SPSR & txbuf_empty)){}//等待发送缓冲区空，查询到txbuf_empty==1时，即TXBUF中无数据时再写入新的数据
	TXBUF = dat;  
	SPIIE |= tx_e_ie;
	while(!clr_if){}	//等待发送缓冲区空,进中断
	clr_if = 0;
	SPIIE |= rx_ne_ie;  //使能接收缓冲区满中断
	while(!(SPSR & rxbuf_full)){} //等待接收缓冲区满，1: RXBUF中有数据，接收完一个BYTE
	rx_data = RXBUF;	
	while(!clr_if){}	//等待接收缓冲区空,已读完数据
	clr_if = 0;
	delay_us(1);
	return rx_data;
}

/************************************************************************
	函数名称:SPI_write_enable_int
	函数功能:使能中断时写数据使能
	入口参数:无
	出口参数:无
	备    注:无
************************************************************************/
void SPI_write_enable_int(void)
{

	CS_CLR();
	delay_us(1);
	SPI_RW_byte_int(0x06);
	delay_us(1);
	CS_SET();
}

/************************************************************************
	函数名称:SPI_write_enable
	函数功能:写数据使能
	入口参数:无
	出口参数:无
	备    注:无
************************************************************************/
void SPI_write_enable(void)
{
	CS_CLR();

	SPI_RW_byte(0x06);//wr enable 

	CS_SET();
}
/************************************************************************
	函数名称:SPI_write_disable
	函数功能:写数据禁止
	入口参数:无
	出口参数:无
	备    注:无
************************************************************************/
void SPI_write_disable(void)
{
	CS_CLR();

	SPI_RW_byte(0x04);//wr disable

	CS_SET();
}

/************************************************************************
	函数名称:SPI_write_disable_int
	函数功能:使能中断时写数据禁止
	入口参数:无
	出口参数:无
	备    注:无
************************************************************************/
void SPI_write_disable_int(void)
{
	CS_CLR();
	delay_us(1);
	SPI_RW_byte_int(0x04);//wr disable
	delay_us(1);
	CS_SET();
}


/************************************************************************
	函数名称:spi_chip_erase
	函数功能:spi flash chip erase
	入口参数:无
	出口参数:无
	备    注:无
************************************************************************/
void spi_chip_erase(void)
{
	SPI_write_enable();
	
	CS_CLR();

	SPI_RW_byte(0x60);//chip erase
	
	CS_SET();

}

void SPI_write_enable_dma(void)
{
	INT8U send_buff[1];
	send_buff[0] = 0x06;//wr enable
	
	CS_CLR();
	SPI_DMA_Send_Byte((INT16U)&send_buff[0],1);

	CS_SET();
}

void spi_chip_erase_dma(void)
{
	INT8U send_buff[1];
	send_buff[0] = 0x60;//chip erase
	
	SPI_write_enable_dma(); //write enable
	
	delay_us(1);
	
	CS_CLR();
	SPI_DMA_Send_Byte((INT16U)&send_buff[0],1);

	CS_SET();

}

/********************************************************
*	函数名：write_spi_nbyte                              
*	函数功能：不使能中断时，spi写入长度为wr_length的byte
*   入口参数：地址、数据、写入长度
*   出口参数：无             
********************************************************/
void write_spi_nbyte(uchar addr2,uchar addr1,uchar addr0,uchar *mtd,INT16U wr_length)
{
	uchar j;
	INT32U data wait = 0;
// SPCR3 |= tx_only;	//	
	CS_CLR();

	SPI_RW_byte(0x06);//wr enable

	CS_SET();
	delay_us(1);

	CS_CLR();

	SPI_RW_byte(0x02);//page program   
	SPI_RW_byte(addr2);//addr byte2 
	SPI_RW_byte(addr1);//addr byte1
	SPI_RW_byte(addr0);//addr byte0

	for(j=0;j<wr_length;j++)
	{
		SPI_RW_byte(*mtd);//data 
		mtd++;
	}
	CS_SET();

 	delay(10);//twc=10ms
}

/********************************************************
*	函数名：write_spi_nbyte_int                               
*	函数功能：使能中断时，spi写入长度为wr_length的byte
*   入口参数：地址、数据、写入长度
*   出口参数：无             
********************************************************/
// void write_spi_nbyte_int(uchar addr2,uchar addr1,uchar addr0,uchar *mtd,INT16U wr_length)
// {
// 	uchar j;
// 	INT32U data wait = 0;
// 	
// 	CS_CLR();
// 	SPI_RW_byte_int(0x06);//wr enable
// 	CS_SET();
// 	delay_us(1);

// 	CS_CLR();

// 	SPI_RW_byte_int(0x02);//page program   
// 	SPI_RW_byte_int(addr2);//addr byte2
// 	SPI_RW_byte_int(addr1);//addr byte1
// 	SPI_RW_byte_int(addr0);//addr byte0

// 	for(j=0;j<wr_length;j++)
// 	{
// 		SPI_RW_byte_int(*mtd);//data 
// 		mtd++; 
// 	}
// 	delay_us(1);
// 	CS_SET();

//  	delay(10);//twc=10ms

// }

 /********************************************************
 *	函数名：read_spi_nbyte                              
 *	函数功能：不使能中断时，spi读出长度为wr_length的byte
 *   入口参数：地址、数据、写入长度
 *   出口参数：无             
 ********************************************************/
 void read_spi_nbyte(uchar addr2,uchar addr1,uchar addr0,uchar *mrd,INT16U wr_length)
 {
 	uchar j;

 	CS_CLR();
	 
	if(SPCR1 & lsb_first) 
	{
		SPI_RW_byte(0xc0);//read data
		SPI_RW_byte(addr2);//addr byte2
		SPI_RW_byte(addr1);//addr byte1
		SPI_RW_byte(addr0);//addr byte0
		
		for(j=0;j<wr_length;j++)
		{
			*mrd = SPI_RW_byte(0xff);
			mrd++;
		}	
		
	}
	else
	{
		SPI_RW_byte(0x03);//read data
		SPI_RW_byte(addr2);//addr byte2
		SPI_RW_byte(addr1);//addr byte1
		SPI_RW_byte(addr0);//addr byte0
		
		for(j=0;j<wr_length;j++)
		{
			*mrd = SPI_RW_byte(0xff);
			mrd++;
		}	
	}

 	
 	delay_us(1);
 	CS_SET();
 }

 /********************************************************
 *	函数名：read_spi_nbyte_int                               
 *	函数功能：使能中断时，spi读出长度为wr_length的byte
 *   入口参数：地址、数据、写入长度
 *   出口参数：无             
 ********************************************************/
//  void read_spi_nbyte_int(uchar addr2,uchar addr1,uchar addr0,uchar *mrd,INT16U wr_length)
//  {
//  	uchar j;

//  	CS_CLR();

//  	SPI_RW_byte_int(0x03);//read data
// 	SPI_RW_byte_int(addr2);//addr byte2
// 	SPI_RW_byte_int(addr1);//addr byte1
// 	SPI_RW_byte_int(addr0);//addr byte0
//  	

//  	for(j=0;j<wr_length;j++)
//  	{
//  		*mrd = SPI_RW_byte_int(0xff);
// 		mrd++;
//  	}
//  	
//  	delay_us(1);
//  	CS_SET();
// }
 
 

void SPI_DMA_Config(void)
{
	INT8U i;
	PERICLK_CTRL0 |= dma_clken;   //DMA时钟使能
	for(i=0; i<21; i++)
	{
		XBYTE[DMA_ADDR+i] = 0; //deinit dma reg
	}
	
	GCTRL |= dma_en;  //DMA全局使能
	GCTRL &= ~m2m_en; //FLASH通道禁止

	GCTRL |= prior_ch0; //通道0优先
	CH0_CTRL |= ch_inc;//源端地址递加
	CH0_CTRL |= spi_tx;//CH0:spi发送

	CH1_CTRL |= ch_inc;//源端地址递加
	CH1_CTRL |= spi_rx;//CH1：spi接收

}

//wr_addr:要写入的数据所在存储器地址,12位     
//rd_addr:读出的数据存放的存储器地址,12位 
//ndtr:数据传输量     
void SPI_DMA_RW_Byte(INT16U wr_addr,INT16U rd_addr,INT16U ndtr)
{
// 	INT8U *p = (INT8U *)rd_addr;
	CH0_LEN = ndtr-1;//传输数据长度
	CH0_ADDRH = (INT8U)(wr_addr>>8);
	CH0_ADDRL = (INT8U)(wr_addr&0xff);//mem地址设置

	CH1_LEN = ndtr-1;//传输数据长度
	CH1_ADDRH = (INT8U)(rd_addr>>8);
	CH1_ADDRL = (INT8U)(rd_addr&0xff);//mem地址设置	
//  	UartTx_0(ndtr);
	CH0_CTRL |= ch_en;//使能通道0
	CH1_CTRL |= ch_en;//使能通道1	
	
	while(!((CH0STA & ch_end)&(CH1STA & ch_end)))
	{
		WDT_CLR();
// 		*p++ = 0x55;
	}   //通道传输完毕后	
	CH0STA &= ~ch_end;
	CH1STA &= ~ch_end;
		
	CH0_CTRL &= ~ch_en;
	CH1_CTRL &= ~ch_en;	


	
// 	while(--ndtr)
// 	{
// 		CH1_LEN = 1;//传输数据长度
// 		CH1_ADDRH = (INT8U)(rd_addr>>8);
// 		CH1_ADDRL = (INT8U)(rd_addr&0xff);//mem地址设置	
// 		
// 		while(!(SPSR & txbuf_empty)){}//等待发送缓冲区空，查询到txbuf_empty==1时，即TXBUF中无数据时再写入新的数据
// 		TXBUF = 0xff;  //发送虚假数据
// 		CH1_CTRL |= ch_en;//使能通道1，启动SPI接收
// 		while(!(CH1STA & ch_end)) {WDT_CLR();}   //通道传输完毕后	
// 		CH1STA &= ~ch_end;
// 		CH1_CTRL &= ~ch_en;	
// 	}

	
}

//wr_addr:要写入的数据所在存储器地址,12位     
//ndtr:数据传输量     
void SPI_DMA_Send_Byte(INT16U wr_addr,INT16U ndtr)
{
// 	INT8U *p = (INT8U *)wr_addr;
	CH0_LEN = ndtr-1;//传输数据长度
	CH0_ADDRH = (INT8U)(wr_addr>>8);
	CH0_ADDRL = (INT8U)(wr_addr&0xff);//mem地址设置
	
	CH0_CTRL |= ch_en;//使能通道0
	while(!(CH0STA & ch_end)) 
	{
		WDT_CLR();
// 		*p++ = 0x55;
	}   //通道传输完毕后	
	CH0STA &= ~ch_end;
	CH0_CTRL &= ~ch_en;

}

void write_spi_nbyte_dma(uchar addr2,uchar addr1,uchar addr0,INT16U mtd,INT16U wr_length)
{

	INT8U send_buff[1];
 	INT8U *p;

	send_buff[0] = 0x06;//wr enable
// 	send_buff[1] = 0x02;//page program  
// 	send_buff[2] = addr2;//addr byte2 
// 	send_buff[3] = addr1;//addr byte1
// 	send_buff[4] = addr0;//addr byte0	
	
	p = (INT16U *)(mtd - 4);
	*p = 0x02;
	p++;
	*p = addr2;
	p++;
	*p = addr1;
	p++;
	*p = addr0;
	p = (INT16U *)(mtd - 4);

	CS_CLR();
	SPI_DMA_RW_Byte((INT16U)&send_buff[0],0xa00,1);
// 	SPI_DMA_Send_Byte((INT16U)&send_buff[0],1);
// 	while(!(SPSR & txbuf_empty)){}//等待发送缓冲区空
	CS_SET();
	delay_us(1);

	CS_CLR();
	SPI_DMA_RW_Byte((INT16U)p,0xa00,4+wr_length);
// 	SPI_DMA_Send_Byte((INT16U)&send_buff[1],4);

//  	SPI_DMA_Send_Byte(mtd,wr_length);
// 	while(!(SPSR & txbuf_empty)){}//等待发送缓冲区空
	CS_SET();

 	delay(10);//twc=10ms
}

void read_spi_nbyte_dma(uchar addr2,uchar addr1,uchar addr0,INT16U mrd,INT16U wr_length)
{
//   INT8U *p = (INT8U *)mrd;
	INT8U send_buff[4];
	send_buff[0] = 0x03;//read data
	send_buff[1] = addr2;//addr byte2
	send_buff[2] = addr1;//addr byte1 
	send_buff[3] = addr0;//addr byte0
	
	
	CS_CLR();
	//SPI_DMA_Send_Byte((INT16U)&send_buff[0],4);
	SPI_DMA_RW_Byte((INT16U)&send_buff[0],mrd-4,4+wr_length);
//	SPI_DMA_RW_Byte(0xa00,mrd,wr_length);  //接收,mtd地址是虚假数据
// 	while(!(SPSR & rxbuf_full)){} //等待接收缓冲区满
//  	p = (INT8U *)mrd;
//    *p = RXBUF;
//  	UartTx_0(*p);


	CS_SET();
}

unsigned char spi_master_error(void)
{
	SPIIE |= error_ie;
	return 1;
}

unsigned char spi_txcol_error(void)
{
	SPIIE |= error_ie;
	
	CS_CLR();
	delay_us(1);

	while(!(SPSR & txbuf_empty)){}//等待发送缓冲区空，查询到txbuf_empty==1时，即TXBUF中无数据时再写入新的数据
	TXBUF = 0xaa; 
	TXBUF = 0X55;//未发送完，接着再写一个byte，产生TXBUF_WCOL
	if(flag & txf_wcol)
		return 1;
	else return 0;
	
	
}