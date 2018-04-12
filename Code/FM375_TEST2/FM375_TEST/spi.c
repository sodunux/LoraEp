#include "init.h"

/************************************************************************/
/* ��������INIT_SPI_PAD													*/
/* ����˵����SPI�ܽų�ʼ���ӳ��� 							*/
/* ��ڲ�����N/A													*/
/* ���ڲ�����N/A													*/
/************************************************************************/
void INIT_SPI_PAD(void)
{
// 	PDFCR1 = 0;
// 	PDFCR2 = 0;
// 	PDFCR2 |= BIT4 | BIT5 | BIT6 | BIT7;//{PDFCR2[4]:PDFCR1[4]}=2'b10
// 	AFSELD &= (~BIT4)&(~BIT5)&(~BIT6)&(~BIT7);//0��MOSI��MISO,SCK,SSN  
// 	
// 	PDPPEN = 0;
// 	PDPPEN |= BIT0;//ʹ��PD0�����칦��
// 	PDFCR1 |= BIT0;//{PDFCR2[0]:PDFCR1[0]}=2'b01,ʹ��PD0���,SPI_CS
// 	CS_SET();//	�ر�SPI����
// 	
// // 	PDPUEN |= BIT7;
	
	
	PAFCR1 &= 0XF0;
	PAFCR2 |= BIT0 | BIT1 | BIT2 | BIT3;//{PAFCR2[0]:PAFCR1[0]}=2'b10
	AFSELA &= 0XF0; //0��MOSI��MISO,SCK,SSN  
	
//	PAPUEN |= BIT2;
	
	PAPPEN |= BIT4;//ʹ��PA4�����칦��    
	PAFCR2 &= ~BIT4;
	PAFCR1 |= BIT4;//{PDFCR2[0]:PDFCR1[0]}=2'b01,ʹ��PA4���,SPI_CS
	CS_SET();//	�ر�SPI����
	
// 	PDPUEN |= BIT7; 	
	
	
	
}


/********************************************************
*	��������spi_init                               
*	�������ܣ�spi�Ĵ�����ʼ�� 
*   ��ڲ�������
*   ���ڲ�������             
********************************************************/
void spi_init(void)
{
// 	SPCR1 = 0;
// 	SPCR2 = 0;
// 	SPCR3 = 0;
// 	SPIIE = 0;
//  	SPCR1 |= mstr;  //MASTER 
//  	SPCR1 |= cpha;  //SPIģ���ڴ���ʱ�ӵĵڶ��������ز�������
//  	SPCR1 |= cpol;  //����ͬ��ʱ�ӵĿ���״̬Ϊ�ߵ�ƽ
// 	SPCR1 &= ~ssn_mode;
// 	SPCR1 &= ~lsb_first;//MSB FIRST
// 	SPCR2 |= spi_en | no_wait;	// ssn_mcu_en | //Ӳ������ssn    | sample_p
// // 	SPCR2 &= ~txonly_auo_clr;
// // 	SPCR3 |= ssn_mcu;  //SSN�����д��1
// //  	SPCR3 |= tx_only;
// 	
// // 	SPCR3 |= s_filter;
	
	
	SPCR1 = 0;
	SPCR2 = 0;
	SPCR3 = 0;
	SPCR4 = 0x0f;
	SPIIE = 0;
	SPCR1 |= BIT7;  //100�� fPCLK/32 
	
	SPCR1 |= ssn_mode;   //1'b 1�� ÿ��8Bit�ᱻ���ߣ����ߵ�ʱ��Ϊ����1��SCK Cycle�����WAIT_CNT��Ϊ0����ô���ߵ�ʱ��Ϊ1+WAIT_CNT��SCK Cycle
 	SPCR1 &= ~lsb_first;//MSB FIRST
 	SPCR1 |= mstr;  //MASTER 
 	SPCR1 &= ~cpha;  //SPIģ���ڴ���ʱ�ӵĵ�һ�������ز�������
 	SPCR1 &= ~cpol;  //����ͬ��ʱ�ӵĿ���״̬Ϊ�͵�ƽ
//	SPCR1 &= ~ssn_mode;  //1'b 0�� ��TxbufΪ�ǿգ����Ѿ��������8Bit��SSN���ֵ�
	


	SPCR2 |= spi_en | no_wait;	//    | ssn_mcu_en
	
// 	SPCR2 |= sample_p;
	
// 	SPCR2 &= ~txonly_auo_clr;


//  	SPCR3 |= ssn_mcu;  //SSN�����д��1
//  	SPCR3 |= tx_only;
 	SPCR3 |= s_filter;
	
	
	
}

/*******************************************************
*   ��������wait_no_trans_data                                
*   �������ܣ����жϱ�־���
*   ��ڲ�������
*   ���ڲ�������                          
*******************************************************/
// void wait_no_trans_data(void)
// {
// 	uchar tmp,tmp1;
// 	while(SPSR & busy){}  //��ѯ��busyΪ0����ʾTXBUFΪ�գ���SPIû���ڴ�������
// 	//while(!(SPSR & txbuf_empty)){}  //��ѯ��txbuf_emptyΪ1��TXBUF�������ݣ������ٴ�д��
// }
/*******************************************************
*   ��������wait_if_clear                                
*   �������ܣ����жϱ�־���
*   ��ڲ�������
*   ���ڲ�������                          
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
*	��������SPI_wr_byte                               
*	�������ܣ�spi��������
*   ��ڲ�������
*   ���ڲ�������             
********************************************************/
INT8U SPI_RW_byte( INT8U dat)
{
	INT8U rx_data;
	delay_us(1);
	while(!(SPSR & txbuf_empty)){}//�ȴ����ͻ������գ���ѯ��txbuf_empty==1ʱ����TXBUF��������ʱ��д���µ�����
	TXBUF = dat;
	while(!(SPSR & rxbuf_full)){} //�ȴ����ջ���������1: RXBUF�������ݣ�������һ��BYTE
	rx_data = RXBUF;
	delay_us(1);

	return rx_data;
}

/********************************************************
*	��������SPI_wr_byte_int                               
*	�������ܣ�spi��������
*   ��ڲ�������
*   ���ڲ�������             
********************************************************/
INT8U SPI_RW_byte_int( INT8U dat)
{
	INT8U rx_data;
	delay_us(1);
	clr_if = 0;
	while(!(SPSR & txbuf_empty)){}//�ȴ����ͻ������գ���ѯ��txbuf_empty==1ʱ����TXBUF��������ʱ��д���µ�����
	TXBUF = dat;  
	SPIIE |= tx_e_ie;
	while(!clr_if){}	//�ȴ����ͻ�������,���ж�
	clr_if = 0;
	SPIIE |= rx_ne_ie;  //ʹ�ܽ��ջ��������ж�
	while(!(SPSR & rxbuf_full)){} //�ȴ����ջ���������1: RXBUF�������ݣ�������һ��BYTE
	rx_data = RXBUF;	
	while(!clr_if){}	//�ȴ����ջ�������,�Ѷ�������
	clr_if = 0;
	delay_us(1);
	return rx_data;
}

/************************************************************************
	��������:SPI_write_enable_int
	��������:ʹ���ж�ʱд����ʹ��
	��ڲ���:��
	���ڲ���:��
	��    ע:��
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
	��������:SPI_write_enable
	��������:д����ʹ��
	��ڲ���:��
	���ڲ���:��
	��    ע:��
************************************************************************/
void SPI_write_enable(void)
{
	CS_CLR();

	SPI_RW_byte(0x06);//wr enable 

	CS_SET();
}
/************************************************************************
	��������:SPI_write_disable
	��������:д���ݽ�ֹ
	��ڲ���:��
	���ڲ���:��
	��    ע:��
************************************************************************/
void SPI_write_disable(void)
{
	CS_CLR();

	SPI_RW_byte(0x04);//wr disable

	CS_SET();
}

/************************************************************************
	��������:SPI_write_disable_int
	��������:ʹ���ж�ʱд���ݽ�ֹ
	��ڲ���:��
	���ڲ���:��
	��    ע:��
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
	��������:spi_chip_erase
	��������:spi flash chip erase
	��ڲ���:��
	���ڲ���:��
	��    ע:��
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
*	��������write_spi_nbyte                              
*	�������ܣ���ʹ���ж�ʱ��spiд�볤��Ϊwr_length��byte
*   ��ڲ�������ַ�����ݡ�д�볤��
*   ���ڲ�������             
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
*	��������write_spi_nbyte_int                               
*	�������ܣ�ʹ���ж�ʱ��spiд�볤��Ϊwr_length��byte
*   ��ڲ�������ַ�����ݡ�д�볤��
*   ���ڲ�������             
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
 *	��������read_spi_nbyte                              
 *	�������ܣ���ʹ���ж�ʱ��spi��������Ϊwr_length��byte
 *   ��ڲ�������ַ�����ݡ�д�볤��
 *   ���ڲ�������             
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
 *	��������read_spi_nbyte_int                               
 *	�������ܣ�ʹ���ж�ʱ��spi��������Ϊwr_length��byte
 *   ��ڲ�������ַ�����ݡ�д�볤��
 *   ���ڲ�������             
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
	PERICLK_CTRL0 |= dma_clken;   //DMAʱ��ʹ��
	for(i=0; i<21; i++)
	{
		XBYTE[DMA_ADDR+i] = 0; //deinit dma reg
	}
	
	GCTRL |= dma_en;  //DMAȫ��ʹ��
	GCTRL &= ~m2m_en; //FLASHͨ����ֹ

	GCTRL |= prior_ch0; //ͨ��0����
	CH0_CTRL |= ch_inc;//Դ�˵�ַ�ݼ�
	CH0_CTRL |= spi_tx;//CH0:spi����

	CH1_CTRL |= ch_inc;//Դ�˵�ַ�ݼ�
	CH1_CTRL |= spi_rx;//CH1��spi����

}

//wr_addr:Ҫд����������ڴ洢����ַ,12λ     
//rd_addr:���������ݴ�ŵĴ洢����ַ,12λ 
//ndtr:���ݴ�����     
void SPI_DMA_RW_Byte(INT16U wr_addr,INT16U rd_addr,INT16U ndtr)
{
// 	INT8U *p = (INT8U *)rd_addr;
	CH0_LEN = ndtr-1;//�������ݳ���
	CH0_ADDRH = (INT8U)(wr_addr>>8);
	CH0_ADDRL = (INT8U)(wr_addr&0xff);//mem��ַ����

	CH1_LEN = ndtr-1;//�������ݳ���
	CH1_ADDRH = (INT8U)(rd_addr>>8);
	CH1_ADDRL = (INT8U)(rd_addr&0xff);//mem��ַ����	
//  	UartTx_0(ndtr);
	CH0_CTRL |= ch_en;//ʹ��ͨ��0
	CH1_CTRL |= ch_en;//ʹ��ͨ��1	
	
	while(!((CH0STA & ch_end)&(CH1STA & ch_end)))
	{
		WDT_CLR();
// 		*p++ = 0x55;
	}   //ͨ��������Ϻ�	
	CH0STA &= ~ch_end;
	CH1STA &= ~ch_end;
		
	CH0_CTRL &= ~ch_en;
	CH1_CTRL &= ~ch_en;	


	
// 	while(--ndtr)
// 	{
// 		CH1_LEN = 1;//�������ݳ���
// 		CH1_ADDRH = (INT8U)(rd_addr>>8);
// 		CH1_ADDRL = (INT8U)(rd_addr&0xff);//mem��ַ����	
// 		
// 		while(!(SPSR & txbuf_empty)){}//�ȴ����ͻ������գ���ѯ��txbuf_empty==1ʱ����TXBUF��������ʱ��д���µ�����
// 		TXBUF = 0xff;  //�����������
// 		CH1_CTRL |= ch_en;//ʹ��ͨ��1������SPI����
// 		while(!(CH1STA & ch_end)) {WDT_CLR();}   //ͨ��������Ϻ�	
// 		CH1STA &= ~ch_end;
// 		CH1_CTRL &= ~ch_en;	
// 	}

	
}

//wr_addr:Ҫд����������ڴ洢����ַ,12λ     
//ndtr:���ݴ�����     
void SPI_DMA_Send_Byte(INT16U wr_addr,INT16U ndtr)
{
// 	INT8U *p = (INT8U *)wr_addr;
	CH0_LEN = ndtr-1;//�������ݳ���
	CH0_ADDRH = (INT8U)(wr_addr>>8);
	CH0_ADDRL = (INT8U)(wr_addr&0xff);//mem��ַ����
	
	CH0_CTRL |= ch_en;//ʹ��ͨ��0
	while(!(CH0STA & ch_end)) 
	{
		WDT_CLR();
// 		*p++ = 0x55;
	}   //ͨ��������Ϻ�	
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
// 	while(!(SPSR & txbuf_empty)){}//�ȴ����ͻ�������
	CS_SET();
	delay_us(1);

	CS_CLR();
	SPI_DMA_RW_Byte((INT16U)p,0xa00,4+wr_length);
// 	SPI_DMA_Send_Byte((INT16U)&send_buff[1],4);

//  	SPI_DMA_Send_Byte(mtd,wr_length);
// 	while(!(SPSR & txbuf_empty)){}//�ȴ����ͻ�������
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
//	SPI_DMA_RW_Byte(0xa00,mrd,wr_length);  //����,mtd��ַ���������
// 	while(!(SPSR & rxbuf_full)){} //�ȴ����ջ�������
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

	while(!(SPSR & txbuf_empty)){}//�ȴ����ͻ������գ���ѯ��txbuf_empty==1ʱ����TXBUF��������ʱ��д���µ�����
	TXBUF = 0xaa; 
	TXBUF = 0X55;//δ�����꣬������дһ��byte������TXBUF_WCOL
	if(flag & txf_wcol)
		return 1;
	else return 0;
	
	
}