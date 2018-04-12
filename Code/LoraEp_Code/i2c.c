#include "init.h"

void INIT_I2C_PAD(void)
{
	PEFCR1 = 0;
	PEFCR2 = 0;
	PEFCR2 |= BIT5 | BIT6;//{PEFCR2[5]:PEFCR1[5]}=2'b10
	AFSELE |= BIT5 | BIT6;//1��SCL,SDA
	
	PEPPEN = 0;
	PEPPEN |= BIT7;//ʹ��PE7�����칦��
	PEFCR1 |= BIT7;//{PEFCR2[7]:PEFCR1[7]}=2'b01,ʹ��PE0���,WP
	PEDATA &= ~BIT7;
}


/********************************************************
*	��������i2c_init                               
*	�������ܣ�i2c�Ĵ�����ʼ�� 
*   ��ڲ�������
*   ���ڲ�������             
********************************************************/
void i2c_init(void)
{	
//  	SSPIR |= i2cie;   //I2CIE = 1��ʹ���ж�
//  	SSPERR |= errie;
	
	
	SSPERR = 0x00;
//	SSPBUF = 0xa0;	
//	SSPCON |= BIT1;  
//	SSPCON |= BIT3;
//	SSPCON |= BIT4;  //��IDLE״̬�²���ierr��־
	
	SSPCON = 0;
	//SSPCON:|I2CEN|LLEN|HLEN|ACKEN|RCEN|PEN|RSEN|SEN|
	SSPCON |= i2cen;  //I2C_EN=1

	//��������������λ�����������
//	SSPBRG = FOSC/(4*band)-1;  
//	SSPBRG = 0x04;		//8mʱ�ӣ�400k����Ƭ����ʱ�̶�
// 	SSPBRG = 0x06;    //3mʱ�ӣ�100k
	//SSPSTAT:|WCOL|--|R/W|P|S|BF|ACKSAT|ACKDT|
	SSPSTAT = 0x00;
	//SSPIR:|--|--|--|--|--|--|I2CIE|I2CIF|
	SSPFSM = 0x00;

}

/*******************************************************
*   ��������i2c_start                                  
*   �������ܣ�startʱ�����  
*   ��ڲ�������
*   ���ڲ�������                          
*******************************************************/
void i2c_start(void)
{
	SSPCON |= sen;   //SEN=1;I2C����
	do{
	;
	}
	while((SSPIR & i2cif) != i2cif);  //SSPIF==0;
	SSPIR &= ~i2cif;   //SSPIF=0;
}

/*******************************************************
*   ��������i2c_stop                                  
*   �������ܣ�stopʱ�����
*   ��ڲ�������
*   ���ڲ�������                           
*******************************************************/
void i2c_stop(void)
{
	SSPCON |= pen;    //PEN=1;I2Cֹͣ
	do{
	;
	}
	while((SSPIR & i2cif) != i2cif);  //SSPIF==0;
	SSPIR &= ~i2cif;   //SSPIF=0;
}

/*******************************************************
*   ��������i2c_restart                                  
*   �������ܣ�restartʱ����� 
*   ��ڲ�������
*   ���ڲ�������                           
*******************************************************/
void i2c_restart(void)
{
	SSPCON |= rsen;   //RSEN=1;�ظ�����
	do{
	;
	}
	while((SSPIR & i2cif) != i2cif);  //SSPIF==0;
	SSPIR &= ~i2cif;   //SSPIF=0;
}

/*******************************************************
*   ��������i2c_int                                  
*   �������ܣ��жϲ�ѯ�ȴ������ 
*   ��ڲ�������
*   ���ڲ�������                          
*******************************************************/
void i2c_int(void)  
{
	do{
	;
	}
	while((SSPIR & i2cif) != i2cif);  //SSPIF==0;
	SSPIR &= ~i2cif;   //SSPIF=0;
}

/*******************************************************
*   ��������wait_if_clear                                
*   �������ܣ����жϱ�־���
*   ��ڲ�������
*   ���ڲ�������                          
*******************************************************/
// uchar wait_if_clear()
// {
// 	//===���Ӷ�ʱ����
// 	INIT_T0();							//timer0��������					
// 	Timer0_Test_Cnt = 1;					//5ms��ʱ��ʱ
// 	Timer0_Test_Statues = 0;
// 	TR0=1;	
// 	while(!clr_if)
// 	{
// 		WDT_CLR();
// 		if(Timer0_Test_Statues==1)			//��5ms���޷����
// 		{	
// 			TR0=0;
// 			TMOD &= 0xF0;
// 			return NO_I2C_IF;
// 			break;
// 		}
// 	}
// 	clr_if = 0;
// 	TR0=0;
// 	TMOD &= 0xF0;
// 	return YES_I2C_IF;
// }

/*******************************************************
*   ��������wait_if_clear                                
*   �������ܣ����жϱ�־���
*   ��ڲ�������
*   ���ڲ�������                          
*******************************************************/
void wait_i2cif_clear(void)
{
	while(!clr_if){WDT_CLR();}
	clr_if = 0;
	
}

/*****************************************************************************************************
*    ��������i2c_write    
*    �������ܣ�д����
*    ��ڲ����� ctrlwΪ�����ֽڣ�����Ѱַ�������д����������
*     eepromaddrΪ���ݵ�ַ����дһ��ҳ�����ݵ�ַ������ÿҳ���׵�ַ����׼ȷд�루����һҳ�������ݻᷭת���ǣ���
*     mtdΪ���������ݻ�������ַ
*     wlengthΪд�������ֽڳ���
*	  EepromTypeΪ���������ͣ�ö�ٱ������ֱ�Ϊ2401~24256
*    ���ڲ���������0��ʾ�������󣬷���1��ʾ�ɹ�д��
*****************************************************************************************************/
bit i2c_write1(INT8U ctrlw, INT16U eepromaddr, INT8U *mtd, INT16U wlength) 
{
	i2c_init();           //��ʼ��
	i2c_start();          //��������
	SSPBUF = ctrlw;       //ex:дEEPROM��0xA0 
	i2c_int();            //�ȴ����ͣ�������ɵھŸ�scl�½��ز����ж�
    
  if ((SSPSTAT & ackstat) == ackstat)        //if(ACKSTAT)
		ERROR();   //ackstat==1ʱ��Ӧ�𣬴���
  SSPBUF = eepromaddr>>8;              //ȡEEPROM���ݵ�ַ���ֽ�
	i2c_int(); //����
	if ((SSPSTAT & ackstat) == ackstat)        //if(ACKSTAT)
    ERROR();   //��Ӧ�𣬴���
	SSPBUF = (INT8U)eepromaddr;            //��FM24C08��10λ���ݵ�ַ��2λ�ڿ����ֽ��У���8λ����һ�ֽڼ���
	i2c_int(); //����
	
	while(wlength--)
	{
		if ((SSPSTAT & ackstat) == ackstat)        //if(ACKSTAT) 
			ERROR();       //��Ӧ�𣬴���
		else if((SSPSTAT & ackstat) != ackstat)   //if(!ACKSTAT)
			SSPBUF = *mtd++;     //*(mtd+wlength%PAGE)   //д������
		i2c_int();    //����
  }
	if ((SSPSTAT & ackstat) == ackstat)        //if(ACKSTAT) 
  		ERROR();  
	else if((SSPSTAT & ackstat) != ackstat)   //if(!ACKSTAT)
    	i2c_stop();   //����ֹͣλ������EEPROM�ڲ�д����
 
	do{               //Ӧ���ѯ���ȴ��ڲ�д����
	i2c_start();
	SSPBUF = 0xA0; 
	i2c_int();
	if((SSPSTAT & ackstat) != ackstat)  //if(!ACKSTAT)���л�Ӧ˵����д�꣬����
	    break;
	i2c_stop();
	}while((SSPSTAT & ackstat) == ackstat);  //while(ACKSTAT);�޻�Ӧ�������ѯ

	return I2C_SUCCESS;
}

// void I2C_DMA_Config(void)
// {
// 	INT8U i;
// 	PERICLK_CTRL0 |= dma_clken;   //DMAʱ��ʹ��
// 	for(i=0; i<21; i++)
// 	{
// 		XBYTE[DMA_ADDR+i] = 0; //deinit dma reg
// 	}
// 	
// 	GCTRL |= dma_en;  //DMAȫ��ʹ��
// 	GCTRL &= ~m2m_en; //FLASHͨ����ֹ

// 	GCTRL |= prior_ch0; //ͨ��0����
// 	CH0_CTRL |= ch_inc;//Դ�˵�ַ�ݼ�
// 	CH0_CTRL |= spi_tx;//CH0:I2C����

// 	CH1_CTRL |= ch_inc;//Դ�˵�ַ�ݼ�
// 	CH1_CTRL |= spi_rx;//CH1��I2C����

// }
/*****************************************************************************************************
*    ��������i2c_write    
*    �������ܣ�д����
*    ��ڲ����� ctrlwΪ�����ֽڣ�����Ѱַ�������д����������
*     eepromaddrΪ���ݵ�ַ����дһ��ҳ�����ݵ�ַ������ÿҳ���׵�ַ����׼ȷд�루����һҳ�������ݻᷭת���ǣ���
*     mtdΪ���������ݻ�������ַ
*     wlengthΪд�������ֽڳ���
*	  EepromTypeΪ���������ͣ�ö�ٱ������ֱ�Ϊ2401~24256
*    ���ڲ���������0��ʾ�������󣬷���1��ʾ�ɹ�д��
*****************************************************************************************************/
bit i2c_write_DMA(INT8U ctrlw, INT16U eepromaddr, INT8U near *mtd, INT16U wlength) 
{
	INT8U send_buff[2];
	send_buff[0] = eepromaddr>>8;     //ȡEEPROM���ݵ�ַ���ֽ�
	send_buff[1] = (INT8U)eepromaddr; //ȡEEPROM���ݵ�ַ���ֽ�
	
	i2c_init();           //��ʼ��
	i2c_start();          //��������	
	
	DMA_Start(CH0,(INT16U)&ctrlw,1,i2c_tx);  //���ͣ�дEEPROM��0xA0 
	
	i2c_int();            //�ȴ����ͣ�������ɵھŸ�scl�½��ز����ж�
  if ((SSPSTAT & ackstat) == ackstat)        //if(ACKSTAT)
		ERROR();   //ackstat==1ʱ��Ӧ�𣬴���	
	
	DMA_Start(CH0,(INT16U)&send_buff[0],1,i2c_tx);  
	
	i2c_int(); //����
  if ((SSPSTAT & ackstat) == ackstat)        //if(ACKSTAT)
		ERROR();   //ackstat==1ʱ��Ӧ�𣬴���	
	
	DMA_Start(CH0,(INT16U)&send_buff[1],1,i2c_tx);  
	
	i2c_int(); //����
	
	while(wlength--)
	{
		if ((SSPSTAT & ackstat) == ackstat)        //if(ACKSTAT) 
			ERROR();       //��Ӧ�𣬴���
		else if((SSPSTAT & ackstat) != ackstat)   //if(!ACKSTAT)
		{ DMA_Start(CH0,(INT16U)mtd,1,i2c_tx); }
			
		mtd++;     //*(mtd+wlength%PAGE)   //д������
		i2c_int();    //����
  }
	if ((SSPSTAT & ackstat) == ackstat)        //if(ACKSTAT) 
  		ERROR();  
	else if((SSPSTAT & ackstat) != ackstat)   //if(!ACKSTAT)
    	i2c_stop();   //����ֹͣλ������EEPROM�ڲ�д����	

 
	do{               //Ӧ���ѯ���ȴ��ڲ�д����
		i2c_start();
			
		DMA_Start(CH0,(INT16U)&ctrlw,1,i2c_tx);  //���ͣ�дEEPROM��0xA0
			
		i2c_int();
		if((SSPSTAT & ackstat) != ackstat)  //if(!ACKSTAT)���л�Ӧ˵����д�꣬����
				break;
		i2c_stop();
	}while((SSPSTAT & ackstat) == ackstat);  //while(ACKSTAT);�޻�Ӧ�������ѯ

	return I2C_SUCCESS;
}


/*****************************************************************************************************
*    ��������i2c_write_conflict    
*    �������ܣ�д���ݳ�ͻ
*    ��ڲ����� ��
*    ���ڲ���������1����ʾ����д��ͻ��WCOL����
*****************************************************************************************************/
bit i2c_write_conflict(void)
{
	i2c_init();
	
	AIE3 = 1;
	SSPIR |= i2cie;   //I2CIE = 1��ʹ���ж�
 	SSPERR |= errie;
	
	
	SSPSTAT &= ~i2c_wcol; //WCOL�������
	SSPBUF = 0xAA;   //startδ׼����ʱ�������ݣ���ɳ�ͻ
	wait_i2cif_clear();
	if(flag & i2c_wcol)
		return 1;	
// UartTx_0(flag);	
// 	if((SSPSTAT & i2c_wcol) == i2c_wcol)
// 	{
// 		return 1;
// 	}
	SSPCON |= sen;  //SEN=1����ʼSTART
	wait_i2cif_clear();
	SSPBUF = 0xA0;         //д������ֽ�
	wait_i2cif_clear();           //�ȴ����ͣ�������ɵھŸ�scl�½��ز����ж�

	if((SSPSTAT & ackstat) == ackstat)        //if(ACKSTAT)
		ERROR();    //��Ӧ�𣬴���
	else if((SSPSTAT & ackstat) != ackstat)   //if(!ACKSTAT)
		SSPBUF = 0x00;            //д�����ݵ�ַ
	wait_i2cif_clear(); //����
	if((SSPSTAT & ackstat) == ackstat)        //if(ACKSTAT)
		ERROR();    //��Ӧ�𣬴���
	else if((SSPSTAT & ackstat) != ackstat)   //if(!ACKSTAT)
		SSPBUF = 0x00;            //д�����ݵ�ַ
	wait_i2cif_clear(); //����
	
	if((SSPSTAT & ackstat) == ackstat)        //if(ACKSTAT) 
		ERROR();      //��Ӧ�𣬴���
	else if((SSPSTAT & ackstat) != ackstat)   //if(!ACKSTAT)
	    SSPBUF = 0x55;          //д������

	wait_i2cif_clear();    //����
	       
	if((SSPSTAT & ackstat) == ackstat)        //if(ACKSTAT) 
		ERROR();
	else if((SSPSTAT & ackstat) != ackstat)   //if(!ACKSTAT)
		i2c_stop();   //����ֹͣλ������EEPROM�ڲ�д����
	    
	do{               //Ӧ���ѯ���ȴ��ڲ�д����
		i2c_start();
		SSPBUF = 0xA0;
		wait_i2cif_clear();
		if((SSPSTAT & ackstat) != ackstat)  //if(!ACKSTAT)���л�Ӧ˵����д�꣬����
			 break;
		i2c_stop();
	}while((SSPSTAT & ackstat) == ackstat);  //while(ACKSTAT);�޻�Ӧ�������ѯ

	return 0;
}

/*****************************************************************************************************
*    ��������i2c_write_conflict2    
*    �������ܣ�д���ݳ�ͻ
*    ��ڲ����� ��
*    ���ڲ���������1����ʾ����д��ͻ��WCOL����
*****************************************************************************************************/
bit i2c_write_conflict2(void)
{
	i2c_init();
	AIE3 = 1;
	SSPIR |= i2cie;   //I2CIE = 1��ʹ���ж�
 	SSPERR |= errie;
	
	
	SSPSTAT &= ~i2c_wcol; //WCOL�������
	SSPCON |= sen;  //SEN=1����ʼSTART
	wait_i2cif_clear();
	SSPBUF = 0xA0;         //д������ֽ�
	wait_i2cif_clear();            //�ȴ����ͣ�������ɵھŸ�scl�½��ز����ж�

	if((SSPSTAT & ackstat) == ackstat)        //if(ACKSTAT)
		ERROR();    //��Ӧ�𣬴���
	else if((SSPSTAT & ackstat) != ackstat)   //if(!ACKSTAT)
		SSPBUF = 0x00;            //д�����ݵ�ַ
	wait_i2cif_clear(); //����
	if((SSPSTAT & ackstat) == ackstat)        //if(ACKSTAT)
		ERROR();    //��Ӧ�𣬴���
	else if((SSPSTAT & ackstat) != ackstat)   //if(!ACKSTAT)
		SSPBUF = 0x00;            //д�����ݵ�ַ
	wait_i2cif_clear(); //����
	
	if((SSPSTAT & ackstat) == ackstat)        //if(ACKSTAT) 
		ERROR();      //��Ӧ�𣬴���
	else if((SSPSTAT & ackstat) != ackstat)   //if(!ACKSTAT)
	    SSPBUF = 0x55;          //д������
		SSPBUF = 0xAA;   //һ֡����δ����ʱ��д���ݣ����д��ͻ
	wait_i2cif_clear();
	if(flag & i2c_wcol)
		return 1;	
	
// 	if((SSPSTAT & i2c_wcol) == i2c_wcol)
// 	{
// 		return 1;
// 	}
// 	wait_i2cif_clear();    //����
	       
	if((SSPSTAT & ackstat) == ackstat)        //if(ACKSTAT) 
		ERROR();
	else if((SSPSTAT & ackstat) != ackstat)   //if(!ACKSTAT)
		i2c_stop();   //����ֹͣλ������EEPROM�ڲ�д����
	    
	do{               //Ӧ���ѯ���ȴ��ڲ�д����
		i2c_start();
		SSPBUF = 0xA0;
		i2c_int();
		if((SSPSTAT & ackstat) != ackstat)  //if(!ACKSTAT)���л�Ӧ˵����д�꣬����
			 break;
		i2c_stop();
	}while((SSPSTAT & ackstat) == ackstat);  //while(ACKSTAT);�޻�Ӧ�������ѯ

	return 0;
}

/*****************************************************************************************************
*    ��������i2c_oierr    
*    �������ܣ�OP_IDLE״̬�´����־λ
*    ��ڲ����� ��
*    ���ڲ���������1����ʾOP_IDLE״̬�´����־λ����
*****************************************************************************************************/
bit i2c_oierr(void)
{
	AIE3 = 1;
	SSPERR = 0x00;
	SSPIR |= i2cie;   //I2CIE = 1��ʹ���ж�
 	SSPERR |= errie;
	SSPCON = 0;
	SSPCON |= i2cen;  //I2C_EN=1
	SSPSTAT = 0x00;
	SSPFSM = 0x00;
	
	
	SSPCON |= sen;  //SEN=1����ʼSTART
	wait_i2cif_clear();
	SSPBUF = 0xA0;         //д������ֽ�
	wait_i2cif_clear();
	
	SSPCON |= sen;  //SEN=1����ʼSTART
	wait_i2cif_clear();

	if(TX_Buf[8] & oierr)
		return 1;
	
	
	if((SSPSTAT & ackstat) == ackstat)        //if(ACKSTAT)
		ERROR();    //��Ӧ�𣬴���
	else if((SSPSTAT & ackstat) != ackstat)   //if(!ACKSTAT)
		SSPBUF = 0x00;            //д�����ݵ�ַ
	wait_i2cif_clear();

	SSPCON |= sen;  //SEN=1����ʼSTART
	wait_i2cif_clear();
	if(TX_Buf[8] && oierr)
		return 1;	
	else return 0;
}

/*****************************************************************************************************
*    ��������i2c_sderr    
*    �������ܣSSTART_DONE״̬�´����־λ
*    ��ڲ����� ��
*    ���ڲ���������1��START_DONE״̬�´����־λ����
*****************************************************************************************************/
bit i2c_sderr(void)
{
	AIE4 = 1;
	SSPIR |= i2cie;   //I2CIE = 1��ʹ���ж�
 	SSPERR |= errie;

	SSPCON = 0;
	SSPCON |= i2cen;  //I2C_EN=1
	SSPSTAT = 0x00;
	SSPFSM = 0x00;
	
	
	SSPCON |= sen;  //SEN=1����ʼSTART
	wait_i2cif_clear();
	SSPCON |= BIT0; 
	SSPCON |= BIT1;  
	SSPCON |= BIT2;
	SSPCON |= BIT3;
	SSPCON |= BIT4;  //��IDLE״̬�²���sderr��־
	
	wait_i2cif_clear();
	if(TX_Buf[8] & sderr)
		return 1;	
	else return 0;	
	
}

/*****************************************************************************************************
*    ��������i2c_ierr    
*    �������ܣ�IDLE״̬�´����־λ
*    ��ڲ����� ��
*    ���ڲ���������1����ʾIDLE״̬�´����־λ����
*****************************************************************************************************/
bit i2c_ierr(void)
{
	AIE4 = 1;
	SSPIR |= i2cie;   //I2CIE = 1��ʹ���ж�
 	SSPERR |= errie;

	SSPCON = 0;
	SSPCON |= i2cen;  //I2C_EN=1
	SSPSTAT = 0x00;
	SSPFSM = 0x00;
	

	SSPCON |= BIT1;  
	SSPCON |= BIT3;
	SSPCON |= BIT4;  //��IDLE״̬�²���ierr��־
	
	wait_i2cif_clear();
	if(TX_Buf[8] & ierr)
		return 1;	
	else return 0;	
	
}

/*****************************************************************************************************
*    ��������i2c_write    
*    �������ܣ�д����
*    ��ڲ����� ctrlwΪ�����ֽڣ�����Ѱַ�������д����������
*     eepromaddrΪ���ݵ�ַ����дһ��ҳ�����ݵ�ַ������ÿҳ���׵�ַ����׼ȷд�루����һҳ�������ݻᷭת���ǣ���
*     mtdΪ���������ݻ�������ַ
*     wlengthΪд�������ֽڳ���
*	  EepromTypeΪ���������ͣ�ö�ٱ������ֱ�Ϊ2401~24256
*    ���ڲ���������0��ʾ�������󣬷���1��ʾ�ɹ�д��
*****************************************************************************************************/
bit i2c_write(INT8U ctrlw, INT16U eepromaddr, INT8U *mtd, INT16U wlength, eetype EepromType) 
{
	i2c_init();           //��ʼ��
	i2c_start();          //��������
	SSPBUF = ctrlw;       //ex:дEEPROM��0xA0 
	i2c_int();            //�ȴ����ͣ�������ɵھŸ�scl�½��ز����ж�
    
  if ((SSPSTAT & ackstat) == ackstat)        //if(ACKSTAT)
     ERROR();   //��Ӧ�𣬴���
  else if ((SSPSTAT & ackstat) != ackstat)  //if(!ACKSTAT)
	{	
		if(EepromType > FM24C16)
		{
		    SSPBUF = eepromaddr>>8;              //ȡEEPROM���ݵ�ַ���ֽ�
		    i2c_int(); //����
		}
		SSPBUF = (INT8U)eepromaddr;            //��FM24C08��10λ���ݵ�ַ��2λ�ڿ����ֽ��У���8λ����һ�ֽڼ���
		i2c_int(); //����
  }
	
	while(wlength--)
	{
		if ((SSPSTAT & ackstat) == ackstat)        //if(ACKSTAT) 
			ERROR();       //��Ӧ�𣬴���
		else if((SSPSTAT & ackstat) != ackstat)   //if(!ACKSTAT)
			SSPBUF = *mtd++;     //*(mtd+wlength%PAGE)   //д������
		i2c_int();    //����
  }
	if ((SSPSTAT & ackstat) == ackstat)        //if(ACKSTAT) 
  		ERROR();  
	else if((SSPSTAT & ackstat) != ackstat)   //if(!ACKSTAT)
    	i2c_stop();   //����ֹͣλ������EEPROM�ڲ�д����
 
	do{               //Ӧ���ѯ���ȴ��ڲ�д����
	i2c_start();
	SSPBUF = 0xA0; 
	i2c_int();
	if((SSPSTAT & ackstat) != ackstat)  //if(!ACKSTAT)���л�Ӧ˵����д�꣬����
	    break;
	i2c_stop();
	}while((SSPSTAT & ackstat) == ackstat);  //while(ACKSTAT);�޻�Ӧ�������ѯ

//	delay(5);
	return I2C_SUCCESS;
}

/***********************************************************************************
*    ��������i2c_random_read  
*    �������ܣ����ɶ�������������/���ɶ�һ�ֽ�
*    ��ڲ�����ctrlΪ�����ֽڣ�����Ѱַ������Ͷ�/д����������
*     eepromaddrΪ���ݵ�ַ
*     mrdΪ���������ݻ�������ַ
*     rlengthΪ���������ֽ�����rlength�����ڴ洢������ַ���ȣ����ᷭת����С��ַ
*	  EepromTypeΪ���������ͣ�ö�ٱ������ֱ�Ϊ2401~24256
*	���ڲ���������0��ʾ�������󣬷���1��ʾ��ȡ�ɹ�
************************************************************************************/
bit i2c_random_read(INT8U ctrl, INT16U eepromaddr, INT8U *mrd, INT16U rlength, eetype EepromType)
{
	i2c_init();
	i2c_start();
	SSPBUF = ctrl;        //��д������ex:EEPROM��0xA0 
	i2c_int(); 
    
  if ((SSPSTAT & ackstat) == ackstat)        //if(ACKSTAT)
     ERROR();   //��Ӧ�𣬴���
  else if ((SSPSTAT & ackstat) != ackstat)  //if(!ACKSTAT)
	{	
		if(EepromType > FM24C16)
		{
		    SSPBUF = eepromaddr>>8;              //ȡEEPROM���ݵ�ַ���ֽ�
		    i2c_int(); //����
		}
		SSPBUF = (INT8U)eepromaddr;            //��FM24C08��10λ���ݵ�ַ��2λ�ڿ����ֽ��У���8λ����һ�ֽڼ���
		i2c_int(); //����
    }
    
	if((SSPSTAT & ackstat) == ackstat)        //if(ACKSTAT)
		ERROR();  //��Ӧ�𣬴���
	else if((SSPSTAT & ackstat) != ackstat)   //if(!ACKSTAT)
		i2c_restart();                    
	SSPBUF = (ctrl ^ 0x01);       //������������,ex:EEPROM��0xA1
	i2c_int();
    
	if((SSPSTAT & ackstat) == ackstat)        //if(ACKSTAT) Ӧ��
		ERROR();
	else if((SSPSTAT & ackstat) != ackstat)   //if(!ACKSTAT)
	while(--rlength)     //��rlength-1�ֽ�����
	{
		SSPCON |= rcen;   //RCEN=1;
		i2c_int();
		*mrd++ = SSPBUF;
		SSPSTAT &= ~ackdt;      //ACKDT=0�����ؽ�����Ӧ��״̬
		SSPCON |= acken; 	 //ACKEN=1�����ؽ�����Ӧ��ʹ��
		i2c_int();
	}
	SSPCON |= rcen;   //RCEN=1;
	i2c_int();
    *mrd = SSPBUF;
	SSPSTAT |= ackdt;      //ACKDT=1�����ؽ�����Ӧ��״̬
	SSPCON |= acken; 	 //ACKEN=1�����ؽ�����Ӧ��ʹ��
	i2c_int();

	i2c_stop();         //���꣬����

	return I2C_SUCCESS;
}

bit i2c_random_read_DMA(INT8U ctrl, INT16U eepromaddr, INT8U near *mrd, INT16U rlength)
{
	INT8U send_buff[2];
	send_buff[0] = eepromaddr>>8;     //ȡEEPROM���ݵ�ַ���ֽ�
	send_buff[1] = (INT8U)eepromaddr; //ȡEEPROM���ݵ�ַ���ֽ�
	
	i2c_init();           //��ʼ��
	i2c_start();          //��������	
	
	DMA_Start(CH0,(INT16U)&ctrl,1,i2c_tx);  //���ͣ�дEEPROM��0xA0 
	
	i2c_int();            //�ȴ����ͣ�������ɵھŸ�scl�½��ز����ж�
  if ((SSPSTAT & ackstat) == ackstat)        //if(ACKSTAT)
		ERROR();   //ackstat==1ʱ��Ӧ�𣬴���	
	
	DMA_Start(CH0,(INT16U)&send_buff[0],1,i2c_tx);  
	
	i2c_int(); //����
  if ((SSPSTAT & ackstat) == ackstat)        //if(ACKSTAT)
		ERROR();   //ackstat==1ʱ��Ӧ�𣬴���	
	
	DMA_Start(CH0,(INT16U)&send_buff[1],1,i2c_tx);  
	
	i2c_int(); //����
	if((SSPSTAT & ackstat) == ackstat)        //if(ACKSTAT)
		ERROR();  //��Ӧ�𣬴���
	else if((SSPSTAT & ackstat) != ackstat)   //if(!ACKSTAT)
		i2c_restart();     

	ctrl = (ctrl ^ 0x01);       //������������,ex:EEPROM��0xA1
	DMA_Start(CH0,(INT16U)&ctrl,1,i2c_tx);  //���ͣ�дEEPROM��0xA0 
	i2c_int();
    
	if((SSPSTAT & ackstat) == ackstat)        //if(ACKSTAT) Ӧ��
		ERROR();
	else if((SSPSTAT & ackstat) != ackstat)   //if(!ACKSTAT)
	while(--rlength)     //��rlength-1�ֽ�����
	{
		SSPCON |= rcen;   //RCEN=1;
		i2c_int();
		DMA_Start(CH0,(INT16U)mrd,1,i2c_rx);  //DMA���� 
		mrd++;
		SSPSTAT &= ~ackdt;      //ACKDT=0�����ؽ�����Ӧ��״̬
		SSPCON |= acken; 	 //ACKEN=1�����ؽ�����Ӧ��ʹ��
		i2c_int();
	}
	SSPCON |= rcen;   //RCEN=1;
	i2c_int();
  DMA_Start(CH0,(INT16U)mrd,1,i2c_rx);  //DMA����
	SSPSTAT |= ackdt;      //ACKDT=1�����ؽ�����Ӧ��״̬
	SSPCON |= acken; 	 //ACKEN=1�����ؽ�����Ӧ��ʹ��
	i2c_int();

	i2c_stop();         //���꣬����

	return I2C_SUCCESS;
}


/***********************************************************************************
*    ��������i2c_current_read  
*    �������ܣ���ǰ��ַ��������������/��ǰ��ַ��һ�ֽ�
*    ��ڲ�����ctrlrΪ�����ֽڣ�����Ѱַ������Ͷ�����������
*     mrdΪ���������ݻ�������ַ
*     rlengthΪ���������ֽ�����rlength�����ڴ洢������ַ���ȣ����ᷭת����С��ַ
*	���ڲ���������0��ʾ�������󣬷���1��ʾ��ȡ�ɹ�
************************************************************************************/
bit i2c_current_read(INT8U ctrlr, INT8U *mrd, INT16U rlength)
{
	i2c_init();           //��ʼ��
	i2c_start();          //��������
	SSPBUF = ctrlr;         //ex:��EEPROM��0xA1 
	i2c_int();            //�ȴ����ͣ�������ɵھŸ�scl�½��ز����ж�

	if((SSPSTAT & ackstat) == ackstat)        //if(ACKSTAT) 
		ERROR();
	else if((SSPSTAT & ackstat) != ackstat)   //if(!ACKSTAT)
		while(--rlength)     //��ǰrlength-1�ֽ�����
		{
			SSPCON |= rcen;       //RCEN=1;����������
			i2c_int();            //���ݴ�SSPSR�Ĵ���ת�ص�SSPBUF�������ж�
			*mrd++ = SSPBUF;      //��������
			SSPSTAT &= ~ackdt;      //ACKDT=0�����ؽ�����Ӧ��״̬
			SSPCON |= acken; 	 //ACKEN=1�����ؽ�����Ӧ��ʹ��
			i2c_int();           //ACKʱ����ɣ������ж�
		}
//������Ҫ����ȷ�Ľ���ʱ��.���յ����һ�������ֽں����ؽ������������ط�����Ӧ��ʹ֮�ͷ������ߣ��Ա�������stop
	SSPCON |= rcen;    //RCEN=1;
	i2c_int();
	*mrd = SSPBUF;
	SSPSTAT |= ackdt;      //ACKDT=1
	SSPCON |= acken; 	 //ACKEN=1
  	i2c_int();	

	i2c_stop();         //���꣬����

	return I2C_SUCCESS;
}

/***********************************************************************************
*    ��������i2c_random_readwrong  
*    �������ܣ����ɶ������������������ؽ�������Ӧ�𣬺���EEPROM���ݽ����ᱻ����
*    ��ڲ�����ctrlΪ�����ֽڣ�����Ѱַ������Ͷ�/д����������
*     eepromaddrΪ���ݵ�ַ
*     mrdΪ���������ݻ�������ַ
*     rlengthΪ���������ֽ���
*	  EepromTypeΪ���������ͣ�ö�ٱ������ֱ�Ϊ2401~24256
*	 ���ڲ���������0��ʾ�������󣬷���1��ʾ��ȡ�ɹ�
************************************************************************************/
bit i2c_random_readwrong(INT8U ctrl, INT16U eepromaddr, INT8U *mrd, INT16U rlength, eetype EepromType)
{
	i2c_init();
	i2c_start();
	SSPBUF = ctrl;        //��д������ex:EEPROM��0xA0 
	i2c_int(); 
    
  if ((SSPSTAT & ackstat) == ackstat)        //if(ACKSTAT)
    ERROR();   //��Ӧ�𣬴���
  else if ((SSPSTAT & ackstat) != ackstat)  //if(!ACKSTAT)
	{	
		if(EepromType > FM24C16)
		{
		    SSPBUF = eepromaddr>>8;              //ȡEEPROM���ݵ�ַ���ֽ�
		    i2c_int(); //����
		}
		SSPBUF = (INT8U)eepromaddr;            //��FM24C08��10λ���ݵ�ַ��2λ�ڿ����ֽ��У���8λ����һ�ֽڼ���
		i2c_int(); //����
    }
    
	if((SSPSTAT & ackstat) == ackstat)        //if(ACKSTAT)
    	ERROR();  //��Ӧ�𣬴���
	else if((SSPSTAT & ackstat) != ackstat)   //if(!ACKSTAT)
    	i2c_restart();
	SSPBUF = (ctrl^0x01);       //������������,ex:EEPROM��0xA1
	i2c_int();
    
	if((SSPSTAT & ackstat) == ackstat)        //if(ACKSTAT) Ӧ��
		ERROR();
	else if((SSPSTAT & ackstat) != ackstat)   //if(!ACKSTAT)
	while(rlength--)     //��rlength�ֽ�����
	{
		SSPCON |= rcen;   //RCEN=1;
		i2c_int();
		*mrd++ = SSPBUF;
		SSPSTAT |= ackdt;      //ACKDT=1����һ�ֽڼ���Ӧ��
		SSPCON |= acken; 	 //ACKEN=1�����ؽ�����Ӧ��ʹ��
		i2c_int();
	}
	i2c_stop();         //���꣬����

	return I2C_SUCCESS;
}
