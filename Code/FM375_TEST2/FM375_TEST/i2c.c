#include "init.h"

void INIT_I2C_PAD(void)
{
	PEFCR1 = 0;
	PEFCR2 = 0;
	PEFCR2 |= BIT5 | BIT6;//{PEFCR2[5]:PEFCR1[5]}=2'b10
	AFSELE |= BIT5 | BIT6;//1：SCL,SDA
	
	PEPPEN = 0;
	PEPPEN |= BIT7;//使能PE7的推挽功能
	PEFCR1 |= BIT7;//{PEFCR2[7]:PEFCR1[7]}=2'b01,使能PE0输出,WP
	PEDATA &= ~BIT7;
}


/********************************************************
*	函数名：i2c_init                               
*	函数功能：i2c寄存器初始化 
*   入口参数：无
*   出口参数：无             
********************************************************/
void i2c_init(void)
{	
//  	SSPIR |= i2cie;   //I2CIE = 1；使能中断
//  	SSPERR |= errie;
	
	
	SSPERR = 0x00;
//	SSPBUF = 0xa0;	
//	SSPCON |= BIT1;  
//	SSPCON |= BIT3;
//	SSPCON |= BIT4;  //在IDLE状态下产生ierr标志
	
	SSPCON = 0;
	//SSPCON:|I2CEN|LLEN|HLEN|ACKEN|RCEN|PEN|RSEN|SEN|
	SSPCON |= i2cen;  //I2C_EN=1

	//波特率设置由上位机命令单独配置
//	SSPBRG = FOSC/(4*band)-1;  
//	SSPBRG = 0x04;		//8m时钟，400k，样片测试时固定
// 	SSPBRG = 0x06;    //3m时钟，100k
	//SSPSTAT:|WCOL|--|R/W|P|S|BF|ACKSAT|ACKDT|
	SSPSTAT = 0x00;
	//SSPIR:|--|--|--|--|--|--|I2CIE|I2CIF|
	SSPFSM = 0x00;

}

/*******************************************************
*   函数名：i2c_start                                  
*   函数功能：start时序产生  
*   入口参数：无
*   出口参数：无                          
*******************************************************/
void i2c_start(void)
{
	SSPCON |= sen;   //SEN=1;I2C启动
	do{
	;
	}
	while((SSPIR & i2cif) != i2cif);  //SSPIF==0;
	SSPIR &= ~i2cif;   //SSPIF=0;
}

/*******************************************************
*   函数名：i2c_stop                                  
*   函数功能：stop时序产生
*   入口参数：无
*   出口参数：无                           
*******************************************************/
void i2c_stop(void)
{
	SSPCON |= pen;    //PEN=1;I2C停止
	do{
	;
	}
	while((SSPIR & i2cif) != i2cif);  //SSPIF==0;
	SSPIR &= ~i2cif;   //SSPIF=0;
}

/*******************************************************
*   函数名：i2c_restart                                  
*   函数功能：restart时序产生 
*   入口参数：无
*   出口参数：无                           
*******************************************************/
void i2c_restart(void)
{
	SSPCON |= rsen;   //RSEN=1;重复启动
	do{
	;
	}
	while((SSPIR & i2cif) != i2cif);  //SSPIF==0;
	SSPIR &= ~i2cif;   //SSPIF=0;
}

/*******************************************************
*   函数名：i2c_int                                  
*   函数功能：中断查询等待，清除 
*   入口参数：无
*   出口参数：无                          
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
*   函数名：wait_if_clear                                
*   函数功能：等中断标志清除
*   入口参数：无
*   出口参数：无                          
*******************************************************/
// uchar wait_if_clear()
// {
// 	//===增加定时控制
// 	INIT_T0();							//timer0参数配置					
// 	Timer0_Test_Cnt = 1;					//5ms计时延时
// 	Timer0_Test_Statues = 0;
// 	TR0=1;	
// 	while(!clr_if)
// 	{
// 		WDT_CLR();
// 		if(Timer0_Test_Statues==1)			//在5ms内无法完成
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
*   函数名：wait_if_clear                                
*   函数功能：等中断标志清除
*   入口参数：无
*   出口参数：无                          
*******************************************************/
void wait_i2cif_clear(void)
{
	while(!clr_if){WDT_CLR();}
	clr_if = 0;
	
}

/*****************************************************************************************************
*    函数名：i2c_write    
*    函数功能：写数据
*    入口参数： ctrlw为控制字节，包括寻址命令码和写操作命令码
*     eepromaddr为数据地址。若写一整页，数据地址必须是每页的首地址才能准确写入（超过一页长度数据会翻转覆盖）！
*     mtd为主发送数据缓冲区首址
*     wlength为写入数据字节长度
*	  EepromType为从器件类型，枚举变量，分别为2401~24256
*    出口参数：返回0表示发生错误，返回1表示成功写入
*****************************************************************************************************/
bit i2c_write1(INT8U ctrlw, INT16U eepromaddr, INT8U *mtd, INT16U wlength) 
{
	i2c_init();           //初始化
	i2c_start();          //启动操作
	SSPBUF = ctrlw;       //ex:写EEPROM，0xA0 
	i2c_int();            //等待发送，发送完成第九个scl下降沿产生中断
    
  if ((SSPSTAT & ackstat) == ackstat)        //if(ACKSTAT)
		ERROR();   //ackstat==1时无应答，错误！
  SSPBUF = eepromaddr>>8;              //取EEPROM数据地址高字节
	i2c_int(); //发送
	if ((SSPSTAT & ackstat) == ackstat)        //if(ACKSTAT)
    ERROR();   //无应答，错误
	SSPBUF = (INT8U)eepromaddr;            //对FM24C08，10位数据地址高2位在控制字节中，低8位发送一字节即可
	i2c_int(); //发送
	
	while(wlength--)
	{
		if ((SSPSTAT & ackstat) == ackstat)        //if(ACKSTAT) 
			ERROR();       //无应答，错误！
		else if((SSPSTAT & ackstat) != ackstat)   //if(!ACKSTAT)
			SSPBUF = *mtd++;     //*(mtd+wlength%PAGE)   //写入数据
		i2c_int();    //发送
  }
	if ((SSPSTAT & ackstat) == ackstat)        //if(ACKSTAT) 
  		ERROR();  
	else if((SSPSTAT & ackstat) != ackstat)   //if(!ACKSTAT)
    	i2c_stop();   //产生停止位，启动EEPROM内部写过程
 
	do{               //应答查询，等待内部写周期
	i2c_start();
	SSPBUF = 0xA0; 
	i2c_int();
	if((SSPSTAT & ackstat) != ackstat)  //if(!ACKSTAT)，有回应说明已写完，跳出
	    break;
	i2c_stop();
	}while((SSPSTAT & ackstat) == ackstat);  //while(ACKSTAT);无回应则继续查询

	return I2C_SUCCESS;
}

// void I2C_DMA_Config(void)
// {
// 	INT8U i;
// 	PERICLK_CTRL0 |= dma_clken;   //DMA时钟使能
// 	for(i=0; i<21; i++)
// 	{
// 		XBYTE[DMA_ADDR+i] = 0; //deinit dma reg
// 	}
// 	
// 	GCTRL |= dma_en;  //DMA全局使能
// 	GCTRL &= ~m2m_en; //FLASH通道禁止

// 	GCTRL |= prior_ch0; //通道0优先
// 	CH0_CTRL |= ch_inc;//源端地址递加
// 	CH0_CTRL |= spi_tx;//CH0:I2C发送

// 	CH1_CTRL |= ch_inc;//源端地址递加
// 	CH1_CTRL |= spi_rx;//CH1：I2C接收

// }
/*****************************************************************************************************
*    函数名：i2c_write    
*    函数功能：写数据
*    入口参数： ctrlw为控制字节，包括寻址命令码和写操作命令码
*     eepromaddr为数据地址。若写一整页，数据地址必须是每页的首地址才能准确写入（超过一页长度数据会翻转覆盖）！
*     mtd为主发送数据缓冲区首址
*     wlength为写入数据字节长度
*	  EepromType为从器件类型，枚举变量，分别为2401~24256
*    出口参数：返回0表示发生错误，返回1表示成功写入
*****************************************************************************************************/
bit i2c_write_DMA(INT8U ctrlw, INT16U eepromaddr, INT8U near *mtd, INT16U wlength) 
{
	INT8U send_buff[2];
	send_buff[0] = eepromaddr>>8;     //取EEPROM数据地址高字节
	send_buff[1] = (INT8U)eepromaddr; //取EEPROM数据地址低字节
	
	i2c_init();           //初始化
	i2c_start();          //启动操作	
	
	DMA_Start(CH0,(INT16U)&ctrlw,1,i2c_tx);  //发送，写EEPROM，0xA0 
	
	i2c_int();            //等待发送，发送完成第九个scl下降沿产生中断
  if ((SSPSTAT & ackstat) == ackstat)        //if(ACKSTAT)
		ERROR();   //ackstat==1时无应答，错误！	
	
	DMA_Start(CH0,(INT16U)&send_buff[0],1,i2c_tx);  
	
	i2c_int(); //发送
  if ((SSPSTAT & ackstat) == ackstat)        //if(ACKSTAT)
		ERROR();   //ackstat==1时无应答，错误！	
	
	DMA_Start(CH0,(INT16U)&send_buff[1],1,i2c_tx);  
	
	i2c_int(); //发送
	
	while(wlength--)
	{
		if ((SSPSTAT & ackstat) == ackstat)        //if(ACKSTAT) 
			ERROR();       //无应答，错误！
		else if((SSPSTAT & ackstat) != ackstat)   //if(!ACKSTAT)
		{ DMA_Start(CH0,(INT16U)mtd,1,i2c_tx); }
			
		mtd++;     //*(mtd+wlength%PAGE)   //写入数据
		i2c_int();    //发送
  }
	if ((SSPSTAT & ackstat) == ackstat)        //if(ACKSTAT) 
  		ERROR();  
	else if((SSPSTAT & ackstat) != ackstat)   //if(!ACKSTAT)
    	i2c_stop();   //产生停止位，启动EEPROM内部写过程	

 
	do{               //应答查询，等待内部写周期
		i2c_start();
			
		DMA_Start(CH0,(INT16U)&ctrlw,1,i2c_tx);  //发送，写EEPROM，0xA0
			
		i2c_int();
		if((SSPSTAT & ackstat) != ackstat)  //if(!ACKSTAT)，有回应说明已写完，跳出
				break;
		i2c_stop();
	}while((SSPSTAT & ackstat) == ackstat);  //while(ACKSTAT);无回应则继续查询

	return I2C_SUCCESS;
}


/*****************************************************************************************************
*    函数名：i2c_write_conflict    
*    函数功能：写数据冲突
*    入口参数： 无
*    出口参数：返回1，表示发生写冲突，WCOL置起
*****************************************************************************************************/
bit i2c_write_conflict(void)
{
	i2c_init();
	
	AIE3 = 1;
	SSPIR |= i2cie;   //I2CIE = 1；使能中断
 	SSPERR |= errie;
	
	
	SSPSTAT &= ~i2c_wcol; //WCOL软件清零
	SSPBUF = 0xAA;   //start未准备好时即发数据，造成冲突
	wait_i2cif_clear();
	if(flag & i2c_wcol)
		return 1;	
// UartTx_0(flag);	
// 	if((SSPSTAT & i2c_wcol) == i2c_wcol)
// 	{
// 		return 1;
// 	}
	SSPCON |= sen;  //SEN=1，开始START
	wait_i2cif_clear();
	SSPBUF = 0xA0;         //写入控制字节
	wait_i2cif_clear();           //等待发送，发送完成第九个scl下降沿产生中断

	if((SSPSTAT & ackstat) == ackstat)        //if(ACKSTAT)
		ERROR();    //无应答，错误！
	else if((SSPSTAT & ackstat) != ackstat)   //if(!ACKSTAT)
		SSPBUF = 0x00;            //写入数据地址
	wait_i2cif_clear(); //发送
	if((SSPSTAT & ackstat) == ackstat)        //if(ACKSTAT)
		ERROR();    //无应答，错误！
	else if((SSPSTAT & ackstat) != ackstat)   //if(!ACKSTAT)
		SSPBUF = 0x00;            //写入数据地址
	wait_i2cif_clear(); //发送
	
	if((SSPSTAT & ackstat) == ackstat)        //if(ACKSTAT) 
		ERROR();      //无应答，错误！
	else if((SSPSTAT & ackstat) != ackstat)   //if(!ACKSTAT)
	    SSPBUF = 0x55;          //写入数据

	wait_i2cif_clear();    //发送
	       
	if((SSPSTAT & ackstat) == ackstat)        //if(ACKSTAT) 
		ERROR();
	else if((SSPSTAT & ackstat) != ackstat)   //if(!ACKSTAT)
		i2c_stop();   //产生停止位，启动EEPROM内部写过程
	    
	do{               //应答查询，等待内部写周期
		i2c_start();
		SSPBUF = 0xA0;
		wait_i2cif_clear();
		if((SSPSTAT & ackstat) != ackstat)  //if(!ACKSTAT)，有回应说明已写完，跳出
			 break;
		i2c_stop();
	}while((SSPSTAT & ackstat) == ackstat);  //while(ACKSTAT);无回应则继续查询

	return 0;
}

/*****************************************************************************************************
*    函数名：i2c_write_conflict2    
*    函数功能：写数据冲突
*    入口参数： 无
*    出口参数：返回1，表示发生写冲突，WCOL置起
*****************************************************************************************************/
bit i2c_write_conflict2(void)
{
	i2c_init();
	AIE3 = 1;
	SSPIR |= i2cie;   //I2CIE = 1；使能中断
 	SSPERR |= errie;
	
	
	SSPSTAT &= ~i2c_wcol; //WCOL软件清零
	SSPCON |= sen;  //SEN=1，开始START
	wait_i2cif_clear();
	SSPBUF = 0xA0;         //写入控制字节
	wait_i2cif_clear();            //等待发送，发送完成第九个scl下降沿产生中断

	if((SSPSTAT & ackstat) == ackstat)        //if(ACKSTAT)
		ERROR();    //无应答，错误！
	else if((SSPSTAT & ackstat) != ackstat)   //if(!ACKSTAT)
		SSPBUF = 0x00;            //写入数据地址
	wait_i2cif_clear(); //发送
	if((SSPSTAT & ackstat) == ackstat)        //if(ACKSTAT)
		ERROR();    //无应答，错误！
	else if((SSPSTAT & ackstat) != ackstat)   //if(!ACKSTAT)
		SSPBUF = 0x00;            //写入数据地址
	wait_i2cif_clear(); //发送
	
	if((SSPSTAT & ackstat) == ackstat)        //if(ACKSTAT) 
		ERROR();      //无应答，错误！
	else if((SSPSTAT & ackstat) != ackstat)   //if(!ACKSTAT)
	    SSPBUF = 0x55;          //写入数据
		SSPBUF = 0xAA;   //一帧数据未发完时再写数据，造成写冲突
	wait_i2cif_clear();
	if(flag & i2c_wcol)
		return 1;	
	
// 	if((SSPSTAT & i2c_wcol) == i2c_wcol)
// 	{
// 		return 1;
// 	}
// 	wait_i2cif_clear();    //发送
	       
	if((SSPSTAT & ackstat) == ackstat)        //if(ACKSTAT) 
		ERROR();
	else if((SSPSTAT & ackstat) != ackstat)   //if(!ACKSTAT)
		i2c_stop();   //产生停止位，启动EEPROM内部写过程
	    
	do{               //应答查询，等待内部写周期
		i2c_start();
		SSPBUF = 0xA0;
		i2c_int();
		if((SSPSTAT & ackstat) != ackstat)  //if(!ACKSTAT)，有回应说明已写完，跳出
			 break;
		i2c_stop();
	}while((SSPSTAT & ackstat) == ackstat);  //while(ACKSTAT);无回应则继续查询

	return 0;
}

/*****************************************************************************************************
*    函数名：i2c_oierr    
*    函数功能：OP_IDLE状态下错误标志位
*    入口参数： 无
*    出口参数：返回1，表示OP_IDLE状态下错误标志位置起
*****************************************************************************************************/
bit i2c_oierr(void)
{
	AIE3 = 1;
	SSPERR = 0x00;
	SSPIR |= i2cie;   //I2CIE = 1；使能中断
 	SSPERR |= errie;
	SSPCON = 0;
	SSPCON |= i2cen;  //I2C_EN=1
	SSPSTAT = 0x00;
	SSPFSM = 0x00;
	
	
	SSPCON |= sen;  //SEN=1，开始START
	wait_i2cif_clear();
	SSPBUF = 0xA0;         //写入控制字节
	wait_i2cif_clear();
	
	SSPCON |= sen;  //SEN=1，开始START
	wait_i2cif_clear();

	if(TX_Buf[8] & oierr)
		return 1;
	
	
	if((SSPSTAT & ackstat) == ackstat)        //if(ACKSTAT)
		ERROR();    //无应答，错误！
	else if((SSPSTAT & ackstat) != ackstat)   //if(!ACKSTAT)
		SSPBUF = 0x00;            //写入数据地址
	wait_i2cif_clear();

	SSPCON |= sen;  //SEN=1，开始START
	wait_i2cif_clear();
	if(TX_Buf[8] && oierr)
		return 1;	
	else return 0;
}

/*****************************************************************************************************
*    函数名：i2c_sderr    
*    函数功能START_DONE状态下错误标志位
*    入口参数： 无
*    出口参数：返回1，START_DONE状态下错误标志位置起
*****************************************************************************************************/
bit i2c_sderr(void)
{
	AIE4 = 1;
	SSPIR |= i2cie;   //I2CIE = 1；使能中断
 	SSPERR |= errie;

	SSPCON = 0;
	SSPCON |= i2cen;  //I2C_EN=1
	SSPSTAT = 0x00;
	SSPFSM = 0x00;
	
	
	SSPCON |= sen;  //SEN=1，开始START
	wait_i2cif_clear();
	SSPCON |= BIT0; 
	SSPCON |= BIT1;  
	SSPCON |= BIT2;
	SSPCON |= BIT3;
	SSPCON |= BIT4;  //在IDLE状态下产生sderr标志
	
	wait_i2cif_clear();
	if(TX_Buf[8] & sderr)
		return 1;	
	else return 0;	
	
}

/*****************************************************************************************************
*    函数名：i2c_ierr    
*    函数功能：IDLE状态下错误标志位
*    入口参数： 无
*    出口参数：返回1，表示IDLE状态下错误标志位置起
*****************************************************************************************************/
bit i2c_ierr(void)
{
	AIE4 = 1;
	SSPIR |= i2cie;   //I2CIE = 1；使能中断
 	SSPERR |= errie;

	SSPCON = 0;
	SSPCON |= i2cen;  //I2C_EN=1
	SSPSTAT = 0x00;
	SSPFSM = 0x00;
	

	SSPCON |= BIT1;  
	SSPCON |= BIT3;
	SSPCON |= BIT4;  //在IDLE状态下产生ierr标志
	
	wait_i2cif_clear();
	if(TX_Buf[8] & ierr)
		return 1;	
	else return 0;	
	
}

/*****************************************************************************************************
*    函数名：i2c_write    
*    函数功能：写数据
*    入口参数： ctrlw为控制字节，包括寻址命令码和写操作命令码
*     eepromaddr为数据地址。若写一整页，数据地址必须是每页的首地址才能准确写入（超过一页长度数据会翻转覆盖）！
*     mtd为主发送数据缓冲区首址
*     wlength为写入数据字节长度
*	  EepromType为从器件类型，枚举变量，分别为2401~24256
*    出口参数：返回0表示发生错误，返回1表示成功写入
*****************************************************************************************************/
bit i2c_write(INT8U ctrlw, INT16U eepromaddr, INT8U *mtd, INT16U wlength, eetype EepromType) 
{
	i2c_init();           //初始化
	i2c_start();          //启动操作
	SSPBUF = ctrlw;       //ex:写EEPROM，0xA0 
	i2c_int();            //等待发送，发送完成第九个scl下降沿产生中断
    
  if ((SSPSTAT & ackstat) == ackstat)        //if(ACKSTAT)
     ERROR();   //无应答，错误！
  else if ((SSPSTAT & ackstat) != ackstat)  //if(!ACKSTAT)
	{	
		if(EepromType > FM24C16)
		{
		    SSPBUF = eepromaddr>>8;              //取EEPROM数据地址高字节
		    i2c_int(); //发送
		}
		SSPBUF = (INT8U)eepromaddr;            //对FM24C08，10位数据地址高2位在控制字节中，低8位发送一字节即可
		i2c_int(); //发送
  }
	
	while(wlength--)
	{
		if ((SSPSTAT & ackstat) == ackstat)        //if(ACKSTAT) 
			ERROR();       //无应答，错误！
		else if((SSPSTAT & ackstat) != ackstat)   //if(!ACKSTAT)
			SSPBUF = *mtd++;     //*(mtd+wlength%PAGE)   //写入数据
		i2c_int();    //发送
  }
	if ((SSPSTAT & ackstat) == ackstat)        //if(ACKSTAT) 
  		ERROR();  
	else if((SSPSTAT & ackstat) != ackstat)   //if(!ACKSTAT)
    	i2c_stop();   //产生停止位，启动EEPROM内部写过程
 
	do{               //应答查询，等待内部写周期
	i2c_start();
	SSPBUF = 0xA0; 
	i2c_int();
	if((SSPSTAT & ackstat) != ackstat)  //if(!ACKSTAT)，有回应说明已写完，跳出
	    break;
	i2c_stop();
	}while((SSPSTAT & ackstat) == ackstat);  //while(ACKSTAT);无回应则继续查询

//	delay(5);
	return I2C_SUCCESS;
}

/***********************************************************************************
*    函数名：i2c_random_read  
*    函数功能：自由读启动的连续读/自由读一字节
*    入口参数：ctrl为控制字节，包括寻址命令码和读/写操作命令码
*     eepromaddr为数据地址
*     mrd为主接收数据缓冲器首址
*     rlength为待读数据字节数，rlength若大于存储器最大地址长度，将会翻转到最小地址
*	  EepromType为从器件类型，枚举变量，分别为2401~24256
*	出口参数：返回0表示发生错误，返回1表示读取成功
************************************************************************************/
bit i2c_random_read(INT8U ctrl, INT16U eepromaddr, INT8U *mrd, INT16U rlength, eetype EepromType)
{
	i2c_init();
	i2c_start();
	SSPBUF = ctrl;        //假写操作。ex:EEPROM，0xA0 
	i2c_int(); 
    
  if ((SSPSTAT & ackstat) == ackstat)        //if(ACKSTAT)
     ERROR();   //无应答，错误！
  else if ((SSPSTAT & ackstat) != ackstat)  //if(!ACKSTAT)
	{	
		if(EepromType > FM24C16)
		{
		    SSPBUF = eepromaddr>>8;              //取EEPROM数据地址高字节
		    i2c_int(); //发送
		}
		SSPBUF = (INT8U)eepromaddr;            //对FM24C08，10位数据地址高2位在控制字节中，低8位发送一字节即可
		i2c_int(); //发送
    }
    
	if((SSPSTAT & ackstat) == ackstat)        //if(ACKSTAT)
		ERROR();  //无应答，错误！
	else if((SSPSTAT & ackstat) != ackstat)   //if(!ACKSTAT)
		i2c_restart();                    
	SSPBUF = (ctrl ^ 0x01);       //读操作命令码,ex:EEPROM，0xA1
	i2c_int();
    
	if((SSPSTAT & ackstat) == ackstat)        //if(ACKSTAT) 应答
		ERROR();
	else if((SSPSTAT & ackstat) != ackstat)   //if(!ACKSTAT)
	while(--rlength)     //读rlength-1字节数据
	{
		SSPCON |= rcen;   //RCEN=1;
		i2c_int();
		*mrd++ = SSPBUF;
		SSPSTAT &= ~ackdt;      //ACKDT=0，主控接收器应答状态
		SSPCON |= acken; 	 //ACKEN=1，主控接收器应答使能
		i2c_int();
	}
	SSPCON |= rcen;   //RCEN=1;
	i2c_int();
    *mrd = SSPBUF;
	SSPSTAT |= ackdt;      //ACKDT=1，主控接收器应答状态
	SSPCON |= acken; 	 //ACKEN=1，主控接收器应答使能
	i2c_int();

	i2c_stop();         //读完，结束

	return I2C_SUCCESS;
}

bit i2c_random_read_DMA(INT8U ctrl, INT16U eepromaddr, INT8U near *mrd, INT16U rlength)
{
	INT8U send_buff[2];
	send_buff[0] = eepromaddr>>8;     //取EEPROM数据地址高字节
	send_buff[1] = (INT8U)eepromaddr; //取EEPROM数据地址低字节
	
	i2c_init();           //初始化
	i2c_start();          //启动操作	
	
	DMA_Start(CH0,(INT16U)&ctrl,1,i2c_tx);  //发送，写EEPROM，0xA0 
	
	i2c_int();            //等待发送，发送完成第九个scl下降沿产生中断
  if ((SSPSTAT & ackstat) == ackstat)        //if(ACKSTAT)
		ERROR();   //ackstat==1时无应答，错误！	
	
	DMA_Start(CH0,(INT16U)&send_buff[0],1,i2c_tx);  
	
	i2c_int(); //发送
  if ((SSPSTAT & ackstat) == ackstat)        //if(ACKSTAT)
		ERROR();   //ackstat==1时无应答，错误！	
	
	DMA_Start(CH0,(INT16U)&send_buff[1],1,i2c_tx);  
	
	i2c_int(); //发送
	if((SSPSTAT & ackstat) == ackstat)        //if(ACKSTAT)
		ERROR();  //无应答，错误！
	else if((SSPSTAT & ackstat) != ackstat)   //if(!ACKSTAT)
		i2c_restart();     

	ctrl = (ctrl ^ 0x01);       //读操作命令码,ex:EEPROM，0xA1
	DMA_Start(CH0,(INT16U)&ctrl,1,i2c_tx);  //发送，写EEPROM，0xA0 
	i2c_int();
    
	if((SSPSTAT & ackstat) == ackstat)        //if(ACKSTAT) 应答
		ERROR();
	else if((SSPSTAT & ackstat) != ackstat)   //if(!ACKSTAT)
	while(--rlength)     //读rlength-1字节数据
	{
		SSPCON |= rcen;   //RCEN=1;
		i2c_int();
		DMA_Start(CH0,(INT16U)mrd,1,i2c_rx);  //DMA接收 
		mrd++;
		SSPSTAT &= ~ackdt;      //ACKDT=0，主控接收器应答状态
		SSPCON |= acken; 	 //ACKEN=1，主控接收器应答使能
		i2c_int();
	}
	SSPCON |= rcen;   //RCEN=1;
	i2c_int();
  DMA_Start(CH0,(INT16U)mrd,1,i2c_rx);  //DMA接收
	SSPSTAT |= ackdt;      //ACKDT=1，主控接收器应答状态
	SSPCON |= acken; 	 //ACKEN=1，主控接收器应答使能
	i2c_int();

	i2c_stop();         //读完，结束

	return I2C_SUCCESS;
}


/***********************************************************************************
*    函数名：i2c_current_read  
*    函数功能：当前地址读启动的连续读/当前地址读一字节
*    入口参数：ctrlr为控制字节，包括寻址命令码和读操作命令码
*     mrd为主接收数据缓冲器首址
*     rlength为待读数据字节数，rlength若大于存储器最大地址长度，将会翻转到最小地址
*	出口参数：返回0表示发生错误，返回1表示读取成功
************************************************************************************/
bit i2c_current_read(INT8U ctrlr, INT8U *mrd, INT16U rlength)
{
	i2c_init();           //初始化
	i2c_start();          //启动操作
	SSPBUF = ctrlr;         //ex:读EEPROM，0xA1 
	i2c_int();            //等待发送，发送完成第九个scl下降沿产生中断

	if((SSPSTAT & ackstat) == ackstat)        //if(ACKSTAT) 
		ERROR();
	else if((SSPSTAT & ackstat) != ackstat)   //if(!ACKSTAT)
		while(--rlength)     //读前rlength-1字节数据
		{
			SSPCON |= rcen;       //RCEN=1;启动读操作
			i2c_int();            //数据从SSPSR寄存器转载到SSPBUF，产生中断
			*mrd++ = SSPBUF;      //读出数据
			SSPSTAT &= ~ackdt;      //ACKDT=0，主控接收器应答状态
			SSPCON |= acken; 	 //ACKEN=1，主控接收器应答使能
			i2c_int();           //ACK时序完成，产生中断
		}
//读操作要有正确的结束时序.接收到最后一个数据字节后，主控接收器不给被控发送器应答，使之释放数据线，以便主控器stop
	SSPCON |= rcen;    //RCEN=1;
	i2c_int();
	*mrd = SSPBUF;
	SSPSTAT |= ackdt;      //ACKDT=1
	SSPCON |= acken; 	 //ACKEN=1
  	i2c_int();	

	i2c_stop();         //读完，结束

	return I2C_SUCCESS;
}

/***********************************************************************************
*    函数名：i2c_random_readwrong  
*    函数功能：自由读启动的连续读，主控接收器不应答，后续EEPROM数据将不会被读出
*    入口参数：ctrl为控制字节，包括寻址命令码和读/写操作命令码
*     eepromaddr为数据地址
*     mrd为主接收数据缓冲器首址
*     rlength为待读数据字节数
*	  EepromType为从器件类型，枚举变量，分别为2401~24256
*	 出口参数：返回0表示发生错误，返回1表示读取成功
************************************************************************************/
bit i2c_random_readwrong(INT8U ctrl, INT16U eepromaddr, INT8U *mrd, INT16U rlength, eetype EepromType)
{
	i2c_init();
	i2c_start();
	SSPBUF = ctrl;        //假写操作。ex:EEPROM，0xA0 
	i2c_int(); 
    
  if ((SSPSTAT & ackstat) == ackstat)        //if(ACKSTAT)
    ERROR();   //无应答，错误！
  else if ((SSPSTAT & ackstat) != ackstat)  //if(!ACKSTAT)
	{	
		if(EepromType > FM24C16)
		{
		    SSPBUF = eepromaddr>>8;              //取EEPROM数据地址高字节
		    i2c_int(); //发送
		}
		SSPBUF = (INT8U)eepromaddr;            //对FM24C08，10位数据地址高2位在控制字节中，低8位发送一字节即可
		i2c_int(); //发送
    }
    
	if((SSPSTAT & ackstat) == ackstat)        //if(ACKSTAT)
    	ERROR();  //无应答，错误！
	else if((SSPSTAT & ackstat) != ackstat)   //if(!ACKSTAT)
    	i2c_restart();
	SSPBUF = (ctrl^0x01);       //读操作命令码,ex:EEPROM，0xA1
	i2c_int();
    
	if((SSPSTAT & ackstat) == ackstat)        //if(ACKSTAT) 应答
		ERROR();
	else if((SSPSTAT & ackstat) != ackstat)   //if(!ACKSTAT)
	while(rlength--)     //读rlength字节数据
	{
		SSPCON |= rcen;   //RCEN=1;
		i2c_int();
		*mrd++ = SSPBUF;
		SSPSTAT |= ackdt;      //ACKDT=1，第一字节即不应答
		SSPCON |= acken; 	 //ACKEN=1，主控接收器应答使能
		i2c_int();
	}
	i2c_stop();         //读完，结束

	return I2C_SUCCESS;
}
