#include "init.h"

 INT8U near T0_Over;
 INT8U near Ic_Comm_RxBuf[U7816BUFLEN]	;	//7816���ջ�����
 INT8U near Ic_Comm_RxPtr		;	//7816����ָ��

 INT8U near Ic_Comm_TxBuf[U7816BUFLEN]	;	//7816���ͻ�����
 INT8U near Ic_Comm_TxPtr		;	//7816����ָ��
 INT16U near dlength;



/************************************************************************/
/* ��������INIT_7816_PAD													*/
/* ����˵����7816�ܽų�ʼ���ӳ��� 							*/
/* ��ڲ�����N/A													*/
/* ���ڲ�����N/A													*/
/************************************************************************/
void INIT_7816_PAD(void)
{
	PEFCR1 = 0;
	PEFCR2 = 0;
	PEFCR2 |= BIT0 | BIT1 | BIT2;//{PEFCR2[0]:PEFCR1[0]}=2'b10
	AFSELE |= BIT0 | BIT1 | BIT2;//1��7816clk,7816io0��7816io1
	
	PEPPEN = 0;
	PEPPEN |= BIT3;//ʹ��PE3�����칦��
	PEFCR1 |= BIT3;//{PEFCR2[3]:PEFCR1[3]}=2'b01,ʹ��PE3���,7816rst
		
}

/************************************************************************/
/* ��������INIT_7816												    */
/* ����˵����7816ģ���ʼ��     					    		*/
/* ��ڲ�����N/A													    */
/* ���ڲ�����N/A													    */
/************************************************************************/
void INIT_7816(void)
{
		//����ʹ��|����ʹ��|ʱ�����ʹ��|ͨ��ѡ��|����ǿ����������Ч|--|ǿ����ʹ��|������ʹ��|
	//|UTXEN|URXEN|UCLKOUT_EN|PSEL| HPUAT0|--|HPUEN0|LPUEN0|
	U7816CTRL0 |= utxen | urxen | uclkout_en | psel | hpuen;// 0xE2;
	//U7816CTRL0 = 0xE3;
	//|--|--|--|--|ͨ��1����ǿ������Ч|--|ͨ��1ǿ����ʹ��|ͨ��1������ʹ��|
	//|--|--|--|--|HPUAT1|--|HPUEN1|LPUEN1|
	U7816CTRL1 |= hpuen1;// 0x02;
	//U7816CTRL1 = 0x08;
	//|BGTʹ��|�Զ��ط�����|��żУ������||GuardTime����|������żУ�������|������żУ�������|�������
	//|BGTEN|REP_T|PAR|PAR|FREN|TREPEN|RREPEN|DICONV|
	U7816FRMCTRL0 = 0x0e;
	//U7816FRMCTRL0 |= par_even | rrepen | trepen;////  0x06;fren
	//|--|--|--|��д����λ||error signal���|error          signal��guardtime���|
	//|--|--|--|RFU1|RFU0|ERSW1|ERSW0|ERSGD|
	U7816FRMCTRL1 = 0x00;
	//U7816FRMCTRL1 |= err_2etu;// | ersgd;
	//����ʱ�����EGT��Ŀ(��etuΪ��λ)
	U7816EGTCTRL = 0x00;
	//ʱ�������Ƶ���ƣ�f7816=ϵͳʱ��/(CLKDIV+1)
	//|--|--|--|--|CLKDIV3|CLKDIV2|CLKDIV1|CLKDIV0|
	CLK_DIV = 0x01;       
	//|���η�Ƶ��|Ԥ��Ƶ��������Baud=f7816/((prediv+1))  PDIV = {pdivh,pdivl}
	//|---|---|---|---|Pdivh3|Pdivh2|Pdivh1|Pdivh0|
	//|Pdivl7|Pdivl6|Pdivl5|Pdivl4|Pdivl3|Pdivl2|Pdivl1|Pdivl0|
	PRE_DIVH = 0x01;
	PRE_DIVL = 0x73;   //8M mclk,9600

//	U7816INTEN = 0x07;
	U7816PRIMARYSTATUS = 0;
	U7816ERRSTATUS = 0;
	U7816SECONDSTATUS = 0;
}

/*void send_byte(uchar value)
{
	U7816TXBUF = value;
	while(!(U7816PRIMARYSTATUS & txflag)){}
}

/*void receive_byte(INT8U *Data)
{
	while(!(U7816PRIMARYSTATUS & rxflag)){}
	*Data = U7816RXBUF; 
}*/

// void Delay_Ten_Per_etu( INT16U time )
// {
// 	TR0 = 0;		//��ʱ��0������ֹ
// 	TMOD &= 0xF0;
// 	TMOD |= 0x01; 	//��ʱ��0����ģʽ1: 16bits timer.
//    // TH0 = ((0xFFFF-time+26)>>8);	
// 	//TL0 = ((0xFFFF-time+26)&0x00FF);
//   TH0 = ((0xFFFF-63*time)>>8);	
// 	TL0 = ((0xFFFF-63*time)&0x00FF);

// 	TR0 = 1;		//������ʱ��0
// 	ET0 = 1;		//����ʱ��0�ж�

// 	T0_Over = 0;
// 	while( T0_Over == 0 );
// }

void Delay_Ten_Per_etu( INT16U time )
{
	INT16U i,j;
	for (i=0;i<time;i++)
	{
 		WDT_CLR();
		for (j=0;j<68;j++)
		{
			_nop_();
		}
	}
}

INT8U Rcv_7816_Byte( INT8U *Data )
{
	INT16U near i;
	for(i=0;i<20000;i++ )
	{                //|--|--|--|--|--|ERROR_FLAG|TX_FLAG|RX_FLAG|
		if( U7816PRIMARYSTATUS & rxflag ) //RX_FLAG�Ѿ���λ
		{
			//UartTx_0(0x11);
			//delay(10);
			*Data = U7816RXBUF;            		
			return RCV_SUCCESS; //�ɹ�
		}

		if( U7816PRIMARYSTATUS & errflag) //ERROR_FLAG�Ѿ���λ
		{
	   // temper =U7816RXBUF;
	   			//|--|--|--|--|TPAR_ERR|RPAR_ERR|FRAME_ERR|OE_ERR|
			switch( U7816ERRSTATUS ) //���б�־λ: Ӳ����λ��д0����
			{
				case oeerr: //������������־λ
					break;

				case framerr: //����֡��ʽ�����־λ
					break;

				case rparerr: //����������żУ������־λ
					break;

				case tparerr: //����������żУ������־λ
					break;

				default:
					break;
			}
		return RCV_FAIL; //error
		}


		WDT_CLR();

	}

	return RCV_TOOLONG;
	//--------------------------------------------
}
INT8U Snd_7816_Byte( INT8U Data )
{
	//U7816INTEN |= lsie|txie;	                 //�����ж�ʹ��|--|--|--|--|--|LSIE|TXIE|RXIE|
INT16U near i;
	U7816TXBUF = Data;

	for(i=0;i<20000;i++ )                        //while( T0_Over == 0 )
	{
		if( U7816PRIMARYSTATUS & txflag )	//TX_FLAG��λ
		{
			return SND_SUCCESS; //�ɹ�
		}

		if( U7816PRIMARYSTATUS & 0x04)	//ERROR_FLAG��λ
		{
			switch( U7816ERRSTATUS ) //���б�־λ: Ӳ����λ��д0����
			{
				case 0x01: //������������־λ
					break;

				case 0x02: //����֡��ʽ�����־λ
					break;

				case 0x04: //����������żУ������־λ
					break;

				case 0x08: //����������żУ������־λ
					break;

				default:
					break;
			}
			return SND_FAIL; //error
		}

	WDT_CLR();
	}

	return SND_TOOLONG;
}

INT8U ColdReset(INT8U reset_rxbuf[])
{
	INT8U n, Result, Temp_ATR, T0, TD1, TD2;//, tck;

	Ic_Comm_RxPtr = 0x00; //��λ��Ϣ������ָ��

	memset( Ic_Comm_RxBuf, 0x00, U7816BUFLEN );  
	memset( reset_rxbuf, 0x00, U7816BUFLEN );  

	RST7816_0();
	Delay_Ten_Per_etu(1);  //1=1etu
	U7816CTRL0 |= uclkout_en;  //-------- T0ʱ��, CLK���ʹ�� --------
	Delay_Ten_Per_etu(6);//6=6etu, 400clk = 100uS,�ȴ�Vcc��ѹ�ȶ�
	
	
//	Delay_Ten_Per_etu( 1 ); //400��clk, 0.1*108 = 10.8etu, (40000~45000��CLK)

	//-------- T1ʱ��, RST����ߵ�ƽ --------
	RST7816_1();

	while((U7816SECONDSTATUS & rxbusy) == 0){}
	//--------------- Receive TS ----------------
	Result = Rcv_7816_Byte( &Temp_ATR ); //receive TS.
		
// 	UartTx_0(Result); 
// 	UartTx_0(Temp_ATR);		
		
	if( Result == RCV_FAIL)  //receive failure.
	{ 
		return RCV_FAIL; 
	}

	Ic_Comm_RxBuf[ Ic_Comm_RxPtr++ ] = Temp_ATR;

	
	if( Temp_ATR == 0x3B );//����
	else if( Temp_ATR == 0x3F);	//����
	else
	{
		return TS_WRONG;	//�쳣: �����TS
	}
	
	//--------------- Receive T0 ----------------
	Result = Rcv_7816_Byte( &T0 ); //receive T0
	if( Result == RCV_FAIL ) //receive failure.
	{ 
		return RCV_FAIL; 
	}
	//UartTx_0(0x01); 
//	UartTx_0(T0);           
//		UartTx_0(Result);
//	UartTx_0(0x02); 
	Ic_Comm_RxBuf[ Ic_Comm_RxPtr++ ] = T0;

	if( (T0&0x10) > 0 )      //T0 = |b8|b7|b6|b5|b4|b3|b2|b1,���b5==1��TA1����
	{
		//TA1 is existed. F and D
		Result = Rcv_7816_Byte( &Temp_ATR ); //receive TA1
		if( Result == RCV_FAIL ) //receive failure.
		{ 
			return RCV_FAIL; 
		}
		Ic_Comm_RxBuf[ Ic_Comm_RxPtr++ ] = Temp_ATR;

	}

	if( (T0&0x20) > 0 )      //���b6==1��TB1����
	{
		//TB1 is existed.IC���ı�̵�ѹ������̵���
		Result = Rcv_7816_Byte( &Temp_ATR ); //receive TB1
		if( Result == RCV_FAIL ) //receive failure.
		{ 
			return RCV_FAIL; 
		}
		Ic_Comm_RxBuf[ Ic_Comm_RxPtr++ ] = Temp_ATR;
	}

	if( (T0&0x40) > 0 )    //���b7==1��TC1����
	{
		//TC1 is existed. TC1 ���� N ֵ, N ���ڱ�ʾ���ӵ���С����ʱ��Ķ��Ᵽ��ʱ��,
		//�˴�����С����ʱ���ʾ���ն˷��͵�IC ����, ��Ϊ������Ϣ���������������ַ�����ʼλ������֮���ʱ��
		Result = Rcv_7816_Byte( &Temp_ATR ); //receive TC1
		if( Result == RCV_FAIL ) //receive failure.
		{ 
			return RCV_FAIL; 
		}
		Ic_Comm_RxBuf[ Ic_Comm_RxPtr++ ] = Temp_ATR;
	}

	if( (T0&0x80) > 0 )   // ���b8==1��TD1����. TD1 = |b8|b7|b6|b5|b4|b3|b2|b1
	{
		//TD1 is existed.
		Result = Rcv_7816_Byte( &TD1 ); //receive TD1
		if( Result == RCV_FAIL ) //receive failure.
		{ 
			return RCV_FAIL; 
		}
		else
		{
			Ic_Comm_RxBuf[ Ic_Comm_RxPtr++ ] = TD1;

			if( (TD1&0x0F) > 0x01 )	    //TD1��4λΪTֵ
			{
				return 4;	//�����Э��
			}
//			if((TD1 & 0x0F) != 0x00)
//			{
//				tck = 1;
//			}
			
			if( (TD1&0x10) > 0x00 )     
			{
				Result = Rcv_7816_Byte( &Temp_ATR ); //receive TA2
				if( Result == RCV_FAIL ) //receive failure.
				{ 
					return RCV_FAIL ; 
				}
				Ic_Comm_RxBuf[ Ic_Comm_RxPtr++ ] = Temp_ATR;
			}

			if( (TD1&0x20) > 0x00 )
			{
				Result = Rcv_7816_Byte( &Temp_ATR ); //receive TB2
				if( Result == RCV_FAIL ) //receive failure.
				{ 
					return RCV_FAIL ; 
				}
				Ic_Comm_RxBuf[ Ic_Comm_RxPtr++ ] = Temp_ATR;
			}
			
			if( (TD1&0x40) > 0x00 )
			{
				Result = Rcv_7816_Byte( &Temp_ATR ); //receive TC2
				if( Result == RCV_FAIL ) //receive failure.
				{ 
					return RCV_FAIL ; 
				}
				Ic_Comm_RxBuf[ Ic_Comm_RxPtr++ ] = Temp_ATR;
			}
			
			if( (TD1&0x80) > 0x00 )
			{
				Result = Rcv_7816_Byte( &TD2 ); //receive TD2
				if( Result == RCV_FAIL ) //receive failure.
				{ 
					return RCV_FAIL ; 
				}
				Ic_Comm_RxBuf[ Ic_Comm_RxPtr++ ] = TD2;

				if( ((TD2&0x0F) != 0x01) || ((TD2&0x0F) != 0x0E) )
				{
					return 4;	//�����Э��
				}
		//		if((TD2 & 0x0F) != 0x00)
		//		{
		//			tck = 1;
		//		}

				if( (TD2&0x10) > 0x00 )
				{
					Result = Rcv_7816_Byte( &Temp_ATR ); //receive TA3
					if( Result == RCV_FAIL ) //receive failure.
					{ 
						return RCV_FAIL ; 
					}
					Ic_Comm_RxBuf[ Ic_Comm_RxPtr++ ] = Temp_ATR;
				}

				if( (TD2&0x20) > 0x00 )
				{
					Result = Rcv_7816_Byte( &Temp_ATR ); //receive TB3
					if( Result == RCV_FAIL ) //receive failure.
					{ 
						return RCV_FAIL ; 
					}
					Ic_Comm_RxBuf[ Ic_Comm_RxPtr++ ] = Temp_ATR;
				}
				
				if( (TD2&0x40) > 0x00 )
				{
					Result = Rcv_7816_Byte( &Temp_ATR ); //receive TC3
					if( Result == RCV_FAIL ) //receive failure.
					{ 
						return RCV_FAIL ; 
					}
					Ic_Comm_RxBuf[ Ic_Comm_RxPtr++ ] = Temp_ATR;
				}
			}
		}
	}	

	//---------------- receive history data. -------------------
	for( n=0x00; n<(T0&0x0F); n++ )
	{
		Result = Rcv_7816_Byte( &Temp_ATR );
		if( Result == RCV_FAIL ) //;Receive fail
		{	
			return RCV_FAIL ;	
		}
		Ic_Comm_RxBuf[ Ic_Comm_RxPtr++ ] = Temp_ATR;
	}

	for(n=0; n<33; n++)                //ATR�33�ֽ�
	{
		reset_rxbuf[n] = Ic_Comm_RxBuf[n];
		
		//UartTx_0(Ic_Comm_RxBuf[n]);
		//delay(10);
	}
		

	return RESET_FINISH;	//;Receive success
}

//command-response pairs
INT8U CardTxRxFrame( INT8U Lc, INT8U Le )
{
	INT8U result, Temp_SW1, Temp_SW2, Proc_Byte;
	INT8U k, Temp_Data;
	INT8U ins, ins1, fins, fins1;
	ins = Ic_Comm_TxBuf[1];                    //INS  
	fins = ins + 1;				//INS+1       
	ins1 = ins ^ 0xff;			//INS��     
	fins1 = fins ^ 0xff;                 //INS+1��   

	memset( Ic_Comm_RxBuf, 0x00, U7816BUFLEN );               //���ջ���������
	
// 	UartTx_0(Lc);
// 	UartTx_0(Le);
// 	UartTx_0(0x02);	
	
//	Delay_Ten_Per_etu( 40 ); //0.1*40 = 4etu, �˴�Ĭ��ΪExtTime=0,���ExtTime��Ϊ0�����ڴ�������ʱExtTime��etu��

	for( k=0; k<5; k++ )	//send command header (5bytes)
	{	
		result = Snd_7816_Byte( Ic_Comm_TxBuf[ k ] );	//;ȡ����������
		if( result == SND_FAIL )
			{ return SND_FAIL; } //send failure.

	}
	
//  	UartTx_0(result);
// 	UartTx_0(0x03);	
	
	
	for( ;; )
	{
		WDT_CLR();

//		U7816FRAMCTRL0 |= 0x10;
		result = Rcv_7816_Byte( &Proc_Byte ); //receive procedure byte.
		
// 		receive_byte(&Proc_Byte);
// 		UartTx_0(result);
//  		UartTx_0(Proc_Byte);
//  		UartTx_0(0x03);
		
		if( result == RCV_FAIL)
			{ return RCV_FAIL; } //receive failure.



		if( Proc_Byte == 0x60 ) //Null byte
		{
			//do nothing. continue to receive.
		}
		else if( (Proc_Byte == ins) ||(Proc_Byte == fins) ) //procedure byte = INS
		{
			if( k < (Lc+5) )	//send remaining data, then wait for procedure byte.
			{
				for( ; k<(Lc+5); )
				{
					result = Snd_7816_Byte( Ic_Comm_TxBuf[ k++ ] ); //;ȡ����������
					if( result == SND_FAIL ) 
						{ return SND_FAIL ; }	//send failure.
				}
			}
			else
			{
				//;�����ֽ���֮ǰ�ѽ��գ�ֻ����2��״̬�ֽڼ��ɡ�
				for( Ic_Comm_RxPtr=0x00; Ic_Comm_RxPtr<( Le+2 ); Ic_Comm_RxPtr++ )
				{
					result = Rcv_7816_Byte( &Temp_Data );
					if( result == RCV_FAIL)
						{ return RCV_FAIL; }	//;Receive fail			
					Ic_Comm_RxBuf[ Ic_Comm_RxPtr ] = Temp_Data;
				}				
				return CMD_RESPONSE;
			}
		}
		else if( (Proc_Byte == ins1)||(Proc_Byte == fins1) ) //procedure byte = (INS XOR FFH)
		{
			if( k < (Lc+5) )	//send next one data, then wait for procedure byte.
			{
				result = Snd_7816_Byte( Ic_Comm_TxBuf[ k++ ] ); //;ȡ����������
				if( result == SND_FAIL )
					{ return SND_FAIL ;}	//send failure.
			}
			else
			{
				//;�����ֽ���֮ǰ�ѽ��գ�ֻ����2��״̬�ֽڼ��ɡ�
				for( Ic_Comm_RxPtr=0x00; Ic_Comm_RxPtr<( Le+2 ); Ic_Comm_RxPtr++ )
				{
					result = Rcv_7816_Byte( &Temp_Data );
					if( result == RCV_FAIL)	
						{return RCV_FAIL;}	//;Receive fail			
					Ic_Comm_RxBuf[ Ic_Comm_RxPtr ] = Temp_Data;
				}
				return CMD_RESPONSE;
			}
		}
		else
		{
			Temp_SW1 = ( Proc_Byte & 0xF0 );
			
			if( (Temp_SW1 == 0x60) || (Temp_SW1 == 0x90) ) //procedure byte = 6X or 9X (ex. 60H)	
			{
				result = Rcv_7816_Byte( &Temp_SW2 ); //receive sw2.
				if( result == RCV_FAIL )
					{ return RCV_FAIL; } //receive failure.
		
				// 61H: send get response command, P3<=XX
				// 6CH: send last command again, P3 = XX;
				// other 6X or 9X, this communication is over.

				//'9000' command normally completed
				//'6E00' CLA not supported
				//'6D00' CLA supported, but INS not programmed or invalid
				//'6B00' CLA INS supported, but P1 P2 incorrect
				//'6700' CLA INS P1 P2 supported, but P3 incorrect
				//'6F00' command not supported and no precise diagnosis given
		
				Ic_Comm_RxBuf[0] = Proc_Byte;
				Ic_Comm_RxBuf[1] = Temp_SW2;
				Ic_Comm_RxPtr = 2;
				
				return CMD_RESPONSE;
			}
			else
			{
				return CMD_RESPONSE_ERROR;
			}
		}
	}

	return CMD_RESPONSE_ERROR;
}


INT8U U7816_comm1( INT8U near comm_rxbuf[], INT8U near WriteESAM[] )
{
	INT8U result,i;
//	memset( Ic_Comm_RxBuf, 0x00, U7816BUFLEN ); 
	memset( Ic_Comm_TxBuf, 0x00, U7816BUFLEN );
//	memset( comm_rxbuf, 0x00, U7816BUFLEN );  

	
	for( Ic_Comm_TxPtr=0x00; Ic_Comm_TxPtr<(WriteESAM[4]+5); Ic_Comm_TxPtr++ )
	{
		Ic_Comm_TxBuf[ Ic_Comm_TxPtr ] = WriteESAM[ Ic_Comm_TxPtr ];
   	//UartTx_0(Ic_Comm_TxBuf[ Ic_Comm_TxPtr ]);		
	}

	result = CardTxRxFrame( Ic_Comm_TxBuf[4], 0x00 );
	
// 	UartTx_0(result);
// 	UartTx_0(0x01);

	if( result != CMD_RESPONSE)
	{
		if( Ic_Comm_RxPtr == 2 )
		{
			if( Ic_Comm_RxBuf[0] == 0x61 )
			{
				// 61H: send get response command, P3<=XX
				Ic_Comm_TxPtr = 0x00;
				Ic_Comm_TxBuf[ Ic_Comm_TxPtr++ ] = 0x00;	//CLA
				Ic_Comm_TxBuf[ Ic_Comm_TxPtr++ ] = 0xC0;	//INS
				Ic_Comm_TxBuf[ Ic_Comm_TxPtr++ ] = 0x00;	//P1
				Ic_Comm_TxBuf[ Ic_Comm_TxPtr++ ] = 0x00;	//P2
				
				Ic_Comm_TxBuf[ Ic_Comm_TxPtr++ ] = Ic_Comm_RxBuf[1];	//Lc

				result = CardTxRxFrame( 0x00, Ic_Comm_RxBuf[1] );
				if( result == CMD_RESPONSE )
				{
				}
			}
			else if( Ic_Comm_RxBuf[0] == 0x6C )
			{
				// 6CH: send last command again, P3 = XX;
				Ic_Comm_TxPtr = 4;
				Ic_Comm_TxBuf[ Ic_Comm_TxPtr++ ] = Ic_Comm_RxBuf[1];	//Lc	
				result = CardTxRxFrame( 0x02, Ic_Comm_RxBuf[1] );
				if( result == CMD_RESPONSE )
				{
				}
			}
			else if( Ic_Comm_RxBuf[0] == 0x90 )
			{
				Ic_Comm_TxPtr = 0x00;
				Ic_Comm_TxBuf[ Ic_Comm_TxPtr++ ] = 0x00;	//CLA
				Ic_Comm_TxBuf[ Ic_Comm_TxPtr++ ] = 0xB0;	//INS
				Ic_Comm_TxBuf[ Ic_Comm_TxPtr++ ] = 0x81;	//P1
				Ic_Comm_TxBuf[ Ic_Comm_TxPtr++ ] = 0x00;	//P2
			
				Ic_Comm_TxBuf[ Ic_Comm_TxPtr++ ] = 0x03;	//Lc
			
				result = CardTxRxFrame( 0x00, 0x03 );
			}
		}
		else
		{
	//		return OVER;
		}
	}
	for(i=0; i<0x02; i++)
	{
		comm_rxbuf[i] = Ic_Comm_RxBuf[i];
// 		UartTx_0(Ic_Comm_RxBuf[i]);
		delay(10);

	}
		
	return OVER;
}

INT8U Snd_7816_Byte_DMA( INT8U Data )
{
	//U7816INTEN |= lsie|txie;	                 //�����ж�ʹ��|--|--|--|--|--|LSIE|TXIE|RXIE|
INT16U near i;
	
	DMA_Start(CH0,(INT16U)&Data,1,u7816_tx);  //U7816TXBUF = Data;

	for(i=0;i<20000;i++ )                        //while( T0_Over == 0 )
	{
		if( U7816PRIMARYSTATUS & txflag )	//TX_FLAG��λ
		{
			return SND_SUCCESS; //�ɹ�
		}

		if( U7816PRIMARYSTATUS & 0x04)	//ERROR_FLAG��λ
		{
			switch( U7816ERRSTATUS ) //���б�־λ: Ӳ����λ��д0����
			{
				case 0x01: //������������־λ
					break;

				case 0x02: //����֡��ʽ�����־λ
					break;

				case 0x04: //����������żУ������־λ
					break;

				case 0x08: //����������żУ������־λ
					break;

				default:
					break;
			}
			return SND_FAIL; //error
		}

	WDT_CLR();
	}

	return SND_TOOLONG;
}

INT8U Rcv_7816_Byte_DMA( INT8U near *Data )
{
	INT16U near i;
	for(i=0;i<20000;i++ )
	{                //|--|--|--|--|--|ERROR_FLAG|TX_FLAG|RX_FLAG|
		if( U7816PRIMARYSTATUS & rxflag ) //RX_FLAG�Ѿ���λ
		{
			DMA_Start(CH0,(INT16U)Data,1,u7816_rx);  //DMA���� *Data = U7816RXBUF;  

			return RCV_SUCCESS; //�ɹ�
		}

		if( U7816PRIMARYSTATUS & errflag) //ERROR_FLAG�Ѿ���λ
		{
	   // temper =U7816RXBUF;
	   			//|--|--|--|--|TPAR_ERR|RPAR_ERR|FRAME_ERR|OE_ERR|
			switch( U7816ERRSTATUS ) //���б�־λ: Ӳ����λ��д0����
			{
				case oeerr: //������������־λ
					break;

				case framerr: //����֡��ʽ�����־λ
					break;

				case rparerr: //����������żУ������־λ
					break;

				case tparerr: //����������żУ������־λ
					break;

				default:
					break;
			}
		return RCV_FAIL; //error
		}


		WDT_CLR();

	}

	return RCV_TOOLONG;
	//--------------------------------------------
}

//command-response pairs
INT8U CardTxRxFrame_DMA( INT8U Lc, INT8U Le )
{
	INT8U result, Temp_SW1, Temp_SW2, Proc_Byte;
	INT8U k, Temp_Data;
	INT8U ins, ins1, fins, fins1;
	ins = Ic_Comm_TxBuf[1];                    //INS  
	fins = ins + 1;				//INS+1       
	ins1 = ins ^ 0xff;			//INS��     
	fins1 = fins ^ 0xff;                 //INS+1��   

	memset( Ic_Comm_RxBuf, 0x00, U7816BUFLEN );               //���ջ���������
	
// 	UartTx_0(Lc);
// 	UartTx_0(Le);
// 	UartTx_0(0x02);	
	
//	Delay_Ten_Per_etu( 40 ); //0.1*40 = 4etu, �˴�Ĭ��ΪExtTime=0,���ExtTime��Ϊ0�����ڴ�������ʱExtTime��etu��

	for( k=0; k<5; k++ )	//send command header (5bytes)
	{	
		result = Snd_7816_Byte_DMA( Ic_Comm_TxBuf[ k ] );	//;ȡ����������
		if( result == SND_FAIL )
			{ return SND_FAIL; } //send failure.

	}
	
//  	UartTx_0(result);
// 	UartTx_0(0x03);	
	
	
	for( ;; )
	{
		WDT_CLR();

//		U7816FRAMCTRL0 |= 0x10;
		result = Rcv_7816_Byte_DMA( &Proc_Byte ); //receive procedure byte.
		
// 		receive_byte(&Proc_Byte);
// 		UartTx_0(result);
//  		UartTx_0(Proc_Byte);
//  		UartTx_0(0x03);
		
		if( result == RCV_FAIL)
			{ return RCV_FAIL; } //receive failure.



		if( Proc_Byte == 0x60 ) //Null byte
		{
			//do nothing. continue to receive.
		}
		else if( (Proc_Byte == ins) ||(Proc_Byte == fins) ) //procedure byte = INS
		{
			if( k < (Lc+5) )	//send remaining data, then wait for procedure byte.
			{
				for( ; k<(Lc+5); )
				{
					result = Snd_7816_Byte_DMA( Ic_Comm_TxBuf[ k++ ] ); //;ȡ����������
					if( result == SND_FAIL ) 
						{ return SND_FAIL ; }	//send failure.
				}
			}
			else
			{
				//;�����ֽ���֮ǰ�ѽ��գ�ֻ����2��״̬�ֽڼ��ɡ�
				for( Ic_Comm_RxPtr=0x00; Ic_Comm_RxPtr<( Le+2 ); Ic_Comm_RxPtr++ )
				{
					result = Rcv_7816_Byte_DMA( &Temp_Data );
					if( result == RCV_FAIL)
						{ return RCV_FAIL; }	//;Receive fail			
					Ic_Comm_RxBuf[ Ic_Comm_RxPtr ] = Temp_Data;
				}				
				return CMD_RESPONSE;
			}
		}
		else if( (Proc_Byte == ins1)||(Proc_Byte == fins1) ) //procedure byte = (INS XOR FFH)
		{
			if( k < (Lc+5) )	//send next one data, then wait for procedure byte.
			{
				result = Snd_7816_Byte_DMA( Ic_Comm_TxBuf[ k++ ] ); //;ȡ����������
				if( result == SND_FAIL )
					{ return SND_FAIL ;}	//send failure.
			}
			else
			{
				//;�����ֽ���֮ǰ�ѽ��գ�ֻ����2��״̬�ֽڼ��ɡ�
				for( Ic_Comm_RxPtr=0x00; Ic_Comm_RxPtr<( Le+2 ); Ic_Comm_RxPtr++ )
				{
					result = Rcv_7816_Byte_DMA( &Temp_Data );
					if( result == RCV_FAIL)	
						{return RCV_FAIL;}	//;Receive fail			
					Ic_Comm_RxBuf[ Ic_Comm_RxPtr ] = Temp_Data;
				}
				return CMD_RESPONSE;
			}
		}
		else
		{
			Temp_SW1 = ( Proc_Byte & 0xF0 );
			
			if( (Temp_SW1 == 0x60) || (Temp_SW1 == 0x90) ) //procedure byte = 6X or 9X (ex. 60H)	
			{
				result = Rcv_7816_Byte_DMA( &Temp_SW2 ); //receive sw2.
				if( result == RCV_FAIL )
					{ return RCV_FAIL; } //receive failure.
		
				// 61H: send get response command, P3<=XX
				// 6CH: send last command again, P3 = XX;
				// other 6X or 9X, this communication is over.

				//'9000' command normally completed
				//'6E00' CLA not supported
				//'6D00' CLA supported, but INS not programmed or invalid
				//'6B00' CLA INS supported, but P1 P2 incorrect
				//'6700' CLA INS P1 P2 supported, but P3 incorrect
				//'6F00' command not supported and no precise diagnosis given
		
				Ic_Comm_RxBuf[0] = Proc_Byte;
				Ic_Comm_RxBuf[1] = Temp_SW2;
				Ic_Comm_RxPtr = 2;
				
				return CMD_RESPONSE;
			}
			else
			{
				return CMD_RESPONSE_ERROR;
			}
		}
	}

	return CMD_RESPONSE_ERROR;
}


INT8U U7816_comm1_DMA( INT8U near comm_rxbuf[], INT8U near WriteESAM[] )
{
	INT8U result,i;
//	memset( Ic_Comm_RxBuf, 0x00, U7816BUFLEN ); 
	memset( Ic_Comm_TxBuf, 0x00, U7816BUFLEN );
//	memset( comm_rxbuf, 0x00, U7816BUFLEN );  

	
	for( Ic_Comm_TxPtr=0x00; Ic_Comm_TxPtr<(WriteESAM[4]+5); Ic_Comm_TxPtr++ )
	{
		Ic_Comm_TxBuf[ Ic_Comm_TxPtr ] = WriteESAM[ Ic_Comm_TxPtr ];
   	//UartTx_0(Ic_Comm_TxBuf[ Ic_Comm_TxPtr ]);		
	}

	result = CardTxRxFrame_DMA( Ic_Comm_TxBuf[4], 0x00 );
	
// 	UartTx_0(result);
// 	UartTx_0(0x01);

	if( result != CMD_RESPONSE)
	{
		if( Ic_Comm_RxPtr == 2 )
		{
			if( Ic_Comm_RxBuf[0] == 0x61 )
			{
				// 61H: send get response command, P3<=XX
				Ic_Comm_TxPtr = 0x00;
				Ic_Comm_TxBuf[ Ic_Comm_TxPtr++ ] = 0x00;	//CLA
				Ic_Comm_TxBuf[ Ic_Comm_TxPtr++ ] = 0xC0;	//INS
				Ic_Comm_TxBuf[ Ic_Comm_TxPtr++ ] = 0x00;	//P1
				Ic_Comm_TxBuf[ Ic_Comm_TxPtr++ ] = 0x00;	//P2
				
				Ic_Comm_TxBuf[ Ic_Comm_TxPtr++ ] = Ic_Comm_RxBuf[1];	//Lc

				result = CardTxRxFrame_DMA( 0x00, Ic_Comm_RxBuf[1] );
				if( result == CMD_RESPONSE )
				{
				}
			}
			else if( Ic_Comm_RxBuf[0] == 0x6C )
			{
				// 6CH: send last command again, P3 = XX;
				Ic_Comm_TxPtr = 4;
				Ic_Comm_TxBuf[ Ic_Comm_TxPtr++ ] = Ic_Comm_RxBuf[1];	//Lc	
				result = CardTxRxFrame_DMA( 0x02, Ic_Comm_RxBuf[1] );
				if( result == CMD_RESPONSE )
				{
				}
			}
			else if( Ic_Comm_RxBuf[0] == 0x90 )
			{
				Ic_Comm_TxPtr = 0x00;
				Ic_Comm_TxBuf[ Ic_Comm_TxPtr++ ] = 0x00;	//CLA
				Ic_Comm_TxBuf[ Ic_Comm_TxPtr++ ] = 0xB0;	//INS
				Ic_Comm_TxBuf[ Ic_Comm_TxPtr++ ] = 0x81;	//P1
				Ic_Comm_TxBuf[ Ic_Comm_TxPtr++ ] = 0x00;	//P2
			
				Ic_Comm_TxBuf[ Ic_Comm_TxPtr++ ] = 0x03;	//Lc
			
				result = CardTxRxFrame_DMA( 0x00, 0x03 );
			}
		}
		else
		{
	//		return OVER;
		}
	}
	for(i=0; i<0x02; i++)
	{
		comm_rxbuf[i] = Ic_Comm_RxBuf[i];
// 		UartTx_0(Ic_Comm_RxBuf[i]);
		delay(10);

	}
		
	return OVER;
}


INT8U U7816_comm2( INT8U near comm_rxbuf[], INT8U near ReadESAM[] )
{
	unsigned char result,i;
	memset( Ic_Comm_RxBuf, 0x00, U7816BUFLEN );  
//	memset( comm_rxbuf, 0x00, U7816BUFLEN );  
	
	for( Ic_Comm_TxPtr=0x00; Ic_Comm_TxPtr<5; Ic_Comm_TxPtr++ )
	{
		Ic_Comm_TxBuf[ Ic_Comm_TxPtr ] = ReadESAM[ Ic_Comm_TxPtr ];
	}
	
	result = CardTxRxFrame( 0x00, Ic_Comm_TxBuf[4] );

	if( result  != CMD_RESPONSE )
	{
		if( Ic_Comm_RxPtr == 2 )
		{
			if( Ic_Comm_RxBuf[0] == 0x61 )
			{
				// 61H: send get response command, P3<=XX
				Ic_Comm_TxPtr = 0x00;
				Ic_Comm_TxBuf[ Ic_Comm_TxPtr++ ] = 0x00;	//CLA
				Ic_Comm_TxBuf[ Ic_Comm_TxPtr++ ] = 0xC0;	//INS
				Ic_Comm_TxBuf[ Ic_Comm_TxPtr++ ] = 0x00;	//P1
				Ic_Comm_TxBuf[ Ic_Comm_TxPtr++ ] = 0x00;	//P2
				
				Ic_Comm_TxBuf[ Ic_Comm_TxPtr++ ] = Ic_Comm_RxBuf[1];	//Lc

				result = CardTxRxFrame( 0x00, Ic_Comm_RxBuf[1] );
				if( result == 0 )
				{
				}
			}
			else if( Ic_Comm_RxBuf[0] == 0x6C )
			{
				// 6CH: send last command again, P3 = XX;
				Ic_Comm_TxPtr = 4;
				Ic_Comm_TxBuf[ Ic_Comm_TxPtr++ ] = Ic_Comm_RxBuf[1];	//Lc	
				result = CardTxRxFrame( 0x02, Ic_Comm_RxBuf[1] );
				if( result == 0 )
				{
				}
			}
			else if( Ic_Comm_RxBuf[0] == 0x90 )
			{
				Ic_Comm_TxPtr = 0x00;
				Ic_Comm_TxBuf[ Ic_Comm_TxPtr++ ] = 0x00;	//CLA
				Ic_Comm_TxBuf[ Ic_Comm_TxPtr++ ] = 0xB0;	//INS
				Ic_Comm_TxBuf[ Ic_Comm_TxPtr++ ] = 0x81;	//P1
				Ic_Comm_TxBuf[ Ic_Comm_TxPtr++ ] = 0x00;	//P2
			
				Ic_Comm_TxBuf[ Ic_Comm_TxPtr++ ] = 0x03;	//Lc
			
				result = CardTxRxFrame( 0x00, 0x03 );
			
//				sendmessage( Ic_Comm_RxBuf, Ic_Comm_RxPtr );
			}
		}
		else
		{
//			return OVER;
		}
	}
	for(i=0; i<(Ic_Comm_TxBuf[4] +2); i++)
		comm_rxbuf[i] = Ic_Comm_RxBuf[i];
	return OVER;
}

INT8U U7816_comm2_DMA( INT8U near comm_rxbuf[], INT8U near ReadESAM[] )
{
	unsigned char result,i;
	memset( Ic_Comm_RxBuf, 0x00, U7816BUFLEN );  
//	memset( comm_rxbuf, 0x00, U7816BUFLEN );  
	
	for( Ic_Comm_TxPtr=0x00; Ic_Comm_TxPtr<5; Ic_Comm_TxPtr++ )
	{
		Ic_Comm_TxBuf[ Ic_Comm_TxPtr ] = ReadESAM[ Ic_Comm_TxPtr ];
	}
	
	result = CardTxRxFrame_DMA( 0x00, Ic_Comm_TxBuf[4] );

	if( result  != CMD_RESPONSE )
	{
		if( Ic_Comm_RxPtr == 2 )
		{
			if( Ic_Comm_RxBuf[0] == 0x61 )
			{
				// 61H: send get response command, P3<=XX
				Ic_Comm_TxPtr = 0x00;
				Ic_Comm_TxBuf[ Ic_Comm_TxPtr++ ] = 0x00;	//CLA
				Ic_Comm_TxBuf[ Ic_Comm_TxPtr++ ] = 0xC0;	//INS
				Ic_Comm_TxBuf[ Ic_Comm_TxPtr++ ] = 0x00;	//P1
				Ic_Comm_TxBuf[ Ic_Comm_TxPtr++ ] = 0x00;	//P2
				
				Ic_Comm_TxBuf[ Ic_Comm_TxPtr++ ] = Ic_Comm_RxBuf[1];	//Lc

				result = CardTxRxFrame_DMA( 0x00, Ic_Comm_RxBuf[1] );
				if( result == 0 )
				{
				}
			}
			else if( Ic_Comm_RxBuf[0] == 0x6C )
			{
				// 6CH: send last command again, P3 = XX;
				Ic_Comm_TxPtr = 4;
				Ic_Comm_TxBuf[ Ic_Comm_TxPtr++ ] = Ic_Comm_RxBuf[1];	//Lc	
				result = CardTxRxFrame_DMA( 0x02, Ic_Comm_RxBuf[1] );
				if( result == 0 )
				{
				}
			}
			else if( Ic_Comm_RxBuf[0] == 0x90 )
			{
				Ic_Comm_TxPtr = 0x00;
				Ic_Comm_TxBuf[ Ic_Comm_TxPtr++ ] = 0x00;	//CLA
				Ic_Comm_TxBuf[ Ic_Comm_TxPtr++ ] = 0xB0;	//INS
				Ic_Comm_TxBuf[ Ic_Comm_TxPtr++ ] = 0x81;	//P1
				Ic_Comm_TxBuf[ Ic_Comm_TxPtr++ ] = 0x00;	//P2
			
				Ic_Comm_TxBuf[ Ic_Comm_TxPtr++ ] = 0x03;	//Lc
			
				result = CardTxRxFrame_DMA( 0x00, 0x03 );
			
//				sendmessage( Ic_Comm_RxBuf, Ic_Comm_RxPtr );
			}
		}
		else
		{
//			return OVER;
		}
	}
	for(i=0; i<(Ic_Comm_TxBuf[4] +2); i++)
		comm_rxbuf[i] = Ic_Comm_RxBuf[i];
	return OVER;
}

