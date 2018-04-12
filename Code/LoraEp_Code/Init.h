#ifndef _INIT_H_
#define _INIT_H_


//include-----------------------------------------------------
#include   		"fm375reg.h"
#include   		"fm375reg_def.h"
#include 		"bin2hex.h"
#include		"Oscpu.h"			//�������Ͷ���
#include 		<intrins.h>			//_nop_();
#include 		<absacc.h>			//XBYTE();
#include 		<stdlib.h>			//rand();
#include 		<string.h>          //memset();
#include 		<math.h>

//macro define -----------------------------------------------

#ifndef TRUE
	#define TRUE  				1		//��
#endif
#ifndef FALSE
	#define FALSE 				0		//��
#endif
#ifndef true
	#define true				TRUE	//��
#endif
#ifndef false
	#define false				FALSE	//��
#endif

#define  	sweep_window    			16     		//
#define 	trim_signal_flash_addr		0XFFFC00		//�洢�������ݵĵ�ַ

//extern FNNC declare
extern void Clk_Adj_Proc( void );
extern void INIT_CLK(void);
extern void SET_MCLK(INT8U mclk,INT8U en_hs_div,INT8U div_num);
extern void Init_SYS(void);
extern void Init_IO_PAD(void);
extern void close_all_module(void);

extern void INIT_T0(void);
extern void INIT_T1(void);
extern unsigned int initial_cap_test_8(void);

extern 	unsigned int  	edata	min_dlt,ossfet_array[sweep_window*2];	//�޸��������
extern 	unsigned char 	edata trim_result,cap_overflow_cnt,over_flow_cnt,ossfet_point,trim_ideal,trim_min,trim_max,trim_set,trim_test;
extern 	unsigned int  	edata cap_00,cap_7F,cap_value,trim_value;
extern 	unsigned long  	edata cap_cal;
extern 	unsigned char 	edata trim_2M,trim_4M,trim_8M;
extern 	unsigned char 	trim_of_min_dlt;
extern  INT8U near  Timer0_Test_Cnt;
extern  INT8U near Timer0_Test_Statues;
extern  INT8U near Timer1_Test_Cnt;
extern  INT8U near Timer1_Test_Statues;
extern  INT16U LP_tim_cnt;

extern  INT8U near clr_if;
extern  INT8U near T0_Over;
extern INT8U near io_if;

//extern Typ_word_bits		ADC_Convert_Data;

#define V1_LED_ON()		do{PADATA &= ~BIT4;}while(0)
#define V1_LED_OFF()	do{PADATA |= BIT4;}while(0)
#define V1_LED_TOG()	do{PADATA ^= BIT4;}while(0)
          
#define V2_LED_ON()		do{PATATA &= ~BIT5;}while(0)
#define V2_LED_OFF()	do{PATATA |= BIT5;}while(0)
#define V2_LED_TOG()	do{PADATA ^= BIT5;}while(0)
          
#define V3_LED_ON()		do{PADATA &= ~BIT6;}while(0)
#define V3_LED_OFF()	do{PADATA |= BIT6;}while(0)
#define V3_LED_TOG()	do{PADATA ^= BIT6;}while(0)
          
#define V4_LED_ON()		do{PADATA &= ~BIT7;}while(0)
#define V4_LED_OFF()	do{PADATA |= BIT7;}while(0)
#define V4_LED_TOG()	do{PADATA ^= BIT7;}while(0)

//----------- WDT ----------
extern void Set_SYSWDT(INT8U wdt_Period);
// #define F251WDT_CLR()	do{WDTRST=0x1E;WDTRST=0xE1;}while(0)

//------------------�жϳ�ʼ��--------------------------------
extern void Init_INT(void);

 
//------------------UART--------------------------------
extern void Init_UART_PAD(void);
extern void Init_UART(void);
extern void UartTx_0(INT8U Value);
extern void UartTx_3(INT8U Value);
extern void Uart0_SendStr(INT8U len, INT8U *ptr);
extern void UartTx_1(INT8U Value);
extern INT8U UartRec_0(void);
extern void UartTx_2(INT8U Value);
extern void UartSendInfo(INT8U len, INT8U *ptr);
extern void Uart1_SendInfo(INT8U len, INT8U *ptr);
extern void UART1_receiveStr(uchar length);
extern void UART2_receiveStr(uchar length);
extern void Uart2_SendStr(INT8U len, INT8U *ptr);
extern void Uart_Proc(void);
extern void Uart0_TX_DMA(INT8U near *ptr, INT8U len);
extern void Uart0_RX_DMA(INT8U near *ptr, INT8U len);
extern void Init_UART2(void);
extern void Uart2_TX_DMA(INT8U near *ptr, INT8U len);
//------------------ I2C ---------------
//global variable and function--------------------------------
typedef enum{FM24C01,
			  FM24C02,
			  FM24C04,
			  FM24C08,
			  FM24C16,
			  FM24C32,
			  FM24C64,
			  FM24C128,
			  FM24C256
			  }eetype;
extern eetype EepromType;
extern void INIT_I2C_PAD(void);
extern void i2c_init(void);
extern void i2c_start(void);
extern void i2c_stop(void);
extern void i2c_restart(void);
extern void i2c_int(void);
extern bit i2c_write1(INT8U ctrlw, INT16U eepromaddr, INT8U *mtd, INT16U wlength); 
extern bit i2c_write_conflict(void);
extern bit i2c_write_conflict2(void);
extern bit i2c_oierr(void);
extern bit i2c_sderr(void);
extern bit i2c_ierr(void);
extern void wait_i2cif_clear(void);
extern bit i2c_write(INT8U ctrlw, INT16U eepromaddr, INT8U *mtd, INT16U wlength, eetype EepromType);
extern bit i2c_random_read(INT8U ctrl, INT16U eepromaddr, INT8U *mrd, INT16U rlength, eetype EepromType);
extern bit i2c_current_read(INT8U ctrlr, INT8U *mrd, INT16U rlength);
extern bit i2c_random_readwrong(INT8U ctrl, INT16U eepromaddr, INT8U *mrd, INT16U rlength, eetype EepromType);
extern bit i2c_write_DMA(INT8U ctrlw, INT16U eepromaddr, INT8U near *mtd, INT16U wlength);
extern bit i2c_random_read_DMA(INT8U ctrl, INT16U eepromaddr, INT8U near *mrd, INT16U rlength); 				
#define NO_I2C_IF	  0x11
#define YES_I2C_IF	  0x22
#define NO_I2C_ACK	  0x33
#define YES_I2C_ACK	  0x44

#define I2C_BF_RW	  		0x66
#define NO_I2C_BF_RW	  0x77
//#define EE24C32
//#define EE24C64
//#define EE24C04
//#define EE24C08
#define EE24C128

#ifdef EE24C32
#define PAGE     32	//24c32һҳ����                    16                                          //24Cxxһҳ����       �˴�Ϊ24c08
#define ALL      128//24c32ҳ��       				64		//24Cxxҳ����<I2C_BUF_SIZE
#endif
#ifdef EE24C64
#define PAGE     32	//24c32һҳ����                    16                                          //24Cxxһҳ����       �˴�Ϊ24c08
#define ALL      256//24c32ҳ��       				64		//24Cxxҳ����<I2C_BUF_SIZE
#endif
#ifdef EE24C04
#define PAGE     16	//24c32һҳ����                    16                                          //24Cxxһҳ����       �˴�Ϊ24c08
#define ALL      32//24c32ҳ��       				64		//24Cxxҳ����<I2C_BUF_SIZE
#endif
#ifdef EE24C08
#define PAGE     16	//24c32һҳ����                    16                                          //24Cxxһҳ����       �˴�Ϊ24c08
#define ALL      64//24c32ҳ��       				64		//24Cxxҳ����<I2C_BUF_SIZE
#endif
#ifdef EE24C128
#define PAGE     64	//24c128һҳ����                    16                                          //24Cxxһҳ����       �˴�Ϊ24c08
#define ALL      256//24c128ҳ��       				64		//24Cxxҳ����<I2C_BUF_SIZE
#endif


#define I2C_ERROR		0
#define I2C_SUCCESS	1
#define ERROR() do{i2c_stop(); return I2C_ERROR;} while(0)   //���ͺ�δ�յ��ӻ�Ӧ�𣬽������β������ɸ���ʵ�������޸Ĵ���ʽ

//------------------SPI------------------
// #define CS_CLR()	do{IO1DATA &= ~BIT7;}while(0)   //TSET BOARD
// #define CS_SET()	do{IO1DATA |= BIT7;}while(0)


#define CS_CLR()	do{PADATA &= ~BIT4;}while(0)  //TEMPERATURE TSET BOARD
#define CS_SET()	do{PADATA |= BIT4;}while(0)

extern void INIT_SPI_PAD(void);
extern void spi_init(void);
extern void SPI_write_enable_int(void);	
extern void SPI_write_enable(void);	
extern void SPI_write_disable_int(void);	
extern void SPI_write_disable(void);		
extern void spi_chip_erase(void);
extern void write_spi_nbyte_int(uchar addr2,uchar addr1,uchar addr0,uchar *mtd,INT16U wr_length);
extern void write_spi_nbyte(uchar addr2,uchar addr1,uchar addr0,uchar *mtd,INT16U wr_length);
extern void read_spi_nbyte_int(uchar addr2,uchar addr1,uchar addr0,uchar *mrd,INT16U wr_length);
extern void read_spi_nbyte(uchar addr2,uchar addr1,uchar addr0,uchar *mrd,INT16U wr_length);
extern void write_spi_nbyte_dma(uchar addr2,uchar addr1,uchar addr0,INT16U mtd,INT16U wr_length);
extern void read_spi_nbyte_dma(uchar addr2,uchar addr1,uchar addr0,INT16U mrd,INT16U wr_length);
extern void spi_chip_erase_dma(void);
extern void SPI_DMA_Config(void);
extern void SPI_DMA_RW_Byte(INT16U wr_addr,INT16U rd_addr,INT16U ndtr);
extern void SPI_DMA_Send_Byte(INT16U wr_addr,INT16U ndtr);
extern unsigned char spi_master_error(void);
extern unsigned char spi_txcol_error(void);
#define SPI_ERROR		0x0
#define SPI_SUCCESS	0xff

//------------------ 7816 FUNC ---------------------------
#define U7816BUFLEN 100
#define JUDGE_EDGE_NO	  0x11      //δ��⵽io������
#define JUDGE_EDGE_YES   0x22				//��⵽io������
#define RCV_SUCCESS	0x33				//��ȡ�ɹ�
#define RCV_FAIL			0x44				//��ȡʧ��
#define RCV_TOOLONG	0x55
#define SND_SUCCESS	0x66				//���ͳɹ�
#define SND_FAIL		0x77				//����ʧ��
#define SND_TOOLONG	0x88
#define RESET_ERROR1	0x99				//��λӦ������
#define RESET_ERROR2	0xaa				//��λӦ������
#define TS_WRONG		0xbb				//��λӦ��TSֵ����
#define RESET_FINISH		0xcc				//��λ���
#define CMD_RESPONSE	0xdd				//������Ӧ
#define CMD_RESPONSE_ERROR  0xee                 //������Ӧ�쳣
#define OVER				0xff
#define RST7816_0()	 do{PEDATA &= ~BIT3;}while(0)
#define RST7816_1()	 do{PEDATA |= BIT3;}while(0)   


extern void INIT_7816_PAD(void);
extern void INIT_7816(void);

extern  INT8U near Ic_Comm_RxBuf[U7816BUFLEN]	;	//7816���ջ�����
extern  INT8U near Ic_Comm_RxPtr		;	//7816����ָ��
extern  INT8U near Ic_Comm_TxBuf[U7816BUFLEN]	;	//7816���ͻ�����
extern  INT8U near Ic_Comm_TxPtr		;	//7816����ָ��



extern void INIT_7816(void);
extern INT8U Rcv_7816_Byte( INT8U *Data );
extern INT8U Snd_7816_Byte( INT8U Data );
extern INT8U ColdReset(INT8U reset_rxbuf[] );
extern INT8U U7816_comm1( INT8U near comm_rxbuf[], INT8U near WriteESAM[] );
extern INT8U U7816_comm2( INT8U near comm_rxbuf[], INT8U near ReadESAM[] );
extern INT8U U7816_comm1_DMA( INT8U near comm_rxbuf[], INT8U near WriteESAM[] );
extern INT8U U7816_comm2_DMA( INT8U near comm_rxbuf[], INT8U near ReadESAM[] );

extern void Delay_Ten_Per_etu(INT16U time);
extern INT8U CardTxRxFrame( INT8U Lc, INT8U Le );

extern  INT8U near T0_Over;
extern  INT8U near T1_Over;

//----------------- RAM,falsh -------------------
#define 	FLASH_ADDR		0xFF0000	
#define  	RAM_LAST_SECTOR				0x000E00		//ram���512�ֽ�
extern uchar xram_wr_check(void);
extern void Write_RAM(INT16U ram_offset_addr,INT8U ram_data,INT8U length);
extern INT8U Read_RAM(INT16U ram_offset_addr);


extern void write_FLASH(INT32U fl_addr,INT8U fl_data);
extern INT8U read_FLASH(INT32U fl_addr);
extern void erase_FLASH(INT32U fl_addr);
extern void erase_FLASH_CHIP(void);
extern void write_FLASH_Buffer(INT32U fl_addr,INT8U *point_data);

extern INT16U flash_wr_check(INT32U fl_addr,INT8U fl_data);
extern INT8U near ram_par_err;



//------------------ ȫ�ֱ��� ----------------------- 
#define 	TX_BUF_SIZE			256				//Uart�շ��ͻ��������ȶ���
extern INT8U TX_Buf[TX_BUF_SIZE] _at_ 0x100;			//ͨ�Ž��շ��ͻ����� 100BYTES
extern INT8U Commandh;
extern INT8U Commandl;	
extern INT8U Uart_test_int;

extern INT8U near RTCTRIM_copy;

//UART�˿����в������ݽṹ(LEN=60)
struct UartRunParam_struct
{	
	INT16U pRecvPoint;			//���ջ���������ָ��
	INT16U pRecvLenth;			//���ջ������ݳ���
	INT16U pSendPoint;			//���ͻ���������ָ��
	INT16U pSendLenth;			//���ͻ������ݳ���
	INT8U pTmOutCnt;			//ͨѶ�ӳ����������
	INT8U pTmOutVal;			//ͨѶ�ӳ�����趨ֵ

	INT8U RecvStatus;			//����״̬��
	BOOLEAN RecvOKFLAG;		//�������״̬
	INT8U SendStatus;			//����״̬��
	BOOLEAN bUpRecordFlag;		//�ϴ���¼���ݱ�־
	BOOLEAN bSendOverFlag;		//�������ݽ�����־
	INT8U bCRC;					//�洢У���

	INT8U BaudRate;				//������
}; 

//����ͨ�Ų�����BUF����
extern struct  UartRunParam_struct near Uart3;		//����ͨ�����ݽṹ

//RTC
extern INT8U near rtc_sec_if;

//------------------ UARTָ��Ӳ����ַ --------------------
#define  UART_ADDRH				1				//���ڵ�ַ�����ڶമ��ϵͳ
#define  UART_ADDRL  			0

//-----PMU------
extern void wake_event_set(INT8U wake_source,INT8U power_mode);
extern void Low_power_Ctrl(INT8U off_vref,INT8U en_cvs,INT8U off_xtlf,INT8U lp_ram_h,INT8U lp_ram_l,INT8U rclp_off,INT8U stop_wake_time,INT8U en_disc);
extern void goto_stop(void);
extern INT8U near p_mode;

#define  LSCLK  			1
#define  RCHF_8M  		2
#define  RCHF_16M  		3
#define  RCHF_24M  		4
#define  RCHF_32M  		5
#define  PLL		  		6


#define  LFDET  		0
#define  LVD_PD  		1
#define  RTCINT  		2
#define  IOINT  		3
#define  WKUP	  		4
#define  IWDG		  		5
#define  LPTIM_INT  	6
#define  RXD2_RX  		7
#define  LVD_PU  		8
#define  NO_WKUP  		9

#define  ACTIVE  			0
#define  LPRUN	  		1
#define  SLEEP  			2
#define  STOP	  			3

//----------- TRNG -----------------------

#define MSB_FIRST

#define  LSB  		0
#define  MSB  		1

#define  LFSR_BUSY  		0
#define  CRC_SUCCESS  		1
#define INIT  0xffffffff
#define XOROT 0xffffffff

extern INT8U near flag;
extern INT8U near dma_end;
extern INT8U near dma_half;
extern INT8U near flash_op_end;
//============flip251=====================

union B16_B08
{
	unsigned char B08[2];
	unsigned int  B16;
};

typedef union
{
    unsigned int word_val;
    unsigned char v[2];    //0�Ǹ�λ
    struct
    {
        unsigned char HB;  //��λ��ַ����λ���ݣ�
        unsigned char LB; 
    } byte;   
} Typ_word_bits;

// extern uchar near PCA_Buf[20];
// extern uchar PCA_Cmp_Cnt;
// extern uchar PCA_Cap_Cnt;
// extern uchar near PCA_Cap_Buf[12];
// extern uchar PCA_PWM_cnt;

// extern uchar PCA_Timer(uchar Num,uchar *PCA_Buf2, uchar Cnt_Max,uchar Cnt_Min);
// extern uchar PCA_Capture(uchar Num,uchar *PCA_buf2,uchar CON,uchar MOD,uchar Cnt_Max,uchar Cnt_Min);

//============TIMER2=====================
extern uchar near T2_Buf[10];
extern uchar T2_Cap_Cnt;
extern uchar near T2_Cap_Buf[12];
extern uchar TIMER2_Capture(uchar *T2_buf2,uchar CON,uchar MOD,INT16U Cnt_Max,INT16U Cnt_Min);
extern INT8U timer0_Cnt(INT8U *T2_buf2,INT8U MOD,INT8U Cnt_Max,INT8U Cnt_Min);
extern INT8U timer1_Cnt(INT8U *T2_buf2,INT8U MOD,INT8U Cnt_Max,INT8U Cnt_Min);
extern INT8U Lptim_Cnt(INT8U lpcfg0,INT8U lpcfg1,INT8U lpcompl,INT8U lpcomph,INT8U lptargtl,INT8U lptargth,INT8U ie_con,INT8U Cnt_Max,INT8U Cnt_Min);
//--------------------- clk gen------------
extern void clk_out_ctl(INT8U option);

//===================== ANA ==============
extern INT8U near ls_if;

extern INT8U near adc_done;

extern char volatile usart_temp[50];

#define CHARGE()	do{PEDATA &= ~BIT0;PEDATA |= BIT1;}while(0)
#define DISCHARGE()	do{PEDATA &= ~BIT1;PEDATA |= BIT0;}while(0)
#define CLOSE_SWTICH()	do{PEDATA |= BIT0;PEDATA |= BIT1;}while(0)


//===================== disp ==============
typedef union UInt {                   
   unsigned int Int;
   unsigned char Char[2];
} UInt;									//���÷�ʽ �ֽ� XXX.Char[0] ��XXX.Char[1] ��λ��ǰ ������ XXX.Int

typedef union MY_long {                   // Byte addressable long
   long Long;
   unsigned int Int[2];
   unsigned char Char[4];				//0�����λ
} MY_long;

typedef union ULong {                  // Byte addressable unsigned long
   unsigned long ULong;
   unsigned int  U16[2];
   unsigned char U8[4];					//0�����λ
} ULong;

//����LCD��λѰַ����
typedef union 
{
	unsigned char	byte_val; 		/* byte unit */
	struct 
	{
		unsigned char b0:1; /* bit0 */
		unsigned char b1:1; /* bit1 */
		unsigned char b2:1; /* bit2 */
		unsigned char b3:1; /* bit3 */
		unsigned char b4:1; /* bit4 */
		unsigned char b5:1; /* bit5 */
		unsigned char b6:1; /* bit6 */
		unsigned char b7:1; /* bit7 */ 		
	}bits; 		/* bit unit */
}Typ_byte_bits; 
//���÷��� 
//����  	Typ_byte_bits TEST_byte
//�ֽ�����	TEST_byte.byte_val=0X55;
//λ����	TEST_byte.bits.b1=1;
 
 
//����LCD��ʾ�����������ݽṹ
typedef union
{
   unsigned char buff[13];				//�ֽڷ���ģʽ   ��Ӧ����FM375��13����ʾ�Ĵ���
   struct 
   {
		Typ_byte_bits buf_DISPDATA0;			//�ɰ��ֽں�BIT����
		Typ_byte_bits buf_DISPDATA1;			//�ɰ��ֽں�BIT����
		Typ_byte_bits buf_DISPDATA2;			//�ɰ��ֽں�BIT����
		Typ_byte_bits buf_DISPDATA3;			//�ɰ��ֽں�BIT����
		Typ_byte_bits buf_DISPDATA4;			//�ɰ��ֽں�BIT����
		Typ_byte_bits buf_DISPDATA5;			//�ɰ��ֽں�BIT����
		Typ_byte_bits buf_DISPDATA6;			//�ɰ��ֽں�BIT����
		Typ_byte_bits buf_DISPDATA7;			//�ɰ��ֽں�BIT����
		Typ_byte_bits buf_DISPDATA8;			//�ɰ��ֽں�BIT����
		Typ_byte_bits buf_DISPDATA9;			//�ɰ��ֽں�BIT����
		Typ_byte_bits buf_DISPDATA10;			//�ɰ��ֽں�BIT����
		Typ_byte_bits buf_DISPDATA11;			//�ɰ��ֽں�BIT����
		Typ_byte_bits buf_DISPDATA12;			//�ɰ��ֽں�BIT����
   }byte_mode;
}LCD_DISP_BUFF;


//����LCD��ʾ�����������ݽṹ
typedef union
{
   unsigned long Flag;						//��ӦҺ����������������ַ�
   struct 
   {
		unsigned char s_buchang				:1; /* bit0 */
		unsigned char s_tiaojiao			:1; /* bit1 */
		unsigned char s_S1					:1; /* bit2 */
		unsigned char s_T2					:1; /* bit3 */
		unsigned char s_T3					:1; /* bit4 */
		unsigned char s_moshi		    	:1; /* bit5 */
		unsigned char s_pinglv				:1; /* bit6 */
		unsigned char s_wendu		    	:1; /* bit7 */
		unsigned char s_banben				:1; /* bit8 */
		unsigned char s_tiaojiaozhi			:1; /* bit9 */
		unsigned char s_qiangdu				:1; /* bit10 */
		unsigned char s_bianshuju	    	:1; /* bit11 */
		unsigned char s_biancheng	    	:1; /* bit12 */
		unsigned char s_qian		    	:1; /* bit13 */
		unsigned char s_hou					:1; /* bit14 */
		unsigned char s_hezhi		    	:1; /* bit15 */
		unsigned char s_sizhao				:1; /* bit16 */
		unsigned char s_miao		    	:1; /* bit17 */
		unsigned char s_S2					:1; /* bit18 */
		unsigned char s_shi					:1; /* bit19 */
		unsigned char s_adc					:1; /* bit20 */
		unsigned char s_P3					:1; /* bit21 */
		unsigned char s_P4					:1; /* bit22 */
		unsigned char s_P5					:1; /* bit23 */
		unsigned char s_P6					:1; /* bit24 */
		unsigned char s_P7					:1; /* bit25 */
		unsigned char s_P8					:1; /* bit26 */
		unsigned char s_P9					:1; /* bit27 */
		unsigned char s_S3					:1; /* bit28 */
		unsigned char s_T1					:1; /* bit29 */
//		unsigned char 						:1; /* bit30 */
//		unsigned char 						:1; /* bit31 */

   }bits;
}LCD_char_flag;

extern LCD_char_flag DISP_char_flag;
extern LCD_DISP_BUFF DISP_buff;
////��ʼ���������ʾ�εĶ���   ʹ��λ������õĲ���  ��϶�Ӧ����ֵ����ɶ�Ӧ�Ĵ����Ķ���  4COMģʽ
////���ϵ���������C0-C1-C2-C3��ӦSEG������  �Ǹ���Һ��Ӳ������ȷ�ϵ�
////C0���SEG1---SEG25�Ķ���    
////��Զ��ס  ���޸ĵ�ֻ�ǵ�һ��  ����޸Ŀ�Һ�����ձ� ���еĹ��ձ���COM�����е�SEG��	
#define	T1					DISP_buff.byte_mode.buf_DISPDATA0.bits.b1	//C0-S1 	//�Ȳ�Һ��Ӳ��COM0 SEG0 ����ʾ T1  			�ٲ�Ĵ�����C0-S0��Ӧ�Ĵ���	DSPDATA0.0	 ��ͬ								
#define	buchang			DISP_buff.byte_mode.buf_DISPDATA0.bits.b2	//C0-S2 		//�Ȳ�Һ��Ӳ��COM0 SEG1 ����ʾ �����ݷ���  	�ٲ�Ĵ�����C0-S0��Ӧ�Ĵ���	DSPDATA0.1	 ��ͬ		
#define	tiaojiao		DISP_buff.byte_mode.buf_DISPDATA0.bits.b3	//C0-S3 		//�Ȳ�Һ��Ӳ��COM0 SEG2 �ٲ�Ĵ�����C2-S3��Ӧ�Ĵ���	DSPDATA2.3	 ��ͬ					              
#define	D1					DISP_buff.byte_mode.buf_DISPDATA0.bits.b4	//C0-S4   	//�Ȳ�Һ��Ӳ��COM1 SEG3 �ٲ�Ĵ�����C3-S3��Ӧ�Ĵ���	DSPDATA3.3				                      
#define	S1					DISP_buff.byte_mode.buf_DISPDATA0.bits.b5	//C0-S5
#define	D2					DISP_buff.byte_mode.buf_DISPDATA0.bits.b6	//C0-S6
#define	T2					DISP_buff.byte_mode.buf_DISPDATA0.bits.b7	//C0-S7 
#define	T3					DISP_buff.byte_mode.buf_DISPDATA4.bits.b0	//C0-S8 
#define	moshi				DISP_buff.byte_mode.buf_DISPDATA4.bits.b1	//C0-S9	
#define	F3					DISP_buff.byte_mode.buf_DISPDATA4.bits.b2	//C0-S10
#define	A3					DISP_buff.byte_mode.buf_DISPDATA4.bits.b3	//C0-S11
#define	F4					DISP_buff.byte_mode.buf_DISPDATA4.bits.b4	//C0-S12
#define	A4					DISP_buff.byte_mode.buf_DISPDATA4.bits.b5	//C0-S13
#define	F5					DISP_buff.byte_mode.buf_DISPDATA4.bits.b6	//C0-S14
#define	A5					DISP_buff.byte_mode.buf_DISPDATA4.bits.b7	//C0-S15
#define	F6					DISP_buff.byte_mode.buf_DISPDATA8.bits.b0	//C0-S16 	
#define	A6					DISP_buff.byte_mode.buf_DISPDATA8.bits.b1	
#define	F7					DISP_buff.byte_mode.buf_DISPDATA8.bits.b2	
#define	A7					DISP_buff.byte_mode.buf_DISPDATA8.bits.b3	
#define	F8					DISP_buff.byte_mode.buf_DISPDATA8.bits.b4	 
#define	A8					DISP_buff.byte_mode.buf_DISPDATA8.bits.b5	 
#define	F9					DISP_buff.byte_mode.buf_DISPDATA8.bits.b6	 
#define	A9					DISP_buff.byte_mode.buf_DISPDATA8.bits.b7	 
#define	F10					DISP_buff.byte_mode.buf_DISPDATA12.bits.b0	//c0-s24      
#define	A10					DISP_buff.byte_mode.buf_DISPDATA12.bits.b1	//c0-s25
        			
////C1���SEG1---SEG25�Ķ���  								
#define	pinglv				DISP_buff.byte_mode.buf_DISPDATA1.bits.b1	 		//�Ȳ�Һ��Ӳ��COM0 SEG0 ����ʾ T1  			�ٲ�Ĵ�����C0-S0��Ӧ�Ĵ���	DSPDATA0.0	 ��ͬ					
#define	wendu					DISP_buff.byte_mode.buf_DISPDATA1.bits.b2	   //�Ȳ�Һ��Ӳ��COM0 SEG1 ����ʾ �����ݷ���  	�ٲ�Ĵ�����C0-S0��Ӧ�Ĵ���	DSPDATA0.1	 ��ͬ	
#define	E1						DISP_buff.byte_mode.buf_DISPDATA1.bits.b3	   //�Ȳ�Һ��Ӳ��COM0 SEG2 �ٲ�Ĵ�����C2-S3��Ӧ�Ĵ���	DSPDATA2.3	 ��ͬ					            
#define	C1						DISP_buff.byte_mode.buf_DISPDATA1.bits.b4	   //�Ȳ�Һ��Ӳ��COM1 SEG3 �ٲ�Ĵ�����C3-S3��Ӧ�Ĵ���	DSPDATA3.3				                    
#define	E2						DISP_buff.byte_mode.buf_DISPDATA1.bits.b5	 
#define	C2						DISP_buff.byte_mode.buf_DISPDATA1.bits.b6	 
#define	banben				DISP_buff.byte_mode.buf_DISPDATA1.bits.b7	 
#define	tiaojiaozhi 	DISP_buff.byte_mode.buf_DISPDATA5.bits.b0	
#define	qiangdu				DISP_buff.byte_mode.buf_DISPDATA5.bits.b1	
#define	G3						DISP_buff.byte_mode.buf_DISPDATA5.bits.b2	
#define	B3						DISP_buff.byte_mode.buf_DISPDATA5.bits.b3	
#define	G4						DISP_buff.byte_mode.buf_DISPDATA5.bits.b4	 
#define	B4						DISP_buff.byte_mode.buf_DISPDATA5.bits.b5	 
#define	G5						DISP_buff.byte_mode.buf_DISPDATA5.bits.b6	 
#define	B5						DISP_buff.byte_mode.buf_DISPDATA5.bits.b7	 
#define	G6						DISP_buff.byte_mode.buf_DISPDATA9.bits.b0	
#define	B6						DISP_buff.byte_mode.buf_DISPDATA9.bits.b1	
#define	G7						DISP_buff.byte_mode.buf_DISPDATA9.bits.b2	
#define	B7						DISP_buff.byte_mode.buf_DISPDATA9.bits.b3	
#define	G8						DISP_buff.byte_mode.buf_DISPDATA9.bits.b4	 
#define	B8						DISP_buff.byte_mode.buf_DISPDATA9.bits.b5	 
#define	G9						DISP_buff.byte_mode.buf_DISPDATA9.bits.b6	 
#define	B9						DISP_buff.byte_mode.buf_DISPDATA9.bits.b7	 
#define	G10						DISP_buff.byte_mode.buf_DISPDATA12.bits.b2  //c1-s24
#define	B10						DISP_buff.byte_mode.buf_DISPDATA12.bits.b3  //c1-s25
        					

////C2���SEG1---SEG25�Ķ���   				
#define	bianshuju 	DISP_buff.byte_mode.buf_DISPDATA2.bits.b1	//�Ȳ�Һ��Ӳ��COM0 SEG0 ����ʾ T1  			�ٲ�Ĵ�����C0-S0��Ӧ�Ĵ���	DSPDATA0.0	 ��ͬ					                                            
#define	biancheng 	DISP_buff.byte_mode.buf_DISPDATA2.bits.b2	 //�Ȳ�Һ��Ӳ��COM0 SEG1 ����ʾ �����ݷ���  	�ٲ�Ĵ�����C0-S0��Ӧ�Ĵ���	DSPDATA0.1	 ��ͬ					                                      
#define	G1					DISP_buff.byte_mode.buf_DISPDATA2.bits.b3	 //�Ȳ�Һ��Ӳ��COM0 SEG2 �ٲ�Ĵ�����C2-S3��Ӧ�Ĵ���	DSPDATA2.3	 ��ͬ					                                                          
#define	B1					DISP_buff.byte_mode.buf_DISPDATA2.bits.b4	 //�Ȳ�Һ��Ӳ��COM1 SEG3 �ٲ�Ĵ�����C3-S3��Ӧ�Ĵ���	DSPDATA3.3				                                                                   
#define	G2					DISP_buff.byte_mode.buf_DISPDATA2.bits.b5	 
#define	B2					DISP_buff.byte_mode.buf_DISPDATA2.bits.b6	 
#define	qian				DISP_buff.byte_mode.buf_DISPDATA2.bits.b7	 
#define	hou					DISP_buff.byte_mode.buf_DISPDATA6.bits.b0	
#define	hezhi				DISP_buff.byte_mode.buf_DISPDATA6.bits.b1	
#define	E3					DISP_buff.byte_mode.buf_DISPDATA6.bits.b2	
#define	C3					DISP_buff.byte_mode.buf_DISPDATA6.bits.b3	
#define	E4					DISP_buff.byte_mode.buf_DISPDATA6.bits.b4	 
#define	C4					DISP_buff.byte_mode.buf_DISPDATA6.bits.b5	 
#define	E5					DISP_buff.byte_mode.buf_DISPDATA6.bits.b6	 
#define	C5					DISP_buff.byte_mode.buf_DISPDATA6.bits.b7	 
#define	E6					DISP_buff.byte_mode.buf_DISPDATA10.bits.b0	
#define	C6					DISP_buff.byte_mode.buf_DISPDATA10.bits.b1	
#define	E7					DISP_buff.byte_mode.buf_DISPDATA10.bits.b2	
#define	C7					DISP_buff.byte_mode.buf_DISPDATA10.bits.b3	
#define	E8					DISP_buff.byte_mode.buf_DISPDATA10.bits.b4	 
#define	C8					DISP_buff.byte_mode.buf_DISPDATA10.bits.b5	 
#define	E9					DISP_buff.byte_mode.buf_DISPDATA10.bits.b6	 
#define	C9					DISP_buff.byte_mode.buf_DISPDATA10.bits.b7	 
#define	E10					DISP_buff.byte_mode.buf_DISPDATA12.bits.b4   //c2-s24
#define	C10					DISP_buff.byte_mode.buf_DISPDATA12.bits.b5   //c2-s25
        				

////C3���SEG1---SEG25�Ķ���  		
#define		sizhao 		DISP_buff.byte_mode.buf_DISPDATA3.bits.b1	 	//�Ȳ�Һ��Ӳ��COM0 SEG0 ����ʾ T1  			�ٲ�Ĵ�����C0-S0��Ӧ�Ĵ���	DSPDATA0.0	 ��ͬ					                                                                           
#define		miao	  	DISP_buff.byte_mode.buf_DISPDATA3.bits.b2	//�Ȳ�Һ��Ӳ��COM0 SEG1 ����ʾ �����ݷ���  	�ٲ�Ĵ�����C0-S0��Ӧ�Ĵ���	DSPDATA0.1	 ��ͬ					                                                                       
#define		F1		  	DISP_buff.byte_mode.buf_DISPDATA3.bits.b3	 //�Ȳ�Һ��Ӳ��COM0 SEG2 �ٲ�Ĵ�����C2-S3��Ӧ�Ĵ���	DSPDATA2.3	 ��ͬ					                                                                                         
#define		A1		  	DISP_buff.byte_mode.buf_DISPDATA3.bits.b4	 //�Ȳ�Һ��Ӳ��COM1 SEG3 �ٲ�Ĵ�����C3-S3��Ӧ�Ĵ���	DSPDATA3.3				                                                                                                 
#define		F2		  	DISP_buff.byte_mode.buf_DISPDATA3.bits.b5	                                                                                                                                                                          
#define		A2		  	DISP_buff.byte_mode.buf_DISPDATA3.bits.b6	 
#define		S2		  	DISP_buff.byte_mode.buf_DISPDATA3.bits.b7	 
#define		shi				DISP_buff.byte_mode.buf_DISPDATA7.bits.b0	
#define		adc				DISP_buff.byte_mode.buf_DISPDATA7.bits.b1	
#define		D3		  	DISP_buff.byte_mode.buf_DISPDATA7.bits.b2	
#define		P3		  	DISP_buff.byte_mode.buf_DISPDATA7.bits.b3	
#define		D4		  	DISP_buff.byte_mode.buf_DISPDATA7.bits.b4	 
#define		P4		  	DISP_buff.byte_mode.buf_DISPDATA7.bits.b5	 
#define		D5		  	DISP_buff.byte_mode.buf_DISPDATA7.bits.b6	 
#define		P5		  	DISP_buff.byte_mode.buf_DISPDATA7.bits.b7	 
#define		D6		  	DISP_buff.byte_mode.buf_DISPDATA11.bits.b0	
#define		P6		  	DISP_buff.byte_mode.buf_DISPDATA11.bits.b1	
#define		D7		  	DISP_buff.byte_mode.buf_DISPDATA11.bits.b2	
#define		P7		  	DISP_buff.byte_mode.buf_DISPDATA11.bits.b3	
#define		D8		  	DISP_buff.byte_mode.buf_DISPDATA11.bits.b4	 
#define		P8		  	DISP_buff.byte_mode.buf_DISPDATA11.bits.b5	 
#define		D9		  	DISP_buff.byte_mode.buf_DISPDATA11.bits.b6	 
#define		P9		  	DISP_buff.byte_mode.buf_DISPDATA11.bits.b7	 
#define		D10				DISP_buff.byte_mode.buf_DISPDATA12.bits.b6 //c3-s24				
#define		S3				DISP_buff.byte_mode.buf_DISPDATA12.bits.b7 //c3-s25				
         		  

#define char_S		0X05
#define char_E		0X0E
#define char_H		16
#define char_h		17
#define char_L		18
#define char_n		19
#define char_N		20
#define char_O		21
#define char_P		22
#define char_q		23
#define char_r		24
#define char_t		25
#define char_U		26
#define char_y		27
#define char_hen	28
#define char__		29

extern void clr_LCD_ram_segment_symbol(void);
extern void write_LCD_symbol_to_LCD_register(void);
extern void write_LCD_ram_to_LCD_register(void)   ;
extern void set_LCD_segment_by_loc_and_value(unsigned char loc,unsigned char disp_char);
extern void set_LCD_segment_by_point_num_byte_loc(unsigned char *point,unsigned char num,unsigned char loc);
extern void set_LCD_segment_by_point_num_compbyte_loc(unsigned char *point,unsigned char num,unsigned char loc);
extern void LCD_display_delay(unsigned char del_val);
extern void chip_poweron_display(void);
extern void  LCD_all_reg_0xFF(void);
extern void  LCD_all_reg_0x00(void);
extern void set_LCD_ram_by_loc_segment(unsigned char loc,unsigned char segment_data);
#define		LCD_disp_delay	1			//������ʾʱ�䵥λ0.5S



#define SEG25	1
#define SEG24	0
#define SEG23	7
#define SEG22	6
#define SEG21	5
#define SEG20	4
#define SEG19	3
#define SEG18	2
#define SEG17	1
#define SEG16	0
#define SEG15	7
#define SEG14	6
#define SEG13	5
#define SEG12	4
#define SEG11	3
#define SEG10	2
#define SEG9	1
#define SEG8	0
#define SEG7	7
#define SEG6	6
#define SEG5	5
#define SEG4	4
#define SEG3	3
#define SEG2	2
#define SEG1	1

//------variables declaration---------
extern  INT8U  Dispdata0Buf;
extern  INT8U  Dispdata1Buf;
extern  INT8U  Dispdata2Buf;
extern  INT8U  Dispdata3Buf;
extern  INT8U  Dispdata4Buf;
extern  INT8U  Dispdata5Buf;
extern  INT8U  Dispdata6Buf;
extern  INT8U  Dispdata7Buf;
extern  INT8U  Dispdata8Buf;
extern  INT8U  Dispdata9Buf;
extern  INT8U  Dispdata10Buf;
extern  INT8U  Dispdata11Buf;						
extern  INT8U  Dispdata12Buf;
extern  INT8U  Dispdata13Buf;
extern  INT8U  Dispdata14Buf;
extern  INT8U  Dispdata15Buf;
extern  INT8U  Dispdata16Buf;
extern  INT8U  Dispdata17Buf;
//----------clr disp buf-----------
#define ClrDispBuf()    do{Dispdata0Buf = 0;\
						   Dispdata1Buf = 0;\  
							Dispdata2Buf = 0;\
						   Dispdata3Buf = 0;\
								Dispdata4Buf = 0;\	   
							Dispdata5Buf = 0;\
							Dispdata6Buf = 0;\
    					   Dispdata7Buf = 0;\
							Dispdata8Buf = 0;\		   
							Dispdata9Buf = 0;\
						   Dispdata10Buf = 0;\
								Dispdata11Buf = 0;\
						   Dispdata12Buf = 0;\
						   Dispdata13Buf = 0;\
							Dispdata14Buf = 0;\
						   Dispdata15Buf = 0;\
						   Dispdata16Buf = 0;\
							 Dispdata17Buf = 0;\
						}while(0)		

//----------refresh disp buf-----------
#define RefreshDispBuf()    do{DISPDATA0 = Dispdata0Buf;\
							   DISPDATA1 = Dispdata1Buf;\
									DISPDATA2 = Dispdata2Buf;\
							   DISPDATA3 = Dispdata3Buf;\
									DISPDATA4 = Dispdata4Buf;\
							   DISPDATA5 = Dispdata5Buf;\
	    					   DISPDATA6 = Dispdata6Buf;\
							   DISPDATA7 = Dispdata7Buf;\
								DISPDATA8 = Dispdata8Buf;\
							   DISPDATA9 = Dispdata9Buf;\
	    					   DISPDATA10 = Dispdata10Buf;\
							   DISPDATA11 = Dispdata11Buf;\
	    					   DISPDATA12 = Dispdata12Buf;\
							   DISPDATA13 = Dispdata13Buf;\
								DISPDATA14 = Dispdata14Buf;\
							   DISPDATA15 = Dispdata15Buf;\
	    					   DISPDATA16 = Dispdata16Buf;\
									 DISPDATA17 = Dispdata17Buf;\
                            }while(0)   	

//----------clr display buf-------------
#define DISP_ALL_OFF()	do{DISPDATA0 = 0;\
						   DISPDATA1 = 0;\
						   DISPDATA2 = 0;\
						   DISPDATA3 = 0;\
						   DISPDATA4 = 0;\
						   DISPDATA5 = 0;\
						   DISPDATA6 = 0;\
						   DISPDATA7 = 0;\
						   DISPDATA8 = 0;\
						   DISPDATA9 = 0;\
						   DISPDATA10 = 0;\
						   DISPDATA11 = 0;\
								DISPDATA12 = 0;\
						   DISPDATA13 = 0;\
						   DISPDATA14 = 0;\
						   DISPDATA15 = 0;\
						   DISPDATA16 = 0;\
							 DISPDATA17 = 0;\
						}while(0)		


//-----------display all ---------------
#define DISP_ALL_ON()	do{DISPDATA0 = 0xff;\
						   DISPDATA1 = 0xff;\
						   DISPDATA2 = 0xff;\
						   DISPDATA3 = 0xff;\
						   DISPDATA4 = 0xff;\
						   DISPDATA5 = 0xff;\
						   DISPDATA6 = 0xff;\
						   DISPDATA7 = 0xff;\
						   DISPDATA8 = 0xff;\
						   DISPDATA9 = 0xff;\
						   DISPDATA10 = 0xff;\
						   DISPDATA11 = 0xff;\
							 DISPDATA12 = 0xff;\
						   DISPDATA13 = 0xff;\
						   DISPDATA14 = 0xff;\
						   DISPDATA15 = 0xff;\
						   DISPDATA16 = 0xff;\
							 DISPDATA17 = 0xff;\
						}while(0)			
//�����Ĵ���a��bλ
#define SET(a,b)	(a) |= (1 << (b))
#define CLR(a,b)	(a) &= ~(1 << (b))	

//------functions declaration----------
extern void LcdDisp(INT8U lcdbit, INT8U show);
extern void INIT_DISP_COUT_PAD(void);
extern void INIT_DISP_RIN_PAD(void);
extern void DISP_TEST_EN(INT8U disp_mode);
//============================= DMA =============================//
extern void DMA_Config(INT8U chx,INT16U maddr,INT16U ndtr,INT8U par_txrx);
extern void DMA_TxRx(INT8U par_txrx);
extern void DMA_Enable(INT8U chx);
extern void DMA_Disable(INT8U chx);
extern void DMA_Start(INT8U chx,INT16U maddr,INT16U ndtr,INT8U par_txrx);
extern void DMA2_Start(INT16U saddr,INT16U daddr,INT16U ndtr);
#define CH0 0
#define CH1 1	
						
#endif