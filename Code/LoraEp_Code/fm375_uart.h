/************************************************************************/
/* 	各模块测试函数头文件		    					                */
/* 	主要功能:															*/
/* 		1.串口指令码定义，便于修改    		                            */
/*	硬件平台：	 													  	*/
/*					                        	*/
/* 	编制:xudayong													*/
/* 	编制时间:2016年03月31日												*/
/************************************************************************/

#ifndef _FM375_UART_H_
#define	_FM375_UART_H_

//******************* 通信命令码定义********************
//高位=============================================
#define SYS							0x00			//系统用指令
#define DMA							0x10			//随机数
#define UART						0x20			//串口
#define CLKGEN					0x40			//时钟模块
#define I2C			        0x50           	//i2c模块
#define RAM			        0x60      //RAM
#define SPI							0x70			//SPI模块
#define FLIP						0x80			//flip模块
#define DISP 						0x90			//显示模块
#define ANA							0xA0			//数模接口
#define RST							0x30			//复位模块
#define MOD							0xC0			//模式控制模块
#define RTC							0xD0 			//RTC模块
#define PMU							0xE0 			//tmp_sensor模块
#define PAD							0xF0 			//PADCFG
#define XTMON						0x41			//XTLF monitor
#define IDD							0x51			//功耗测试
#define ADC							0x61      //ADC
#define FLSCTRL					0x71   		//FLSCTRL
#define U7816           0x81            //7816模块
#define CRC           	0x82            //CRC模块
#define LPTIM           0x83            //LPTIM


#define SPCH						0xFF			//special command
#define DATAFLASH					0xB0			//DATAFLASH操作命令


//低位==================================================
//SYS
#define WRITE_EXBYTE 				0x01
#define READ_EXBYTE				0x02
#define IO45_CLK				0x03
#define UART_SELFTEST				0x55	
#define SOFT_RST				0xCC
#define INTC_NEST				0x04
#define BAND_CHANGE				0x05
#define IO_INT				0x06
#define TEMPER_SET				0x07

//DMA
#define READ_ALL_DMAREG			0x01
#define WRITE_ALL_DMAREG			0x02
#define WRITE_ALL_DMAREG_SAME		0x30
#define DMA_UART_TX			0x03
#define RD_TRNG			0x04
#define DMA_UART0_RXTX			0x05
#define DMA2_FLASH_RAM			0x06

//FLSCTRL
#define READ_ALL_FLSCTRLREG			0x01
#define WRITE_ALL_TFLSCTRLREG			0x02
#define WRITE_ALL_FLSCTRLREG_SAME		0x30
#define	WRITE_DF_BYTE					0x03
#define READ_DF_BYTE			0x04
#define ERASE_SECTOR		0x05
#define FLS_WR_CHECK				0x06
#define ERASE_CHIP		0x07
#define WRITE_FLS_BUFFER		0x08

//I2C
#define READ_ALL_I2CREG			0x01
#define WRITE_ALL_I2CREG			0x02
#define I2C_ERR	       0x03

#define WRITE_ALL_I2CREG_SAME		0x30
#define I2C_START					0x10
#define I2C_STOP					0x11
#define I2C_RESTART				0x12
#define I2C_WRITE_BYTE	        	0x13
#define I2C_WRITE_NBYTE			0x14
#define I2C_RAND_READ_BYTE       	0x15
#define I2C_CURR_READ_BYTE  		0x16
#define I2C_RAND_READ_NBYTE       	0x17
#define I2C_CURR_READ_NBYTE  		0x18
#define I2C_WRITE_WR      			0x19
#define I2C_WRITE_CONF  			0x20
#define I2C_READ_WITHOUTACK   		0x21
#define I2C_SCLHL     				0x22
#define I2C_SDAHL   				0x23
#define I2C_BAUD_CTRL				0x24
#define I2C_RW_ALL					0x25
#define I2C_TEMPTEST				0x26
#define I2C_WRITE_NBYTE_DMA					0x04
#define I2C_RAND_READ_NBYTE_DMA	    0x05


//SPI
#define READ_ALL_SPIREG			0x01
#define WRITE_ALL_SPIREG			0x02
#define WRITE_ALL_SPIREG_SAME		0x30
// #define	SPI_INIT					0x03
#define WRITE_25F04_ENABLE			0x04
#define WRITE_25F04_DISABLE		0x05
#define CHIP_ERASE					0x0a
#define WRITE_25F04				0x06
#define READ_25F04				0x07
#define SPI_RW_ALL					0x08 
#define SPI_BAUD_SET				0x09
#define	STC_SLAVE_INIT				0x11
#define STC_WRITE_93C86				0x22
#define STC_READ_93C86				0x33
#define WRITE_25F04_DMA				0x0B
#define READ_25F04_DMA				0x0C
#define SPI_RW_ALL_DMA				0x0D
#define SPI_MASTER_ERR				0x0E
#define SPI_TX_WCOL						0x0F

//XTMON
#define READ_ALL_XTMONREG			0x01
#define WRITE_ALL_XTMONREG			0x02
#define WRITE_ALL_XTMONREG_SAME		0x30
#define XTMON_EN			0x03

//U7816
#define READ_ALL_7816REG			0x01
#define WRITE_ALL_7816REG			0x02
#define WRITE_U7816TXBUF			0x03
#define U7816_INIT					0x10
#define U7816_RESET				0x11
#define U7816_SELEF				0x12
#define U7816_WRITE				0X13
#define U7816_READ					0X14
#define U7816_READRAN				0x15
#define TEST_BGT					0x16
#define TEST_REVERROR				0x17
#define U7816_TESTRW				0x18
#define U7816_TEMPTEST1			0x20
#define U7816_TEMPTEST2			0x21
#define WRITE_ALL_7816REG_SAME	0x30
#define U7816_WRITE_DMA				0X04
#define U7816_READ_DMA			0X05
//RAM
#define READ_ALL_RAM			0x01
#define READ_RAM_BYTE			0x02
#define WRITE_RAM					0x03
#define WR_RAM_CHECK			0x04
#define RAMBIST_ON 				0x05
#define READ_RAMBIST 				0x06
#define STOP_RAM_TEST 				0x07



//disp
#define READ_ALL_DISPREG			0x01
#define WRITE_ALL_DISPREG			0x02
#define DISP_TEST_RIN					0x03
#define DISP_TEST_COUT				0x04
#define SLEEP_DISP_RIN				0x05
#define SLEEP_DISP_COUT				0x06
#define SLEEP_SAME_con				0x07
#define STOP_DISP_REG_TEST				0x08
#define LCD_TEST_MODE				0x09
#define LCD_DISP_MODE				0x0A
#define WRITE_ALL_DISPREG_SAME			0x30


//ana
#define READ_ALL_ANAREG			0x01
#define WRITE_ALL_ANAREG			0x02
#define WRITE_ALL_ANAREG_SAME				0x30
#define LSNS_ENABLE			0x03
#define LSNS_TESTEN			0x04
#define LVD_TEST				0x05
#define BUF4TST_OUT				0x06
#define BUF4TST_BUF_OUT				0x07
#define PDR_EN						0x08
#define BOR_EN						0x09

#define ADC_RUN								0x0A
#define ADC_SDM								0x0B
#define ADC_STATIC_TEST				0x0C
#define ADC_MEA_VDD						0x0D
#define ADC_TIME_TEST						0x0E
#define ADC_MEA_BUF4TST_IN						0x0F 

#define SLEEP_BUF4TST 	   0x10     
#define ANA_SAME_SLEEP     0x11   
#define SLEEP_LVD 			   0x12         
#define TEST_34401 			   0x13

//RST
#define READ_ALL_RSTREG			0x01
#define WRITE_ALL_RSTREG			0x02
#define WRITE_ALL_RSTREG_SAME		0x30
#define SYSWDT_OVFLOW				0x03
#define SYSWDT_CLEAR				0x04 



//MOD
#define READ_ALL_MODREG			0x01
#define WRITE_ALL_MODREG			0x02
#define WRITE_ALL_MODREG_SAME		0x30

//clk
#define READ_ALL_CLKREG			0x01
#define WRITE_ALL_CLKREG			0x02
#define WRITE_ALL_CLKREG_SAME		0x30
#define CLK_OUT			0x03
#define RCHF_TEST			0x04
#define RCLP_TEST			0x05
#define XTLF_FDET			0x06
#define ADDR_OVF			0x07
#define CLK_OUT1			0x08
#define LPRUN_RCHF			0x09
#define SAME_CLK_LPRUN			0x0A
//PMU
#define READ_ALL_PMUREG					0x01
#define WRITE_ALL_PMUREG				0x02
#define WRITE_ALL_RPMUREG_SAME	0x30
#define ACT_LPRUN								0x03
#define ACT_LPRUN_ACT						0x04

#define XTLF_CLOSE_ALL 			0x05
#define XTLF_OPEN_ADC 			0x06
#define XTLF_OPEN_DISP 			0x07
#define XTLF_OPEN_PLL 			0x08
#define XTLF_OPEN_BUF4TST		0x09
#define XTLF_OPEN_RCHF 			0x0C

#define RAM_STANDBY_TEST	 	0x17

#define STB_RCHF			0x1D


#define ACT_SL				0x0D
#define ACT_LPRUN_SL	  0x0E
#define ACT_ST				0x11
#define ACT_LPRUN_ST				0x12

//CRC
#define READ_ALL_CRCREG			0x01
#define WRITE_ALL_CRCREG			0x02
#define WRITE_ALL_CRCREG_SAME				0x30
#define CRC_TEST				0x03
#define CRC_5A5A				0x04
#define CRC_11223344				0x05
#define CRC_FLASH				0x06


//LPTIM
#define READ_ALL_LPTIMREG			0x01
#define WRITE_ALL_LPTIMREG			0x02
#define WRITE_ALL_LPTIMREG_SAME				0x30
#define LPTIM_CNT_N				0x03

//PAD
#define READ_ALL_PADREG			0x01
#define WRITE_ALL_PADREG			0x02
#define WRITE_ALL_PADREG_SAME		0x30
//RTC
#define READ_ALL_RTCREG			0x01
#define WRITE_ALL_RTCREG			0x02
#define WRITE_ALL_RTCREG_SAME		0x30
#define TIME_SET					0x10 			//时间设置
#define TIME_READ1				0x20 			//时间读，方式1
#define TIME_READ2				0x11 			//时间读，方式2
#define RTC_ALARM              0x40				//rtc闹钟
#define RTC_ENABLE				0x50                   	
#define RTC_DISABLE				0x60                   	
#define RTCADJ_SET	    0x70                    	//时钟调教
#define RTC_OUT	    0x80                    	//时标输出
#define RTCADJ_PLL	    0x03                    	//时标输出

//IRC
#define READ_ALL_IRCREG 		0X01
#define WRITE_ALL_IRCREG 		0X02
#define WRITE_ALL_IRCREG_SAME 0X30
//FLIP SFR
#define READ_ALL_SFRREG 		0X01
#define WRITE_ALL_SFRREG 		0X02
#define WRITE_ALL_SFRREG_SAME 0X30
#define PCA_TEST 				0X03
#define TIMER2_TEST 		0X04
#define TIMER0_TEST 		0X05
#define TIMER1_TEST 		0X06
//IDD
#define	ISB_VDD				0x00
#define	IDD_STOP			0x01
#define	IDD_SLEEP			0x02

#define	IDD_PDR				0x04
#define	IDD_REF125			0x05
#define	IDD_BUF125_CMP		0x06
#define	IDD_LVD				0x07

#define	IDD_XTHF_HFEN		0x09
#define	IDD_LCD_RESIN		0x0A
#define	IDD_LCD_RESOUT		0x0B
#define	IDD_LCD_CAPOUT		0x0C
#define	ISB_VBAT1			0x0D
#define	ISB_VBAT2			0x0E
#define	ISB_VBAT3			0x0F
#define	ISB_VBAT4			0x10
#define	ISB_RCSENSOR		0x11




#define	WORK_MIN_IDD		0x1F

#define	VBE			0x55

//#define	IDD_ADCDIS			0x22
#define	IDD_RC8M			0X22
#define	IDD_VREF			0x23
#define	IDD_ADCRUN			0x24
#define	IDD_PTAT			0x25
//RAM_TEST
#define IRAM				0x01
#define PXRAM				0x02
#define XRAM				0x03
#define NVRAM				0x04



//global variable and function--------------------------------

extern void Pro_SYS(void);
extern void Pro_SPI(void);
extern void Pro_I2C(void);
extern void Pro_U7816(void);
extern void Pro_RAM(void);
extern void Pro_UART(void);
extern void Pro_CLKGEN(void);
extern void Pro_TIMER(void);
extern void Pro_DISP(void);
extern void Pro_ANA(void);
extern void Pro_ADC(void);
extern void Pro_SPI(void);
extern void Pro_WDT(void);
extern void Pro_MOD(void);
extern void Pro_RTC(void);
extern void Pro_DMA(void);
extern void Pro_PMU(void);
extern void Pro_PAD(void);
extern void Pro_pmflash(void);
extern void Pro_IRC(void);
extern void Pro_FLIPSFR(void);
extern void Pro_IDD(void);
extern void Pro_SM3(void);
extern void Pro_XTMON(void);
extern void Pro_FLIP(void);
extern void Pro_FLSCTRL(void);
extern void Pro_CRC(void);
extern void Pro_LPTIM(void);
//------------------------------------------------------------
//------------------------------------------------------------
//未定义sfr
sfr VOID91 = 0x91;
sfr VOID92 = 0x92;
sfr VOID93 = 0x93;
sfr VOID94 = 0x94;
sfr VOID95 = 0x95;
sfr VOID9A = 0x9A;
sfr VOID9B = 0x9B;
sfr VOID9C = 0x9C;
sfr VOID9D = 0x9D;
sfr VOID9E = 0x9E;
sfr VOID9F = 0x9F;
sfr VOIDA7 = 0xA7;
sfr VOIDAA = 0xAA;
sfr VOIDAB = 0xAB;
sfr VOIDB4 = 0xB4;
sfr VOIDB5 = 0xB5;
sfr VOIDB6 = 0xB6;
sfr VOIDBA = 0xBA;
sfr VOIDBB = 0xBB;
sfr VOIDBC = 0xBC;
sfr VOIDBD = 0xBD;
sfr VOIDBF = 0xBF;
sfr VOIDC1 = 0xC1;
sfr VOIDC2 = 0xC2;
sfr VOIDC3 = 0xC3;
sfr VOIDC4 = 0xC4;
sfr VOIDC5 = 0xC5;
sfr VOIDC6 = 0xC6;
sfr VOIDC7 = 0xC7;
sfr VOIDCE = 0xCE;
sfr VOIDCF = 0xCF;
sfr VOIDD3 = 0xD3;
sfr VOIDEF = 0xEF;
sfr VOIDF4 = 0xF4;
sfr VOIDFF = 0xFF;
//有定义251未用
sfr IGNORE98 = 0x98;	//SCON
sfr IGNORE99 = 0x99;  //SBUF
sfr IGNOREA9 = 0x99;  //SADDR
sfr IGNOREB1 = 0xB1;  //SPCR
sfr IGNOREB2 = 0xB2;  //SPDR
sfr IGNOREB3 = 0xB3;  //SPSR
sfr IGNOREB9 = 0xB9;  //SADEN
sfr IGNORED2 = 0xD2;  //MIEN1
sfr IGNORED4 = 0xD4;  //MCADDR
sfr IGNOREE1 = 0xE1;  //MCON
sfr IGNOREE2 = 0xE2;  //MRXBUF
sfr IGNOREE3 = 0xE3;  //MTXBUF
sfr IGNOREE4 = 0xE4;  //MPRESC
sfr IGNOREE5 = 0xE5;  //MSTAT0
sfr IGNOREE6 = 0xE6;  //MSTAT1
sfr IGNOREE7 = 0xE7;  //MIEN0
sfr IGNORED5 = 0xD5;  //SIEN0
sfr IGNORED6 = 0xD6;  //SIEN1
sfr IGNORED7 = 0xD7;  //SSADDR
sfr IGNOREF1 = 0xF1;  //STCON
sfr IGNOREF2 = 0xF2;  //SRXBUF
sfr IGNOREF3 = 0xF3;  //STXBUF
sfr IGNOREF5 = 0xF5;  //SSTAT0
sfr IGNOREF6 = 0xF6;  //SSTAT1


#define read_all_sfr()  do{\
							TX_Buf[7] = P0;\
							TX_Buf[8] = SP;\
							TX_Buf[9] = DPL;\
							TX_Buf[10] = DPH;\
							TX_Buf[11] = DPXL;\
							TX_Buf[12] = XTALCON;\
							TX_Buf[13] = CLKCON;\
							TX_Buf[14] = PCON;\
							TX_Buf[15] = TCON;\
							TX_Buf[16] = TMOD;\
							TX_Buf[17] = TL0;\
							TX_Buf[18] = TL1;\
							TX_Buf[19] = TH0;\
							TX_Buf[20] = TH1;\
							TX_Buf[21] = CCMCON;\
							TX_Buf[22] = CCMVAL;\
							TX_Buf[23] = P1;\
							TX_Buf[24] = NMIFLAG;\
							TX_Buf[25] = INT0FLAG;\
							TX_Buf[26] = EXTRA0;\
							TX_Buf[27] = EXTRA1;\
							TX_Buf[28] = EXTRA2;\
							TX_Buf[29] = CPUINFO;\
							TX_Buf[30] = MMCON;\
							TX_Buf[31] = IGNORE98;\
							TX_Buf[32] = IGNORE99;\
							TX_Buf[33] = VOID9A;\
							TX_Buf[34] = VOID9B;\
							TX_Buf[35] = VOID9C;\
							TX_Buf[36] = VOID9D;\
							TX_Buf[37] = VOID9E;\
							TX_Buf[38] = VOID9F;\
							TX_Buf[39] = P2;\
							TX_Buf[40] = MPAGE;\
							TX_Buf[41] = PWMC;\
							TX_Buf[42] = PWMDCLSB;\
							TX_Buf[43] = PWMDCMSB;\
							TX_Buf[44] = WDTCON;\
							TX_Buf[45] = WDTRST;\
							TX_Buf[46] = VOIDA7;\
							TX_Buf[47] = IE;\
							TX_Buf[48] = IGNOREA9;\
							TX_Buf[49] = VOIDAA;\
							TX_Buf[50] = VOIDAB;\
							TX_Buf[51] = P0_DIR;\
							TX_Buf[52] = P1_DIR;\
							TX_Buf[53] = P2_DIR;\
							TX_Buf[54] = P3_DIR;\
							TX_Buf[55] = P3;\
							TX_Buf[56] = IGNOREB1;\
							TX_Buf[57] = IGNOREB2;\
							TX_Buf[58] = IGNOREB3;\
							TX_Buf[59] = VOIDB4;\
							TX_Buf[60] = VOIDB5;\
							TX_Buf[61] = VOIDB6;\
							TX_Buf[62] = IPH;\
							TX_Buf[63] = IPL;\
							TX_Buf[64] = IGNOREB9;\
							TX_Buf[65] = VOIDBA;\
							TX_Buf[66] = VOIDBB;\
							TX_Buf[67] = VOIDBC;\
							TX_Buf[68] = VOIDBD;\
							TX_Buf[69] = SPH;\
							TX_Buf[70] = VOIDBF;\
							TX_Buf[71] = AIF;\
							TX_Buf[72] = VOIDC1;\
							TX_Buf[73] = VOIDC2;\
							TX_Buf[74] = VOIDC3;\
							TX_Buf[75] = VOIDC4;\
							TX_Buf[76] = VOIDC5;\
							TX_Buf[77] = VOIDC6;\
							TX_Buf[78] = VOIDC7;\
							TX_Buf[79] = T2CON;\
							TX_Buf[80] = T2MOD;\
							TX_Buf[81] = RCAP2L;\
							TX_Buf[82] = RCAP2H;\
							TX_Buf[83] = TL2;\
							TX_Buf[84] = TH2;\
							TX_Buf[85] = VOIDCE;\
							TX_Buf[86] = VOIDCF;\
							TX_Buf[87] = PSW;\
							TX_Buf[88] = PSW1;\
							TX_Buf[89] = IGNORED2;\
							TX_Buf[90] = VOIDD3;\
							TX_Buf[91] = IGNORED4;\
							TX_Buf[92] = IGNORED5;\
							TX_Buf[93] = IGNORED6;\
							TX_Buf[94] = IGNORED7;\
							TX_Buf[95] = CCON;\
							TX_Buf[96] = CMOD;\
							TX_Buf[97] = CCAPM0;\
							TX_Buf[98] = CCAPM1;\
							TX_Buf[99] = CCAPM2;\
							TX_Buf[100] = CCAPM3;\
							TX_Buf[101] = CCAPM4;\
							TX_Buf[102] = CCAPO;\
							TX_Buf[103] = ACC;\
							TX_Buf[104] = IGNOREE1;\
							TX_Buf[105] = IGNOREE2;\
							TX_Buf[106] = IGNOREE3;\
							TX_Buf[107] = IGNOREE4;\
							TX_Buf[108] = IGNOREE5;\
							TX_Buf[109] = IGNOREE6;\
							TX_Buf[110] = IGNOREE7;\
							TX_Buf[111] = AIE;\
							TX_Buf[112] = CL;\
							TX_Buf[113] = CCAP0L;\
							TX_Buf[114] = CCAP1L;\
							TX_Buf[115] = CCAP2L;\
							TX_Buf[116] = CCAP3L;\
							TX_Buf[117] = CCAP4L;\
							TX_Buf[118] = VOIDEF;\
							TX_Buf[119] = B;\
							TX_Buf[120] = IGNOREF1;\
							TX_Buf[121] = IGNOREF2;\
							TX_Buf[122] = IGNOREF3;\
							TX_Buf[123] = VOIDF4;\
							TX_Buf[124] = IGNOREF5;\
							TX_Buf[125] = IGNOREF6;\
							TX_Buf[126] = AIPH;\
							TX_Buf[127] = AIPL;\
							TX_Buf[128] = CH;\
							TX_Buf[129] = CCAP0H;\
							TX_Buf[130] = CCAP1H;\
							TX_Buf[131] = CCAP2H;\
							TX_Buf[132] = CCAP3H;\
							TX_Buf[133] = CCAP4H;\
							TX_Buf[134] = VOIDFF;\
							}while(0)

			
#endif
