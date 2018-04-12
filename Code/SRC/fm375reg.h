/*
* Copyright (c) 2001,上海复旦微电子股份有限公司设计部
* All rights reserved.
*
* 文件名称： fm375Reg.h
* 文件标识：
* 摘要：FM375扩展寄存器及内部SFR地址定义
*
* 当前版本： 1.0
* 作者： xxx
*
*/

#ifndef _FM375REG_INCLUDED_
#define _FM375REG_INCLUDED_

///////////////////////////////////////////
//Flip80251 Special Functions Registers
/*  BYTE Registers  */
//IO PORTS SFRs
sfr P0        = 0x80;	//reset FFH
sfr P1        = 0x90;	//reset FFH
sfr P2        = 0xA0;	//reset FFH
sfr P3        = 0xB0;	//reset FFH
//PORT DIRECTION SFRs
sfr P0_DIR    = 0xAC;	//reset FFH
sfr P1_DIR    = 0xAD;	//reset FFH
sfr P2_DIR    = 0xAE;	//reset FFH
sfr P3_DIR    = 0xAF;	//reset FFH
//CORE SFRs
sfr SP        = 0x81;	//reset 07H
sfr DPL       = 0x82;
sfr DPH       = 0x83;
sfr DPXL      = 0x84;	//reset 01H
sfr PCON      = 0x87;
sfr TCON      = 0x88;
sfr MPAGE     = 0xA1;
sfr IE        = 0xA8;
sfr IPH       = 0xB7;
sfr IPL       = 0xB8;
sfr SPH       = 0xBE;
sfr PSW       = 0xD0;
sfr PSW1      = 0xD1;
sfr ACC       = 0xE0;
sfr B         = 0xF0;
//CPMU SFRs
sfr XTALCON   = 0x85;
sfr CLKCON    = 0x86;
//TIMER0/1 SFRs
sfr TMOD      = 0x89;
sfr TL0       = 0x8A;
sfr TL1       = 0x8B;
sfr TH0       = 0x8C;
sfr TH1       = 0x8D;

//TIMER2 SFRs
sfr T2CON     = 0xC8;
sfr T2MOD     = 0xC9;
sfr RCAP2L    = 0xCA;
sfr RCAP2H    = 0xCB;
sfr TL2       = 0xCC;
sfr TH2       = 0xCD;

//BIRD/POC SFRs
sfr CCMCON    = 0x8E;
sfr CCMVAL    = 0x8F;
sfr CPUINFO   = 0x96;
sfr MMCON     = 0x97;	//reset 07H

//PWM SFRs
sfr PWMC      = 0xA2;
sfr PWMDCLSB  = 0xA3;
sfr PWMDCMSB  = 0xA4;
//WDT SFRs
sfr WDTCON    = 0xA5;	//reset 07H
sfr WDTRST    = 0xA6;

//ADDITIONAL IRQ SFRs
sfr AIF       = 0xC0;
sfr AIE       = 0xE8;
sfr AIPH      = 0xF7;
sfr AIPL      = 0xF8;

//INCT SFR
sfr NMIFLAG		= 0x91;//NMI中断标志寄存器
sfr INT0FLAG	= 0x92;//INT0中断标志寄存器
sfr EXTRA0		= 0x93;//
sfr EXTRA1		= 0x94;//
sfr EXTRA2		= 0x95;//
//TIMER2 SFRs
sfr T2CON     = 0xC8;
sfr T2MOD     = 0xC9;
sfr RCAP2L    = 0xCA;
sfr RCAP2H    = 0xCB;
sfr TL2       = 0xCC;
sfr TH2       = 0xCD;
//PCA
sfr CCON      = 0xD8;
sfr CMOD      = 0xD9;
sfr CCAPM0    = 0xDA;
sfr CCAPM1    = 0xDB;
sfr CCAPM2    = 0xDC;
sfr CCAPM3    = 0xDD;
sfr CCAPM4    = 0xDE;
sfr CCAPO     = 0xDF;
sfr CL        = 0xE9;
sfr CCAP0L    = 0xEA;
sfr CCAP1L    = 0xEB;
sfr CCAP2L    = 0xEC;
sfr CCAP3L    = 0xED;
sfr CCAP4L    = 0xEE;
sfr CH        = 0xF9;
sfr CCAP0H    = 0xFA;
sfr CCAP1H    = 0xFB;
sfr CCAP2H    = 0xFC;
sfr CCAP3H    = 0xFD;
sfr CCAP4H    = 0xFE;

/*  BIT Registers  */
//P0
sbit P07      = P0^7;
sbit P06      = P0^6;
sbit P05      = P0^5;
sbit P04      = P0^4;
sbit P03      = P0^3;
sbit P02      = P0^2;
sbit P01      = P0^1;
sbit P00      = P0^0;
//P1
sbit P17      = P1^7;
sbit P16      = P1^6;
sbit P15      = P1^5;
sbit P14      = P1^4;
sbit P13      = P1^3;
sbit P12      = P1^2;
sbit P11      = P1^1;
sbit P10      = P1^0;
//P2
sbit P27      = P2^7;
sbit P26      = P2^6;
sbit P25      = P2^5;
sbit P24      = P2^4;
sbit P23      = P2^3;
sbit P22      = P2^2;
sbit P21      = P2^1;
sbit P20      = P2^0;
//P3
sbit P37      = P3^7;
sbit P36      = P3^6;
sbit P35      = P3^5;
sbit P34      = P3^4;
sbit P33      = P3^3;
sbit P32      = P3^2;
sbit P31      = P3^1;
sbit P30      = P3^0;

sbit RD       = P3^7;
sbit WR       = P3^6;
sbit T1       = P3^5;
sbit T0       = P3^4;
sbit INT1     = P3^3;
sbit INT0     = P3^2;
sbit TXD      = P3^1;
sbit RXD      = P3^0;
//TCON
sbit TF1      = TCON^7; //Timer1 overflow flag
sbit TR1      = TCON^6; //Timer1 run control bit
sbit TF0      = TCON^5; //Timer0 overflow flag
sbit TR0      = TCON^4; //Timer0 run control bit
sbit IE1      = TCON^3; //External interrupt 1 edge flag.Hardware controlled
sbit IT1      = TCON^2; //External interrupt 1 signal type control bit
sbit IE0      = TCON^1; //External interrupt 0 edge flag.Hardware controlled
sbit IT0      = TCON^0; //External interrupt 0 signal type control bit
//T2CON
sbit TF2      = T2CON^7; //Timer2 overflow flag
sbit EXF2     = T2CON^6; //Timer2 external flag
sbit RCLK     = T2CON^5; //Receive clock bit
sbit TCLK     = T2CON^4; //Transmit clock bit
sbit EXEN2    = T2CON^3; //Timer2 external enable bit
sbit TR2      = T2CON^2; //Timer2 run control bit
sbit CT2      = T2CON^1; //Timer2 counter/timer select
sbit CPRL2    = T2CON^0; //Capture reload bit
//IE
sbit EA       = IE^7; //Global interrupt enable
sbit EC       = IE^6; //PCA interrupt enable
sbit ET2      = IE^5; //Timer2 interrupt enable
sbit ES       = IE^4; //Serial port interrupt enable
sbit ET1      = IE^3; //Timer1 interrupt enable
sbit EX1      = IE^2; //External interrupt 1 interrupt enable
sbit ET0      = IE^1; //Timer0 interrupt enable
sbit EX0      = IE^0; //External interrupt 0 interrupt enable
//IPL
sbit IPLC     = IPL^6;
sbit IPLT2    = IPL^5;
sbit IPLS     = IPL^4;
sbit IPLT1    = IPL^3;
sbit IPLX1    = IPL^2;
sbit IPLT0    = IPL^1;
sbit IPLX0    = IPL^0;
//PSW
sbit CY       = PSW^7;
sbit AC       = PSW^6;
sbit F0       = PSW^5;
sbit RS1      = PSW^4;
sbit RS0      = PSW^3;
sbit OV       = PSW^2;
sbit UD       = PSW^1;
sbit P        = PSW^0;
//ACC
sbit ACC7     = ACC^7;
sbit ACC6     = ACC^6;
sbit ACC5     = ACC^5;
sbit ACC4     = ACC^4;
sbit ACC3     = ACC^3;
sbit ACC2     = ACC^2;
sbit ACC1     = ACC^1;
sbit ACC0     = ACC^0;
//AIF
sbit AIF6     = AIF^6; //Additonal interrupt 6 flag
sbit AIF5     = AIF^5; //Additonal interrupt 5 flag
sbit AIF4     = AIF^4; //Additonal interrupt 4 flag
sbit AIF3     = AIF^3; //Additonal interrupt 3 flag
sbit AIF2     = AIF^2; //Additonal interrupt 2 flag
sbit AIF1     = AIF^1; //Additonal interrupt 1 flag
sbit AIF0     = AIF^0; //Additonal interrupt 0 flag
//AIE
sbit AIE6     = AIE^6; //Additonal interrupt 6 enable
sbit AIE5     = AIE^5; //Additonal interrupt 5 enable
sbit AIE4     = AIE^4; //Additonal interrupt 4 enable
sbit AIE3     = AIE^3; //Additonal interrupt 3 enable
sbit AIE2     = AIE^2; //Additonal interrupt 2 enable
sbit AIE1     = AIE^1; //Additonal interrupt 1 enable
sbit AIE0     = AIE^0; //Additonal interrupt 0 enable
//AIPL
sbit AIPL6    = AIPL^6; //Additional interrupt 6 Priority Less significant bit
sbit AIPL5    = AIPL^5; //Additional interrupt 5 Priority Less significant bit
sbit AIPL4    = AIPL^4; //Additional interrupt 4 Priority Less significant bit
sbit AIPL3    = AIPL^3; //Additional interrupt 3 Priority Less significant bit
sbit AIPL2    = AIPL^2; //Additional interrupt 2 Priority Less significant bit
sbit AIPL1    = AIPL^1; //Additional interrupt 1 Priority Less significant bit
sbit AIPL0    = AIPL^0; //Additional interrupt 0 Priority Less significant bit


///////////////////////////////////////////


///////////////////////////////////////////
//FM340 Function modules Registers	 
//address for tb and C communication
#define	PRG_ADDR				EBYTE[0x07F0]	//程序地址
#define PRG_DATA				EBYTE[0x07F1]	//程序数据
#define PRG_FLAG				EBYTE[0x07F2]	//
#define TB_ADDR					EBYTE[0x07F3]	//TB地址
#define TB_DATA					EBYTE[0x07F4]	//TB数据
#define TB_FLAG					EBYTE[0x07F5]	//
#define CASE_NUM				EBYTE[0x07F6]	//
#define TEMP1					EBYTE[0x07F7]	//
#define TEMP2					EBYTE[0x07F8]	//
#define TEMP3					EBYTE[0x07F9]	//
#define TEMP4					EBYTE[0x07FA]	//
#define TEMP5					EBYTE[0x07FE]	//
#define TEMP6					EBYTE[0x07FF]	//
#define TEMP7					EBYTE[0x07E0]	//
#define TEMP8					EBYTE[0x07E1]	//
#define SYN						EBYTE[0x07FB]	//
#define DEBUG_C					EBYTE[0x07FE]	//
#define tbc						EBYTE[0x07FC] 	//TB产生的随机数传输给c
#define	tbc_flag				EBYTE[0x07FD]

//Mode Control Unit
#define AUTH  					XBYTE[0x0000]	//芯片权限模式寄存器
#define RDN0					XBYTE[0x0001]   //冗余扇区映射寄存器0
#define RDN1					XBYTE[0x0002]   //冗余扇区映射寄存器1
#define WTCFG					XBYTE[0x0003]	//
#define MDCSTA					    XBYTE[0x0004]	//

//PMU
#define PMU_CFG  				XBYTE[0x0020]   //
#define RAMLPCFG				XBYTE[0x0021]   //
#define PMU_WKSRC			    XBYTE[0x0022]   //
#define LPRUNERR					XBYTE[0x0023]	//
#define LPRUNERRIE			    XBYTE[0x0024]   //
#define PMU_PDCTRL					XBYTE[0x0030]	//
//Clock Control Unit
//RCC
#define HSDIVSEL				XBYTE[0x0040]
#define MCLKSEL				    XBYTE[0x0041]
#define LSCLKSEL				XBYTE[0x0042]
#define CKSRC_CTRL				XBYTE[0x0043]
#define RCHFADJ                 XBYTE[0x0044]
#define XTLF_IPWRB				XBYTE[0x0045]
#define RCLP_TRIM				XBYTE[0x0046]
#define PERICLK_CTRL0		    XBYTE[0x0047]	//时钟门控寄存器
#define	PERICLK_CTRL1			XBYTE[0x0048]
#define PERICLK_CTRL2           XBYTE[0x0049]
#define PLLDBH				    XBYTE[0x004a]	
#define PLLDBL				    XBYTE[0x004b]
#define SOFTRST                 XBYTE[0x004c]
#define RSTCFG     		        XBYTE[0x004d]    //AO PD
#define RSTFLAG     		    XBYTE[0x004e]


//LCD Display Unit
#define DISPCTRL                XBYTE[0x0060]
#define LCDTEST				    XBYTE[0x0061]	//显示测试控制寄存器
#define LCDDF					XBYTE[0x0062]	//显示频率控制寄存器
#define TON						XBYTE[0x0063]	//显示点亮时间寄存器
#define TOFF					XBYTE[0x0064]	//显示熄灭时间寄存器
#define DISPIE					XBYTE[0x0065] 	//显示中断使能寄存器
#define DISPIF					XBYTE[0x0066]	//显示中断标志寄存器
#define LCDSET					XBYTE[0x0067]	//LCD显示设置寄存器
#define ENMODE					XBYTE[0x0068]	//LCD驱动模式寄存器
#define DISPDATA0				XBYTE[0x0069]	//显示数据寄存器0
#define DISPDATA1				XBYTE[0x006A]	//显示数据寄存器1
#define DISPDATA2				XBYTE[0x006B]	//显示数据寄存器2
#define DISPDATA3				XBYTE[0x006C]	//显示数据寄存器3
#define DISPDATA4				XBYTE[0x006D]	//显示数据寄存器4
#define DISPDATA5				XBYTE[0x006E]	//显示数据寄存器5
#define DISPDATA6				XBYTE[0x006F]	//显示数据寄存器6
#define DISPDATA7				XBYTE[0x0070]	//显示数据寄存器7
#define DISPDATA8				XBYTE[0x0071]	//显示数据寄存器8
#define DISPDATA9				XBYTE[0x0072]	//显示数据寄存器9
#define DISPDATA10				XBYTE[0x0073]	//显示数据寄存器10
#define DISPDATA11				XBYTE[0x0074]	//显示数据寄存器11
#define DISPDATA12				XBYTE[0x0075]	//显示数据寄存器12
#define DISPDATA13				XBYTE[0x0076]	//显示数据寄存器13
#define DISPDATA14				XBYTE[0x0077]	//显示数据寄存器14
#define DISPDATA15				XBYTE[0x0078]	//显示数据寄存器15
#define DISPDATA16				XBYTE[0x0079]	//显示数据寄存器16
#define DISPDATA17				XBYTE[0x007A]	//显示数据寄存器17
#define LCDBIAS					XBYTE[0x007B]	//LCD显示灰度设置寄存器
#define COMSEG_EN1			    XBYTE[0x007C]	//LCD输出使能控制寄存器1
#define COMSEG_EN2			    XBYTE[0x007D]	//LCD输出使能控制寄存器2
#define COMSEG_EN3			    XBYTE[0x007E]	//LCD输出使能控制寄存器3
#define COMSEG_EN4			    XBYTE[0x007F]	//LCD输出使能控制寄存器4

//LPTIM
#define LPTCFG0					XBYTE[0x0080]
#define LPTCFG1					XBYTE[0x0081]
#define LPTCNTL					XBYTE[0x0082]
#define LPTCNTH					XBYTE[0x0083]
#define LPTCMPL					XBYTE[0x0084]
#define LPTCMPH					XBYTE[0x0085]
#define TARGETL					XBYTE[0x0086]
#define TARGETH					XBYTE[0x0087]
#define LPTIMIE				  XBYTE[0x0088]
#define LPTIMIF				  XBYTE[0x0089]
#define LPTCTRL					XBYTE[0x008A]

//RTC
#define RTCWE					XBYTE[0x00A0]
#define RTCIE1					XBYTE[0x00A1]
#define RTCIF1					XBYTE[0x00A2]
#define RTCIE2					XBYTE[0x00A3]
#define RTCIF2					XBYTE[0x00A4]
#define BCDSEC                  XBYTE[0x00A5]
#define BCDMIN                  XBYTE[0x00A6]
#define BCDHOUR                 XBYTE[0x00A7]
#define BCDDATE                 XBYTE[0x00A8]
#define BCDWEEK                 XBYTE[0x00A9]
#define BCDMONTH                XBYTE[0x00AA]
#define BCDYEAR                 XBYTE[0x00AB]
#define ALARMSEC                XBYTE[0x00AC]
#define ALARMMIN                XBYTE[0x00AD]
#define ALARMHOUR               XBYTE[0x00AE]
#define FSEL                    XBYTE[0x00AF]
#define ADJUST                  XBYTE[0x00B0]
#define ADJUST1                 XBYTE[0x00B1]
#define ADSIGN                  XBYTE[0x00B2]
#define PRLSEN                  XBYTE[0x00B3]

//Watch Dog Timer
#define WDTSERV					XBYTE[0x00C0]	
#define WDTCFG					XBYTE[0x00C1]	
#define WDTCNT0					XBYTE[0x00C2]
#define WDTCNT1					XBYTE[0x00C3]
#define WDTCNT2					XBYTE[0x00C4]

//Flash Control Unit
#define ERCSR					XBYTE[0x00E0]	//Flash擦除控制寄存器
#define PRCSR					XBYTE[0x00E1]	//Flash编程控制寄存器
#define FLSKEY                  XBYTE[0x00E3]   //Flash key
#define FLSIE					XBYTE[0x00E4]	//Flash中断使能寄存器
#define EPFLAG				    XBYTE[0x00E6]	//FLash擦写标志寄存器

//Pad Control Unit
#define    PAPPEN		        XBYTE[0x0100]
#define    PBPPEN				XBYTE[0x0101]
#define    PCPPEN		        XBYTE[0x0102]
#define    PDPPEN				XBYTE[0x0103]
#define    PEPPEN		        XBYTE[0x0104]
#define    PFPPEN				XBYTE[0x0105]
#define    PGPPEN		        XBYTE[0x0106]
#define    PH0PPEN		        XBYTE[0x0107]
#define    PAPUEN		        XBYTE[0x0108]
#define    PBPUEN		        XBYTE[0x0109]
#define    PCPUEN		        XBYTE[0x010A]
#define    PDPUEN		        XBYTE[0x010B]
#define    PEPUEN		        XBYTE[0x010C]
#define    PFPUEN		        XBYTE[0x010D]
#define    PGPUEN		        XBYTE[0x010E]
#define    PH0PUEN		        XBYTE[0x010F]
#define    PBODEN		        XBYTE[0x0110]
#define    PEODEN		        XBYTE[0x0111]
#define    PAFCR1		        XBYTE[0x0112]
#define    PBFCR1		        XBYTE[0x0113]
#define    PCFCR1		        XBYTE[0x0114]
#define    PDFCR1		        XBYTE[0x0115]
#define    PEFCR1		        XBYTE[0x0116]
#define    PFFCR1		        XBYTE[0x0117]
#define    PGFCR1		        XBYTE[0x0118]
#define    PHFCR1		        XBYTE[0x0119]
#define    PAFCR2		        XBYTE[0x011A]
#define    PBFCR2		        XBYTE[0x011B]
#define    PCFCR2		        XBYTE[0x011C]
#define    PDFCR2		        XBYTE[0x011D]
#define    PEFCR2		        XBYTE[0x011E]
#define    PFFCR2		        XBYTE[0x011F]
#define    PGFCR2		        XBYTE[0x0120]
#define    PHFCR2		        XBYTE[0x0121]
#define    PADATA               XBYTE[0x0122]
#define    PBDATA               XBYTE[0x0123]
#define    PCDATA               XBYTE[0x0124]
#define    PDDATA               XBYTE[0x0125]
#define    PEDATA               XBYTE[0x0126]
#define    PFDATA               XBYTE[0x0127]
#define    PGDATA               XBYTE[0x0128]
#define    PH0DATA              XBYTE[0x0129]
#define    PADIN                XBYTE[0x012A]
#define    PBDIN                XBYTE[0x012B]
#define    PCDIN                XBYTE[0x012C]
#define    PDDIN                XBYTE[0x012D]
#define    PEDIN                XBYTE[0x012E]
#define    PFDIN                XBYTE[0x012F]
#define    PGDIN                XBYTE[0x0130]
#define    PH0DIN               XBYTE[0x0131]
#define    AFSELA               XBYTE[0x0132]
#define    AFSELB               XBYTE[0x0133]
#define    AFSELC               XBYTE[0x0134]
#define    AFSELD               XBYTE[0x0135]
#define    AFSELE               XBYTE[0x0136]
#define    AFSELF               XBYTE[0x0137]
#define    AFSELG               XBYTE[0x0138]
#define	   GPIO_EXTI_SEL0		XBYTE[0x0139]
#define	   GPIO_EXTI_SEL1		XBYTE[0x013A]
#define	   GPIO_EXTI_SEL2		XBYTE[0x013B]
#define	   GPIO_EXTI_SEL3		XBYTE[0x013C]
#define	   GPIO_EXTIL_ES0		XBYTE[0x013D]
#define	   GPIO_EXTIL_ES1		XBYTE[0x013E]
#define	   GPIO_EXTIH_ES0		XBYTE[0x013F]
#define	   GPIO_EXTIH_ES1		XBYTE[0x0140]
#define	   EXTILIF		        XBYTE[0x0141]
#define	   EXTIHIF		        XBYTE[0x0142]
#define	   FOUT0_SEL		   		XBYTE[0x0143]
#define	   FOUT1_SEL		    	XBYTE[0x0144]
#define    PAOS                 XBYTE[0x0145]
#define    PBOS                 XBYTE[0x0146]
#define    PCOS                 XBYTE[0x0147]
#define    PDOS                 XBYTE[0x0148]
#define    PEOS                 XBYTE[0x0149]
#define    PFOS                 XBYTE[0x014A]
#define    PGOS                 XBYTE[0x014B]
#define    PH0OS                XBYTE[0x014C]

//CRC
#define    CRC_DRL			    XBYTE[0x0200]
#define    CRC_DRH				XBYTE[0x0201]
#define    CRC_CR			    XBYTE[0x0202]
#define    CRC_CAL0				XBYTE[0x0203]
#define    CRC_CAL1				XBYTE[0x0204]
#define    CRC_XOR0			    XBYTE[0x0205]
#define    CRC_XOR1				XBYTE[0x0206]
#define    CRC_FLSCRC			XBYTE[0x0207]

//DMA
#define     GCTRL			    XBYTE[0x0220]
#define     CH0_CTRL			XBYTE[0x0221]
#define     CH0_LEN			    XBYTE[0x0222]
#define     CH0_ADDRH			XBYTE[0x0223]
#define     CH0_ADDRL			XBYTE[0x0224]
#define     CH1_CTRL			XBYTE[0x0225]
#define     CH1_LEN			    XBYTE[0x0226]
#define     CH1_ADDRH			XBYTE[0x0227]
#define     CH1_ADDRL			XBYTE[0x0228]
#define     CH2_CTRL			XBYTE[0x0229]
#define     CH2_LEN			    XBYTE[0x022A]
#define     CH2_SADDRH			XBYTE[0x022B]
#define     CH2_SADDRL			XBYTE[0x022C]
#define     CH2_DADDRH			XBYTE[0x022D]
#define     CH2_DADDRL			XBYTE[0x022E]
#define     CH0STA				XBYTE[0x022F]
#define     CH0IE				XBYTE[0x0230]
#define     CH1STA				XBYTE[0x0231]
#define     CH1IE				XBYTE[0x0232]
#define     CH2STA				XBYTE[0x0233]
#define     CH2IE				XBYTE[0x0234]

//U7816
#define U7816CTRL0               XBYTE[0x0240]   //U7816通道0控制寄存器
#define U7816CTRL1               XBYTE[0x0241]   //U7816通道1端口寄存器
#define U7816FRMCTRL0      			 XBYTE[0x0242]	//U7816帧格式控制寄存器0
#define U7816FRMCTRL1      			 XBYTE[0x0243]   //U7816帧格式控制寄存器1
#define U7816EGTCTRL     				 XBYTE[0x0244]   //U7816 EGT配置寄存器
#define CLK_DIV                  XBYTE[0x0245]   //U7816工作时钟分频控制寄存器
#define PRE_DIVH                 XBYTE[0x0246]	//U7816预分频控制寄存器高位
#define PRE_DIVL                 XBYTE[0x0247]	//U7816预分频控制寄存器低位
#define U7816RXBUF              XBYTE[0x0248]   //U7816通道0数据接收缓冲寄存器
#define U7816TXBUF              XBYTE[0x0249]   //U7816通道0数据发送缓冲寄存器
#define U7816INTEN              XBYTE[0x024A]	//U7816中断标志寄存器
#define U7816PRIMARYSTATUS     	 XBYTE[0x024B]   //U7816主状态寄存器
#define U7816ERRSTATUS       		 XBYTE[0x024C]	//U7816错误信息寄存器
#define U7816SECONDSTATUS   		 XBYTE[0x024D]   //U7816次状态寄存器

//I2C
#define SSPCON					XBYTE[0x0260]	
#define SSPSTAT					XBYTE[0x0261]
#define SSPBRG					XBYTE[0x0262]
#define SSPBUF					XBYTE[0x0263]
#define SSPIR						XBYTE[0x0264]
#define SSPBRGH					XBYTE[0x0265]
#define SSPFSM					XBYTE[0x0266]
#define SSPERR					XBYTE[0x0267]

//SPI
#define SPCR1                   XBYTE[0x0280]   //SPI控制寄存器1
#define SPCR2                   XBYTE[0x0281]   //SPI控制寄存器2
#define SPCR3                   XBYTE[0x0282]	 //SPI控制寄存器3
#define SPCR4                   XBYTE[0x0283]   //SPI状态寄存器4
#define SPIIE                   XBYTE[0x0284]   //SPI中断使能寄存器
#define SPSR                    XBYTE[0x0285]   //SPI状态寄存器
#define TXBUF                   XBYTE[0x0286]	 //SPI发送数据寄存器
#define RXBUF	                  XBYTE[0x0287]   //SPI接收数据寄存器

//UART
#define UARTIE					XBYTE[0x02A0]	//UART中断允许寄存器
#define UARTIF					XBYTE[0x02A1]	//UART中断标志寄存器
#define TZBRGH					XBYTE[0x02A2]	//UART红外调制频率高位寄存器
#define TZBRGL					XBYTE[0x02A3]	//UART红外调制频率低位寄存器
#define RXSTA0					XBYTE[0x02A4]	//接收状态控制寄存器0
#define RXSTA1					XBYTE[0x02A5]	//接收状态控制寄存器1
#define RXSTA2					XBYTE[0x02A6]	//接收状态控制寄存器2
#define RXSTA3					XBYTE[0x02A7]	//接收状态控制寄存器3

#define TXSTA0 					XBYTE[0x02A8]	//发送状态控制寄存器0
#define TXSTA1 					XBYTE[0x02A9]	//发送状态控制寄存器1
#define TXSTA2 					XBYTE[0x02AA]	//发送状态控制寄存器2
#define TXSTA3 					XBYTE[0x02AB]	//发送状态控制寄存器3

#define RXREG0					XBYTE[0x02AC]	//接收数据缓冲寄存器0
#define RXREG1					XBYTE[0x02AD]	//接收数据缓冲寄存器1
#define RXREG2					XBYTE[0x02AE]	//接收数据缓冲寄存器2
#define RXREG3					XBYTE[0x02AF]	//接收数据缓冲寄存器3

#define TXREG0					XBYTE[0x02B0]	//发送数据缓冲寄存器0
#define TXREG1					XBYTE[0x02B1]	//发送数据缓冲寄存器1
#define TXREG2					XBYTE[0x02B2]	//发送数据缓冲寄存器2
#define TXREG3					XBYTE[0x02B3]	//发送数据缓冲寄存器3

#define SPBRGH0					XBYTE[0x02B4]	//波特率产生器高位寄存器0
#define SPBRGH1					XBYTE[0x02B5]	//波特率产生器高位寄存器1
#define SPBRGH2					XBYTE[0x02B6]	//波特率产生器高位寄存器2
#define SPBRGH3					XBYTE[0x02B7]	//波特率产生器高位寄存器3

#define SPBRGL0					XBYTE[0x02B8]	//波特率产生器低位寄存器0
#define SPBRGL1					XBYTE[0x02B9]	//波特率产生器低位寄存器1
#define SPBRGL2					XBYTE[0x02BA]	//波特率产生器低位寄存器2
#define SPBRGL3					XBYTE[0x02BB]	//波特率产生器低位寄存器3

#define TXFIFOSTA0				XBYTE[0x02BC]	//发送FIFO状态控制寄存器0
#define TXFIFOSTA1				XBYTE[0x02BD]	//发送FIFO状态控制寄存器1
#define TXFIFOSTA2				XBYTE[0x02BE]	//发送FIFO状态控制寄存器2
#define TXFIFOSTA3				XBYTE[0x02BF]	//发送FIFO状态控制寄存器3

#define RXFIFOSTA0				XBYTE[0x02C0]	//接收FIFO状态控制寄存器0
#define RXFIFOSTA1				XBYTE[0x02C1]	//接收FIFO状态控制寄存器1
#define RXFIFOSTA2				XBYTE[0x02C2]	//接收FIFO状态控制寄存器2
#define RXFIFOSTA3				XBYTE[0x02C3]	//接收FIFO状态控制寄存器3

#define RTXCON0					XBYTE[0x02C4]	//接收发送取反控制寄存器0
#define RTXCON1					XBYTE[0x02C5]	//接收发送取反控制寄存器1
#define RTXCON2					XBYTE[0x02C6]	//接收发送取反控制寄存器2
#define RTXCON3					XBYTE[0x02C7]	//接收发送取反控制寄存器3

//Rambist
#define RAMBISTCTL				XBYTE[0x02E0]	//RAMBIST控制寄存器
#define RAMBISTSTA				XBYTE[0x02E1]	//RAMBIST状态寄存器
#define RAMPARITY			    XBYTE[0x02E2]	//RAMBIST校验错误寄存器
#define RAMPARITYIE		    XBYTE[0x02E3]	//

 //ETIM
#define  ET1CTRL1               XBYTE[0x0300]
#define  ET1CTRL2               XBYTE[0x0301]
#define  ET1CFG1                XBYTE[0x0302]
#define  ET1CFG2                XBYTE[0x0303]
#define  ET1PRESCALE            XBYTE[0x0304]
#define  ET1LOADCTRL            XBYTE[0x0305]
#define  ET1CNTL                XBYTE[0x0306]
#define  ET1CNTH                XBYTE[0x0307]
#define  ET1PRESETL             XBYTE[0x0308]
#define  ET1PRESETH             XBYTE[0x0309]
#define  ET1LOADL               XBYTE[0x030A]
#define  ET1LOADH               XBYTE[0x030B]
#define  ET1CMPL                XBYTE[0x030C]
#define  ET1CMPH                XBYTE[0x030D]
#define  ET1OUTCNTL             XBYTE[0x030E]
#define  ET1OUTCNTH             XBYTE[0x030F]
#define  ET1OUTCTRL             XBYTE[0x0310]
#define  ET1IE                  XBYTE[0x0311]
#define  ET1IF                  XBYTE[0x0312]

#define  ET2CTRL1               XBYTE[0x0313]
#define  ET2CTRL2               XBYTE[0x0314]
#define  ET2CFG1                XBYTE[0x0315]
#define  ET2CFG2                XBYTE[0x0316]
#define  ET2PRESCALE            XBYTE[0x0317]
#define  ET2LOADCTRL            XBYTE[0x0318]
#define  ET2CNTL                XBYTE[0x0319]
#define  ET2CNTH                XBYTE[0x031A]
#define  ET2PRESETL             XBYTE[0x031B]
#define  ET2PRESETH             XBYTE[0x031C]
#define  ET2LOADL               XBYTE[0x031D]
#define  ET2LOADH               XBYTE[0x031E]
#define  ET2CMPL                XBYTE[0x031F]
#define  ET2CMPH                XBYTE[0x0320]
#define  ET2OUTCNTL             XBYTE[0x0321]
#define  ET2OUTCNTH             XBYTE[0x0322]
#define  ET2OUTCTRL             XBYTE[0x0323]
#define  ET2IE                  XBYTE[0x0324]
#define  ET2IF                  XBYTE[0x0325]

#define  ET3CTRL                XBYTE[0x0326]
#define  ET3INSEL               XBYTE[0x0327]
#define  ET3PRESCALE1           XBYTE[0x0328]
#define  ET3PRESCALE2           XBYTE[0x0329]
#define  ET3STARTL              XBYTE[0x032A]
#define  ET3STARTH              XBYTE[0x032B]
#define  ET3IE                  XBYTE[0x032C]
#define  ET3IF                  XBYTE[0x032D]

#define  ET4CTRL                XBYTE[0x032E]
#define  ET4INSEL               XBYTE[0x032F]
#define  ET4PRESCALE1           XBYTE[0x0330]
#define  ET4PRESCALE2           XBYTE[0x0331]
#define  ET4STARTL              XBYTE[0x0332]
#define  ET4STARTH              XBYTE[0x0333]
#define  ET4IE                  XBYTE[0x0334]
#define  ET4IF                  XBYTE[0x0335]

//PCA
#define  CCON_REG                    XBYTE[0x0340]    
#define  CMOD_REG                    XBYTE[0x0341]
#define  CH_REG                      XBYTE[0x0342]
#define  CL_REG                      XBYTE[0x0343]
#define  CCAP0H_REG                  XBYTE[0x0344]
#define  CCAP1H_REG                  XBYTE[0x0345]
#define  CCAP2H_REG                  XBYTE[0x0346]
#define  CCAP3H_REG                  XBYTE[0x0347]
#define  CCAP4H_REG                  XBYTE[0x0348]  
#define  CCAP0L_REG                  XBYTE[0x0349]
#define  CCAP1L_REG                  XBYTE[0x034A]
#define  CCAP2L_REG                  XBYTE[0x034B]
#define  CCAP3L_REG                  XBYTE[0x034C]
#define  CCAP4L_REG                  XBYTE[0x034D]
#define  CCAPM0_REG                  XBYTE[0x034E]
#define  CCAPM1_REG                  XBYTE[0x034F]
#define  CCAPM2_REG                  XBYTE[0x0350]
#define  CCAPM3_REG                  XBYTE[0x0351]
#define  CCAPM4_REG                  XBYTE[0x0352]

#define  CCAPO_REG                   XBYTE[0x0353]
#define  PWMPREL_REG                 XBYTE[0x0354]
#define  ECISAMPLE_REG               XBYTE[0x0355]

//ANALOG CTRL
#define  PDRCTL     XBYTE[0x0360]
#define  LDOCTL		XBYTE[0x0361]
#define  BGCTL		XBYTE[0x0362]

#define  IPWRCTL	XBYTE[0x0365]
#define  ULPCTL		XBYTE[0x0366]
#define  LVDCTRL	XBYTE[0x0367]
#define  LVDSTAT    XBYTE[0x0368]
#define  LVDLPC	    XBYTE[0x0369]
#define  FDETIE	    XBYTE[0x036a]
#define  FDETIF	    XBYTE[0x036b]
#define  BORCTL	    XBYTE[0x036c]

#define  ADCCTL     XBYTE[0x036e]
#define  ADCTRIM1   XBYTE[0x036f]
#define  ADCTRIM2   XBYTE[0x0370]
#define  ADCDATA1   XBYTE[0x0371]
#define  ADCDATA2   XBYTE[0x0372]
#define  ADCIF      XBYTE[0x0373]
#define  ANATESTSEL XBYTE[0x0374]            

//module register addr----------------------------------------
#define PAD_ADDR		0x0100
#define DISP_ADDR		0x0060
#define MOD_ADDR		0x0000
#define ADC_ADDR		0x0300
#define ANA_ADDR		0x0360
#define RTC_ADDR		0x00A0
#define WDT_ADDR		0x00C0
#define I2C_ADDR		0x0120
#define SPI_ADDR		0x0280
#define PMU_ADDR		0x0020
#define DMA_ADDR		0x0220
#define U7816_ADDR	0x0240
#define RCC_ADDR		0x0040
#define CRC_ADDR		0x0200
#define LPTIM_ADDR		0x0080
//------------------------------------------------------------

//RAM_ADDR--------------------
#define RAM_ADDR		0x000000    
// #define RAM_ADDR		0x001000


#endif






