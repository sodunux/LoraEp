/************************************************************************/
/* 	寄存器位定义头文件									                */
/* 	主要功能:															*/
/* 		1.描述寄存器位功能，便于修改		                            */
/*	硬件平台：	 													  	*/
/*		FM375		                        	*/
/* 	编制:xudayong														*/
/* 	编制时间:2016年03月30日												*/
/************************************************************************/

#ifndef _FM375REG_DEFINE_H_
#define _FM375REG_DEFINE_H_

#include "bin2hex.h"
//-------------------------------------------------------------
//Pad Control Unit---------------------------------------------------
//PortX functional config
//AFSELA
#define afsela7_p07 		  B0000_0000
#define afsela7_t12in0 		B1000_0000
#define afsela6_p06  			B0000_0000
#define afsela6_pcacapt0	B0100_0000
#define afsela3_ssn   		B0000_0000
#define afsela3_pcacomp1  B0000_1000
#define afsela2_sck   		B0000_0000
#define afsela2_pcacomp0  B0000_0100
#define afsela1_miso   		B0000_0000
#define afsela1_txd1   		B0000_0010
#define afsela0_mosi   		B0000_0000
#define afsela0_rxd1   		B0000_0001
//AFSELB
#define afselb7_sda			 	B0000_0000
#define afselb7_wkup1		 	B1000_0000
#define afselb6_scl       B0000_0000
#define afselb6_lptrg     B0100_0000
#define afselb5_p05       B0000_0000
#define afselb5_lptout    B0010_0000
#define afselb4_p04       B0000_0000
#define afselb4_lpti      B0001_0000
#define afselb3_p03       B0000_0000
#define afselb3_7816io0   B0000_1000
#define afselb2_p02       B0000_0000
#define afselb2_7816clk   B0000_0100
#define afselb1_p01       B0000_0000
#define afselb1_txd0      B0000_0010
#define afselb0_p00       B0000_0000
#define afselb0_rxd0      B0000_0001
//AFSELF
#define afself6_rtccalib	B0000_0000
#define afself6_wkup3 		B0100_0000
#define afself5_fout0 		B0000_0000
#define afself5_pcaeci 		B0010_0000
#define afself4_txd0   		B0000_0000
#define afself4_it2ckout  B0001_0000
#define afself3_rxd0   		B0000_0000
#define afself3_it2in   	B0000_1000
#define afself2_txd1   		B0000_0000
#define afself2_it1in   	B0000_0100
#define afself1_rxd1   		B0000_0000
#define afself1_it0in   	B0000_0010
//AFSELG
#define afselg6_txd3				B0000_0000
#define afselg6_pcacapt2 		B0100_0000
#define afselg5_rxd3 				B0000_0000
#define afselg5_pcacapt1 		B0010_0000
#define afselg4_fout0   		B0000_0000
#define afselg4_pcacapt0  	B0001_0000
//AFSELC
#define afselc7_p17			 	B0000_0000
#define afselc7_t3in0		 	B1000_0000
#define afselc6_p16       B0000_0000
#define afselc6_t3in1     B0100_0000
#define afselc5_p15       B0000_0000
#define afselc5_t4in0    B0010_0000
#define afselc4_p14       B0000_0000
#define afselc4_t4in1      B0001_0000
#define afselc3_p13       B0000_0000
#define afselc3_fou1   		B0000_1000
#define afselc2_p12       B0000_0000
#define afselc2_pcacomp0   B0000_0100
#define afselc0_p10       B0000_0000
#define afselc0_t12in1      B0000_0001



//-------------------------------------------------------------
//WDT ---------------------------------------------------
//WDTCFG
#define wdtov_time_125MS    B0000_0000  //125MS
#define wdtov_time_500MS    B0000_0001  //500MS
#define wdtov_time_2S    		B0000_0010  //2S
#define wdtov_time_8S    		B0000_0011  //8S
#define wdtov_time_4096S    B0000_0111  //4096S

//-------------------------------------------------------------
//UART ---------------------------------------------------
//UARTIF
#define rxif3	B1000_0000
#define txif3	B0100_0000
#define rxif2	B0010_0000
#define txif2	B0001_0000
#define rxif1	B0000_1000
#define txif1	B0000_0100
#define rxif0	B0000_0010
#define txif0	B0000_0001
//UARTIE
#define rxie3	B1000_0000
#define txie3	B0100_0000
#define rxie2	B0010_0000
#define txie2	B0001_0000
#define rxie1	B0000_1000
#define txie1	B0000_0100
#define rxie0	B0000_0010
#define txie0	B0000_0001
//RXSTA
#define pdsel_8_n	  	B0000_0000		//模式选择
#define pdsel_8_even 	B0100_0000
#define pdsel_8_odd  	B1000_0000
#define pdsel_9_n			B1100_0000
#define uarterrie			B0010_0000	//错误中断允许
#define rxen					B0001_0000	//接收使能
#define perr					B0000_1000	//奇偶校验错标志
#define ferr					B0000_0100	//帧格式错标志
#define oerr					B0000_0010	//溢出错标志
#define rx9d					B0000_0001	//接收第9位
//TXSTA

#define stopsel		B0100_0000	//停止位选择位
#define txis			B0010_0000	//发送中断选择
#define txen			B0001_0000	//发送使能
#define iren			B0000_1000	//发送红外调制使能
#define tsrf			B0000_0010	//发送移位寄存器空标志
#define tx9d			B0000_0001	//发送第9位
//RTXCON0
#define rxdflag  	B0000_0010
#define txdflag  	B0000_0001
//TXFIFOSTA
#define fifo_empty_int		 B0000_1000
#define fifo_tsr_empty_int B0000_0100
#define fifo_cmpta_old_int B0000_0000
#define txff						  	B0000_0001
//RXFIFOSTA
#define fifo_full_int B0000_0100

//---------------------------------------------
//MCLKSEL
#define	hsclk_sel_div				B0000_0010
#define	hsclk_no_sel_div		B0000_0000
#define	mclk_hsclk					B0000_0000
#define	mclk_lsclk					B0000_0001
//LSCLKSEL
#define lptim_fcken					B0000_0100    //计数源选择系统时钟时，需使能此位
#define	svd_clken						B0000_0010
#define	svd_lcd_clk_rclp		B0000_0000
#define	svd_lcd_clk_xtlf		B0000_0001
//HSDIVSEL
#define hsclk_rchf					B0000_0000
#define	hsclk_pll						B0000_0100
#define	hsclk_div2					B0000_0000
#define	hsclk_div4					B0000_0001
#define	hsclk_div8					B0000_0010
#define	hsclk_div16					B0000_0011
//CKSRC_CTRL
#define rchf_8M							B0000_0000
#define	rchf_16M						B0100_0000
#define	rchf_24M						B1000_0000
#define	rchf_32M						B1100_0000
#define	pll_en							B0000_0100
#define	rchf_en							B0000_0010
#define	lp_rclpctrl					B0000_0001
//PERICLK_CTRL0
#define crc_clken							B1000_0000
#define et1_clken							B0100_0000
#define et2_clken							B0010_0000
#define et34_clken						B0001_0000
#define dma_clken   					B0000_1000
#define flsc_clken   					B0000_0100
#define rambist_clken   			B0000_0010
#define lcd_clken   					B0000_0001
//PERICLK_CTRL1
#define uart_per_clken			B1000_0000
#define u7816_clken					B0100_0000
#define i2c_clken						B0010_0000
#define spi_clken						B0001_0000
#define	uart3_clken   			B0000_1000
#define uart2_clken   			B0000_0100
#define uart1_clken   			B0000_0010
#define uart0_clken   			B0000_0001
//PERICLK_CTRL2
#define lptim_rcken					B1000_0000  //总线时钟
#define adc_clken					B0100_0000
#define pca_clken					B0010_0000
#define pdc_clken					B0001_0000
#define	wdt_clken   			B0000_1000
#define anac_clken   			B0000_0100
#define rtc_clken   			B0000_0010
#define adc_clk_512k   		B0000_0000
#define adc_clk_1M   			B0000_0001


//--------------------------------------------------------
//PMU_CFG
#define active_mode					B0000_0000
#define	lprun_mode					B0100_0000
#define	sleep_mode					B1000_0000
#define	stop_mode						B1100_0000
#define	vref_off						B0010_0000
#define	cvs_en							B0001_0000
#define	xtlf_off						B0000_0001
//RAMLPCFG
#define	ram1_lpmode				B0000_0010
#define	ram0_lpmode				B0000_0001
//PCON
#define idl   		B0000_0001
#define pwd   		B0000_0010
//-------------------------------------------------------------
//CRC module---------------------------------------------------
//CRC_CR
#define	REFLECTIN				B0100_0000
#define REFLECTOUT   		B0010_0000
#define crc_busy      	B0000_1000
#define XOROUT 	      	B0000_0100
#define CRC_16					B0000_0000
#define CRC_8						B0000_0001
#define CRC_CCITT				B0000_0010
//FLSCRCEN
#define FLSCRCEN 	      	B0000_0100

//-------------------------------------------------------------

//RAMBISTCTL
#define rambist_start B0000_0001
//RAMBISTSTA
#define rambist_error B0000_0010
#define rambist_done	B0000_0001
//RAMPARITY
#define ramparity_eif	B0000_0001
//RAMPARITYIE
#define ramparity_ie	B0000_0001

//-------------------------------------------------------------


//---------------------------------------------------------
//ADCCTL
#define adc_ie						B1000_0000
#define	adc_vana_en				B0000_0010
#define	adc_en						B0000_0001
#define adc_if   					B0000_0001
//-------------------------------------------------------------
//ANA module---------------------------------------------------
//FDETIE
#define lfdet_ie   					B0000_0001
//FDETIF
#define lfdet_out_b						B0100_0000
#define lfdet_if   					B0000_0001

//LVDCTRL
#define lvdpu_ie					B1000_0000
#define	lvdpd_ie					B0100_0000
#define lvd_enable   		B0010_0000
#define lvd_disable  		B0000_0000
#define lvd_en_batch 		B0001_0000

#define lvd_1_8v      B0000_0000
#define lvd_2_03v      B0000_0001
#define lvd_2_26v      B0000_0010
#define lvd_2_49v      B0000_0011
#define lvd_2_72v      B0000_0100
#define lvd_2_95v      B0000_0101
#define lvd_3_19v      B0000_0110
#define lvd_3_42v      B0000_0111
#define lvd_3_69v      B0000_1000
#define lvd_3_88v      B0000_1001
#define lvd_4_11v      B0000_1010
#define lvd_4_34v      B0000_1011
#define lvd_4_57v      B0000_1100
#define lvd_4_80v      B0000_1101
#define lvd_svs0      B0000_1110
#define lvd_svs1      B0000_1111
//LVDLPC
#define lvd_testen					B1000_0000
#define dsef_4s  			B0000_0000
#define dsef_2s   		B0000_0001
#define dsef_1s   		B0000_0010
#define dsef_512ms   	B0000_0011
#define dsef_256ms    B0000_0100
#define dsef_128ms    B0000_0101  
#define dsef_64ms    	B0000_0110
#define dsef_32ms    	B0000_0111
//LVDSTAT
#define lvdout_b   					B0000_0100
#define lvdpu_if   					B0000_0010
#define lvdpd_if   					B0000_0001

//PDRCTRL
#define pdr_en    B0000_0001
//BORCTRL
#define off_bor    B0000_0001

//NMIFLAG
#define lfdet_int 				B0000_0010
#define pmu_int 					B0000_0001
//RAMPARITY
#define ramparity_if 					B0000_0001
//PMU_WKSRC
#define wkflag_clr_n				B1000_0000
#define	testen_wkup_flag		B0000_1000
#define wkup1_b7   					B0000_0001
#define wkup2_d1	  				B0000_0010
#define wkup3_f6   					B0000_0011
#define wkup4_h0   					B0000_0100
#define wkup_rxd2   				B0000_0101
#define mul_wkup   					B0000_0110
#define err_wkup   					B0000_0111
//LPRUNERR
#define lp_run_err 					B0000_0001
//LPRUNERRIE
#define lprun_ie 					B0000_0001
//PMU_PDCTRL
#define disc_en 					B0000_0001

//-------------------------------------------------------------

//SFR----------------------------------------------------------
//T2MOD
#define DCEN     B0000_0001
#define T2OE     B0000_0010
//CMOD
#define CIDL     B1000_0000 //1-idle模式定时器停止
#define WDTE     B0100_0000 //1-pca看门狗使能
#define CPS_MOD0     B0000_0000 //输入CLKDIV12
#define CPS_MOD1     B0000_0010 //输入CLKDIV4
#define CPS_MOD2     B0000_0100 //输入TIMER0OVERFLOW
#define CPS_MOD3     B0000_0110 //输入ECI(最大频率为系统时钟8分频)
#define ECF      		 B0000_0001 //1-PCA定时计数溢出中断
//PCON
#define POF						B0011_0000     //POWER OFF FLAG
#define GF1   				B0000_1000
#define GF0   				B0000_0100
#define PD   					B0000_0010
#define IDL   				B0000_0001


//-------------------------------------------------------------
//RTCIE1
#define adj128_ie 	B0001_0000
#define alarm_ie  	B0000_1000
#define hz1k_ie		  B0000_0100
#define hz256_ie 		B0000_0010
#define hz64_ie 		B0000_0001
//RTCIF1
#define adj128_if 	B0001_0000
#define alarm_if  	B0000_1000
#define hz1k_if		  B0000_0100
#define hz256_if 		B0000_0010
#define hz64_if 		B0000_0001
//RTCIE2
#define hz16_ie B0000_0001
#define hz8_ie  B0000_0010
#define hz4_ie  B0000_0100
#define hz2_ie  B0000_1000
#define secie 	B0001_0000
#define minie  	B0010_0000
#define hourie  B0100_0000
#define dayie 	B1000_0000
//RTCIF2
#define hz16_if B0000_0001
#define hz8_if  B0000_0010
#define hz4_if  B0000_0100
#define hz2_if  B0000_1000
#define secif 	B0001_0000
#define minif  	B0010_0000
#define hourif  B0100_0000
#define dayif 	B1000_0000

//FSEL
#define 	tm1s_sqr         	B0000_0000  //pll得到的精确的秒时标
#define 	tm80ms_pul       	B0000_0001  //PLL分频的高电平宽度80ms的秒时标
#define 	bcdsec_ovf   			B0000_0010  //输出秒计数器进位信号，高电平宽度1s
#define 	bcdmin_ovf  			B0000_0011  //输出分计数器进位信号，高电平宽度1s
#define 	bcdhour_ovf  			B0000_0100  //输出小时计数器进位信号，高电平宽度1s
#define 	bcddate_ovf  			B0000_0101  //输出天计数器进位信号，高电平宽度1s
#define 	alm_match      		B0000_0110  //输出闹钟匹配信号
#define 	rtc_tm128s       	B0000_0111  //输出128秒方波信号
#define 	r_tm80ms_pul      B0000_1000  //反向输出PLL分频的高电平宽度80ms的秒时标
#define 	r_bcdsec_ovf 			B0000_1001  //反向输出秒计数器进位信号
#define 	r_bcdmin_ovf			B0000_1010  //反向输出分计数器进位信号
#define 	r_bcdhour_ovf 		B0000_1011  //反向输出小时计数器进位信号
#define 	r_bcddate_ovf 		B0000_1100  //反向输出天计数器进位信号
#define 	r_alm_match     	B0000_1101  //反向输出闹钟匹配信号
#define 	r_tm1s_sqr        B0000_1110  //反向输出PLL分频的精确1s方波信号
#define 	rtc_tm1s 					B0000_1111  //输出RTC内部秒时标方波

//PRLSEN
#define pr1sen 		B0000_0001

//INT0FLAG
#define lptim_int B0000_0001
#define rtc_int  	B0000_0010

//GPIO_EXTI_SEL0
#define io_exti_sel_pg7     B1100_0000 
#define io_exti_sel_pf7     B1000_0000 
#define io_exti_sel_pe7     B0100_0000 
#define io_exti_sel_pd7     B0000_0000 

#define io_exti_sel_pg6     B0011_0000 
#define io_exti_sel_pf6     B0010_0000 
#define io_exti_sel_pe6     B0001_0000 
#define io_exti_sel_pd6     B0000_0000

#define io_exti_sel_pc7     B0000_1000 
#define io_exti_sel_pb7     B0000_0100 
#define io_exti_sel_pa7    	B0000_0000
     
#define io_exti_sel_pc6     B0000_0010     
#define io_exti_sel_pb6     B0000_0001     
#define io_exti_sel_pa6    	B0000_0000
//GPIO_EXTI_SEL1
#define io_exti_sel_pg5     B1100_0000 
#define io_exti_sel_pf5     B1000_0000 
#define io_exti_sel_pe5     B0100_0000 
#define io_exti_sel_pd5     B0000_0000 

#define io_exti_sel_pg4     B0011_0000 
#define io_exti_sel_pf4     B0010_0000 
#define io_exti_sel_pe4     B0001_0000 
#define io_exti_sel_pd4     B0000_0000

#define io_exti_sel_pc5     B0000_1000 
#define io_exti_sel_pb5     B0000_0100 
#define io_exti_sel_pa5    	B0000_0000
     
#define io_exti_sel_pc4     B0000_0010     
#define io_exti_sel_pb4     B0000_0001     
#define io_exti_sel_pa4    	B0000_0000     
     
//GPIO_EXTI_SEL2
#define io_exti_sel_pg3     B1100_0000 
#define io_exti_sel_pf3     B1000_0000 
#define io_exti_sel_pe3     B0100_0000 
#define io_exti_sel_pd3     B0000_0000 

#define io_exti_sel_pg2     B0011_0000 
#define io_exti_sel_pf2     B0010_0000 
#define io_exti_sel_pe2     B0001_0000 
#define io_exti_sel_pd2     B0000_0000

#define io_exti_sel_pc3     B0000_1000 
#define io_exti_sel_pb3     B0000_0100 
#define io_exti_sel_pa3    	B0000_0000
     
#define io_exti_sel_pc2     B0000_0010     
#define io_exti_sel_pb2     B0000_0001     
#define io_exti_sel_pa2    	B0000_0000      
     
//GPIO_EXTI_SEL3
#define io_exti_sel_pg1     B1100_0000 
#define io_exti_sel_pf1     B1000_0000 
#define io_exti_sel_pe1     B0100_0000 
#define io_exti_sel_pd1     B0000_0000 

#define io_exti_sel_pg0     B0011_0000 
#define io_exti_sel_pf0     B0010_0000 
#define io_exti_sel_pe0     B0001_0000 
#define io_exti_sel_pd0     B0000_0000

#define io_exti_sel_pc1     B0000_1000 
#define io_exti_sel_pb1     B0000_0100 
#define io_exti_sel_pa1    	B0000_0000
     
#define io_exti_sel_pc0     B0000_0010     
#define io_exti_sel_pb0     B0000_0001     
#define io_exti_sel_pa0    	B0000_0000 

//GPIOA/B/C选出的8bit用于产生EXTIL[7:0]，GPIOD/E/F/G选出的8bit用于产生EXTIH[7:0]
//GPIO_EXTIL_ES0
#define extil3_disable	    B1111_1111 
#define extil3_both_edge    B1011_1111 
#define extil3_fall_edge    B0111_1111 
#define extil3_rise_edge    B0011_1111 
                                      
#define extil2_disable	    B1111_1111 
#define extil2_both_edge    B1110_1111 
#define extil2_fall_edge    B1101_1111 
#define extil2_rise_edge    B1100_1111
                                      
#define extil1_disable	    B1111_1111 
#define extil1_both_edge    B1111_1011 
#define extil1_fall_edge   	B1111_0111
#define extil1_rise_edge    B1111_0011
                                      
#define extil0_disable	    B1111_1111 
#define extil0_both_edge    B1111_1110 
#define extil0_fall_edge   	B1111_1101 
#define extil0_rise_edge    B1111_1100
    
//GPIO_EXTIL_ES1
#define extil7_disable	    B1111_1111 
#define extil7_both_edge    B1011_1111 
#define extil7_fall_edge    B0111_1111 
#define extil7_rise_edge    B0011_1111 

#define extil6_disable	    B1111_1111 
#define extil6_both_edge    B1110_1111 
#define extil6_fall_edge    B1101_1111 
#define extil6_rise_edge    B1100_1111

#define extil5_disable	    B1111_1111 
#define extil5_both_edge    B1111_1011 
#define extil5_fall_edge   	B1111_0111
#define extil5_rise_edge    B1111_0011

#define extil4_disable	    B1111_1111 
#define extil4_both_edge    B1111_1110 
#define extil4_fall_edge   	B1111_1101 
#define extil4_rise_edge    B1111_1100

//GPIO_EXTIH_ES0
#define extih3_disable	    B1111_1111 
#define extih3_both_edge    B1011_1111 
#define extih3_fall_edge    B0111_1111 
#define extih3_rise_edge    B0011_1111 
                                      
#define extih2_disable	    B1111_1111 
#define extih2_both_edge    B1110_1111 
#define extih2_fall_edge    B1101_1111 
#define extih2_rise_edge    B1100_1111
                                      
#define extih1_disable	    B1111_1111 
#define extih1_both_edge    B1111_1011 
#define extih1_fall_edge   	B1111_0111
#define extih1_rise_edge    B1111_0011
                                      
#define extih0_disable	    B1111_1111 
#define extih0_both_edge    B1111_1110 
#define extih0_fall_edge   	B1111_1101 
#define extih0_rise_edge    B1111_1100 

//GPIO_EXTIH_ES1
#define extih7_disable	    B1111_1111 
#define extih7_both_edge    B1011_1111 
#define extih7_fall_edge    B0111_1111 
#define extih7_rise_edge    B0011_1111 
                                      
#define extih6_disable	    B1111_1111 
#define extih6_both_edge    B1110_1111 
#define extih6_fall_edge    B1101_1111 
#define extih6_rise_edge    B1100_1111
                                      
#define extih5_disable	    B1111_1111 
#define extih5_both_edge    B1111_1011 
#define extih5_fall_edge   	B1111_0111
#define extih5_rise_edge    B1111_0011
                                      
#define extih4_disable	    B1111_1111 
#define extih4_both_edge    B1111_1110 
#define extih4_fall_edge   	B1111_1101 
#define extih4_rise_edge    B1111_1100 

//FOUT0_SEL
#define fout0_xtlf				    			B0000_0000 
#define fout0_rclp				    			B0000_0001 
#define fout0_rchf_div64	    			B0000_0010 
#define fout0_lsclk	   							B0000_0011
#define fout0_coreclk_div64				  B0000_0100 
#define fout0_rtctm				    			B0000_0101 
#define fout0_pllo_div64	    			B0000_0110 
#define fout0_rtc_clk64hz   				B0000_0111
//FOUT1_SEL                                    
#define fout1_lsclk				    			B0000_0000 
#define fout1_pllo				    			B0000_0001 
#define fout1_pllo_div4	    				B0000_0010 
#define fout1_rchf	   							B0000_0011 

//EXTILIF
#define extil7_if	    B1000_0000 
#define extil6_if	    B0100_0000 
#define extil5_if	    B0010_0000 
#define extil4_if	    B0001_0000 
#define extil3_if	    B0000_1000 
#define extil2_if	    B0000_0100 
#define extil1_if	    B0000_0010 
#define extil0_if	    B0000_0001 

//EXTIHIF
#define extih7_if	    B1000_0000 
#define extih6_if	    B0100_0000 
#define extih5_if	    B0010_0000 
#define extih4_if	    B0001_0000 
#define extih3_if	    B0000_1000 
#define extih2_if	    B0000_0100 
#define extih1_if	    B0000_0010 
#define extih0_if	    B0000_0001 

//LPTCFG0
#define trig_both_edge    			B1100_0000 
#define trig_neg_edge     			B0100_0000 
#define trig_pos_edge     			B0000_0000 
#define lptin_pos_edge_cnt     	B0000_0000 
#define lptin_neg_edge_cnt     	B0010_0000 
#define lptim_cnt_src_lsclk     B0000_0000 
#define lptim_cnt_src_rclp	    B0000_1000 
#define lptim_cnt_src_coreclk   B0001_0000 
#define lptim_cnt_src_lptin	    B0001_1000 
#define cnt_clk_div1				    B0000_0000 
#define cnt_clk_div2				    B0000_0001 
#define cnt_clk_div4	    			B0000_0010 
#define cnt_clk_div8	   				B0000_0011
#define cnt_clk_div16				    B0000_0100 
#define cnt_clk_div32				    B0000_0101 
#define cnt_clk_div64	    			B0000_0110 
#define cnt_clk_div128   				B0000_0111

//LPTCFG1
#define word_mode_wave_out_timer    B0000_0000 
#define word_mode_pulse_trig_cnt	  B0000_1000    
#define word_mode_ext_pulse_cnt		  B0001_0000  //外部异步脉冲计数模式
#define word_mode_time_out				  B0001_1000  // Timeout模式

#define cnt_mode_continue		    B0000_0000 
#define cnt_mode_single			    B0000_0100 

#define pwm_mode_square_wave_out		    B0000_0000 
#define pwm_mode_pwm_wave_out	   				B0000_0010

#define pos_pol_out							B0000_0000 
#define neg_pol_out				    	B0000_0001 

//LPTIMIE
#define trig_ie			    B0000_0100 
#define ovie	   				B0000_0010
#define compie				  B0000_0001 

//LPTIMIF
#define trig_if			    B0000_0100 
#define ovif	   				B0000_0010
#define compif				  B0000_0001 

//LPTCTRL
#define lpten				  B0000_0001 

//-----------------spi=====================
//SPCR1
#define ssn_mode	 	B0001_0000
#define lsb_first 	B0000_1000
#define mstr 				B0000_0100
#define cpol 				B0000_0010
#define cpha 				B0000_0001
//SPCR2
#define sample_p		 				B1000_0000
#define txonly_auo_clr		 	B0100_0000
#define spi_en     					B0010_0000 
#define ssn_mcu_en     			B0001_0000 
#define no_wait							B0000_0000 
#define wait_2cycle							B0000_0100 
#define wait_3cycle							B0000_1000 
#define wait_4cycle							B0000_1100 
//SPCR3
#define tx_only 				B0000_1000
#define ssn_mcu 				B0000_0100
#define s_filter 				B0000_0010
#define send_p	 				B0000_0001
//SPCR4
#define clr_rxbuf 				B0000_1000
#define clr_txbuf 				B0000_0100
#define clr_m_err 				B0000_0010
#define clr_s_err	 				B0000_0001
//SPIIE
#define error_ie 					B0000_0100
#define tx_e_ie 					B0000_0010
#define rx_ne_ie	 				B0000_0001
//SPSR
#define m_error		 				B0100_0000
#define s_error     			B0010_0000 
#define rxf_wcol     			B0001_0000 
#define txf_wcol 					B0000_1000
#define busy 							B0000_0100
#define txbuf_empty 			B0000_0010
#define rxbuf_full	 			B0000_0001


//i2c interface------------------------------------------------
#define i2cen			B1000_0000
#define sclhl			B0100_0000
#define sdahl			B0010_0000
#define acken			B0001_0000
#define rcen			B0000_1000
#define pen				B0000_0100
#define rsen			B0000_0010
#define sen				B0000_0001
#define i2c_wcol 	B1000_0000
#define i2c_rw  	B0010_0000
#define i2c_p   	B0001_0000
#define i2c_s   	B0000_1000
#define i2c_bf  	B0000_0100
#define ackstat 	B0000_0010
#define ackdt			B0000_0001
#define i2cie			B0000_0010
#define i2cif			B0000_0001
#define errie			B1000_0000
#define oierr			B0000_0100
#define sderr			B0000_0010
#define ierr			B0000_0001
#define sspbrgh		B1000_0000
//-------------------------------------------------------------

//--------------clkctrl-----------------
//PERCKEN0
#define crc_clken		 				B1000_0000
#define et1_clken		 				B0100_0000
#define et2_clken     			B0010_0000 
#define et34_clken     			B0001_0000 
#define dma_clken 					B0000_1000
#define fls_clken 					B0000_0100
#define rambist_clken 			B0000_0010
#define lcd_clken			 			B0000_0001
//RSTCFG
#define addr_rst_en 			B0000_0010
#define emc_rst_en		 			B0000_0001


//--------------DMA-----------------
//GCTRL
#define dma_en 					B0000_0100
#define m2m_en 					B0000_0010				//FLASH通道使能，高有效
#define prior_ch1	 			B0000_0001
#define prior_ch0	 			B0000_0000

//CH01_CTRL
#define ch_en     								B0010_0000 
#define ch_inc     							B0000_0000    //源端地址递加
#define ch_dec     							B0001_0000    //源端地址递减
#define u7816_tx				    	B0000_0000 
#define i2c_tx				    	B0000_0001 
#define spi_tx	    				B0000_0010 
#define uart0_tx	   				B0000_0011
#define uart1_tx				    B0000_0100 
#define uart2_tx				    B0000_0101 
#define uart3_tx	    			B0000_0110 
#define uart3_1_tx	   			B0000_0111
#define u7816_rx							B0000_1000
#define i2c_rx	 						B0000_1001
#define spi_rx	 						B0000_1010
#define uart0_rx 						B0000_1011
#define uart1_rx 						B0000_1100
#define uart2_rx 						B0000_1101
#define uart3_rx 						B0000_1110
#define u_adc	      					B0000_1111
//CH2_CTRL
#define ch2_dest_inc				    	B0000_0000 
#define ch2_dest_dec				    	B0000_0001 
#define ch2_src_dec	    				B0000_0010 
#define ch2_src_inc				    	B0000_0000 
//CH012STA
#define ch_half	    				B0000_0010 
#define ch_end				    	B0000_0001 

//CH012IE
#define ch_half_ie	    		B0000_0010 
#define ch_end_ie				    B0000_0001 

//7816 module-----------------------------------------------
//U7816CTRL0
#define utxen  				B1000_0000
#define urxen   			B0100_0000
#define uclkout_en   	B0010_0000
#define psel   				B0001_0000
#define hpuat   			B0000_1000
#define hpuen   			B0000_0010
#define lpuen   			B0000_0001
//U7816CTRL1
#define hpuat1   			B0000_1000
#define hpuen1   			B0000_0010
#define lpuen1   			B0000_0001
//U7816FRMCTRL0
#define bgten  				B1000_0000
#define rep_t   			B0100_0000
#define par_even   		B0000_0000
#define par_odd   		B0001_0000
#define par_1	   			B0010_0000
#define par_n   			B0011_0000
#define fren   				B0000_1000
#define trepen   			B0000_0100
#define rrepen   			B0000_0010
#define diconv   			B0000_0001
//U7816FRMCTRL1
#define err_2etu   		B0000_0000
#define err_2etu1   	B0000_0010
#define err_1_5etu   	B0000_0100
#define err_1etu   			B0000_0110
#define ersgd   				B0000_0001
//U7816PREDIV
#define fi_msb  				B1000_0000

//U7816PRIMARYSTATUS
#define errflag     B0000_0100
#define txflag     	B0000_0010
#define rxflag     	B0000_0001
//U7816ERRSTATUS
#define tparerr     B0000_1000
#define rparerr     B0000_0100
#define framerr     B0000_0010
#define oeerr     	B0000_0001
//U7816SECONDSTATUS
#define waitrpt     B0001_0000  //等待数据重发标志位
#define txbusy	    B0000_0010  //发送数据忙标志。发送完成后自动清零
#define rxbusy     	B0000_0001   //接收数据忙标志。接收完成后自动清零
//U7816INTEN
#define lsie     	B0000_0100   //error int enable
#define txie     	B0000_0010
#define rxie     	B0000_0001


//--------- flash control -----------------
//ERCSR
#define Era_Type_c	B1000_0000   //chip erase
#define Era_Type_s	B0000_0000   //sector erase
#define Era_Req 		B0000_0001
//PRCSR
#define Pro_Req 		B0000_0001
//FLSIE
#define fls_ie			B1000_0000
#define err_ie			B0100_0000

//EPFLAG
#define Auth_err		B1000_0000   
#define Key_err			B0100_0000
#define clk_err			B0010_0000
#define fls_if			B0001_0000


//================= ANACTRL =================
//BORCTL
#define off_bor 		B0000_0001
//ANATESTSEL
#define pden_vbat	    			B1000_0000
#define buf4tst_en     			B0010_0000
#define buf4tst_bypass		 	B0001_0000
	
#define vbe1_adc    			B0000_0000    
#define vbe2_adc       		B0000_0001    
#define vbe3_adc      		B0000_0010
#define vcm_adc_buf     	B0000_0011
#define vddx_xtlf       	B0000_0100
#define fdet_out_b        B0000_0101
#define vlcd         			B0000_0110
#define vdd15		      		B0000_0111
#define vref08						B0000_1000
#define seg9_an0		     	B0000_1011
#define an1				     		B0000_1100
#define vcin2_an2		      B0000_1101
#define vdd					   		B0000_1110

//-------------------------------------------------------------
//disp module---------------------------------------------------
//DISPCTRL
#define antipolar	  	B1000_0000
#define disp_lcd_en		B0100_0000
#define flicker		 		B0000_0100
#define disp_test 		B0000_0010
#define disp_dispmod 	B0000_0001
//LCDTEST

#define disp_lcdtest_lcctrl	 	B1000_0000
#define disp_lcdtesten    		B0000_0001

//LCDSET
#define vlcd_en						B0000_0100
#define disp_lcd_a   			B0000_0000
#define disp_lcd_b   			B0000_0010
#define disp_lcd_6com    	B0000_0001
#define disp_lcd_4com 		B0000_0000
//DISPIE
#define disp_donie   B0000_0010
#define disp_doffie  B0000_0001
//DISPIF
#define disp_donif   B0000_0010
#define disp_doffif  B0000_0001


//ENMODE
#define disp_scf_fx2xc 				B0000_0000
#define disp_dispclk_8 				B0010_0000
#define disp_dispclk_16 			B0100_0000
#define disp_dispclk_32 			B0110_0000
#define disp_dispclk_64 			B1000_0000
#define disp_dispclk_128 			B1010_0000
#define disp_dispclk_256 			B1100_0000
#define disp_dispclk_512 			B1110_0000

#define disp_lcd_cout_drive1 B0000_0000
#define disp_lcd_cout_drive2 B0000_1000
#define disp_lcd_cout_drive4 B0001_0000
#define disp_lcd_cout_driven B0001_1000

#define disp_lcd_icctrl_00 B0000_0000
#define disp_lcd_icctrl_01 B0000_0010
#define disp_lcd_icctrl_10 B0000_0100
#define disp_lcd_icctrl_11 B0000_0110
#define disp_lcd_mode_cout B0000_0000
#define disp_lcd_mode_rin  B0000_0001
//-------------------------------------------------------------
//TIMER module---------------------------------------------------
//ET12CTRL1
#define et12_chen    B1000_0000   //高位定时器使能
#define et12_clen    B0100_0000   //低位定时器使能
#define et12_mode    B0010_0000   //工作模式选择，1=捕捉模式
#define et12_edgesel B0001_0000   //计数沿和捕捉沿选择，1=下降沿
#define et12_capmod  B0000_1000   //捕捉模式控制，1=脉宽捕捉
#define et12_capclr  B0000_0100   //带清零捕捉模式控制
#define et12_caponce B0000_0010   //单次捕捉控制
//ET12CTRL2
#define et12_sig2sel_grp2 B1000_0000  //计数源内部信号2源选择
#define et12_sig2sel_grp1 B0000_0000
#define et12_sig1sel_grp1 B0100_0000  //捕捉源内部信号1源选择
#define et12_sig1sel_grp2 B0000_0000
#define et12_cnthsel_cntl B0000_0000  //高位计数器源选择
#define et12_cnthsel_capsrc   B0001_0000
#define et12_cnthsel_cnthsrc  B0010_0000
#define et12_cnthsel_cntl2		  B0011_0000
#define et12_diren         B0000_1000
#define et12_stdir         B0000_0100
#define et12_srcsel        B0000_0010
#define et12_dirpo         B0000_0001
//ET12CFG1
#define et12_rtcsel2_32768 B0000_0000  //时钟2选择
#define et12_rtcsel2_sec   B0100_0000
#define et12_rtcsel2_min   B1000_0000
#define et12_rtcsel2_rtctmr1 B1100_0000
#define et12_rtcsel1_32768 B0000_0000  //时钟1选择
#define et12_rtcsel1_sec   B0001_0000
#define et12_rtcsel1_min   B0010_0000
#define et12_rtcsel1_rtctmr0 B0011_0000
#define et12_grp2sel_mclk  B0000_0000  //grp2选择
#define et12_grp2sel_rtcout2  B0000_0100
#define et12_grp2sel_insel2  B0000_1000
#define et12_grp2sel_exsel2  B0000_1100
#define et12_grp1sel_mclk  B0000_0000  //grp1选择
#define et12_grp1sel_rtcout1  B0000_0001
#define et12_grp1sel_insel1  B0000_0010
#define et12_grp1sel_exsel1  B0000_0011
//ET12CFG2
#define et12_exsel2_exin1  B0000_0000  //外部输入信号2选择
#define et12_exsel2_exin2  B0100_0000
#define et12_exsel2_exin3  B1000_0000
#define et12_exsel2_exin4  B1100_0000
#define et12_exsel1_exin1  B0000_0000  //外部输入信号1选择
#define et12_exsel1_exin2  B0001_0000
#define et12_exsel1_exin3  B0010_0000
#define et12_exsel1_exin4  B0011_0000
#define et12_insel2_rxd1   B0000_0000  //内部输入信号2选择
#define et12_insel2_rxd2   B0000_0100
#define et12_insel2_clkti_rcsen  B0000_1000
#define et12_insel2_pcacmp0  B0000_1100
#define et12_insel1_rxd0  B0000_0000  //内部输入信号1选择
#define et12_insel1_rxd1  B0000_0001
#define et12_insel1_rc125  B0000_0010
#define et12_insel1_clkptat_et1out  B0000_0011
//ET12LOADCTRL
#define et12_lhen     B0001_0000
#define et12_llen     B0000_0001
//ET12OUTCTRL
#define et12_outclr   B0001_0000  //输出清零
#define et12_outrev   B0000_1000  //输出反向
#define et12_outmod   B0000_0100  //输出模式，1=反向电平，0=输出脉冲
#define et12_outsel_cmph   B0000_0000  //输出信号选择
#define et12_outsel_cmpl   B0000_0001
#define et12_outsel_grp1in   B0000_0010
#define et12_outsel_grp2in   B0000_0011
//ET12IE
#define et12_cmphie   B0001_0000
#define et12_cmplie   B0000_1000
#define et12_ovhie    B0000_0100
#define et12_ovlie    B0000_0010
#define et12_capie    B0000_0001
//ET12IF
#define et12_capedgesta  B0010_0000    //1=捕到下沿
#define et12_cmphif   B0001_0000
#define et12_cmplif   B0000_1000
#define et12_ovhif    B0000_0100
#define et12_ovlif    B0000_0010
#define et12_capif    B0000_0001
//ET34CTRL
#define et34_cen      B1000_0000    //启动定时器
#define et34_mode     B0100_0000    //工作模式，1=捕捉
#define et34_edgesel  B0001_0000    //计数沿选择，1=下降沿
#define et34_capmod   B0000_1000    //捕捉模式，1=脉宽捕捉
#define et34_capclr   B0000_0100    //捕捉清零
#define et34_caponce  B0000_0010    //单次捕捉
#define et34_capedge  B0000_0001    //捕捉沿选择，1=下降沿
//ET34INSEL
#define et34_sig2sel_grp1 B1000_0000
#define et34_sig2sel_grp2 B0000_0000
#define et34_sig1sel_grp2 B0100_0000
#define et34_sig1sel_grp1 B0000_0000
#define et34_grp2sel_exin2_exin4 B0000_0000
#define et34_grp2sel_32768_32768 B0000_0100
#define et34_grp2sel_rxd1_rxd2  B0000_1000
#define et34_grp2sel_rcsec_rxd0 B0000_1100
#define et34_grp2sel_exin2_7816rxd0 B0001_0000
#define et34_grp2sel_32768_7816rxd1 B0001_0100
#define et34_grp2sel_rxd1_clkti    B0001_1000
#define et34_grp2sel_rcsec_rtctmr0 B0001_1100
#define et34_grp1sel_exin1_exin3 B0000_0000
#define et34_grp1sel_mclk        B0000_0001
#define et34_grp1sel_xthf_clkptat  B0000_0010
#define et34_grp1sel_rc125_rtctmr1 B0000_0011
//ET34IE
#define et34_capie    B0000_0010
#define et34_ovlie    B0000_0001
//ET34IF
#define et34_capedgesta  	B0000_0100    //1=捕到下沿
#define et34_capif    		B0000_0010
#define et34_ovlif    		B0000_0001
//PCACAPCTRL
#define pca_capsel3_rxd0 		B0000_0000
#define pca_capsel3_rxd1 		B0100_0000
#define pca_capsel3_rc125 	B1000_0000
#define pca_capsel3_et1o 		B1100_0000
#define pca_capsel2_rxd1 		B0000_0000
#define pca_capsel2_rxd2		B0001_0000
#define pca_capsel2_rcsen 	B0010_0000
#define pca_capsel2_et2o 		B0011_0000
#define pca_capsel1_exin1 	B0000_0000
#define pca_capsel1_exin2 	B0000_0100
#define pca_capsel1_min 		B0000_1000
#define pca_capsel1_rtctmr1 B0000_1100
#define pca_capsel0_32768 	B0000_0000
#define pca_capsel0_sec 		B0000_0001
#define pca_capsel0_exin3		B0000_0010
#define pca_capsel0_exin4 	B0000_0011
//PCACLKCTRL
#define pca_clksel3_rxd0 B0000_0000
#define pca_clksel3_rxd1 B0100_0000
#define pca_clksel3_rc125 B1000_0000
#define pca_clksel3_et1o B1100_0000
#define pca_clksel2_rtctmr1 B0000_0000
#define pca_clksel2_rxd2 B0001_0000
#define pca_clksel2_rcsen B0010_0000
#define pca_clksel2_et2o B0011_0000
#define pca_clksel1_exin1 B0000_0000
#define pca_clksel1_exin2 B0000_0100
#define pca_clksel1_exin3 B0000_1000
#define pca_clksel1_exin4 B0000_1100
#define pca_clksel0_32768 B0000_0000
#define pca_clksel0_sec B0000_0001
#define pca_clksel0_min B0000_0010
#define pca_clksel0_rtctmr0 B0000_0011
//PCACTRL
#define pca_capsel_pcacapt0 B0000_0000
#define pca_capsel_pcacapt1 B0000_0100
#define pca_capsel_pcacapt2 B0000_1000
#define pca_capsel_pcacapt3 B0000_1100
#define pca_ecisel_pcaclk0 B0000_0000
#define pca_ecisel_pcaclk1 B0000_0001
#define pca_ecisel_pcaclk2 B0000_0010
#define pca_ecisel_pcaclk3 B0000_0011
//FLIP251 PCA REG
//CMODD
#define pca_clkdiv12	 B0000_0000
#define pca_clkdiv4		 B0000_0010
#define pca_timer0ovflow		 B0000_0100
#define pca_eci		 B0000_0110


#endif
