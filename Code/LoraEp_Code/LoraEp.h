#ifndef __LORA__H
#define __LORA__H

#include "Init.h"

#define RLED BIT4
#define GLED BIT5
#define LED_OFF 0
#define LED_ON  1
#define Lora_OFF 0
#define Lora_ON	1


//#define LORA_ID 0x15
#define LORA_ID 0x16
//#define LORA_ID 0x17
//#define LORA_ID 0x18
//#define LORA_ID 0x19
//#define LORA_ID 0x1B
//#define LORA_ID 0x1C
//#define LORA_ID 0x1D
//#define LORA_ID 0x1E
#define LORA_IDLE 0x00
#define LORA_BUSY 0x10
//#define LORA_WORK	0x55

//#define RTC_ON 


extern INT8U LoraStatus;
extern INT16U LP_tim_cnt;
extern INT8U LowVolt;
extern INT8U Rtc_work_flag; //指示是否为上班时间,1为上班，0为下班

void LedSignal();
void EnterSleepMode();
void LpTimConfig();
void LoraSend(INT8U *datbuf,INT8U len);
void LpTimHander();
void SVDHander();
void WkupHander();
void Uart_Config();
void LedConfig();
void LedCtrl(INT8U Ledn,INT8U LED_Status);
void LoraSVDConfig();
void LoraInit();
void close_IO_PAD();
void delay(INT16U dlength) reentrant;
void WDT_CLR();
void Led_LowVolt();
void INIT_CLK(void);
void Led_LoraON();
void RtcConfig();
void RtcHander();


#endif











