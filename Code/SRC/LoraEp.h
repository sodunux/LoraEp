#ifndef __LORA__H
#define __LORA__H

#define RLED BIT4
#define GLED BIT5
#define LED_OFF 0
#define LED_ON  1
#define Lora_OFF 0
#define Lora_ON	1

extern bool LoraStatus;
extern INT16U LP_tim_cnt;


void EnterStopMode();
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

#endif











