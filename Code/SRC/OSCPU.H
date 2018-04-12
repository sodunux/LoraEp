//类型定义头文件
/*--------------------------------------------------------------------------
OSCPU.H

Header file for type re-defien
Copyright (c) 1988-2004 Shanghai FuDan MicroElectronic Inc, Ltd.
All rights reserved.
Designed by Tang yiyong
2004-04-10
--------------------------------------------------------------------------*/
#ifndef _OS_CPU_H
	#define _OS_CPU_H

	//In C51.PDF P176
	typedef unsigned char  	BOOLEAN;       //Dont't use bit define,not useful in struct
	typedef unsigned char  	INT8U;         //unsigned 8 bit data
	typedef signed   char  	INT8S;         //signed 8 bit data
	typedef unsigned int   	INT16U;        //unsigned 16 bit data
	typedef signed   int   	INT16S;        //signed 16 bit data
	typedef unsigned long  	INT32U;        //unsigned 32 bit data
	typedef signed   long  	INT32S;        //signed 32 bit data
	typedef float          	FP32;          //single float
	typedef double         	FP64;          //double float
	
	#define BYTE           	INT8S          //signed 8 bits
	#define UBYTE          	INT8U          //unsigned 8 bits
	#define WORD           	INT16S		   //signed 16 bits
	#define UWORD         	INT16U		   //nsigned 16 bits
	#define LONG           	INT32S		   //signed 32 bits
	#define ULONG          	INT32U		   //unsigned 32 bits
	
	#define uchar          	INT8U          //unsigned 8 bits
	#define uint          	INT16U         //unsigned 16 bits
#endif
//_OS_CPU_H                              

