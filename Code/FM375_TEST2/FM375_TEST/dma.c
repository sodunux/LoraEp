#include "init.h"

 

/*
//===============fm340_CRC32函数为FM340 CRC32算法的C实现 ==============//
INT32U fm340_CRC32(INT32U reg_init, INT8U *message, INT32U len)
{
		INT8U i;
    INT32U crc_data;
		INT32U crc_reg = reg_init;

    crc_data = ((INT32U)message[0] << 24) + ((INT32U)message[1]<<16) + ((INT32U)message[2]<<8) + message[3];

		len <<= 3;
		for(i=0;i<32;i++)
		{
			
			if (((long)(crc_data^crc_reg)) < 0)
			{
				crc_reg = ((crc_reg<<1) ^ 0x04c11DB7);
			}
			else
			{
 				crc_reg <<= 1;
			}
			crc_data <<= 1;
		}
	
	
// 		for (i = 0; i < len; i++)
//     {
//         crc_data = (INT32U)message[i] << 24;
//         for (j = 0; j < 8; j++)
//         {
//           crc_data <<= 1;
// 					if (((long)(crc_data^crc_reg)) < 0)
// 					{
// 						crc_reg = ((crc_reg<<1) ^ 0x04811DB7);
// 					}
// 					else
// 					{
// 						crc_reg <<= 1;
// 					}
//         
//         }
//     }
		
		return crc_reg;
		
}
*/


//===============  以下方法为标准CRC32算法实现 ==============//
/*
//----------------多项式 0xEDB88320(消息先传输LSB) CRC32 直接计算法 -----------------------------------//
//-----------------多项式 0xEDB88320 对应的查找表的求法 ---------//
 INT32U   CRC32[256];
 INT8U   init = 0;
void init_table(void)
{
//     INT16U   i,j;
//     INT32U   crc;
// // 		INT32U CRC32[256];
// 	
//     for(i=0;i<256;i++)
//     {
//          crc = i;
//         for(j = 0;j < 8;j++)
//         {
//             if(crc & 1)
//             {
//                  crc = (crc >> 1) ^ 0xEDB88320;  
//             }
//             else
//             {
//                  crc = crc >> 1;
//             }
//         }
//          CRC32[i] = crc;
//     }
		
		INT16U   i,j;
    INT32U   crc,current;
//  		INT32U CRC32[256];
	
    for(i=0;i<256;i++)
    {
         crc = 0;
			current = i;
        for(j = 0;j < 8;j++)
        {
            if((current^crc) & 1)
            {
                 crc = (crc >> 1) ^ 0xEDB88320;  
            }
            else
            {
                 crc = crc >> 1;
            }
						current>>=1;
        }
         CRC32[i] = crc;
    }
}
//crc32实现函数
INT32U crc_32(INT8U *buf, INT32U len)
{
    INT32U ret = 0xFFFFFFFF;
    INT32U   i;
    if( !init )
    {
         init_table();
         init = 1;
    }
    for(i = 0; i < len;i++)
    {
         ret = CRC32[((ret & 0xFF) ^ buf[i])] ^ (ret >> 8);
    }
     ret = ~ret;
    return ret;
}
*/

/*
//----------------多项式 0x04C11DB7(消息先传输MSB) CRC32 直接计算法 -----------------------------------//
//-----------------多项式 0x04C11DB7 对应的查找表的求法 ---------//
 INT32U   CRC32[256];
 INT8U   init = 0;
void init_table(void)
{
		
// #define CRC_WIDTH 32  //查找表中每个单元的宽度
// 		
// 	INT16U i,j;
// 	INT32U crc;
// // 	INT32U CRC32[256];
// 	for(i=0;i<256;i++)
// 	{
// 		crc = 0;
// 		for(j=8;j>0;j--)
// 		{
// 			if(((i>>(j-1))^(crc>>(32-1)))&0x01)
// 				crc = (crc<<1)^0x04C11DB7;
// 			else crc <<= 1;
// 			
// 		}
// 		CRC32[i]=crc;
// 	}
	
	INT16U i,j;
	INT32U crc,current;

	for(i=0;i<256;i++)
	{
		crc = 0;
		current = ((INT32U)i<<24);
		for(j=8;j>0;j--)
		{
			if((long)(current^crc)<0)     //第一次时，crc为0，与0异或后值不变，即判断current的最高位是否为1.
				crc = (crc<<1)^0x04C11DB7;
			else crc <<= 1;
			
			current<<=1;
		}
		CRC32[i]=crc;
	}	
	
	
}
//crc32实现函数
INT32U crc_32(INT8U *buf, INT32U len)
{
    INT32U ret = 0xFFFFFFFF;
    INT32U   i;
    if( !init )
    {
         init_table();
         init = 1;
    }
    for(i = 0; i < len;i++)
    {
//          ret = CRC32[((ret & 0xFF) ^ buf[i])] ^ (ret << 8);
				ret = CRC32[((ret>>24) ^ buf[i])] ^ (ret << 8);
    }
     ret = ~ret;
    return ret;
}
*/


/*
//------------   CRC 直接计算方法 ---------------//
//---假设待测数据为“11 0101 1011”，长度为10位；生成多项式为10011，W=4，则需要一个4bit的寄存器；-------------//
#define CRC_WIDTH 4
#define DATA_WIDTH 10
#define CRC_POLY 0x03    //生成多项式，去掉最高位，因为最高位必然会被异或成0
void crc_caluate(void)
{
	INT16U crcdata;
	INT8U shift_bit;
	INT8U regs = 0;
	INT8U sreg;
	
	crcdata = 0x035b;	    //待运算数据
	crcdata = (crcdata << CRC_WIDTH);//待运算数据左移W位
	regs = (crcdata>>DATA_WIDTH);   //先在寄存器中移入高4位数据“1101”
	
	for(shift_bit=DATA_WIDTH;shift_bit>0;shift_bit--)      //需要循环10次
	{
		regs = (regs<<1)|((crcdata>>(shift_bit-1))&0x01);    //寄存器左移一位，并在最低位移入待运算数据的下一位
		sreg = (regs>>CRC_WIDTH);   //求出上一步中寄存器左移移出的那一位
		regs &= 0x0f;  //把左移移出的那一位清零
		if(sreg)   //如果移出的那一位为1，则寄存器与CRC多项式进行异或
		{
			regs = (regs^CRC_POLY);
		}
	}
}
*/


/*
//-----------------------  最原始的直接计算法 -----------------------------------//
//------用CRC32-CCITT的生成多项式0x04C11DB7(消息先传输MSB)，其C代码如下--------------//
INT32U do_crc32(INT8U *message, INT32U len)
{
		INT32U i, j;
    INT32U crc_reg;

    crc_reg = ((INT32U)message[0] << 24) + ((INT32U)message[1]<<16) + ((INT32U)message[2]<<8) + message[3];
    for (i = 0; i < len; i++)
    {
        if (i < len - 4)
            for (j = 0; j <= 7; j++)
            {
                if ((long)crc_reg < 0)     //把无符号数crc_reg强制转换成有符号数，如果它小于0，说明最高位为1
                    crc_reg = ((crc_reg << 1) + (message[i + 4] >> (7 - i))) ^ 0x04C11DB7;
                else
                    crc_reg = (crc_reg << 1) + (message[i + 4] >> (7 - i));     
            }
         else
            for (j = 0; j <= 7; j++)
            {
                if ((long)crc_reg < 0)
                    crc_reg = (crc_reg << 1) ^ 0x04C11DB7;
                else
                    crc_reg <<= 1;            
            }        
    } 

    return crc_reg;
} 

//-----------------------  改进的直接计算法 -----------------------------------//
//---由于异或运算满足交换律和结合律，以及与0异或无影响，消息可以不移入寄存器，---------//
//----而在每次内循环的时候，寄存器首位再与对应的消息位异或。改进的代码如下  ------//

INT32U crc32_ccitt(INT8U MOVE, INT32U reg_init, INT8U *message, INT32U len)
{
	INT32U i, j;
  INT32U crc_reg;
  INT32U current;
	
	crc_reg = reg_init;
	
	if(MOVE == MSB)   //------用CRC32-CCITT的生成多项式0x04C11DB7(消息先传输MSB)--------------//
	{
    for (i = 0; i < len; i++)
    {
        current = (INT32U)message[i] << 24;
        for (j = 0; j < 8; j++)
        {
            if ((long)(crc_reg ^ current) < 0)
                crc_reg = (crc_reg << 1) ^ 0x04C11DB7;
            else
                crc_reg <<= 1;
            current <<= 1;           
        }
    }
	
	}
	else if(MOVE == LSB)   //------用CRC32-CCITT的生成多项式0xEDB88320(消息先传输LSB)--------------//
	{
    for (i = 0; i < len; i++)
    {
        current = message[i];
        for (j = 0; j < 8; j++)
        {
            if ((crc_reg ^ current) & 0x0001)    //内循环中，寄存器的首位与消息进行异或
                crc_reg = (crc_reg >> 1) ^ 0xEDB88320;
            else
                crc_reg >>= 1;
            current >>= 1;             
        }
    }
		crc_reg = ~crc_reg;

	}
	return crc_reg;	
}
*/

/*
//--------------------------- 查表法 ------------------------//
#ifdef MSB_FIRST
//--------------------- CRC计算 多项式为0x04C11DB7(消息先传输MSB)  ------------//
static const INT32U CRC32_Table[256] =
{
0X00000000L,  0X04c11db7L,  0X09823b6eL,  0X0d4326d9L,  
0X130476dcL,  0X17c56b6bL,  0X1a864db2L,  0X1e475005L,  
0X2608edb8L,  0X22c9f00fL,  0X2f8ad6d6L,  0X2b4bcb61L,  
0X350c9b64L,  0X31cd86d3L,  0X3c8ea00aL,  0X384fbdbdL,  
0X4c11db70L,  0X48d0c6c7L,  0X4593e01eL,  0X4152fda9L,  
0X5f15adacL,  0X5bd4b01bL,  0X569796c2L,  0X52568b75L,
0X6a1936c8L,  0X6ed82b7fL,  0X639b0da6L,  0X675a1011L,  
0X791d4014L,  0X7ddc5da3L,  0X709f7b7aL,  0X745e66cdL,  
0X9823b6e0L,  0X9ce2ab57L,  0X91a18d8eL,  0X95609039L,  
0X8b27c03cL,  0X8fe6dd8bL,  0X82a5fb52L,  0X8664e6e5L,  
0Xbe2b5b58L,  0Xbaea46efL,  0Xb7a96036L,  0Xb3687d81L,  
0Xad2f2d84L,  0Xa9ee3033L,  0Xa4ad16eaL,  0Xa06c0b5dL,
0Xd4326d90L,  0Xd0f37027L,  0Xddb056feL,  0Xd9714b49L,  
0Xc7361b4cL,  0Xc3f706fbL,  0Xceb42022L,  0Xca753d95L,  
0Xf23a8028L,  0Xf6fb9d9fL,  0Xfbb8bb46L,  0Xff79a6f1L,  
0Xe13ef6f4L,  0Xe5ffeb43L,  0Xe8bccd9aL,  0Xec7dd02dL,  
0X34867077L,  0X30476dc0L,  0X3d044b19L,  0X39c556aeL,  
0X278206abL,  0X23431b1cL,  0X2e003dc5L,  0X2ac12072L,  
0X128e9dcfL,  0X164f8078L,  0X1b0ca6a1L,  0X1fcdbb16L,  
0X018aeb13L,  0X054bf6a4L,  0X0808d07dL,  0X0cc9cdcaL,  
0X7897ab07L,  0X7c56b6b0L,  0X71159069L,  0X75d48ddeL,  
0X6b93dddbL,  0X6f52c06cL,  0X6211e6b5L,  0X66d0fb02L,  
0X5e9f46bfL,  0X5a5e5b08L,  0X571d7dd1L,  0X53dc6066L,
0X4d9b3063L,  0X495a2dd4L,  0X44190b0dL,  0X40d816baL,  
0Xaca5c697L,  0Xa864db20L,  0Xa527fdf9L,  0Xa1e6e04eL,  
0Xbfa1b04bL,  0Xbb60adfcL,  0Xb6238b25L,  0Xb2e29692L,  
0X8aad2b2fL,  0X8e6c3698L,  0X832f1041L,  0X87ee0df6L,  
0X99a95df3L,  0X9d684044L,  0X902b669dL,  0X94ea7b2aL,  
0Xe0b41de7L,  0Xe4750050L,  0Xe9362689L,  0Xedf73b3eL,
0Xf3b06b3bL,  0Xf771768cL,  0Xfa325055L,  0Xfef34de2L,  
0Xc6bcf05fL,  0Xc27dede8L,  0Xcf3ecb31L,  0Xcbffd686L,  
0Xd5b88683L,  0Xd1799b34L,  0Xdc3abdedL,  0Xd8fba05aL,  
0X690ce0eeL,  0X6dcdfd59L,  0X608edb80L,  0X644fc637L,  
0X7a089632L,  0X7ec98b85L,  0X738aad5cL,  0X774bb0ebL,  
0X4f040d56L,  0X4bc510e1L,  0X46863638L,  0X42472b8fL,  
0X5c007b8aL,  0X58c1663dL,  0X558240e4L,  0X51435d53L,  
0X251d3b9eL,  0X21dc2629L,  0X2c9f00f0L,  0X285e1d47L,  
0X36194d42L,  0X32d850f5L,  0X3f9b762cL,  0X3b5a6b9bL,  
0X0315d626L,  0X07d4cb91L,  0X0a97ed48L,  0X0e56f0ffL,  
0X1011a0faL,  0X14d0bd4dL,  0X19939b94L,  0X1d528623L,
0Xf12f560eL,  0Xf5ee4bb9L,  0Xf8ad6d60L,  0Xfc6c70d7L,  
0Xe22b20d2L,  0Xe6ea3d65L,  0Xeba91bbcL,  0Xef68060bL,  
0Xd727bbb6L,  0Xd3e6a601L,  0Xdea580d8L,  0Xda649d6fL,  
0Xc423cd6aL,  0Xc0e2d0ddL,  0Xcda1f604L,  0Xc960ebb3L,  
0Xbd3e8d7eL,  0Xb9ff90c9L,  0Xb4bcb610L,  0Xb07daba7L,  
0Xae3afba2L,  0Xaafbe615L,  0Xa7b8c0ccL,  0Xa379dd7bL,
0X9b3660c6L,  0X9ff77d71L,  0X92b45ba8L,  0X9675461fL,  
0X8832161aL,  0X8cf30badL,  0X81b02d74L,  0X857130c3L,  
0X5d8a9099L,  0X594b8d2eL,  0X5408abf7L,  0X50c9b640L,  
0X4e8ee645L,  0X4a4ffbf2L,  0X470cdd2bL,  0X43cdc09cL,  
0X7b827d21L,  0X7f436096L,  0X7200464fL,  0X76c15bf8L,  
0X68860bfdL,  0X6c47164aL,  0X61043093L,  0X65c52d24L,  
0X119b4be9L,  0X155a565eL,  0X18197087L,  0X1cd86d30L,  
0X029f3d35L,  0X065e2082L,  0X0b1d065bL,  0X0fdc1becL,  
0X3793a651L,  0X3352bbe6L,  0X3e119d3fL,  0X3ad08088L,  
0X2497d08dL,  0X2056cd3aL,  0X2d15ebe3L,  0X29d4f654L,  
0Xc5a92679L,  0Xc1683bceL,  0Xcc2b1d17L,  0Xc8ea00a0L,
0Xd6ad50a5L,  0Xd26c4d12L,  0Xdf2f6bcbL,  0Xdbee767cL,
0Xe3a1cbc1L,  0Xe760d676L,  0Xea23f0afL,  0Xeee2ed18L,
0Xf0a5bd1dL,  0Xf464a0aaL,  0Xf9278673L,  0Xfde69bc4L,
0X89b8fd09L,  0X8d79e0beL,  0X803ac667L,  0X84fbdbd0L,
0X9abc8bd5L,  0X9e7d9662L,  0X933eb0bbL,  0X97ffad0cL,
0Xafb010b1L,  0Xab710d06L,  0Xa6322bdfL,  0Xa2f33668L,
0Xbcb4666dL,  0Xb8757bdaL,  0Xb5365d03L,  0Xb1f740b4L
};
#endif

#ifdef LSB_FIRST
//--------------------- CRC计算 多项式为0xEDB88320(消息先传输LSB)  ------------//
 static const INT32U CRC32_Table[256] =
 {
    0x00000000, 0x77073096, 0xEE0E612C, 0x990951BA,
    0x076DC419, 0x706AF48F, 0xE963A535, 0x9E6495A3,
    0x0EDB8832, 0x79DCB8A4, 0xE0D5E91E, 0x97D2D988,
    0x09B64C2B, 0x7EB17CBD, 0xE7B82D07, 0x90BF1D91,
    0x1DB71064, 0x6AB020F2, 0xF3B97148, 0x84BE41DE,
    0x1ADAD47D, 0x6DDDE4EB, 0xF4D4B551, 0x83D385C7,
    0x136C9856, 0x646BA8C0, 0xFD62F97A, 0x8A65C9EC,
    0x14015C4F, 0x63066CD9, 0xFA0F3D63, 0x8D080DF5,
    0x3B6E20C8, 0x4C69105E, 0xD56041E4, 0xA2677172,
    0x3C03E4D1, 0x4B04D447, 0xD20D85FD, 0xA50AB56B,
    0x35B5A8FA, 0x42B2986C, 0xDBBBC9D6, 0xACBCF940,
    0x32D86CE3, 0x45DF5C75, 0xDCD60DCF, 0xABD13D59,
    0x26D930AC, 0x51DE003A, 0xC8D75180, 0xBFD06116,
    0x21B4F4B5, 0x56B3C423, 0xCFBA9599, 0xB8BDA50F,
    0x2802B89E, 0x5F058808, 0xC60CD9B2, 0xB10BE924,
    0x2F6F7C87, 0x58684C11, 0xC1611DAB, 0xB6662D3D,
    0x76DC4190, 0x01DB7106, 0x98D220BC, 0xEFD5102A,
    0x71B18589, 0x06B6B51F, 0x9FBFE4A5, 0xE8B8D433,
    0x7807C9A2, 0x0F00F934, 0x9609A88E, 0xE10E9818,
    0x7F6A0DBB, 0x086D3D2D, 0x91646C97, 0xE6635C01,
    0x6B6B51F4, 0x1C6C6162, 0x856530D8, 0xF262004E,
    0x6C0695ED, 0x1B01A57B, 0x8208F4C1, 0xF50FC457,
    0x65B0D9C6, 0x12B7E950, 0x8BBEB8EA, 0xFCB9887C,
    0x62DD1DDF, 0x15DA2D49, 0x8CD37CF3, 0xFBD44C65,
    0x4DB26158, 0x3AB551CE, 0xA3BC0074, 0xD4BB30E2,
    0x4ADFA541, 0x3DD895D7, 0xA4D1C46D, 0xD3D6F4FB,
    0x4369E96A, 0x346ED9FC, 0xAD678846, 0xDA60B8D0,
    0x44042D73, 0x33031DE5, 0xAA0A4C5F, 0xDD0D7CC9,
    0x5005713C, 0x270241AA, 0xBE0B1010, 0xC90C2086,
    0x5768B525, 0x206F85B3, 0xB966D409, 0xCE61E49F,
    0x5EDEF90E, 0x29D9C998, 0xB0D09822, 0xC7D7A8B4,
    0x59B33D17, 0x2EB40D81, 0xB7BD5C3B, 0xC0BA6CAD,
    0xEDB88320, 0x9ABFB3B6, 0x03B6E20C, 0x74B1D29A,
    0xEAD54739, 0x9DD277AF, 0x04DB2615, 0x73DC1683,
    0xE3630B12, 0x94643B84, 0x0D6D6A3E, 0x7A6A5AA8,
    0xE40ECF0B, 0x9309FF9D, 0x0A00AE27, 0x7D079EB1,
    0xF00F9344, 0x8708A3D2, 0x1E01F268, 0x6906C2FE,
    0xF762575D, 0x806567CB, 0x196C3671, 0x6E6B06E7,
    0xFED41B76, 0x89D32BE0, 0x10DA7A5A, 0x67DD4ACC,
    0xF9B9DF6F, 0x8EBEEFF9, 0x17B7BE43, 0x60B08ED5,
    0xD6D6A3E8, 0xA1D1937E, 0x38D8C2C4, 0x4FDFF252,
    0xD1BB67F1, 0xA6BC5767, 0x3FB506DD, 0x48B2364B,
    0xD80D2BDA, 0xAF0A1B4C, 0x36034AF6, 0x41047A60,
    0xDF60EFC3, 0xA867DF55, 0x316E8EEF, 0x4669BE79,
    0xCB61B38C, 0xBC66831A, 0x256FD2A0, 0x5268E236,
    0xCC0C7795, 0xBB0B4703, 0x220216B9, 0x5505262F,
    0xC5BA3BBE, 0xB2BD0B28, 0x2BB45A92, 0x5CB36A04,
    0xC2D7FFA7, 0xB5D0CF31, 0x2CD99E8B, 0x5BDEAE1D,
    0x9B64C2B0, 0xEC63F226, 0x756AA39C, 0x026D930A,
    0x9C0906A9, 0xEB0E363F, 0x72076785, 0x05005713,
    0x95BF4A82, 0xE2B87A14, 0x7BB12BAE, 0x0CB61B38,
    0x92D28E9B, 0xE5D5BE0D, 0x7CDCEFB7, 0x0BDBDF21,
    0x86D3D2D4, 0xF1D4E242, 0x68DDB3F8, 0x1FDA836E,
    0x81BE16CD, 0xF6B9265B, 0x6FB077E1, 0x18B74777,
    0x88085AE6, 0xFF0F6A70, 0x66063BCA, 0x11010B5C,
    0x8F659EFF, 0xF862AE69, 0x616BFFD3, 0x166CCF45,
    0xA00AE278, 0xD70DD2EE, 0x4E048354, 0x3903B3C2,
    0xA7672661, 0xD06016F7, 0x4969474D, 0x3E6E77DB,
    0xAED16A4A, 0xD9D65ADC, 0x40DF0B66, 0x37D83BF0,
    0xA9BCAE53, 0xDEBB9EC5, 0x47B2CF7F, 0x30B5FFE9,
    0xBDBDF21C, 0xCABAC28A, 0x53B39330, 0x24B4A3A6,
    0xBAD03605, 0xCDD70693, 0x54DE5729, 0x23D967BF,
    0xB3667A2E, 0xC4614AB8, 0x5D681B02, 0x2A6F2B94,
    0xB40BBE37, 0xC30C8EA1, 0x5A05DF1B, 0x2D02EF8D
 };

#endif


INT32U calculate_CRC32(INT8U MOVE, INT32U reg_init, INT8U *pData, INT32U uSize)
{

	INT32U crc32;

	crc32 = reg_init;
	if(MOVE==LSB)
	{
		while(uSize--)
		{
			crc32 = CRC32_Table[(crc32 & 0xFF) ^ *pData++] ^ (crc32 >> 8);
		}
		
		crc32 = ~crc32;

	}
	else if(MOVE==MSB)
	{
		while(uSize--)
		{
			crc32 = CRC32_Table[( crc32 >> 24 )^ *pData++] ^ ( crc32 << 8 );
		}
			
	}
	return crc32;	
}
*/


//DMAx的各通道配置
//这里的传输形式是固定的,这点要根据不同的情况来修改
//从存储器->外设模式/8位数据宽度/存储器增量模式
//chx:DMA通道选择,01:ch0;10:ch1;11:ch0和ch1
//maddr:存储器地址,12位      
//ndtr:数据传输量     
//par_txrx:外设,0~f,发送or接收
void DMA_Config(INT8U chx,INT16U maddr,INT16U ndtr,INT8U par_txrx)
{
	INT8U i;
	PERICLK_CTRL0 |= dma_clken;   //DMA时钟使能
	for(i=0; i<21; i++)
	{
		XBYTE[DMA_ADDR+i] = 0; //deinit dma reg
	}
	
	GCTRL |= dma_en;  //DMA全局使能
	GCTRL &= ~m2m_en; //FLASH通道禁止
	
	if(chx==0)
	{
		GCTRL |= prior_ch0; //通道0优先
		CH0_CTRL |= ch_inc;//源端地址递加
		CH0_CTRL |= par_txrx;
		CH0_LEN = ndtr-1;//传输数据长度
		CH0_ADDRH = (INT8U)(maddr>>8);
		CH0_ADDRL = (INT8U)(maddr&0xff);//mem地址设置
		AIE6 = 1;
		CH0IE = ch_half_ie | ch_end_ie;
	}
	else if(chx==1)
	{
		GCTRL |= prior_ch1; //通道1优先
		CH1_CTRL |= ch_inc;//源端地址递加
		CH1_CTRL |= par_txrx;
		CH1_LEN = ndtr-1;//传输数据长度
		CH1_ADDRH = (INT8U)(maddr>>8);
		CH1_ADDRL = (INT8U)(maddr&0xff);//mem地址设置
		AIE6 = 1;
		CH1IE = ch_half_ie | ch_end_ie;
	}
	
}

void DMA_Enable(INT8U chx)
{
	if(chx==0)
		CH0_CTRL |= ch_en;//使能通道0
	else if(chx==1)
		CH1_CTRL |= ch_en;//使能通道1
}

void DMA_Disable(INT8U chx)
{
	if(chx==0)
		CH0_CTRL &= ~ch_en;//通道0
	else if(chx==1)
		CH1_CTRL &= ~ch_en;//通道1
}

void DMA_Start(INT8U chx,INT16U maddr,INT16U ndtr,INT8U par_txrx)
{
// 	INT8U *p = (INT8U *)maddr;
	dma_end = 0;
	dma_half = 0;
	DMA_Config(chx,maddr,ndtr,par_txrx);  //配置DMA通道，传输字节

	DMA_Enable(chx);//使能CHx，启动传输
	while(!dma_half){WDT_CLR();}
	dma_half = 0;	
	while(!dma_end){WDT_CLR();}
	dma_end = 0;
		
// 	while(!((CH0STA & ch_end)|(CH1STA & ch_end))) {}   //通道传输完毕后	*p = 0x55;
// 	CH0STA &= ~ch_end;
// 	CH1STA &= ~ch_end;
		
	DMA_Disable(chx);	
}

  

void DMA2_Config(INT16U saddr,INT16U daddr,INT16U ndtr)
{
	INT8U i;
	PERICLK_CTRL0 |= dma_clken;   //DMA时钟使能
	for(i=0; i<21; i++)
	{
		XBYTE[DMA_ADDR+i] = 0; //deinit dma reg
	}
	
	GCTRL |= dma_en;  //DMA全局使能
	
	CH2_CTRL = ch2_src_inc | ch2_dest_inc;//通道2源端(FLASH)地址加,通道2目标端(RAM)地址加
	CH2_LEN = ndtr-1;//传输数据长度
	CH2_SADDRH = (INT8U)(saddr>>8);
	CH2_SADDRL = (INT8U)(saddr&0xff);//源端flash起始地址设置
	CH2_DADDRH = (INT8U)(daddr>>8);
	CH2_DADDRL = (INT8U)(daddr&0xff);//目标端ram起始地址设置
	
	AIE6 = 1;
	CH2IE = ch_half_ie | ch_end_ie;
}

void DMA2_Start(INT16U saddr,INT16U daddr,INT16U ndtr)
{
	dma_end = 0;
	dma_half = 0;
	
	DMA2_Config(saddr,daddr,ndtr);  //配置DMA通道，传输字节

	GCTRL |= m2m_en; //FLASH通道使能，启动传输
	
	while(!dma_half){}
	dma_half = 0;	
	while(!dma_end){}
	dma_end = 0;	
	
// 	while(!(CH2STA & ch_end)) {}   //通道传输完毕后	
// 	CH2STA &= ~ch_end;
	
	GCTRL &= ~m2m_en; //FLASH通道禁止
}
