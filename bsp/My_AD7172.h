#ifndef __MY_AD7172_H__
#define __MY_AD7172_H__
#include "stdint.h"
#include <stdbool.h>
#include "ad717x_Frame.h" 

//建立一个枚举 其包含失调和增益
enum ad717x_soft_calib_mode
{
    OFFSET_CALIB,
    GAIN_CALIB
};

void AD7172ParmInit(void);
void AD7172Loop(void);

uint8_t ReadAD7172_ID(void);
int32_t GetAD7172ADCChannel(uint8_t channel);
uint8_t AD7172_Calib(enum ad717x_mode  calib_mode);
void AD7172_DebugFunction(void);
void AD7172_SoftCalib(enum ad717x_soft_calib_mode mode);
void ADA7172_SOFT_Loop(void);//经过软件校准的ADC 输出值
#endif 

