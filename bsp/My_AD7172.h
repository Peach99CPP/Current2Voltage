#ifndef __MY_AD7172_H__
#define __MY_AD7172_H__
#include "stdint.h"
#include <stdbool.h>
#include "ad717x_Frame.h" 

void AD7172ParmInit(void);
void AD7172Loop(void);

uint8_t ReadAD7172_ID(void);
int32_t GetAD7172ADCChannel(uint8_t channel);
uint8_t AD7172_Calib(enum ad717x_mode  calib_mode);
void AD7172_DebugFunction(void);
#endif 

