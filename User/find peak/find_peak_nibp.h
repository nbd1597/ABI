#ifndef FIND_PEAK_NIBP_H
#define FIND_PEAK_NIBP_H

//#include "pulse_data.h"
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include "arm_math.h"
#include "User_Structs.h"
//#include "lp_envelop.h"
//#define float32_t float

#define MAX_LENGTH 3500

uint16_t peak_step[MAX_LENGTH];
float32_t peak_envelop[MAX_LENGTH];

extern uint16_t S_peak[MAX_LENGTH] ; // peak function --- significant of each index in pulse signal, use this to find peak
extern uint16_t peak[MAX_LENGTH] ; // contain peak as same value from signal, and non peak as zeroes

//extern signed int pulse_no_DC[MAX_LENGTH];

void parameters_init();
void find_peak (NIBP_Struct* NIBP);
//void fir_lp(uint16_t signal[]);
//void find_envelop(uint16_t signal[]);
float32_t find_MAP (float32_t signal[], uint16_t dc_signal[]);
//void remove_DC(uint16_t signal[], float a);
float32_t find_SYS(NIBP_Struct *NIBP);
void find_envelop(FIR_filter_Struct *filter);
uint16_t CDC_Buf[16];





#endif
