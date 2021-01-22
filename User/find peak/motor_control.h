#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "main.h"
#include "arm_math.h"
#include "tim.h"
#include "adc.h"
#include "find_peak_nibp.h"
#include <stdio.h>

int tmp;
void ready_state();
float32_t start(uint16_t *pressure, float32_t limp_bp[], FIR_filter_Struct *filter, envelop_filter_Struct *envelop_filter, NIBP_Struct *NIBP);

uint8_t ready, recording;
uint16_t _pressure;
//uint16_t state;
//uint16_t _limp = 0;

#endif
