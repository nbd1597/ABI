/*
 * User_Structs.h
 *
 *  Created on: May 15, 2020
 *      Author: Duy
 */


#ifndef USER_STRUCTS_H_
#define USER_STRUCTS_H_

#define MAX_LENGTH 3000
#define BLOCK_SIZE 16
#define COEFF_SIZE 64

typedef struct
{

	uint16_t pressure[MAX_LENGTH];
	float32_t pulse_prefilterred [MAX_LENGTH];
	float32_t pulse_filterred[MAX_LENGTH];
	float32_t peak[MAX_LENGTH];
	float32_t S_peak[MAX_LENGTH];
	float32_t peak_step[MAX_LENGTH];
	float32_t peak_envelop[MAX_LENGTH];
}NIBP_Struct;


typedef struct
{
	//float32_t pulse[MAX_LENGTH];
	float32_t state[COEFF_SIZE + BLOCK_SIZE -1];
	float32_t *coeff;
	uint16_t blockSize;
	uint16_t num_taps;
	arm_fir_instance_f32 S;
} FIR_filter_Struct;

typedef struct
{
	//float32_t pulse[MAX_LENGTH];
	float32_t state[128 + 128 -1];
	float32_t *coeff;
	uint16_t blockSize;
	uint16_t num_taps;
	arm_fir_instance_f32 S;
} envelop_filter_Struct;



//pulse_filter = {.blockSize =  BLOCK_SIZE};






#endif /* USER_STRUCTS_H_ */
