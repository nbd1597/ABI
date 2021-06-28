/*
 * User_Structs.h
 *
 *  Created on: May 15, 2020
 *      Author: Duy
 */


#ifndef USER_STRUCTS_H_
#define USER_STRUCTS_H_

#define MAX_LENGTH 2500
#define BLOCK_SIZE 16
#define COEFF_SIZE 32

typedef struct
{
	uint16_t pressure[MAX_LENGTH];					//seperate array for each channel
	float32_t pulse_prefilterred [MAX_LENGTH];
}ADC_Input_t;

typedef struct
{

//	uint16_t pressure[MAX_LENGTH];					//seperate array for each channel
//	int16_t pulse_prefilterred [MAX_LENGTH];
//	uint16_t pressure2[MAX_LENGTH];
//	int16_t pulse_prefilterred2 [MAX_LENGTH];
//	uint16_t pressure3[MAX_LENGTH];
//	int16_t pulse_prefilterred3 [MAX_LENGTH];
//	uint16_t pressure4[MAX_LENGTH];
//	int16_t pulse_prefilterred4 [MAX_LENGTH];
	ADC_Input_t Limp[4];

	float32_t pulse_filterred[MAX_LENGTH]; //shared bw each channel
	float32_t peak[MAX_LENGTH];
	float32_t S_peak[MAX_LENGTH];
	float32_t peak_step[MAX_LENGTH];
	float32_t peak_envelop[MAX_LENGTH];
}NIBP_Struct;


typedef struct
{
	//float32_t pulse[MAX_LENGTH];
//	float32_t state[COEFF_SIZE + BLOCK_SIZE -1];
	float32_t *state;
	float32_t *coeff;
	uint16_t blockSize;
	uint16_t num_taps;
	arm_fir_instance_f32 S;
	uint16_t num_block;
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
