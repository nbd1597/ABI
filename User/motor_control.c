#include "motor_control.h"



void ready_state()
{

	htim5.Instance->CCR2 = 0;
	htim5.Instance->CCR3 = 0;
	parameters_init();

}
float32_t start(uint16_t *pressure, float32_t limp_bp[], FIR_filter_Struct *filter, FIR_filter_Struct *envelop_filter, NIBP_Struct *NIBP)
{
	//uint16_t pressureee;

	recording = 0;
	htim5.Instance->CCR2 = 100;
	htim5.Instance->CCR3 = 100;
	while(1)
	{
		//pressureee = HAL_ADC_GetValue(&hadc1);
		if(*pressure > 2800) break;

	}

	htim5.Instance->CCR2 = 0;
	htim5.Instance->CCR3 = 100;
	HAL_Delay(1000);

	recording = 1;
	_pressure = 0;
	GPIOD->BSRR = GPIO_PIN_15;
	htim5.Instance->CCR2 = 0;
	htim5.Instance->CCR3 = 90;
	while(1)
	{
		//pressureee = HAL_ADC_GetValue(&hadc1);
		if (*pressure < 700) break;
	}

	HAL_Delay(200);
	recording = 0;
	GPIOD->BSRR = GPIO_PIN_15 << 16U;
	htim5.Instance->CCR2 = 0;
	htim5.Instance->CCR3 = 0;
	for (int i = 0; i < MAX_LENGTH/BLOCK_SIZE; i++)
	{
		arm_fir_f32(&filter->S, NIBP->pulse_prefilterred + (i*BLOCK_SIZE) , NIBP->pulse_filterred + (i*BLOCK_SIZE), filter->blockSize);
	}

	/*
	 * find MAP, SYS and ABI
	 */
	//uint16_t MAP;
	static uint32_t _limp;

	find_peak(NIBP); //
	find_MAP(NIBP->pulse_filterred, NIBP->pressure);//
	find_envelop(envelop_filter);
	limp_bp[_limp++] = find_SYS(NIBP);//
    for (int i = 0; i <= 3500; i++)
    {
    	sprintf((char*)CDC_Buf, "%f\r", peak[i]);
    	CDC_Transmit_FS(CDC_Buf, sizeof(CDC_Buf));
    	HAL_Delay(1);

    }


	while(1) /*hold until next measure*/
	{
		if (state == 0);
		break;
	}
	return 0;

}


