#include "motor_control.h"

#define LIMP_NUMBER 1

void ready_state()
{

//	htim5.Instance->CCR2 = 0;
//	htim5.Instance->CCR3 = 0;
	//parameters_init();

}
float32_t start(uint16_t *adc, float32_t limp_bp[], FIR_filter_Struct *filter, envelop_filter_Struct *envelop_filter, NIBP_Struct *NIBP)
{

	recording = 0;
//	for(uint8_t i; i <4; i++)
//	{
//		GPIOD->BSRR = GPIO_PIN_8 << i;
//	}
	GPIOD->BSRR = GPIO_PIN_8;
	GPIOD->BSRR = GPIO_PIN_9;
	GPIOD->BSRR = GPIO_PIN_10;
	GPIOD->BSRR = GPIO_PIN_11;
	htim3.Instance->CCR1 = 100;
	htim3.Instance->CCR2 = 100;
	htim3.Instance->CCR3 = 100;
	htim3.Instance->CCR4 = 100;
	while(1)
	{
//		if(*pressure > 2800) break;
		for(uint8_t i = 0; i <LIMP_NUMBER; i++)
		{
//			if (adc[i + 4] > 2800)
//			{
//				GPIOD->BSRR = GPIO_PIN_11 << (16 - i);
//			}
			if(adc[4] > 2800)
			{
				GPIOD->BSRR = GPIO_PIN_11 << 16U;
				break;
			}
		}
//		if((GPIOD->ODR & (GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11)) == 0)
//		{
//			break;
//		}

	}

//	htim5.Instance->CCR2 = 0;
//	htim5.Instance->CCR3 = 100;
	HAL_Delay(1000);

	htim3.Instance->CCR1 = 90;
	htim3.Instance->CCR2 = 90;
	htim3.Instance->CCR3 = 90;
	htim3.Instance->CCR4 = 90;

//	recording = 1;
	_pressure = 0;
//	GPIOD->BSRR = GPIO_PIN_15;
//	htim5.Instance->CCR2 = 0;
//	htim5.Instance->CCR3 = 90;
	int counter = 0;
	while(1)
	{
		//pressureee = HAL_ADC_GetValue(&hadc1);
//		if (*pressure < 700) break;

		if(adc[0] < 700)
		{
			htim3.Instance->CCR1 = 100;
			counter++;
		}
		if(adc[1] < 700)
		{
			htim3.Instance->CCR2 = 100;
			counter++;
		}
//		if(adc[2] < 700)
//		{
//			htim3.Instance->CCR3 = 100;
//			counter++;
//		}
//		if(adc[3] < 700)
//		{
//			htim3.Instance->CCR4 = 100;
//			counter++;
//		}
//		if (counter == 4)
//		{
//			break;
//		}
	}

	HAL_Delay(200);
//	recording = 0;
//	GPIOD->BSRR = GPIO_PIN_15 << 16U;
//	htim5.Instance->CCR2 = 0;
//	htim5.Instance->CCR3 = 0;
//	for (int i = 0; i < MAX_LENGTH/BLOCK_SIZE; i++)
//	{
//		arm_fir_f32(&filter->S, NIBP->pulse_prefilterred + (i*BLOCK_SIZE) , NIBP->pulse_filterred + (i*BLOCK_SIZE), filter->blockSize);
//	}
	for(uint8_t _limp; _limp <LIMP_NUMBER; _limp++)
	{
		for (int i = 0; i < MAX_LENGTH/BLOCK_SIZE; i++)
		{
			arm_fir_f32(&filter->S, (float32_t*)NIBP->Limp[_limp].pulse_prefilterred + (i*BLOCK_SIZE) , NIBP->pulse_filterred + (i*BLOCK_SIZE), filter->blockSize);
			find_peak(NIBP, _limp); //
			find_MAP((float32_t*)NIBP->Limp[_limp].pulse_prefilterred, NIBP->Limp[_limp].pressure, NIBP);//
			find_envelop(envelop_filter, NIBP);
			limp_bp[_limp] = find_SYS(NIBP, _limp);
		}
	}

	/*
	 * find MAP, SYS and ABI
	 */
	//uint16_t MAP;
//	static uint32_t _limp;
//
//	find_peak(NIBP); //
//	find_MAP(NIBP->pulse_filterred, NIBP->pressure, NIBP);//
//	find_envelop(envelop_filter, NIBP);
//	limp_bp[_limp++] = find_SYS(NIBP);//
//    for (int i = 0; i <= 3000; i++)
//    {
//    	sprintf((char*)CDC_Buf, "%.3f\r", NIBP->pulse_filterred[i]);
//    	CDC_Transmit_FS(CDC_Buf, sizeof(CDC_Buf));
//    	HAL_Delay(1);
//
//    }


    state = 0;
	return 0;

}


