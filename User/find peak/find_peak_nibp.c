#include "find_peak_nibp.h"


/*
 * Calibrate equation
 *
 * mmHg = calib_a*adc - calib_b
 * R2 = 0.9961
 */
#define calib_a	 	(float32_t)0.073
#define calib_b		(float32_t)21.271

#define _start 100
static int _stop;
int pos; // index to run loop
int k = 5; //window
int h = 1.5; // constant to choose peak
int MAP_pos;
int MAP;
static float left_hill = 0;
//uint16_t S_peak[MAX_LENGTH]={0};
//uint16_t peak[MAX_LENGTH]={0};
//uint16_t peak_step[MAX_LENGTH]={0};
//float32_t peak_envelop[MAX_LENGTH]={0};
//signed int pulse_no_DC[MAX_LENGTH]= {0};

static int trough;
static int _trough;
static int trough_pos;


/*void parameters_init()
{
	for(int i = 0; i< MAX_LENGTH; i++)
	{
		S_peak[i] = 0;
		peak[i] = 0;
		peak_step[i] = 0;
		peak_envelop[i] = 0;
		NIBP->pressure[i] = 0;
		NIBP->pulse_filterred[i] = 0;
		NIBP->pulse_prefilterred[i] = 0;

	}
}*/
void find_peak (NIBP_Struct* NIBP, uint8_t index)
{
    /*find stop point*/
    for (pos = 0; pos < MAX_LENGTH; pos++ )
    {

        if (NIBP->Limp[index].pressure[pos] == 0)
        {
            //break;
        	_stop = pos -1;
            continue;
        }
        
    }
    
    //printf("stop at:%d \n", _stop);
    
    /* find peak funtion*/
    int k = 5; //window
    int h = 1.5; // constant to choose peak
    //int len = _stop - _start;
    int maxL, maxR;
    int _k; // index when run loop in k window
    /*peak function*/
    for (pos= _start+k; pos <= _stop-k; pos++)
    {
        
        maxL = NIBP->pulse_filterred[pos] - NIBP->pulse_filterred[pos-1];
        maxR = NIBP->pulse_filterred[pos] - NIBP->pulse_filterred[pos+1];
        for (_k = 1; _k <= k; _k++)
        {
            if(maxL < NIBP->pulse_filterred[pos]-NIBP->pulse_filterred[pos-_k])
            {
                maxL = NIBP->pulse_filterred[pos]-NIBP->pulse_filterred[pos-_k]; // find max of left window
            }
            if(maxR < NIBP->pulse_filterred[pos]-NIBP->pulse_filterred[pos+_k])
            {
                maxR = NIBP->pulse_filterred[pos]-NIBP->pulse_filterred[pos+_k]; // find max of right window
            }
                
        }
        

        NIBP->S_peak[pos] = (maxL+ maxR)/2; //
    }
    

    /*mean of peak function*/
    uint16_t total_S;

    for (pos = _start+k; pos <= _stop-k; pos++)
    {
        total_S += NIBP->S_peak[pos];
    }
    float mean_S = total_S/((_stop-k) - (_start+k) +1);
    
    /*standar deviation of S*/
    float  dev_S;
    uint16_t total_dev;
    for (pos = _start+k; pos <= _stop-k; pos++)
    {
        total_dev += pow((NIBP->S_peak[pos] - (int)mean_S), 2);

    }
    dev_S = sqrt(total_dev/((_stop-k) - (_start+k) +1));
    //printf("%f %d %f", mean_S, total_dev, dev_S);

    /* filter out peaks */

    for (pos = _start+k; pos <= _stop-k; pos++)
    {
        if ( (NIBP->S_peak[pos] > 0) && ((NIBP->S_peak[pos] - mean_S) > (h*dev_S)) && NIBP->pulse_filterred > 0) //if the peak function satisfy this is a local peak
        {
            NIBP->peak[pos] = NIBP->pulse_filterred[pos]; // put value of that pos to peak array

            
        }
        else NIBP->peak[pos] = 0;

    }

    /* filter out smaller peak in a pair of peaks in a k window*/
    int k2 = 15;
    for (pos = _start+k; pos <= _stop-k; pos++)
    {
        if (NIBP->peak[pos] != 0 /*a peak*/)
        {
            for (_k = 1; _k <= k2; _k++) //search for peaks within k
            {
                if (NIBP->peak[pos + _k] != 0) // remove smaller peak within k
                {
                    if (NIBP->peak[pos] <= NIBP->peak[pos + _k])
                    {
                        NIBP->peak[pos] = 0;
                    }
                    else if (NIBP->peak[pos] >= NIBP->peak[pos + _k])
                    {
                        NIBP->peak[pos+_k] = 0;
                    }
                    

                    
                }
            }

        }
    }
    /***find trough***/
    
    //MAP = NIBP->pulse_filterred[MAP_pos];
    //printf("map pos is %d\n", MAP_pos);


}

float32_t find_MAP (float32_t signal[], uint16_t dc_signal[], NIBP_Struct *NIBP)
{
    MAP_pos = 0;
    float32_t* temp1 = NULL;
    float32_t* temp2 = NULL;
    for (pos= _start+k; pos <= _stop-k; pos++)
    {
        if (NIBP->peak[pos] != 0)
        {
            if (NIBP->peak[pos] > NIBP->peak[MAP_pos])
            {
                MAP_pos = pos;
            }
            /*
             * while searching for MAP, ,we also find smaller peak in between two taller and average it
             */
            if (NIBP->peak[pos] > *temp2 && temp2 != NULL)
            {
            	*temp2 = (NIBP->peak[pos] + *temp1)/2;
            	temp1 = temp2;
            }
            if (NIBP->peak[pos] < *temp1) 	temp2 = &NIBP->peak[pos];
            else
            {
            	temp1 = &NIBP->peak[pos];
            	temp2 = NULL;
            }
        }
    }
    //printf(" highest: %d \n", MAP_pos);
    int left_peak = 0;
    int temp_map;
    //int temp_map_left;
    //int temp_map_right;
    temp_map = MAP_pos;
    pos = temp_map;
    left_hill = 0;
    while (left_peak < 4) // check all the peak at the left size
    {
        if (NIBP->peak[pos] != 0 /*a peak*/)
        {
            //printf("peak at: %d\n", pos);
            _trough = 1;
            trough_pos = pos;
            trough = NIBP->peak[pos];
            //printf("checkpoint1\n");
            while ((NIBP->peak[pos - _trough] == 0) && ((pos - _trough) > _start+k))
            {
                
                if (signal[pos - _trough] < trough) // find minima
                {
                    trough = signal[pos - _trough];
                    trough_pos = pos - _trough;
                }
                _trough++;
                //printf("%d\n", _trough);
            }

            if ((signal[pos] - signal[trough_pos])/ (pos - trough_pos) > left_hill) // find the steepest peak on the left
            {
                left_hill = (signal[pos] - signal[pos-_trough])/ (pos - trough_pos);
                temp_map = pos;
            }
            //printf("keft_peak:%d\n", left_peak);
            left_peak++;
            pos = pos - _trough;
            //printf("map pos = %d\n", MAP_pos);

        }  
        pos--;
    } 
    //printf("right peaks start\n");
    //temp_map = MAP_pos;
    pos = MAP_pos;
    trough = 0;
    int right_peak = 0;
    while (right_peak < 4) //check peak at right size
    {
        if (NIBP->peak[pos] != 0)// a peak
        {
            //printf("peak at: %d %f\n", pos, signal[pos]);
            _trough = 1;
            trough_pos = pos;
            trough = NIBP->peak[pos];
            while ((NIBP->peak[pos + _trough] == 0) && ((pos + _trough) < _stop+k))
            {
                if (signal[pos + _trough] < trough)
                {
                    trough = signal[pos + _trough];
                    trough_pos = pos + _trough;
                }
                _trough++;
            }
            if ((signal[pos + _trough] - signal[trough_pos])/ (pos + _trough - trough_pos) > left_hill)
            {
                left_hill = (signal[pos + _trough] - signal[trough_pos])/ (pos + _trough - trough_pos);
                temp_map = pos;
            }
            //printf("right_peak:%d\n", right_peak);
            right_peak++;
            pos = pos + _trough;
        }
        pos++;
    }
    MAP_pos = temp_map;

    MAP = (float32_t)dc_signal[MAP_pos]*calib_a - calib_b;
    //printf("map pos = %d\n", MAP_pos);
    return MAP;
}
void find_envelop(envelop_filter_Struct *filter, NIBP_Struct *NIBP)
{
	uint16_t temp;
	for (int i = _start; i < MAP_pos; i++)
	{
		if (NIBP->peak[i] != 0 && NIBP->peak[i] > temp )
		{
			temp = NIBP->peak[i];
		}
		NIBP->peak_step[i] = temp;
	}
	temp = 0;
	for (int i = _stop; i > MAP_pos; i--)
	{
		if (NIBP->peak[i] != 0 && NIBP->peak[i] > temp)
		{
			temp = NIBP->peak[i];
		}
		NIBP->peak_step[i] = temp;
	}
	NIBP->peak_step[MAP_pos] = NIBP->peak[MAP_pos];

	for(int i = 0; i < MAX_LENGTH/128; i++)
	{
		arm_fir_f32(&filter->S, (float32_t *)&NIBP->peak_step[0] + (i*128) , &NIBP->peak_envelop[0] + (i*128), filter->blockSize);
	}
	/*
	for (int i = _start; i < MAX_LENGTH; i++) //avarage filter
	{
		float32_t sum = 0;
		for(int j = 0; j < 40; j++) //average filter in window k = 20
		{
			sum = sum + NIBP->peak_step[i + j];
		}
		NIBP->peak_envelop[i] = sum / 40;
	}
	*/

}
float32_t find_SYS(NIBP_Struct *NIBP, uint8_t index)
{
	float32_t Ks;
	float32_t Sys_pulse, Sys;
	uint16_t Sys_pos;
	if (MAP > 200) Ks = 0.5;
	else if(MAP < 200 && MAP > 150) Ks = 0.29;
	else if(MAP < 150 && MAP > 135) Ks = 0.45;
	else if(MAP < 135 && MAP > 120) Ks = 0.52;
	else if(MAP < 120 && MAP > 110) Ks = 0.57;
	else if(MAP < 110 && MAP > 70)  Ks = 0.58;
	else if(MAP < 70) 				Ks = 0.64;

	//Sys_pulse = ((Ks * NIBP->pulse_filterred[MAP_pos]) + calib_b) / calib_a;
	Sys_pulse = Ks * NIBP->pulse_filterred[MAP_pos];
	float32_t temp = Sys_pulse;
	for (int i = _start; i < MAP_pos; i++)
	{
		if(abs(Sys_pulse - NIBP->peak[i]) < temp && NIBP->peak != 0)
		{
			temp = Sys_pulse - NIBP->pulse_filterred[i];
			Sys_pos = i;
		}
		/*
		else if (Sys_pulse - NIBP->peak_envelop[i] < 0)
		{
			if (NIBP->peak_envelop[i] - Sys_pulse < temp) Sys_pos = i;
			else Sys_pos = i -1;
			break;
		}
		*/
	}
	Sys = NIBP->Limp[index].pressure[Sys_pos] * calib_a - calib_b;
	return Sys;

}


/*void remove_DC(uint16_t pulse[], float a)
{
    signed int w[MAX_LENGTH] = {0};
    for (int i =0 ; i < MAX_LENGTH; i++)
    {
        
        if ((pulse[i] < 1560) & ((i < 66) || (i>2400)))
        {
            pulse[i] = 1500;
        } 
        if (i > 0)
        {
            w[i] = pulse[i] + a*w[i-1];
            pulse_no_DC[i] = w[i] - w[i-1];
        }
    }
    for (int i = 0; i < MAX_LENGTH; i++)
    {
        if ((pulse_no_DC[i] > 1000) & ((i < 66) || (i>2400)))
        {
            pulse_no_DC[i] = 0;
        } 
    }
}*/

