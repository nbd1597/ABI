#include "find_abi.h"

float32_t get_ABI (float32_t Limp[])
{
	uint16_t lower_BP, upper_BP;
	float32_t ABI;
	upper_BP = Limp[0];
	if (Limp[1] > Limp[2])
	{
		lower_BP = Limp[1];
	}
	else lower_BP = Limp[2];
	ABI = upper_BP/lower_BP;
	return ABI;

}
