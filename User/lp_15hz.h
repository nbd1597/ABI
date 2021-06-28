/*
 * Filter Coefficients (C Source) generated by the Filter Design and Analysis Tool
 * Generated by MATLAB(R) 9.4 and Signal Processing Toolbox 8.0.
 * Generated on: 20-Feb-2020 21:49:25
 */

/*
 * Discrete-Time FIR Filter (real)
 * -------------------------------
 * Filter Structure  : Direct-Form FIR
 * Filter Length     : 64
 * Stable            : Yes
 * Linear Phase      : Yes (Type 2)
 */

/* General type conversion for MATLAB generated C-code  */
//#include "tmwtypes.h"
#include "arm_math.h"
/* 
 * Expected path to tmwtypes.h 
 * D:\MATLAB\extern\include\tmwtypes.h 
 */
/*
 * Warning - Filter coefficients were truncated to fit specified data type.  
 *   The resulting response may not match generated theoretical response.
 *   Use the Filter Design & Analysis Tool to design accurate
 *   single-precision filter coefficients.
 */
/*
 * 5hz
 */
const int numTaps = 32;
float32_t fir_coeff[32] = {
		-3.894450856e-06,1.810794856e-05, 0.000150942913,0.0004820677859,0.0008766679093,
		  0.0006792319473,-0.001271738554,-0.005845753942, -0.01204451453, -0.01558841858,
		  -0.009243382141,  0.01411272399,  0.05659143627,   0.1114861742,   0.1637688428,
		     0.1958314925,   0.1958314925,   0.1637688428,   0.1114861742,  0.05659143627,
		    0.01411272399,-0.009243382141, -0.01558841858, -0.01204451453,-0.005845753942,
		  -0.001271738554,0.0006792319473,0.0008766679093,0.0004820677859, 0.000150942913,
		  1.810794856e-05,-3.894450856e-06
};
