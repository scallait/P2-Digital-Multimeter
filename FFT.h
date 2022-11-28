/*
 * FFT.h
 *
 *  Created on: Nov 27, 2022
 *      Author: Seb
 */

#ifndef SRC_P2_DIGITAL_MULTIMETER_FFT_H_
#define SRC_P2_DIGITAL_MULTIMETER_FFT_H_

#include <stdio.h>
#include <math.h>
#include <complex.h>
#include "main.h"

typedef double complex cplx;

void show(const char * s, cplx buf[], int arraySize);

int findFreq(int arraySize, int samplingFreq, uint16_t ADC_arr[]);

#endif /* SRC_P2_DIGITAL_MULTIMETER_FFT_H_ */
