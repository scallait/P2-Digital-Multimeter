/*
 * ADC.h
 *
 *  Created on: Nov 11, 2022
 *      Author: Seb
 */

#ifndef SRC_P2_DIGITAL_MULTIMETER_ADC_H_
#define SRC_P2_DIGITAL_MULTIMETER_ADC_H_

#include "main.h"

void ADC_init();

int ADC_Conversion(uint16_t analog_Val);

void ADC_Avg(uint16_t * ADC_Arr, int array_length, int * output);

int find_Freq(uint16_t * ADC_Arr, uint16_t * zero_sample_Num, int array_len , int * output);

#endif /* SRC_P2_DIGITAL_MULTIMETER_ADC_H_ */
