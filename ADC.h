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

double ADC_Conversion(uint16_t analog_Val);

void ADC_Avg(double * ADC_Arr, double * output);

#endif /* SRC_P2_DIGITAL_MULTIMETER_ADC_H_ */
