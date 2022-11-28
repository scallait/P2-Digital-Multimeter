/*
 * ADC.c
 *
 *  Created on: Nov 4, 2022
 *      Author: Seb
 */
#include "ADC.h"
#include "USART.h"
#include <stdio.h>
#include <string.h>

double calibration_factor = 1.0;

void ADC_init(){
	// Turn on clock to ADC
	RCC->AHB2ENR |= RCC_AHB2ENR_ADCEN;
	ADC123_COMMON->CCR &= ~(ADC_CCR_CKMODE);
	ADC123_COMMON->CCR |= (1 << ADC_CCR_CKMODE_Pos);
	/* Pg. 614 This configuration must be enabled only if the AHB
	clock prescaler is set to 1 (HPRE[3:0] = 0xxx in RCC_CFGR register) */

	// Supply power to ADC (Pg. 515)
	ADC1->CR &= ~(ADC_CR_DEEPPWD); //Clear bit to start ADC operations
	ADC1->CR |= ADC_CR_ADVREGEN;
	HAL_Delay(1); // ADCVR Start up time

	// Calibrate ADC (Pg. 516-7)
	ADC1->CR &= ~(ADC_CR_ADEN);		// Ensure ADC is not enabled
	ADC1->CR &= ~ADC_CR_ADCALDIF; 	// single-ended input conversions
	ADC1->CR |= ADC_CR_ADCAL;		// Start calibration sequence
	while(ADC1->CR & ADC_CR_ADCAL); // Wait until calibration sequence is complete
	/* Note - Can read calibration factor from ADC_CALFACT register */

	// Turn on ADC (Pg. 519)
	ADC1->ISR |= ADC_ISR_ADRDY; 		// Clear the ARDY bit by setting to 1 in ISR
	ADC1->CR |= ADC_CR_ADEN;
	while(!(ADC1->ISR & ADC_ISR_ADRDY)); // Set once ADC is ready

	// Configure the ADC
	ADC1->SQR1 &= ~(0b11111 << ADC_SQR1_SQ1_Pos);
	ADC1->SQR1 |= 0x1 << ADC_SQR1_SQ1_Pos; // Set channel 1?
	ADC1->SMPR1 &= ~(0b111 << ADC_SMPR1_SMP1_Pos); // Clear SMPR (Sets it to 2.5 ADC Clock Cycles)
	ADC1->SMPR1 |= 7 << ADC_SMPR1_SMP1_Pos; // Set to 640.5 ADC clock cycles

	// Configure GPIO for ADC (Can be done any time)
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN;
	GPIOC->MODER |= GPIO_MODER_MODE0; // Set to Analog Mode (11)
	GPIOC->ASCR |= GPIO_ASCR_ASC0;	// Connect analog switch to the ADC input

	// Configure interrupts
	NVIC->ISER[0] |= 1 << (0x1F & ADC1_2_IRQn);
	ADC1->IER |= ADC_IER_EOCIE;
	ADC1->ISR &= ~(ADC_IER_EOCIE);

	__enable_irq();
}

#define MAX_ANALOG 4095.0
#define REF_VOLTAGE 330

int ADC_Conversion(uint16_t dig_Val){
	// Calibration
	if(dig_Val < 434){ // 0 -> 0.35 V ?
		dig_Val *= 1.02;
	}
	if(dig_Val < 1861){ // 0.35 -> 1.5 V
		dig_Val *= 1.008;
	}
	else{ // 1.5 -> 3.3 V
		dig_Val *= 1.0028;
	}

	// Calculation
	int analog_Val = (dig_Val / MAX_ANALOG) * REF_VOLTAGE + 1; //converting analog to digital

	return analog_Val;
}

void ADC_Avg(uint16_t * ADC_Arr, int array_length, int * output){
	//Finding Min/Max/Avg of 20 sample points
	//output is MIN, MAX, AVG
	int total = 0; //total to be used to find Avg
	output[0] = ADC_Arr[0]; //Setting Original min to compare to
	output[1] = ADC_Arr[0];

	for(int i = 0; i < array_length; i++){
		if(ADC_Arr[i] < output[0]){ //checking for new Min
			output[0] = ADC_Arr[i];
		}
		if(ADC_Arr[i] > output[1]){ //checking for new Max
			output[1] = ADC_Arr[i];
		}
		total += ADC_Arr[i]; //Adding val to total
	}

	output[2] = total / array_length; //Finding Avg of Sample set
}

#define MAX_VOLTAGE 330
#define MIN_VOLTAGE 0
#define V_TOLERANCE 2


int find_Freq(uint16_t ADC_Arr[], uint16_t * zeros, int array_len, int t_max, int t_min){
	//output is [MAX1, MIN1, MAX2, MIN2]
	int output[4] = { 0 };

	int index = 0;
	int max_index = 0;		// Minimum Possible Voltage Value
	int min_index = 0; 	// Maximum possible voltage value
	uint8_t min_flag = 0;
	int min_found = 0;
	int max_found = 0;

	for(int i = 0; i < array_len; i++){
		if(t_max - ADC_Arr[i] <= 1){
			max_index = i;
			max_found = 1;
		}
		if(ADC_Arr[i] - t_min <= 1){
			min_index = i;
			min_found = 1;
		}
		if(max_found == 1 && min_found == 1){
			min_found = 0;
			max_found = 0;
			output[index] = max_index;
			index++;
			output[index] = min_index;
			index++;
		}
//		// Reaches a threshold, so write to array
//		if(i == zeros[index]){
//			if(min_flag){	// Left a min region
//				output[index] = min_index;
//			}
//			else{	// Left a max region
//				output[index] = max_index;
//			}
//			index++;
//		}
//
//		if(ADC_Arr[i] < ADC_Arr[min_index] ){ //checking for new min_index
//			min_index = i;
//			min_flag = 1;
//		}
//		if(ADC_Arr[i] > ADC_Arr[max_index]){ // checking for new Max
//			max_index = i;
//			min_flag = 0;
//		}
	}

	int diff1 = output[3] - output[1];
	int diff2 = output[2] - output[0];
	int avg_diff = (diff1 + diff2) / 2;

	// 0.00026667 seconds between indices -> sampling rate ~3750 Hz

	int index_diff = avg_diff; //gives the differences in index between every other crossing
	int freq = (100000000/(index_diff * 273333)); //translation to Frequency

	return freq;
}

