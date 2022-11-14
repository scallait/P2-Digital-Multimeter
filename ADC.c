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
	ADC1->ISR |= ADC_ISR_ADRDY; // Clear the ARDY bit by setting to 1 in ISR
	ADC1->CR |= ADC_CR_ADEN;
	while(!(ADC1->ISR & ADC_ISR_ADRDY)); // Set once ADC is ready

	// Configure the ADC
	ADC1->SQR1 &= ~(0b11111 << ADC_SQR1_SQ1_Pos);
	ADC1->SQR1 |= 0x1 << ADC_SQR1_SQ1_Pos; // Set channel 1?
	ADC1->SMPR1 &= ~(0b111 << ADC_SMPR1_SMP1_Pos); // Clear SMPR (2.5 ADC Clock Cycles)
	ADC1->SMPR1 |= 6 << ADC_SMPR1_SMP1_Pos; // Set to 247.5 ADC clock cycles

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

#define CALIBRATION_FACTOR 0.985
#define MAX_ANALOG 4095.0
#define REF_VOLTAGE 330

int ADC_Conversion(uint16_t analog_Val){
	int dig_Val = (analog_Val / MAX_ANALOG) * REF_VOLTAGE * CALIBRATION_FACTOR; //converting analog to digital
	return dig_Val;
}

void ADC_Avg(int * ADC_Arr, int * output){
	//Finding Min/Max/Avg of 20 sample points
	//output is MIN, MAX, AVG
	int total = 0; //total to be used to find Avg
	output[0] = ADC_Arr[0]; //Setting Original min to compare to
	output[1] = ADC_Arr[0];

	for(int i = 0; i < 20; i++){
		if(ADC_Arr[i] < output[0]){ //checking for new Min
			output[0] = ADC_Arr[i];
		}
		if(ADC_Arr[i] > output[1]){ //checking for new Max
			output[1] = ADC_Arr[i];
		}
		total += ADC_Arr[i]; //Adding val to total
	}

	output[2] = total/ 20; //Finding Avg of Sample set

}
