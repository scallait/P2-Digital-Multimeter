/*
 * InputCapture.c
 *
 *  Created on: Nov 13, 2022
 *      Author: Sebas
 */
#include "InputCapture.h"

void InputCapture_init(){
	//Configure TIM3 using PA6 as an input to Input Capture Mode

	//Clock configure for GPIOA
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;

	//set PA6 to input (recommended to do the by Benson, assume its the input pin we are linking)
	GPIOA->MODER &= ~(GPIO_MODER_MODE6);

	//Link TI1 to CCR1
	TIM3->CCMR1 &= ~(TIM_CCMR1_CC1S);
	TIM3->CCMR1 |= (TIM_CCMR1_CC1S_0);

	//Input Capture prescaler / CCR1 will take on CNT every trigger
	TIM3->CCMR1 &= ~(TIM_CCMR1_IC1PSC);

	//Binding the read edge to the rising edge (look at CCER in manual)
	TIM3->CCER &= ~(TIM_CCER_CC1NP);
	TIM3->CCER &= ~(TIM_CCER_CC1P);

	//Have Input capture run at same speed as sys clock
	TIM3->CCMR1 &= ~(TIM_CCMR1_IC1F);

	//Enable Compare Capture Mode
	TIM3->CCER |= TIM_CCER_CC1E;

	//Currently not enabling interrupts for CCR1 so no need for IRQ Handler

}
