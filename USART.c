/*
 * USART.c
 *
 *  Created on: Nov 11, 2022
 *      Author: Seb
 */

#include "USART.h"

void USART_init(){
	// Enable clock to GPIO pins
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;

	// Configure PA2/PA3 in AF Mode
	GPIOA->MODER &= ~(GPIO_MODER_MODE2 | GPIO_MODER_MODE3);

	GPIOA->MODER |= GPIO_MODER_MODE2_1 | GPIO_MODER_MODE3_1;

	// Configure AFR Registers
	GPIOA->AFR[0] &= ~(GPIO_AFRL_AFSEL2 | GPIO_AFRL_AFSEL3);

	GPIOA->AFR[0] |= (7 << GPIO_AFRL_AFSEL2_Pos) | (7 << GPIO_AFRL_AFSEL3_Pos);

	// Enable clock to USART2
	RCC->APB1ENR1 |=  RCC_APB1ENR1_USART2EN;

	// 1 Start Bit, 8 Data bits, n stop bits
	USART2->CR1 &= ~(USART_CR1_M1) | ~(USART_CR1_M0);

	// Clear Oversampling bit
	USART2->CR1 &= ~USART_CR1_OVER8;

	// Set Baud Rate of 417 ~115.2 kBps @48 MHz (Pg. 1355)
	USART2->BRR = 416;	// 48 MHz / 115.2 KBps

	// Enable interrupts for receiver
	USART2->CR1 |= USART_CR1_RXNEIE;

	// Enable Register, Transmit, and Receive
	USART2->CR1 |= USART_CR1_TE | USART_CR1_RE | USART_CR1_UE; 		// Enable USART

	// Enable interrupt vector
	NVIC->ISER[1] |= 1 << (0x1F & USART2_IRQn);

	// Enable global interrupts
	__enable_irq();
}

void USART_print(char character[]){
	int len = 0, i = 0;

	// Get length of string
	len = strnlen(character, 200);

	// Print string
	while(i < len){
		while(!(USART2->ISR & USART_ISR_TXE));
		USART2->TDR = character[i];
		i++;
	}
}

#define ASCII_OFFSET 48

void USART_print_num(int num){
	int num_int = (int)(num); 	// Truncate

	uint8_t dig1 = num_int / 100;
	uint8_t dig2 = (num_int - (dig1 * 100)) / 10;
	uint8_t dig3 = num_int - (dig1 * 100) - (dig2 * 10);

	// Print the digits
	USART_print_bit(dig1 + ASCII_OFFSET);
	USART_print_bit((uint8_t)'.');			// send "." after first 2 digits
	USART_print_bit(dig2 + ASCII_OFFSET);
	USART_print_bit(dig3 + ASCII_OFFSET);
}

void USART_print_bit(uint8_t character){
	while(!(USART2->ISR & USART_ISR_TXE));
	USART2->TDR = character;
}

void USART_ESC_Code(char escape_code[]){
	uint16_t i = 0x1B;

	while(!(USART2->ISR & USART_ISR_TXE));
	USART2->TDR = i;

	USART_print(escape_code);
}
