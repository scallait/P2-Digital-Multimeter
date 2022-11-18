/*
 * USART.h
 *
 *  Created on: Nov 11, 2022
 *      Author: Seb
 */

#ifndef SRC_P2_DIGITAL_MULTIMETER_USART_H_
#define SRC_P2_DIGITAL_MULTIMETER_USART_H_

#include "main.h"
#include <string.h>

void USART_init();

void USART_print(char character[]);

void USART_ESC_Code(char escape_code[]);

void USART_print_bit(uint8_t character);

void USART_print_num(int number);

void USART_print_freq(int num);

#endif /* SRC_P2_DIGITAL_MULTIMETER_USART_H_ */
