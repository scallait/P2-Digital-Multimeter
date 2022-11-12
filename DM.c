/*
 * DM.c
 *
 *  Created on: Nov 11, 2022
 *      Author: Seb
 */

#include "DM.h"
#include "USART.h"


void GUI_init(){
	USART_ESC_Code("[2J");		// Clear the entire screen
	USART_ESC_Code("[1;0H");	// Move cursor to position (0,0)
	USART_print("-----------------------------------------------------------------");
	USART_ESC_Code("[2;0H");	// Move cursor to position (0,1)
	USART_ESC_Code("[1m");
	USART_print("|                      Digital Multimeter                       |");
	USART_ESC_Code("[0m");
	USART_ESC_Code("[3;0H");	// Move cursor to position (0,2)
	USART_print("-----------------------------------------------------------------");
	USART_ESC_Code("[4;0H");	// Move cursor to position (0,3)
	USART_print("|      DC Characteristics       |      AC Characteristics       |");
	USART_ESC_Code("[5;0H");	// Move cursor to position (0,3)
	USART_print("|---------------------------------------------------------------|");
	USART_ESC_Code("[6;0H");
	USART_print("|                               |                               |");
	USART_ESC_Code("[7;0H");	// Move cursor to position (0,4)
	USART_print("| Min: 0.00 V                   | Vrms: 0.00 V                  |");
	USART_ESC_Code("[8;0H");
	USART_print("|                               |                               |");
	USART_ESC_Code("[9;0H");	// Move cursor to position (0,5)
	USART_print("| Max: 0.00 V                   | PtoP: 0.00 V                  |");
	USART_ESC_Code("[10;0H");	// Move cursor to position (0,5)
	USART_print("|                               |                               |");
	USART_ESC_Code("[11;0H");	// Move cursor to position (0,6)
	USART_print("| Avg: 0.00 V                   | Freq: 0    Hz                 |");
	USART_ESC_Code("[12;0H");	// Move cursor to position (0,7)
	USART_print("|                               |                               |");
	USART_ESC_Code("[13;0H");	// Move cursor to position (0,5)
	USART_print("-----------------------------------------------------------------");
}

void update_DC(int min, int max, int avg){
	// Print Min
	USART_ESC_Code("[7;8H");	// First digit is printed at 7
	USART_print_num(min);

	// Print Max
	USART_ESC_Code("[9;8H");
	USART_print_num(max);

	// Print Avg
	USART_ESC_Code("[11;8H");
	USART_print_num(avg);

	USART_ESC_Code("[14;0H");	// Move cursor out of table
}

void update_AC(int vrms, int ptop, int freq){
	// Print Vrms
	USART_ESC_Code("[7;41H");	// Row 7 Column 41
	USART_print_num(vrms);

	// Print Peak to Peak value
	USART_ESC_Code("[9;41H");
	USART_print_num(ptop);

	// Print Freq
	USART_ESC_Code("[11;41H");
	USART_print_num(freq);

	USART_ESC_Code("[14;0H");	// Move cursor out of table
}

#define SQRT2x2 0.35355339059

void calc_RMS(int PtoP){
	/* Formula Vrms = 1 / ( 2 * sqrt(2)) * Vptop */
	int result = SQRT2x2 * PtoP;
	return result;
}
