/*
 * DM.h
 *
 *  Created on: Nov 11, 2022
 *      Author: Seb
 */

#ifndef SRC_P2_DIGITAL_MULTIMETER_DM_H_
#define SRC_P2_DIGITAL_MULTIMETER_DM_H_

#include "main.h"

void GUI_init();

void update_DC(int min, int max, int avg);

void update_AC();

#endif /* SRC_P2_DIGITAL_MULTIMETER_DM_H_ */
