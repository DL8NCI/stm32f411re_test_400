/*
 * VT100.c
 *
 *  Created on: 01.07.2017
 *      Author: DL8NCI
 */


#include "VT100.h"
#include <stdio.h>


void VT100CursorHome() {
	printf("\e[H");
	}

void VT100CursorGoto(int row, int col) {
	printf("\e[%d;%dH",row,col);
	}

void VT100EraseScreen() {
	printf("\e[2J");
	}

void VT100SetAttributeMode(uint8_t a1, uint8_t a2, uint8_t a3) {
	if ((a2==255) && (a3==255)) {
		printf("\e[%dm",a1);
		}
	else if (a3==255) {
		printf("\e[%d;%dm",a1,a2);
		}
	else {
		printf("\e[%d;%d;%dm",a1,a2,a3);
		}
	}


