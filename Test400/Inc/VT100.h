/*
 * VT100.h
 *
 *  Created on: 01.07.2017
 *      Author: DL8NCI
 */

#ifndef VT100_H_
#define VT100_H_

#include  <sys/unistd.h>


#define VT100_AM_RESET 0
#define VT100_AM_BRIGHT 1
#define VT100_AM_DIM 2
#define VT100_AM_UNDERSCORE 4
#define VT100_AM_BLINK 5
#define VT100_AM_REVERSE 7
#define VT100_AM_HIDDEN 8
#define VT100_AM_IGNORE 255

#define VT100_AM_FG_BLACK 30
#define VT100_AM_FG_RED 31
#define VT100_AM_FG_GREEN 32
#define VT100_AM_FG_YELLOW 33
#define VT100_AM_FG_BLUE 34
#define VT100_AM_FG_MAGENTA 35
#define VT100_AM_FG_CYAN 36
#define VT100_AM_FG_WHITE 37

#define VT100_AM_BG_BLACK 40
#define VT100_AM_BG_RED 41
#define VT100_AM_BG_GREEN 42
#define VT100_AM_BG_YELLOW 43
#define VT100_AM_BG_BLUE 44
#define VT100_AM_BG_MAGENTA 45
#define VT100_AM_BG_CYAN 46
#define VT100_AM_BG_WHITE 47


void VT100CursorHome();
void VT100CursorGoto(int row, int col);
void VT100EraseScreen();
void VT100SetAttributeMode(uint8_t a1, uint8_t a2, uint8_t a3);


#endif /* VT100_H_ */
