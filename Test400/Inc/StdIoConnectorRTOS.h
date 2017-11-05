/*
 * StdConnectorRTOS.h
 *
 *  Created on: 31.10.2017
 *      Author: DL8NCI
 */

#ifndef STDIOCONNECTORRTOS_H_
#define STDIOCONNECTORRTOS_H_

#include "stm32f4xx_hal.h"


void SIOC_Init(UART_HandleTypeDef *huart);
void SIOC_TxCpltCallback(UART_HandleTypeDef *huart);

void SIOC_InitDiag(UART_HandleTypeDef *huart);
void SIOC_SendDiagPort(char *s);
void SIOC_SendDiagPortUInt32(uint32_t x);


#endif /* STDIOCONNECTORRTOS_H_ */
