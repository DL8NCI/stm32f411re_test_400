/*
 * StdConnectorRTOS.c
 *
 *  Created on: 31.10.2017
 *      Author: DL8NCI
 */

#include <sys/unistd.h>
#include <sys/errno.h>
#include <stdlib.h>
#include <string.h>

#include <StdIoConnectorRTOS.h>


#include "cmsis_os.h"

// type declarations
struct TTxQueueItem {
	uint8_t *buff;
	int len;
	int inUse;
	};

// variable declarations
static UART_HandleTypeDef *_huart;
static UART_HandleTypeDef *_huart_diag;
static osMessageQId QuTxQueueHandle;
osThreadId TsSendCharactersHandle;
volatile static struct TTxQueueItem _qi;
static char clr_home[7] = "\e[2J\e[H";

// function prototypes
void TsSendCharactersImpl(void const * argument);
void UART_WaitOnFlag_Simple(UART_HandleTypeDef *huart, uint32_t Flag, FlagStatus Status);
HAL_StatusTypeDef HAL_UART_Transmit_Simple(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);


// Implementation
void SIOC_Init(UART_HandleTypeDef *huart) {
	// clear buffer
	_huart = huart;
	_qi.buff = NULL;
	_qi.inUse = 0; // 0:free, 1:in use, 2:can be freed

	// clear screen
	HAL_UART_Transmit(_huart, &clr_home, 7, HAL_MAX_DELAY);

	// make task and queue
	xTaskCreate(
			(TaskFunction_t)TsSendCharactersImpl,
			"SendCharacters",
			128,
			NULL,
			4,
			&TsSendCharactersHandle);

	QuTxQueueHandle = xQueueCreate(
			32,
			sizeof(struct TTxQueueItem));

	}

void SIOC_InitDiag(UART_HandleTypeDef *huart) {
	_huart_diag = huart;

	// clear screen
	SIOC_SendDiagPort(&clr_home);
	SIOC_SendDiagPort("Diag started\r\n");

	}

void TsSendCharactersImpl(void const * argument) {

	for(;;)  {
		if (_qi.inUse==2) {
			vPortFree(_qi.buff);
			_qi.buff = NULL;
			_qi.len = 0;
			_qi.inUse = 0;
			}

		if (_qi.inUse==0) {
			if (xQueueReceive(QuTxQueueHandle, &_qi, 10)) {
				_qi.inUse = 1;
				HAL_StatusTypeDef rc = HAL_UART_Transmit_IT(_huart, _qi.buff, _qi.len);
				configASSERT( rc==HAL_OK );
				}
			}
		}
	}


void SIOC_TxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart!=_huart) return;
	_qi.inUse = 2;
	}


int _write(int file, char *data, int len) {
	struct TTxQueueItem qi;

	if ((file != STDOUT_FILENO) && (file != STDERR_FILENO)) {
        errno = EBADF;
        return -1;
    	}

	qi.buff = pvPortMalloc(len);
	qi.len = len;
	qi.inUse = 0; // don't care
	memcpy(qi.buff,data,len);

	if( xQueueSendToBack( QuTxQueueHandle, &qi, ( TickType_t ) 1000 ) != pdPASS ) {
        errno = ENOMEM;
        return -1;
    	}

	return len;
    }


void SIOC_SendDiagPort(char *s) {
	HAL_UART_Transmit_Simple(_huart_diag, s, strlen(s));
	}

void SIOC_SendDiagPortUInt32(uint32_t x) {
	char c[12];
	int i,l;
	uint32_t h,r;


	h = x;
	i = 12;
	l = 0;
	while (h>0) {
		r = h % 10;
		h = h / 10;
		c[i]= (char)(48+r);
		i--;
		l++;
		if (i==0) break;
		}
	c[i] = ' ';
	HAL_UART_Transmit_Simple(_huart_diag, &c[i], l);
	}

HAL_StatusTypeDef HAL_UART_Transmit_Simple(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size) {
	uint16_t* tmp;

	/* Check that a Tx process is not already ongoing */
	if(huart->gState == HAL_UART_STATE_READY) {
		if((pData == NULL ) || (Size == 0)) return HAL_ERROR;

		/* Process Locked */
		__HAL_LOCK(huart);

		huart->ErrorCode = HAL_UART_ERROR_NONE;
		huart->gState = HAL_UART_STATE_BUSY_TX;

		huart->TxXferSize = Size;
		huart->TxXferCount = Size;
		while(huart->TxXferCount > 0U) {
			huart->TxXferCount--;
			if(huart->Init.WordLength == UART_WORDLENGTH_9B) {
				UART_WaitOnFlag_Simple(huart, UART_FLAG_TXE, RESET);
				tmp = (uint16_t*) pData;
				huart->Instance->DR = (*tmp & (uint16_t)0x01FF);
				if(huart->Init.Parity == UART_PARITY_NONE)
					pData +=2U;
				else
					pData +=1U;
				}
			else {
				UART_WaitOnFlag_Simple(huart, UART_FLAG_TXE, RESET);
				huart->Instance->DR = (*pData++ & (uint8_t)0xFF);
				}
			}

		UART_WaitOnFlag_Simple(huart, UART_FLAG_TC, RESET);

    /* At end of Tx process, restore huart->gState to Ready */
		huart->gState = HAL_UART_STATE_READY;

    /* Process Unlocked */
		__HAL_UNLOCK(huart);

		return HAL_OK;
		}
	else return HAL_BUSY;
	}


void UART_WaitOnFlag_Simple(UART_HandleTypeDef *huart, uint32_t Flag, FlagStatus Status) {
	while ((__HAL_UART_GET_FLAG(huart, Flag) ? SET : RESET) == Status) { ; }
	}

