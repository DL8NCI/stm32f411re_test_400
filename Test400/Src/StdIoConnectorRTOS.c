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
#include "cmsis_os.h"

#include <StdIoConnectorRTOS.h>


#define EG_SIOC_BUFFER_FREE 1


struct TTxQueueItem {
	uint8_t *buff;
	int len;
	};


static UART_HandleTypeDef *_huart;
static UART_HandleTypeDef *_huart_diag;
static osMessageQId TxQueueHandle;
static osMessageQId _RxQueueHandle;
static osThreadId TsSendCharactersHandle;
static struct TTxQueueItem _qi;
static char clr_home[7] = "\e[2J\e[H";
static EventGroupHandle_t EgSIOCHandle;

void TsSendCharactersImpl(void const * argument);
void UART_WaitOnFlag_Simple(UART_HandleTypeDef *huart, uint32_t Flag, FlagStatus Status);
HAL_StatusTypeDef HAL_UART_Transmit_Simple(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef SIOC_HAL_UART_Receive_IT(UART_HandleTypeDef *huart);

void SIOC_Init(UART_HandleTypeDef *huart, osMessageQId *RxQueueHandle) {

	_huart = huart;
	_RxQueueHandle = RxQueueHandle;

	_qi.buff = NULL;

	HAL_UART_Transmit(_huart, &clr_home, 7, HAL_MAX_DELAY);

	xTaskCreate(
		(TaskFunction_t)TsSendCharactersImpl,
		"SendCharacters",
		128,
		NULL,
		4,
		&TsSendCharactersHandle);

	TxQueueHandle = xQueueCreate( 32, sizeof(struct TTxQueueItem));

	EgSIOCHandle = xEventGroupCreate();
	xEventGroupClearBits(EgSIOCHandle, EG_SIOC_BUFFER_FREE);

	SIOC_HAL_UART_Receive_IT(huart);
	}


void SIOC_InitDiag(UART_HandleTypeDef *huart) {
	_huart_diag = huart;

	SIOC_SendDiagPort(&clr_home);
	SIOC_SendDiagPort("Diag started\r\n");
	}


void TsSendCharactersImpl(void const * argument) {

	HAL_StatusTypeDef rc;
	EventBits_t b;

	for(;;)  {

		if (xQueueReceive(TxQueueHandle, &_qi, 10)) {
			rc = HAL_UART_Transmit_DMA(_huart, _qi.buff, _qi.len);
			configASSERT( rc==HAL_OK );

			b = xEventGroupWaitBits(
				EgSIOCHandle,
				EG_SIOC_BUFFER_FREE,
				pdTRUE,
				pdFALSE,
				portMAX_DELAY );

			vPortFree(_qi.buff);
			_qi.buff = NULL;
			_qi.len = 0;
			}
		}
	}


void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {

	BaseType_t xHigherPriorityTaskWoken, rc;

	if (huart!=_huart) return;

	rc = xEventGroupSetBitsFromISR(
		EgSIOCHandle,
		EG_SIOC_BUFFER_FREE,
		&xHigherPriorityTaskWoken );

	configASSERT( rc !=  pdFAIL );
	portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
	}


void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
	if (huart!=_huart) return;
	configASSERT( 1==1 );
	}

/*
void HAL_UART_TxHalfCpltCallback(UART_HandleTypeDef *huart) {
	if (huart!=_huart) return;
	}
*/

int _write(int file, char *data, int len) {
	struct TTxQueueItem qi;

	if ((file != STDOUT_FILENO) && (file != STDERR_FILENO)) {
        errno = EBADF;
        return -1;
    	}

	qi.buff = pvPortMalloc(len);
	qi.len = len;
	memcpy(qi.buff,data,len);

	if( xQueueSendToBack( TxQueueHandle, &qi, ( TickType_t ) 1000 ) != pdPASS ) {
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
	HAL_UART_Transmit_Simple(_huart_diag, &c[i], l+1);
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






/**
  * @brief  Receives a byte in non blocking mode - 8N1
  * @param  huart pointer to a UART_HandleTypeDef structure that contains
  *                the configuration information for the specified UART module.
  * @retval HAL status
  */
static HAL_StatusTypeDef SIOC_UART_Receive_IT(UART_HandleTypeDef *huart) {
	uint8_t b;
	BaseType_t rc;
	BaseType_t xHigherPriorityTaskWoken;

	if(huart->RxState != HAL_UART_STATE_BUSY_RX) return HAL_BUSY;
	b = (uint8_t)(huart->Instance->DR & (uint8_t)0x00FF);
    rc = xQueueSendToBackFromISR(_RxQueueHandle, &b, &xHigherPriorityTaskWoken);

    configASSERT( rc ==  pdPASS );
	portYIELD_FROM_ISR( xHigherPriorityTaskWoken );

    return HAL_OK;
	}


/**
  * @brief  End ongoing Rx transfer on UART peripheral (following error detection or Reception completion).
  * @param  huart UART handle.
  * @retval None
  */
static void SIOC_UART_EndRxTransfer(UART_HandleTypeDef *huart) {
	/* Disable RXNE, PE and ERR (Frame error, noise error, overrun error) interrupts */
	CLEAR_BIT(huart->Instance->CR1, (USART_CR1_RXNEIE | USART_CR1_PEIE));
	CLEAR_BIT(huart->Instance->CR3, USART_CR3_EIE);

	/* At end of Rx process, restore huart->RxState to Ready */
	huart->RxState = HAL_UART_STATE_READY;
	}

/**
  * @brief  Receives an unlimited amount of data in non blocking mode
  * @param  huart pointer to a UART_HandleTypeDef structure that contains
  *                the configuration information for the specified UART module.
  * @retval HAL status
  */
HAL_StatusTypeDef SIOC_HAL_UART_Receive_IT(UART_HandleTypeDef *huart) {
	/* Check that a Rx process is not already ongoing */
	if(huart->RxState == HAL_UART_STATE_READY) {

		__HAL_LOCK(huart);

		huart->pRxBuffPtr = NULL;
		huart->RxXferSize = 0;
		huart->RxXferCount = 0;
		huart->ErrorCode = HAL_UART_ERROR_NONE;
		huart->RxState = HAL_UART_STATE_BUSY_RX;

		__HAL_UNLOCK(huart);

		/* Enable the UART Error Interrupt: (Frame error, noise error, overrun error) */
		SET_BIT(huart->Instance->CR3, USART_CR3_EIE);

		/* Enable the UART Parity Error and Data Register not empty Interrupts */
		SET_BIT(huart->Instance->CR1, USART_CR1_PEIE | USART_CR1_RXNEIE);

		return HAL_OK;
		}
	else {
		return HAL_BUSY;
		}
	}



/**
  * @brief  This function handles UART interrupt request.
  * @param  huart pointer to a UART_HandleTypeDef structure that contains
  *                the configuration information for the specified UART module.
  * @retval None
  */
void SIOC_IRQ_Handler(UART_HandleTypeDef *huart) {
	uint32_t isrflags   = READ_REG(huart->Instance->SR);
	uint32_t cr1its     = READ_REG(huart->Instance->CR1);
	uint32_t cr3its     = READ_REG(huart->Instance->CR3);
	uint32_t errorflags = 0x00U;

	/* If no error occurs */
	errorflags = (isrflags & (uint32_t)(USART_SR_PE | USART_SR_FE | USART_SR_ORE | USART_SR_NE));
	if(errorflags == RESET) {
		/* UART in mode Receiver -------------------------------------------------*/
		if(((isrflags & USART_SR_RXNE) != RESET) && ((cr1its & USART_CR1_RXNEIE) != RESET)) {
			SIOC_UART_Receive_IT(huart);
			}
		}
	return;


	/* If some errors occur */
	if((errorflags != RESET) && (((cr3its & USART_CR3_EIE) != RESET) || ((cr1its & (USART_CR1_RXNEIE | USART_CR1_PEIE)) != RESET))) {
		/* UART parity error interrupt occurred ----------------------------------*/
		if(((isrflags & USART_SR_PE) != RESET) && ((cr1its & USART_CR1_PEIE) != RESET)) huart->ErrorCode |= HAL_UART_ERROR_PE;

		/* UART noise error interrupt occurred -----------------------------------*/
		if(((isrflags & USART_SR_NE) != RESET) && ((cr3its & USART_CR3_EIE) != RESET)) huart->ErrorCode |= HAL_UART_ERROR_NE;

		/* UART frame error interrupt occurred -----------------------------------*/
		if(((isrflags & USART_SR_FE) != RESET) && ((cr3its & USART_CR3_EIE) != RESET)) huart->ErrorCode |= HAL_UART_ERROR_FE;

		/* UART Over-Run interrupt occurred --------------------------------------*/
		if(((isrflags & USART_SR_ORE) != RESET) && ((cr3its & USART_CR3_EIE) != RESET)) huart->ErrorCode |= HAL_UART_ERROR_ORE;

		/* Call UART Error Call back function if need be --------------------------*/
		if(huart->ErrorCode != HAL_UART_ERROR_NONE) {
		/* UART in mode Receiver -----------------------------------------------*/
			if(((isrflags & USART_SR_RXNE) != RESET) && ((cr1its & USART_CR1_RXNEIE) != RESET)) {
				SIOC_UART_Receive_IT(huart);
				}

			/* If Overrun error occurs, or if any error occurs in DMA mode reception,
			   consider error as blocking */
			if((huart->ErrorCode & HAL_UART_ERROR_ORE) != RESET) {
				/* Blocking error : transfer is aborted
				   Set the UART state ready to be able to start again the process,
				   Disable Rx Interrupts, and disable Rx DMA request, if ongoing */
				SIOC_UART_EndRxTransfer(huart);

				/* Call user error callback */
				HAL_UART_ErrorCallback(huart);
				}
			else {
				/* Non Blocking error : transfer could go on.
				   Error is notified to user through user error callback */
				HAL_UART_ErrorCallback(huart);
				huart->ErrorCode = HAL_UART_ERROR_NONE;
				}
			}
		return;
		} /* End if some error occurs */

	}
