/*
 * SimplifiedUSART.c
 *
 *  Created on: 14.11.2017
 *      Author: DL8NCI
 */

#include "SimplifiedUSART.h"




/**
  * @brief  Receives an unlimited amount of data in non blocking mode
  * @param  huart pointer to a UART_HandleTypeDef structure that contains
  *                the configuration information for the specified UART module.
  * @retval HAL status
  */
HAL_StatusTypeDef S_USART_HAL_UART_Receive_IT(UART_HandleTypeDef *huart) {
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
  * @brief  End ongoing Rx transfer on UART peripheral (following error detection or Reception completion).
  * @param  huart UART handle.
  * @retval None
  */
static void SUSART_UART_EndRxTransfer(UART_HandleTypeDef *huart) {
	/* Disable RXNE, PE and ERR (Frame error, noise error, overrun error) interrupts */
	CLEAR_BIT(huart->Instance->CR1, (USART_CR1_RXNEIE | USART_CR1_PEIE));
	CLEAR_BIT(huart->Instance->CR3, USART_CR3_EIE);

	/* At end of Rx process, restore huart->RxState to Ready */
	huart->RxState = HAL_UART_STATE_READY;
	}





/**
  * @brief  This function handles UART interrupt request.
  * @param  huart pointer to a UART_HandleTypeDef structure that contains
  *                the configuration information for the specified UART module.
  * @retval None
  */
void SUSART_IRQ_Handler(UART_HandleTypeDef *huart) {
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

