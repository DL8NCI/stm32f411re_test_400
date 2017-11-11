/*
 * HIH8000.c
 *
 *  Created on: 06.11.2017
 *      Author: DL8NCI
 */

#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "HIH8000.h"


// definitions used for Event Group
#define EG_I2C_CMD_SENT	     ( 1 << 0 )
#define EG_I2C_DATA_RECEIVED ( 1 << 1 )

// function prototype
void ReadHIH8000Impl(void const * argument);

// local variables
static I2C_HandleTypeDef *_hi2c;
static EventGroupHandle_t EgHandle;


/**
  * @brief  Initialize HIH8000 access
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @retval pdPASS or pdFAIL
  */
BaseType_t HIH8000_Init(I2C_HandleTypeDef *hi2c) {

	BaseType_t rc;

	_hi2c = hi2c;

	rc = xTaskCreate(
		(TaskFunction_t)ReadHIH8000Impl,
		"Read HIH8000",
		196,
		NULL,
		4,
		&ReadHIH8000Handle);

	if (rc!=pdPASS) return pdFAIL;


	EgHandle = xEventGroupCreate();
	if (EgHandle==NULL) return pdFAIL;
	xEventGroupClearBits(EgHandle, EG_I2C_CMD_SENT | EG_I2C_DATA_RECEIVED);

	return pdPASS;

	}



void ReadHIH8000Impl(void const * argument) {

	uint8_t b[4];
	double humidity, temperature;
	EventBits_t bits;
	HAL_StatusTypeDef rc;

	for(;;) {

		ulTaskNotifyTake( pdTRUE, portMAX_DELAY );

		xEventGroupClearBits(EgHandle, EG_I2C_CMD_SENT | EG_I2C_DATA_RECEIVED);

		b[0] = 0x00;
		rc = HAL_I2C_Master_Transmit_IT(_hi2c, 0x4e, &b[0], 1);
		configASSERT( rc == HAL_OK );

		bits = xEventGroupWaitBits(EgHandle, EG_I2C_CMD_SENT, pdTRUE, pdFALSE, 1000);
		configASSERT((bits & EG_I2C_CMD_SENT)!=0);
		vTaskDelay(100);

		rc = HAL_I2C_Master_Receive_IT(_hi2c, 0x4f, &b[0], 4);
		configASSERT( rc == HAL_OK );
		bits = EG_I2C_DATA_RECEIVED & xEventGroupWaitBits(EgHandle, EG_I2C_DATA_RECEIVED, pdTRUE, pdFALSE, 1000);
		configASSERT(bits!=0);

		humidity = (((uint16_t)b[0] & 0x003f) << 8) | (uint16_t)b[1];
		humidity = humidity/163.820;;

		temperature = ((uint16_t)b[2]<<6) | ((uint16_t)b[3]>>2);
		temperature = temperature*165.0/16382.0 - 40.0;

		printf("RH = %4.1f %%   T = %4.1f deg\r\n", humidity, temperature);

		}
	}

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c) {
	if (hi2c->Instance != _hi2c->Instance) return;

	BaseType_t xHigherPriorityTaskWoken, rc;
	xHigherPriorityTaskWoken = pdFALSE;

	rc = xEventGroupSetBitsFromISR(EgHandle, EG_I2C_CMD_SENT, &xHigherPriorityTaskWoken);
	if (rc!=pdFAIL) portYIELD_FROM_ISR( xHigherPriorityTaskWoken );

	}
void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c) {
	if (hi2c->Instance != _hi2c->Instance) return;

	BaseType_t xHigherPriorityTaskWoken, rc;
	xHigherPriorityTaskWoken = pdFALSE;

	rc = xEventGroupSetBitsFromISR(EgHandle, EG_I2C_DATA_RECEIVED, &xHigherPriorityTaskWoken);
	if (rc!=pdFAIL) portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
	}

