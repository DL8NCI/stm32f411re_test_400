/*
 * RTOS_Statistics.c
 *
 *  Created on: 06.11.2017
 *      Author: DL8NCI
 */

#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "RTOSStatistics.h"

void RTOSStatisticsImpl(void const * argument);

static TIM_HandleTypeDef *_htim;
static uint16_t cntr;

void RTOSStatisticsInit(TIM_HandleTypeDef *htim) {
	_htim = htim;
	cntr = 0;

	xTaskCreate(
		(TaskFunction_t)RTOSStatisticsImpl,
		"RTOS Statistics",
		256,
		NULL,
		4,
		&RTOSStatisticsHandle);

	}


void RTOSStatisticsImpl(void const * argument) {
	TaskStatus_t *tsa;
	volatile UBaseType_t as, i;
	unsigned long totalRuntime;

	for(;;) {

		ulTaskNotifyTake( pdTRUE, portMAX_DELAY );

		as = uxTaskGetNumberOfTasks();
		tsa = pvPortMalloc(as*sizeof(TaskStatus_t));

		as = uxTaskGetSystemState( tsa, as, &totalRuntime);


		printf("cntr = %d   tasks=%lu   total runtime = %10.1f s\r\n", cntr, as, (double)totalRuntime/100000.0);
		cntr++;

		totalRuntime /= 100UL;

		if (totalRuntime == 0) {
			printf("TNr            Task-Name Stk    Curr Prio    Base Prio      Runtime\r\n");
			for (i=0; i<as; i++) {
				printf("%3lu %20s %3hu %12lu %12lu %12lu\r\n", tsa[i].xTaskNumber, tsa[i].pcTaskName, tsa[i].usStackHighWaterMark, tsa[i].uxCurrentPriority, tsa[i].uxBasePriority, tsa[i].ulRunTimeCounter);
				}
			}
		else {
			printf("TNr            Task-Name Stk    Curr Prio    Base Prio Runtme\r\n");
			for (i=0; i<as; i++) {
				printf("%3lu %20s %3hu %12lu %12lu %5.1f%%\r\n", tsa[i].xTaskNumber, tsa[i].pcTaskName, tsa[i].usStackHighWaterMark, tsa[i].uxCurrentPriority, tsa[i].uxBasePriority, (double)tsa[i].ulRunTimeCounter/(double)totalRuntime);
				}
			}



		printf("\r\n");

		vPortFree(tsa);

		}
	}

unsigned long getRunTimeCounterValue(void) {
	uint32_t c = _htim->Instance->CNT;
	return c;
	}

void configureTimerForRunTimeStats(void) {
	HAL_StatusTypeDef rc = HAL_TIM_Base_Start(_htim);
	configASSERT( rc == HAL_OK );
	}

