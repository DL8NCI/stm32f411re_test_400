/*
 * HIH8000.h
 *
 *  Created on: 06.11.2017
 *      Author: DL8NCI
 */

#ifndef HIH8000_H_
#define HIH8000_H_


osThreadId ReadHIH8000Handle;
BaseType_t HIH8000_Init(I2C_HandleTypeDef *hi2c);


#endif /* HIH8000_H_ */
