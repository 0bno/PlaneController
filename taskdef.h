/*
 * taskdef.h
 *
 *  Created on: Jan 20, 2023
 *      Author: florentgoutailler
 */

#ifndef TASKDEF_H_
#define TASKDEF_H_

//extern variable
extern uint32_t RTOS_RunTimeCounter;

//Tasks
void vApplicationIdleHook( void );
int growStack( void );

void vTaskInit( void *);
void vTask1a(void *pvParameters);
void vTask2a(void *pvParameters);
void vTask3(void *pvParameters);

#endif /* TASKDEF_H_ */
