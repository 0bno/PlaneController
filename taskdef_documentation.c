/* USER CODE BEGIN Header */

/**
 ******************************************************************************
 * @file           : taskdef.c
 * @brief          : Definition of functions, tasks and others
 ******************************************************************************
 */
/*
 * Mise à jour le 30 mars 2023
 */

/* USER CODE END Header */

#include "main.h"
#include "stm32f429i_discovery_lcd.h"
#include <stdio.h>
#include <stdlib.h>
#include "taskdef.h"
#include "mylib.h"
#include <math.h>
#include <string.h>

TimerHandle_t Tim50Hz; /**< Timer de 50Hz pour lancer la tâche 2a*/
TimerHandle_t Tim5Hz; /**< Timer de 5Hz pour lancer la tâche 3*/
TimerHandle_t Tim5Hz_2; /**< Timer de 5Hz pour lancer la tâche 4*/
TimerHandle_t Tim100Hz; /**< Timer de 100Hz pour lancer la tâche 1a*/

extern I2C_HandleTypeDef hi2c3; /**/

typedef struct {
	    double p_acc[3]; /*Paramètre de l'accéléromètre*/
	    double p_gyro[3]; /*Paramètre du gyromètre*/
	} Message_t; /**<Première structure qui contient les valeurs du gyromètre et de l'accéléromètre pour la message queue*/

typedef struct{
	double roll_angle; /*Calcul du roll angle*/
	double pitch_angle; /*Calcul du pitch angle*/
} Message_calcul; /**<Première structure qui contient les valeurs du pitch et de l'angle pour la message queue*/


/* Déclaration des message queues */
QueueHandle_t xQueue1aTo2a; /**<Message Queue permettant de passer les données du gyromètre et de l'accéléromètre de la tâche 1a vers 2a*/
QueueHandle_t xQueue2aTo3; /**<Message Queue permettant de passer les données du pitch et de l'angle de la tâche 2a vers 3*/
QueueHandle_t xQueue1bTo3; /**<Message Queue permettant de passer les données du magnétomètre de la tâche 1b vers 3*/

/* Déclaration des sémaphores */
SemaphoreHandle_t xMutexI2C; /**<Sémaphore mutex pour protéger le bus I2C.*/
SemaphoreHandle_t xMutexRS232; /**<Sémaphore mutex pour protéger le bus RS232.*<*/
SemaphoreHandle_t xMutexLCD; /**<Sémaphore mutex pour protéger l'affichage des données sur l'écran LCD.*/
SemaphoreHandle_t xSemaphore1aTo2a; /**<Sémaphore binaire pour protéger le passage de la tâche 1a à 2a.*/
SemaphoreHandle_t xSemaphore; /**<Sémaphore binaire pour protéger la tâche 1a.*/
SemaphoreHandle_t xSemaphore2; /**<Sémaphore binaire pour protéger la tâche 1b.*/
SemaphoreHandle_t xSemaphore3; /**<Sémaphore binaire pour protéger la tâche 3.*/
SemaphoreHandle_t xSemaphore4; /**<Sémaphore binaire pour protéger la tâche 4.*/


/**
  * @brief Fonction CallBack pour le timer de 100Hz de la tâche 1a.
  * @param TimerHandle_t xTimer
  * @retval void
  */
void prvTim100HzCallbackFunction( TimerHandle_t xTimer )
{
	xSemaphoreGive(xSemaphore);
}

/**
  * @brief Fonction CallBack pour le timer de 100Hz de la tâche 1b.
  * @param TimerHandle_t xTimer
  * @retval void
  */
void prvTim100HzCallbackFunction_2( TimerHandle_t xTimer )
{
	xSemaphoreGive(xSemaphore);
}


/**
  * @brief Fonction CallBack pour le timer de 50Hz de la tâche 1b.
  * @param TimerHandle_t
  * @retval void
  */
void prvTim50HzCallbackFunction( TimerHandle_t xTimer )
{
	//to be completed...
}

/**
  * @brief Fonction CallBack pour le timer de 5Hz de la tâche 3.
  * @param TimerHandle_t xTimer
  * @retval void
  */
void prvTim5HzCallbackFunction( TimerHandle_t xTimer )
{
	xSemaphoreGive(xSemaphore3);
}

/**
  * @brief Fonction CallBack pour le timer de 5Hz de la tâche 4.
  * @param TimerHandle_t xTimer
  * @retval void
  */
void prvTim5HzCallbackFunction_2( TimerHandle_t xTimer )
{
	xSemaphoreGive(xSemaphore4);
}

/**
  * @brief Task d'initialisation du système : Création de taches, initialisation des timers. La tâche se détruit en fin d'éxécution.
  * @param pvParameters Vide
  * @retval void
  */
void vTaskInit( void *pvParameters)
{
	/*Timers*/
	/* Timer 50Hz */
	Tim50Hz=xTimerCreate("Tim50Hz",pdMS_TO_TICKS(20),pdTRUE,0,prvTim50HzCallbackFunction);
	if (Tim50Hz==NULL) {
		printf("Error creation timer 50Hz\r\n");
		exit(1);
	}

	/* Timer 100Hz */
	Tim100Hz=xTimerCreate("Tim100Hz",pdMS_TO_TICKS(10),pdTRUE,0,prvTim100HzCallbackFunction);
	if (Tim100Hz==NULL) {
		printf("Error creation timer 100Hz\r\n");
		exit(1);
	}

	/* Timer 5Hz */
	Tim5Hz=xTimerCreate("Tim5Hz",pdMS_TO_TICKS(200),pdTRUE,0,prvTim5HzCallbackFunction);
	if (Tim5Hz==NULL) {
		printf("Error creation timer 5Hz\r\n");
		exit(1);
	}

	/* Timer 5Hz */
	Tim5Hz_2=xTimerCreate("Tim5Hz_2",pdMS_TO_TICKS(200),pdTRUE,0,prvTim5HzCallbackFunction_2);
	if (Tim5Hz_2==NULL) {
		printf("Error creation timer 5Hz\r\n");
		exit(1);
	}
	/* End Timers */

	/* Creation Semmaphore */
	xSemaphore = xSemaphoreCreateBinary();
	xMutexI2C = xSemaphoreCreateMutex();
	xMutexRS232 = xSemaphoreCreateMutex();
	xMutexLCD = xSemaphoreCreateMutex();
	xSemaphore3 = xSemaphoreCreateBinary();
	xSemaphore4 = xSemaphoreCreateBinary();

	vQueueAddToRegistry(xSemaphore, "xSemaphore");
	vQueueAddToRegistry(xMutexI2C, "xMutexI2C");
	vQueueAddToRegistry(xMutexRS232, "xMutexRS232");
	vQueueAddToRegistry(xMutexLCD, "xMutexLCD");
	vQueueAddToRegistry(xSemaphore3, "xSemaphore3");
	vQueueAddToRegistry(xSemaphore4, "xSemaphore4");

	/* Creation message queue */
	xQueue1aTo2a = xQueueCreate(1, sizeof(Message_t));
	xQueue2aTo3 = xQueueCreate(1, sizeof(Message_calcul));

	vQueueAddToRegistry(xQueue1aTo2a, "xQueue1aTo2a");
	vQueueAddToRegistry(xQueue2aTo3, "xQueue2aTo3");


	/* Creation Tâches */
	xTaskCreate(vTask1a, "Task1a",1000, NULL, 7, NULL);
	xTaskCreate(vTask2a, "Task2a",1000, NULL, 5, NULL);
	xTaskCreate(vTask3, "Task3",1000, NULL, 3, NULL);
	xTaskCreate(vTask4, "Task4",1000, NULL, 3, NULL);

	xTimerStart(Tim5Hz,portMAX_DELAY);
	xTimerStart(Tim100Hz,portMAX_DELAY);
	xTimerStart(Tim50Hz,portMAX_DELAY);
	xTimerStart(Tim5Hz_2,portMAX_DELAY);

	vTaskDelete(NULL);
}

/**
  * @brief Tâche 1a : acquisition des données de l'accéléromètre et du gyromètre
  * @param pvParameters void
  * @retval void
  */
void vTask1a(void *pvParameters)
{
	double p_acc[3];
	double p_gyro[3];
	Message_t message;

	while (1) {
		if (xSemaphoreTake(xSemaphore, portMAX_DELAY) == pdTRUE) {
			// Accès à la ressource protégée ici
			xSemaphoreTake(xMutexI2C, portMAX_DELAY);

			MeasureA(&hi2c3, p_acc);
			MeasureG(&hi2c3, p_gyro);

			xSemaphoreGive(xMutexI2C);

			// Envoi des valeurs dans la file d'attente

			memcpy(message.p_acc, p_acc, sizeof(p_acc));
			memcpy(message.p_gyro, p_gyro, sizeof(p_gyro));
			xQueueSend(xQueue1aTo2a, &message, portMAX_DELAY);
		}
	}
}

/**
  * @brief Tâche 1b : acquisition des données du magnétomètre + Calcul du yaw
  * @param pvParameters void
  * @retval void
  */

void vTask1b(void *pvParameters)
{
	//Manque de temps nous avons réalisé les calculs directement dans la fonction 1b et
	//envoyer les valeurs directement à la tâche 3. Nous n'avons pas respecté notre schéma.

	double p_mag[3];
	//double p_press[3]={0, 0, 0};
	double yaw;
	//double p_theta[3];
	//long signed int p_Tfine[3];
	double p_bias[3] = {0, 0, 0};
	double p_scale[3] = {1, 1, 1};

	while(1)
	{
		if (xSemaphoreTake(xSemaphore, portMAX_DELAY) == pdTRUE) {
			// Accès à la ressource protégée ici
			xSemaphoreTake(xMutexI2C, portMAX_DELAY);

			MeasureM(&hi2c3, p_mag, p_bias, p_scale);
			//MeasureP(&hi2c3, p_press, MeasureT_BMP280(&hi2c3, p_theta, p_Tfine));
			xSemaphoreGive(xMutexI2C);

			Yaw(&hi2c3, p_mag, &yaw);

			// Envoi des valeurs dans la file d'attente
			xQueueSend(xQueue1bTo3, &yaw, 10);

		}

	}
}

/**
  * @brief Tâche 2a : calcul des angles de roulis, tangage grâce aux données récupérées dans la tâche 1a.
  * @param pvParameters Vide
  * @retval void
  */
void vTask2a(void *pvParameters)
{
	Message_t message;
	Message_calcul message_calcul;
	double p_acc[3];
	double p_gyro[3];
	double roll_angle[2];
	roll_angle[0]= 0;
	double pitch_angle[2];
	roll_angle[0]= 0;

	while (1) {
		 xQueueReceive(xQueue1aTo2a, &message, portMAX_DELAY);
		// Utilisation des valeurs de p_acc et p_gyro
		 memcpy(p_acc, message.p_acc, sizeof(p_acc));
		 memcpy(p_gyro, message.p_gyro, sizeof(p_gyro));
		 roll_angle[1] = 0.9*(roll_angle[0]+0.01*p_gyro[0])+(0.1)*atan2(p_acc[1], p_acc[2]);
		 pitch_angle[1] = 0.9*(pitch_angle[0]-0.01*p_gyro[1])+(0.1)*atan2(p_acc[0], p_acc[2]);

		 roll_angle[0] = roll_angle[1];
		 pitch_angle[0] = pitch_angle[1];

		 memcpy(&message_calcul.roll_angle, &roll_angle[0], sizeof(roll_angle[0]));
		 memcpy(&message_calcul.pitch_angle, &pitch_angle[0], sizeof(pitch_angle[0]));

		 xQueueOverwrite(xQueue2aTo3, &message_calcul);
	}
}

/**
  * @brief Task 3 : communication des données par liaison série RS-232.
  * @param pvParameters Vide
  * @retval void
  */
void vTask3(void *pvParameters)
{
	double roll_angle;
	double pitch_angle;
	Message_calcul message_calcul;

	while (1) {
		if (xSemaphoreTake(xSemaphore3, portMAX_DELAY) == pdTRUE) {
			xQueueReceive(xQueue2aTo3, &message_calcul, portMAX_DELAY);
			roll_angle = (message_calcul.roll_angle*180)/M_PI;
			pitch_angle = (message_calcul.pitch_angle*180)/M_PI;
			xSemaphoreTake(xMutexRS232, portMAX_DELAY);
			printf("Roll Angle : %2.2f \r\n Pitch Angle : %2.2f \r\n", roll_angle, pitch_angle);
			xSemaphoreGive(xMutexRS232);
		}
	}
}

/**
  * @brief Task 4 : affichage des informations sur l'écran LCD.
  * @param pvParameters Vide
  * @retval void
  */
void vTask4(void *pvParameters)
{
	double roll_angle;
	double pitch_angle;
	Message_calcul message_calcul;

	while (1) {
		if (xSemaphoreTake(xSemaphore4, portMAX_DELAY) == pdTRUE) {
			xQueueReceive(xQueue2aTo3, &message_calcul, portMAX_DELAY);
			roll_angle = message_calcul.roll_angle;
			pitch_angle = message_calcul.pitch_angle;
			xSemaphoreTake(xMutexLCD, portMAX_DELAY);
			GUI	(roll_angle, pitch_angle, 0.0, 0.0, 0.0);
			xSemaphoreGive(xMutexLCD);
		}
	}
}

/*Memory error detection*/

/**
* @fn void vApplicationStackOverflowHook ...
* @brief kernel hook here if stack overflow detection
*/
void vApplicationStackOverflowHook( xTaskHandle xTask, signed char *pcTaskName )
{
	printf("\r\nerror stack overflow by ");
	printf((char*) pcTaskName);
	printf(" \r\n");
	while(1) ; // it's a trap
}

/**
* @fn void growStack( void )
* @brief current stack allocation until overflow
*/
int growStack(void)
{
	vTaskDelay( 1 );
	return growStack();
}

/**
* @fn vApplicationMallocFailedHook()
* @brief kernel hook here if malloc allocation failed detection
*/
void vApplicationMallocFailedHook( void )
{
	printf("\r\nerror malloc failed \r\n");

	while(1);	// it's a trap
}







