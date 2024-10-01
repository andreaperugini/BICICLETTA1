/*
 * stmlib.h
 *
 *  Created on: Oct 1, 2024
 *      Author: andra
 */

#ifndef INC_STMLIB_H_
#define INC_STMLIB_H_

// Inclusione del file di intestazione per le periferiche STM32H7 HAL
#include "stm32h7xx_hal.h"

// Struttura per i dati da trasmettere
typedef struct {
 int velocita;
 double accelerazione;
 double tempo;} Dati;

 //dichiarazione esterna delle variabili dati e rx_buffer
extern Dati dati;
extern char rx_buffer[1];

// Dichiarazioni delle funzioni
int write(int file, char *ptr, int len);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void Trasmissione_dati(void* data, size_t size);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);


#endif /* INC_STMLIB_H_ */
