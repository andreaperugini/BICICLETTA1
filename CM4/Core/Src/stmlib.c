/*
 * stmlib.c
 *
 *  Created on: Oct 1, 2024
 *      Author: andra
 */

#include "stmlib.h"
#include "main.h"
#include <stdio.h>


// Dichiarazioni delle variabili per le UART e il timer
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern TIM_HandleTypeDef htim6;
// Buffer per la ricezione dei dati dalla UART2
char rx_buffer[1];
uint8_t trasmissione_attiva = 0;
Dati dati;

// Funzione per trasmettere dati tramite UART2
void Trasmissione_dati(void* data, size_t size) {
HAL_UART_Transmit(&huart2, (uint8_t*)data, size, HAL_MAX_DELAY);
}

// Callback chiamata quando un byte viene ricevuto sulla UART2
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
if (huart == &huart2) {
if (rx_buffer[0] == ’S’) {
// Interrompi la trasmissione
printf("Arrivato: %c\r\n",rx_buffer[0]);
trasmissione_attiva = 0;
}
else if (rx_buffer[0] == ’V’) {
// Avvia la trasmissione
printf("Arrivato: %c\r\n",rx_buffer[0]);
trasmissione_attiva = 1;
}

HAL_UART_Receive_IT(&huart2, (uint8_t *)rx_buffer, 1);
//Riattiva la ricezione
}

}


// Callback chiamata quando il timer TIM6 scade
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
if (htim == &htim6) {
// Gestione dell’invio periodico dei dati
if (trasmissione_attiva==1) {
char buffer[50];

int bytesWritten = sprintf(buffer, "%d,%lf,%lf\n", dati.
 velocita, dati.accelerazione, dati.tempo);
printf("Dati trasmessi: %s\r\n", buffer);
Trasmissione_dati(buffer, bytesWritten);
}
}
}

// Funzione di scrittura per printf
int _write(int file, char *ptr, int len) {
HAL_UART_Transmit(&huart3, (uint8_t*)ptr, len, HAL_MAX_DELAY);
return len;
}

