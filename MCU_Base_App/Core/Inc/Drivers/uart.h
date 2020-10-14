/**
  ******************************************************************************
  * @file           : uart.h
  * @brief          : UART library file
  * @author	    : Anastasis Vagenas
  ******************************************************************************
  * UART helper header file
  * Containts the functions prototypes
  *
  ******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "stm32wbxx_hal.h"
#include <stdio.h>
#include <string.h>

/* External Variables --------------------------------------------------------*/
extern UART_HandleTypeDef huart1;

/* Functions  ----------------------------------------------------------------*/
void transmit_UART_message(uint8_t *buffer);
