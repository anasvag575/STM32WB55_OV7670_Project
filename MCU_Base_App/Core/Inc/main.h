/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Main header file
  * @author			: Anastasis Vagenas
  ******************************************************************************
  * Main's header file.
  * Includes the libraries needed for the project and the header files for the
  * drivers we created
  *
  * This header file declares the handlers of the peripherals, the frame_buffer
  * needed for storing a frame and flags for utilities
  *
  ******************************************************************************
*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32wbxx_hal.h"
#include "stm32wbxx_ll_gpio.h"
#include "Essentials/stm32wbxx_nucleo.h"
#include <stdio.h>
#include "Drivers/OV7670.h"
#include "Drivers/uart.h"
#include "Drivers/sensor_lib.h"

/* Peripherals Handlers ------------------------------------------------------*/
UART_HandleTypeDef huart1;	// UART handler //
ADC_TypeDef *adc_handle;	// ADC handler //
I2C_HandleTypeDef hi2c1;  	// I2C Handler //

/* Global Variables ----------------------------------------------------------*/
extern uint8_t frame_buffer[2 * IMG_ROW * IMG_COL];	// Holds frame from camera //
uint8_t debug_flag;					// Used for debugging //
uint8_t demo_flag;					// Used for the demo //

/* Private function prototypes -----------------------------------------------*/
//--> System Functions
void SystemClock_Config(void);
void MX_GPIO_Init(void);
void MX_I2C1_Init(void);
void MX_USART1_UART_Init(void);
void MX_ADC_Init(void);
void MX_I2C1_Init(void);

//-->User utilities
void get_button_press(Button_TypeDef button);
void initialize_peripherals(uint8_t flag);
void sensor_test(void);
void read_gpio_port(void);
void demo_test(void);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

#ifdef __cplusplus
}
#endif

#endif
