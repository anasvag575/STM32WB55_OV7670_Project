/**
  ******************************************************************************
  * @file           : OV7670.h
  * @brief          : Header file for Camera Utilities
  * @author	    : Anastasis Vagenas
  ******************************************************************************
  *	Camera driver's header file.
  *	Includes needed libraries and the uart driver in order for the confirmation
  *	of the communication with the camera
  *
  * Defines :
  * The addresses of the camera for R/W
  * Important registers -> Contrast, saturation (mostly color transformations)
  *	Size parameters for QVGA resolution
  *
  ******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "stm32wbxx_hal.h"
#include "stm32wbxx_ll_gpio.h"
#include <stdio.h>
#include <string.h>
#include "uart.h"

/* Defines -------------------------------------------------------------------*/

// OV7670 SCCB related //
#define OV7670_REG_NUM 		114
#define OV7670_WRITE_ADDR 	0x42
#define OV7670_READ_ADDR 	0x43

// Image size -> QVGA //
#define IMG_ROW 	240
#define IMG_COL   	320

//* OV7670 Most used registers values
// Clock - Speed Related //
#define CLK_DIV_VAL		0x0a		// Clock divider Reg Value -> Minimum Stable value //
#define CLK_PLL_VAL		0x4a		// Clock PLL - Clock Multiplier reg value //

// Image properties //
#define BRIGHTNESS 		0x00		// Brightness Value //
#define CONTRAST 		0x40		// Contrast Value //
#define SHARPNESS		0x00		// Edge enhancement Value //
#define AWB_RED_GAIN		0x60		// AWB Red channel gain value //
#define AWB_GREEN_GAIN		0x40		// AWB Green channel gain value //
#define AWB_BLUE_GAIN		0x40		// AWB Blue channel gain value //

// OV7670 Most used registers addresses
#define CLK_DIV_REG		0x11		// Clock divider (XCLK) reg address//
#define CLK_PLL_REG		0x6b		// Clock PLL - Clock Multiplier reg address //
#define BRIGHTNESS_REG 		0x55		// Brightness reg address //
#define CONTRAST_REG 		0x56		// Contrast reg address //
#define SHARPNESS_REG 		0x3f		// Edge enhancement reg address //
#define AWB_RED_GAIN_REG	0x02		// AWB Red channel gain reg address //
#define AWB_GREEN_GAIN_REG	0x6a		// AWB Green channel gain reg address //
#define AWB_BLUE_GAIN_REG	0x01		// AWB Blue channel gain reg address //

// 0V7670 GPIO PINS //
// LL_PIN Definitions are used only for the initialization phase //
// The normal pin and port definitions are used for the accessing of the pin register //
// VSYNC - Vertical Sync //
#define VSYNC_PIN		GPIO_PIN_4
#define VSYNC_PORT		GPIOB
#define VSYNC_LL_PIN		LL_GPIO_PIN_4

// HSYNC - Horizontal Sync //
#define HSYNC_PIN		GPIO_PIN_3
#define HSYNC_PORT		GPIOC
#define HSYNC_LL_PIN		LL_GPIO_PIN_3

// PXCLK - Pixel clock //
#define PXCLK_PIN		GPIO_PIN_1
#define PXCLK_PORT		GPIOC
#define PXCLK_LL_PIN		LL_GPIO_PIN_1

// XCLK - Input camera clock //
#define XCLK_PIN		GPIO_PIN_8
#define XCLK_PORT		GPIOA
#define XCLK_LL_PIN		LL_GPIO_PIN_8

// SIO_C - SCCB clock pin (I2C SCL) //
#define SIO_C_PIN		GPIO_PIN_8
#define SIO_C_PORT		GPIOB
#define SIO_C_LL_PIN		LL_GPIO_PIN_8

// SIO_D - SCCB Data pin (I2C SDA) //
#define SIO_D_PIN		GPIO_PIN_9
#define SIO_D_PORT		GPIOB
#define SIO_D_LL_PIN		LL_GPIO_PIN_9

// PWDN - Power down pin //
#define PWDN_PIN		GPIO_PIN_10
#define PWDN_PORT		GPIOC
#define PWDN_LL_PIN		LL_GPIO_PIN_10

// RET - Reset pin //
#define RET_PIN			GPIO_PIN_12
#define RET_PORT		GPIOC
#define RET_LL_PIN		LL_GPIO_PIN_12

// Data0-Data7 - Video parallel output //
#define DATA_PINS		(GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7)
#define DATA_PORT		GPIOA
#define DATA0_LL_PIN		LL_GPIO_PIN_0
#define DATA1_LL_PIN		LL_GPIO_PIN_1
#define DATA2_LL_PIN		LL_GPIO_PIN_2
#define DATA3_LL_PIN		LL_GPIO_PIN_3
#define DATA4_LL_PIN		LL_GPIO_PIN_4
#define DATA5_LL_PIN		LL_GPIO_PIN_5
#define DATA6_LL_PIN		LL_GPIO_PIN_6
#define DATA7_LL_PIN		LL_GPIO_PIN_7

// Modes //
// *Develop mode is basically a NO-UART mode other than the transfer of the image.    //
// Uncommenting DEVELOP_MODE basically allows some basic messages to the terminal.    //
// *Debug mode prints every state of the camera initialization without stopping at    //
// an error. Useful when comms are not working properly 			      //
// Activate DEVELOP and if that's not enough then activate DEBUG (ONLY IN THAT ORDER) //

//#define DEVELOP_MODE	// Basic Messages - Uncomment for activation //
//#define DEBUG_MODE	// Debug - Extra messages - Uncomment for activation //

// Parameters and Constants //
#define MAX_I2C_RETRIES 	30  // Max number of retrials in I2C communication //
#define MAX_I2C_TIMEOUT 	20  // Max timeout time in milliseconds for I2C //
#define PIN_LOW			0x00u

/* Global Variables ---------------------------------------------------------*/
uint8_t frame_buffer[2 * IMG_COL * IMG_ROW];  // Frame holder //
extern I2C_HandleTypeDef hi2c1;  	      // I2C Handler //

/* Functions Prototypes ---------------------------------------------------------*/
uint8_t write_camera_reg(uint8_t reg_address, uint8_t data);
uint8_t read_camera_reg(uint8_t reg_address, uint8_t *data);
uint8_t OV7670_init(void);
void get_frame(void);
void transmit_UART_frame(uint8_t format);
void OV7670_init_peripherals(void);
void OV7670_reset(void);
void OV7670_set_power_mode(uint8_t mode);
uint8_t OV7670_testbench(void);
