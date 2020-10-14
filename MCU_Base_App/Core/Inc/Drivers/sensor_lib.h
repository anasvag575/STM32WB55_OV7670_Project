/**
  ******************************************************************************
  * @file       : sensor_lib.h
  * @brief      : Main header file
  * @author	: Anastasis Vagenas
  ******************************************************************************
  * Sensor_lib header file.
  * Includes the libraries needed for the sensors configuration
  *
  * This header file declares the handlers of the peripherals and flags for utilities
  ******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "Imported_Libs/dwt_delay.h"
#include "stm32wbxx_hal.h"
#include "stm32wbxx_ll_gpio.h"
#include "Essentials/stm32wbxx_nucleo.h"
#include "string.h"
#include "Drivers/uart.h"

/* Defines -------------------------------------------------------------------*/
//Uncomment if specified sensor is not used //
//#define KY_002_SENSOR
//#define KY_018_SENSOR
//#define KY_020_SENSOR
//#define KY_028_SENSOR

/* Defines Used for calculation ----------------------------------------------*/
// General Values //
#define ADC_BIT_RES 	4096
#define VREF 		3.3
#define PIN_LOW 	0x00u

// KY-018 Values //
#define KY_018_SCALE_VALUE 256

// Set light bounds //
#define NO_LIGHT_BOUND 		14
#define LOW_LIGHT_BOUND 	11
#define HIGH_LIGHT_BOUND 	5
#define MAX_LIGHT_BOUND 	2

// Light levels //
#define NO_LIGHT 		0
#define DARK_LIGHT 		1
#define MEDIUM_LIGHT 		2
#define HIGH_LIGHT 		3
#define MAX_LIGHT 		4

// KY-002 Values //
#define KY_002_SAMPLE_MAX 	50
#define KY_002_THRESHOLD_MIN 	35

// KY-020 Values //
#define KY_020_SAMPLE_MAX 	50
#define KY_020_THRESHOLD_MIN 	30

/* GPIOs ---------------------------------------------------------------------*/
/* GPIOs can be configured from port A-D and all Pin numbers as well ADC Channels
 * As long as the combination is valid it can be done:
 *
 * --- GPIO Port ---
 * 1.To change port just change the KY_0xx_PORT_y_CLK (y to A-D)
 * 2.And the KY_0xx_PORT change to GPIOy (y to A-D)
 *
 * --- GPIO Pin number ---
 * 1.To change pin number just change the KY_0y_PIN to GPIO_PIN_y
 *  (y to 0-15)
 * 2.Change in the same spirit KY_0xx_LL_PIN to LL_GPIO_PIN_y
 *
 * --- ADC Channel ---
 * 1.To change the corresponding sensor ADC Channel just change
 * KY_0xx_ADC to ADC_CHANNEL_y (y to 0-15)
 * 2.Change KY_0xx_REG_CONF to (y << 6) where y is the channel number
 */

#define KY_002_PORT_B_CLK 1		// Choose GPIO port clock //
#define KY_002_PORT GPIOB		// Choose GPIO port //
#define KY_002_PIN GPIO_PIN_2		// Choose GPIO pin //
#define KY_002_LL_PIN LL_GPIO_PIN_2	// Choose LL GPIO definition pin //

#define KY_018_PORT_C_CLK 1		// Choose GPIO port clock //
#define KY_018_PORT GPIOC		// Choose GPIO port //
#define KY_018_PIN GPIO_PIN_0		// Choose GPIO pin //
#define KY_018_LL_PIN LL_GPIO_PIN_0	// Choose LL GPIO definition pin //
#define KY_018_ADC LL_ADC_CHANNEL_1	// Choose ADC channel //

#define KY_020_PORT_E_CLK 1		// Choose GPIO port clock //
#define KY_020_PORT GPIOE		// Choose GPIO port //
#define KY_020_PIN GPIO_PIN_4		// Choose GPIO pin //
#define KY_020_LL_PIN LL_GPIO_PIN_4	// Choose LL GPIO definition pin //

#define KY_028_PORT_C_CLK_ 1			// Choose GPIO port clock //
#define KY_028_PORT GPIOC			// Choose GPIO port //
#define KY_028_PIN_ANALOG GPIO_PIN_2		// Choose GPIO Analog pin //
#define KY_028_LL_PIN_ANALOG LL_GPIO_PIN_2	// Choose LL GPIO definition pin //
#define KY_028_PIN_DIGITAL GPIO_PIN_5		// Choose GPIO Digital pin //
#define KY_028_LL_PIN_DIGITAL LL_GPIO_PIN_5	// Choose LL GPIO definition pin //
#define KY_028_ADC LL_ADC_CHANNEL_3		// Choose ADC channel //

/* Structs -------------------------------------------------------------------*/
// The struct that containts the data of the sensors. //
// Can be expanded for more sensors //
typedef struct s_data {
   uint8_t tilt;		// KY-020 Output //
   uint8_t light;		// KY-018 Output //
   uint8_t shock;		// KY-002 Output //
   float temp_dif;		// KY-028 Analog //
   uint8_t temp_thresh;		// KY-028 Digital //
} s_data;

/* Global Variables ----------------------------------------------------------*/
uint32_t ADC_channel_user;		// Value used to know the owner of the ADC //

/* External Variables --------------------------------------------------------*/
extern ADC_TypeDef *adc_handle;		// ADC handler //

/* Functions -----------------------------------------------------------------*/
void ky002_init(void);
void ky018_init(void);
void ky020_init(void);
void ky028_init(void);
uint8_t ky002_read(void);
uint8_t ky018_read(void);
uint8_t ky020_read(void);
uint8_t ky028_read(float *Vout);
void get_sensor_data(s_data *data);
void print_sensor_data(s_data *data);
