/**
  ******************************************************************************
  * @file           : sensor_lib.c
  * @brief          : Source code for sensor drivers
  * @author	    : Anastasis Vagenas
  ******************************************************************************
  * The sensor library source code.
  * Supported sensors and a small description for them:
  *
  * ---- KY002 ----
  * Digital shock sensor.
  * Has 3 pins : VDD, GND, Serial_output
  * We supply the sensor with 5V and its serial output will be a digital value, meaning
  * either HIGH or LOW (OV).
  * Sensor can operate also on 3.3V but due too a large voltage drop, HIGH signal is 1.8V,
  * meaning really close to the detection threshold for a digital HIGH signal.
  *
  * That's why a 5V supply used and 5V tolerant GPIO is used, which reaches about 3.7-3.8V
  * for a digital HIGH (which can be detected by software as digital HIGH).
  *
  * ---- KY015 ----
  * Digital Humidity/Temperature sensor.
  * Has 3 pins : VDD, GND, Serial_output
  * We need a 3.3V supply for it and its serial output is HIGH (3.3V) or LOW (0V).
  * We initialize only the GPIO to appropriate mode and then in order to read from it
  * we need to perform a series of Digital Reads/Writes on the serial pin based on
  * the communication protocol set by the manufacturer.
  *
  * ---- KY018 ----
  * Analog light sensor.
  * Has 3 pins : VDD, GND, Serial_output
  * We supply the sensor with 3.3V and its serial output will be an analog value, meaning
  * between [0,3.3] V .
  * We initialize the appropriate GPIO and the ADC channel configuration needed to
  * perform the reading of the analog value (and convert to a digital one).
  *
  * ---- KY020 ----
  * Digital tilt sensor.
  * Has 3 pins : VDD, GND, Serial_output
  * We supply the sensor with 5V and its serial output will be a digital value, meaning
  * either HIGH or LOW (OV).
  * Sensor can operate also on 3.3V but due too a large voltage drop, HIGH signal is 1.8V,
  * meaning really close to the detection threshold for a digital HIGH signal.
  *
  * That's why a 5V supply used and 5V tolerant GPIO is used, which reaches about 3.7-3.8V
  * for a digital HIGH (which can be detected by software as digital HIGH)
  *
  *  ---- KY028 ----
  * Analog/Digital temperature sensor.
  * Has 4 pins : VDD, GND, Serial_output, Threshold
  * We supply the sensor with 3.3V and its serial output will be an analog value, meaning
  * between [0,3.3] V .
  * The threshold value is HIGH (3.3V) or LOW (OV) whether the sensor recorded the
  * threshold temperature we specified.
  *
  * Then the analog value will show the relative difference from the threshold value.
  * We initialize the appropriate GPIOs and the ADC channel configuration needed to
  * perform the reading of the analog value (and convert to a digital one).
  *
  ******************************************************************************
*/
#include "Drivers/sensor_lib.h"

/* Initialization function for KY-002 Vibration sensor.
 * Initializes the GPIO pin used.
 *
 * KY-002 => PB2 - Data (Also 5V tolerant)
*/
void ky002_init(void)
{

  // Enable GPIO Port clock //
  #ifdef KY_002_PORT_A_CLK
    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
  #elif KY_002_PORT_B_CLK
    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);
  #elif KY_002_PORT_C_CLK
    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOC);
  #elif KY_002_PORT_D_CLK
    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOD);
  #elif KY_002_PORT_E_CLK
    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOE);
  #endif

  // PB2   ------> Data serial //
  // Set Pin mode and Resistor Pull //
  LL_GPIO_SetPinMode(KY_002_PORT, KY_002_LL_PIN, LL_GPIO_MODE_INPUT);
  LL_GPIO_SetPinPull(KY_002_PORT, KY_002_LL_PIN, LL_GPIO_PULL_NO);
}

/* Initialization function for KY-018 Light sensor.
 * Initializes the GPIO pin used and the ADC channel that
 * will be used for the sensor's analog output
 *
 * KY-018 => PC0 - Data - ADC_IN1
*/
void ky018_init(void)
{

  // Enable GPIO Port clock //
  #ifdef KY_018_PORT_A_CLK
    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
  #elif KY_018_PORT_B_CLK
    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);
  #elif KY_018_PORT_C_CLK
    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOC);
  #elif KY_018_PORT_D_CLK
    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOD);
  #elif KY_018_PORT_E_CLK
    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOE);
  #endif

  // PC0   ------> Data serial //
  // Set Pin mode and Resistor Pull //
  LL_GPIO_SetPinMode(KY_018_PORT, KY_018_LL_PIN, LL_GPIO_MODE_ANALOG);
  LL_GPIO_SetPinPull(KY_018_PORT, KY_018_LL_PIN, LL_GPIO_PULL_NO);

  // Channel configuration //
  LL_ADC_REG_SetSequencerRanks(adc_handle, LL_ADC_REG_RANK_1, KY_018_ADC);
  LL_ADC_SetChannelSamplingTime(adc_handle, KY_018_ADC, LL_ADC_SAMPLINGTIME_92CYCLES_5);
  LL_ADC_SetChannelSingleDiff(adc_handle, KY_018_ADC, LL_ADC_SINGLE_ENDED);

  // Change the user //
  ADC_channel_user = KY_018_ADC;
}

/* Initialization function for KY-020 Tilt sensor.
 * Initializes the GPIO pin used.
 *
 * KY-020 => PE4 - Data (Also 5V tolerant)
*/
void ky020_init(void)
{

  // Enable GPIO Port clock //
  #ifdef KY_020_PORT_A_CLK
    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
  #elif KY_020_PORT_B_CLK
    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);
  #elif KY_020_PORT_C_CLK
    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOC);
  #elif KY_020_PORT_D_CLK
    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOD);
  #elif KY_020_PORT_E_CLK
    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOE);
  #endif

  // PC5   ------> Data serial //
  // Set Pin mode and Resistor Pull //
  LL_GPIO_SetPinMode(KY_020_PORT, KY_020_LL_PIN, LL_GPIO_MODE_INPUT);
  LL_GPIO_SetPinPull(KY_020_PORT, KY_020_LL_PIN, LL_GPIO_PULL_NO);
}

/* Initialization function for KY-028 Temperature sensor.
 * Initializes the GPIO pins used and the ADC channel that
 * will be used for the sensor's analog output
 *
 * KY-028 => PC2 - Data - ADC_IN3
 * 	     PC5 - Threshold
*/
void ky028_init(void)
{

  // Enable GPIO Port clock //
  #ifdef KY_028_PORT_A_CLK
    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
  #elif KY_028_PORT_B_CLK
    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);
  #elif KY_028_PORT_C_CLK
    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOC);
  #elif KY_028_PORT_D_CLK
    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOD);
  #elif KY_028_PORT_E_CLK
    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOE);
  #endif

  // PC2   ------> Analog Data //
  // Set Pin mode and Resistor Pull //
  LL_GPIO_SetPinMode(KY_028_PORT, KY_028_LL_PIN_ANALOG, LL_GPIO_MODE_ANALOG);
  LL_GPIO_SetPinPull(KY_028_PORT, KY_028_LL_PIN_ANALOG, LL_GPIO_PULL_NO);

  // PC5   ------> Threshold signal //
  // Set Pin mode and Resistor Pull //
  LL_GPIO_SetPinMode(KY_028_PORT, KY_028_LL_PIN_DIGITAL, LL_GPIO_MODE_INPUT);
  LL_GPIO_SetPinPull(KY_028_PORT, KY_028_LL_PIN_DIGITAL, LL_GPIO_PULL_NO);

  // Channel configuration //
  LL_ADC_REG_SetSequencerRanks(adc_handle, LL_ADC_REG_RANK_1, KY_028_ADC);
  LL_ADC_SetChannelSamplingTime(adc_handle, KY_028_ADC, LL_ADC_SAMPLINGTIME_92CYCLES_5);
  LL_ADC_SetChannelSingleDiff(adc_handle, KY_028_ADC, LL_ADC_SINGLE_ENDED);

  // Change the user //
  ADC_channel_user = KY_028_ADC;
}

/* Driver function for KY-002 Vibration sensor.
 * Sensor sets serial pin to HIGH when vibration is detected.
 * We sample the pin multiple times in order to determine if
 * vibration is really present based on threshold value and return:
 * Vibration presence -> 1
 * No vibration -> 0
*/
uint8_t ky002_read(void)
{
  uint8_t i;
  uint8_t threshold;

  // Sample the GPIO for 50 times //
  for (i = 0, threshold = 0; i < KY_002_SAMPLE_MAX; i++)
  {
    // Direct register GPIO access //
    if ((KY_002_PORT->IDR & KY_002_PIN) != PIN_LOW)
      threshold++;
  }

  // Return SUCCESS (1) if we detect sufficient vibration //
  if (threshold > KY_002_THRESHOLD_MIN)
    return 1;
  else
    return 0;
}

/* Driver function for KY-018 Light sensor.
 * Considering that the sensor has an analog output
 * an ADC channel has to be set in order for this function to work
 * The initialization is performed in the corresponding init function
 *
 * Function calculates the voltage based on the 12-bit value from the ADC
 * and returns an integer that defines the light level
 *
*/
uint8_t ky018_read(void)
{
  uint32_t adc_value = 0;
  uint8_t light_intensity;

  // Check the ADC channel owner //
  // Set the SQR1 register appropriately //
  if (ADC_channel_user != KY_018_ADC)
  {
    LL_ADC_REG_SetSequencerRanks(adc_handle, LL_ADC_REG_RANK_1, KY_018_ADC);
    ADC_channel_user = KY_018_ADC;
  }

  // Enable the ADC peripheral //
  LL_ADC_Enable(adc_handle);

  // Wait for the ADC to get ready //
  while ((LL_ADC_IsActiveFlag_ADRDY(adc_handle)) == PIN_LOW);

  // Clear Flags just to be safe //
  LL_ADC_ClearFlag_EOC(adc_handle);
  LL_ADC_ClearFlag_EOS(adc_handle);
  LL_ADC_ClearFlag_OVR(adc_handle);

  // Start the conversion //
  LL_ADC_REG_StartConversion(adc_handle);

  // Wait for ADC to finish conversion and get value //
  // Wait for the flag that signifies the End Of Conversion (EOC) //
  // When EOC part of ISR register is High -> Get value //
  while ((LL_ADC_IsActiveFlag_EOC(adc_handle)) == PIN_LOW);

  // Direct Register Access to get ADC value //
  adc_value = adc_handle->DR;

  // Calculate the percentage of light intensity //
  light_intensity = adc_value / KY_018_SCALE_VALUE;

  // Map the light intensity to a range of predefined values //
  if (light_intensity < MAX_LIGHT_BOUND )
    light_intensity = MAX_LIGHT;
  else if ((light_intensity < HIGH_LIGHT_BOUND) && (light_intensity >= MAX_LIGHT_BOUND))
    light_intensity = HIGH_LIGHT;
  else if ((light_intensity >= HIGH_LIGHT_BOUND) && (light_intensity < LOW_LIGHT_BOUND))
    light_intensity = MEDIUM_LIGHT;
  else if ((light_intensity < NO_LIGHT_BOUND) && (light_intensity >= LOW_LIGHT_BOUND))
    light_intensity = DARK_LIGHT;
  else
    light_intensity = NO_LIGHT;

  // Sanity check - Will return 0 //
  if (LL_ADC_REG_IsConversionOngoing(adc_handle))
    LL_ADC_REG_StopConversion(adc_handle);

  // Disable the ADC //
  LL_ADC_Disable(adc_handle);

  // Clear the flags for the next activation //
  LL_ADC_ClearFlag_EOSMP(adc_handle);
  LL_ADC_ClearFlag_ADRDY(adc_handle);

  // Wait for ADC Enable signal to go low //
  while (LL_ADC_IsDisableOngoing(adc_handle) != PIN_LOW);

  return light_intensity;
}

/* Driver function for KY-020 Tilt sensor.
 * Sensor sets serial pin to HIGH when tilt is detected.
 *
 * We sample the pin multiple times in order to determine if
 * tilt is really present based on threshold value and return:
 * Tilt presence -> 1
 * No tilt -> 0
*/
uint8_t ky020_read(void)
{
  uint8_t i;
  uint8_t threshold;

  // Sample the GPIO for 50 times //
  for (i = 0, threshold = 0; i < KY_020_SAMPLE_MAX; i++)
  {
    if ((KY_020_PORT->IDR & KY_020_PIN) != PIN_LOW)
      threshold++;
  }

  // Return SUCCESS (1) if we detect sufficient tilt //
  if (threshold > KY_020_THRESHOLD_MIN)
    return 1;
  else
    return 0;
}

/* Driver function for KY-028 Temperature sensor.
 * Considering that the sensor has an analog output
 * an ADC channel has to be set in order for this function to work
 * The initialization is performed in the corresponding init function
 *
 * Sensor just detects if the threshold temperature is reached.
 * Argument float stores the result from the analog read and function's
 * return value whether we reached the threshold temperature or not.
*/
uint8_t ky028_read(float *Vout)
{
  uint32_t adc_value = 0;

  // Check the ADC channel owner //
  // Set the SQR1 register appropriately //
  if (ADC_channel_user != KY_028_ADC)
  {
    LL_ADC_REG_SetSequencerRanks(adc_handle, LL_ADC_REG_RANK_1, KY_028_ADC);
    ADC_channel_user = KY_028_ADC;
  }

  // Enable the ADC peripheral //
  LL_ADC_Enable(adc_handle);

  // Wait for the ADC to get ready //
  while ((LL_ADC_IsActiveFlag_ADRDY(adc_handle)) == PIN_LOW);

  // Clear Flags just to be safe //
  LL_ADC_ClearFlag_EOC(adc_handle);
  LL_ADC_ClearFlag_EOS(adc_handle);
  LL_ADC_ClearFlag_OVR(adc_handle);

  // Start the conversion //
  LL_ADC_REG_StartConversion(adc_handle);

  // Wait for ADC to finish conversion and get value //
  // Wait for the flag that signifies the End Of Conversion (EOC) //
  // When EOC part of ISR register is High -> Get value //
  while ((LL_ADC_IsActiveFlag_EOS(adc_handle)) == PIN_LOW);

  // Direct Register Access to get ADC value //
  adc_value = adc_handle->DR;

  // Calculation of Temperature change //
  *Vout = (((float)adc_value) / ((float) ADC_BIT_RES)) * VREF ;

  // Sanity check - Will return 0 //
  if (LL_ADC_REG_IsConversionOngoing(adc_handle))
    LL_ADC_REG_StopConversion(adc_handle);

  // Disable the ADC //
  LL_ADC_Disable(adc_handle);

  // Clear the Flags for the next activation //
  LL_ADC_ClearFlag_EOSMP(adc_handle);
  LL_ADC_ClearFlag_ADRDY(adc_handle);

  // Wait for ADC Enable signal (ADEN) to go low //
  while (LL_ADC_IsDisableOngoing(adc_handle) != PIN_LOW);

  // Return whether we reached the threshold or not //
  if ((KY_028_PORT->IDR & KY_028_PIN_DIGITAL) != PIN_LOW)
    return 1;
  else
    return 0;

}

/* Helper function that writes the data to the specified struct.
 * Function that is the library's reading function to access the sensors.
 */
void get_sensor_data(s_data *data)
{
  // Read from Shock sensor //
  #ifdef KY_002_SENSOR
    data->shock = ky002_read();
  #endif

  // Read from Shock sensor //
  #ifdef KY_018_SENSOR
    data->light = ky018_read();
  #endif

  // Read from tilt sensor //
  #ifdef KY_020_SENSOR
    data->tilt = ky020_read();
  #endif

  // Read from threshold temperature sensor //
  #ifdef KY_028_SENSOR
    data->temp_thresh = ky028_read(&(data->temp_dif));
  #endif

}

/* Helper function that prints the data of the specified struct to UART.
 * Function that is the library's main printing.
 * Used for debugging purposes and verification.
 */
void print_sensor_data(s_data *data)
{
  uint8_t message_buf[] = "KY-0xx -> Output = 0\n";

  // Print data from Shock sensor //
  #ifdef KY_002_SENSOR
    message_buf [4] = '0';
    message_buf [5] = '2';
    message_buf [19] = '0' + data->shock;
    transmit_UART_message(message_buf);
  #endif

  // Print data from Shock sensor //
  #ifdef KY_018_SENSOR
    message_buf [4] = '1';
    message_buf [5] = '8';
    message_buf [19] = '0' + data->light;
    transmit_UART_message(message_buf);
  #endif

  // Print data from tilt sensor //
  #ifdef KY_020_SENSOR
    message_buf [4] = '2';
    message_buf [5] = '0';
    message_buf [19] = '0' + data->tilt;
    transmit_UART_message(message_buf);
  #endif

  // Print data from threshold temperature sensor 		  //
  // Printf is used due to formatting, can be changed if we learn //
  // the scale of the temperature scale to map the values 	  //
  #ifdef KY_028_SENSOR
    printf("KY-028 -> Output = %f | Threshold = %d\n", data->temp_dif, data->temp_thresh);
  #endif

}
