/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  * @author	    : Anastasis Vagenas
  ******************************************************************************
  * Our project's main.
  * The project is a wireless sensor network, having a small processing unit
  * (MCU) controlling the sensors (peripherals) and then sending the data back
  * to a cloud system or a big processing system (usually FPGA or other embedded system).
  *
  * ------ The sensors/Peripherals------
  * Camera module -> OV7670
  * Sensors -> KY002, KY015, KY020, KY028
  * UART channel -> USB/COM port
  * Radio -> MCU onboard transceiver
  *
  * ------ MCU --------
  * MCU <-> Board ====> STM32WB55RG <-> NUCLEO 68
  *
  * --------------------------- Main functionality -----------------------------
  * Main initializes every peripheral and the MCU itself. After that the drivers
  * of the peripherals are used in order to get the data we want:
  *
  * ------- |Camera -> OV7670.c | OV7670.h |---------
  * Reset the camera and initialize it (by loading our settings) and then use the
  * functionalities to get frames when the MCU is instructed to.
  *
  * ------- |Sensors -> sensor_lib.c | sensor_lib.h |---------
  * Initialize the sensor used or request data from it.
  *
  *
  ****************************************************************************************
  ****************************************************************************************
  ****************************************************************************************
  * ------------- GPIOs Main Table ---------------------------------------------------
  * MCU GPIO  -> Pxy [ x = port letter, y = pin number ]
  * Function  -> Alternate function name and pin name
  * Board Pin -> Txy [ T = (A)rduino or (M)orpho header , x = (L)eft or (R)ight, y = pin number ]
  *
  * ------------------------- Always used --------------------------
  * ----------------------------------------------------------------
  * 		    MCU GPIO ||  Function   || Board Pin
  *-----------------------------------------------------------------
  * 			PB6  ||  USART1_TX  || 	MR34
  * 			PB7  ||  USART1_RX  ||	MR6
  * 			PB8  ||   I2C1_SCL  || 	MR3
  *			PB9  ||   I2C1_SDA  || 	MR5
  *			PA8  ||    XCLK	    || 	MR25
  *			PC10 ||	   PWDN     || 	MR29
  *			PC12 ||    RESET    ||	MR21
  *			PB4  ||	   VSYNC    ||  MR4
  *			PC3  ||	   HSYNC    ||  ML36
  *			PC1  ||	   PXCLK    ||  ML30
  *			PA0  ||	   Data0    ||  ML34
  *			PA1  ||	   Data1    ||  ML32
  *			PA2  ||	   Data2    ||  MR35
  *			PA3  ||	   Data3    ||  MR37
  *			PA4  ||    Data4    ||  MR17
  *  			PA5  ||	   Data5    ||  MR11
  *    			PA6  ||	   Data6    ||  MR13
  *    			PA7  ||	   Data7    ||  MR15
  * -----------------------------------------------------------------
  * ------------------------- Conditional use -----------------------
  * ---------------------- Depends on configuration -----------------
  * -----------------------------------------------------------------
  * 		PC0  ||  Serial-KY018/ADC Channel 1  || ML28
  * 		PB2  ||    Serial - KY002 (5V)       || ML2
  * 		PE4  ||    Serial - KY020 (5V)	     || ML4
  * 		PC2  ||  Analog KY028/ADC Channel 3  || ML38
  *		PA10 ||	     Serial - KY015   	     ||	MR31
  *		PC5  ||       Digital KY028    	     || ML3
  ****************************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "main.h"

int main(void)
{
  // Variables //
  #ifdef DEVELOP_MODE
    uint8_t message_buf[100] = "Entering main..\n----System Config Reporting----\n";
    uint32_t clk_freq;
  #endif
  s_data sensor_data;

  // Mode choice //
  debug_flag = 1;
  demo_flag = 0;

  // Initialization phase //
  initialize_peripherals(debug_flag);

  #ifdef DEVELOP_MODE
    // Preparation phase //
    // Using our function due to reduced cost //
    transmit_UART_message(message_buf);

    // Printing Main clocks on our system //
    // Required due to formating //
    clk_freq = HAL_RCC_GetSysClockFreq();
    printf("Clock Freq: %ldHz\nUART Baud Rate: %ldB/s\nI2C Clock Freq: %ldHz\nCamera Clock Freq: %ldHz\n", clk_freq, huart1.Init.BaudRate, hi2c1.Init.Timing,clk_freq>>1);

    strcpy((char*) message_buf, "Press button to continue\n");
    transmit_UART_message(message_buf);
  #endif

  // Enable Core Counter - Used to count execution time //
  DWT_Init();

  // Main Functionality //
  while (1)
  {

    // Wait for user press //
    get_button_press(BUTTON_SW1);

    // Delay just for functionality //
    HAL_Delay(2500);

    // Reset Counter //
    DWT->CYCCNT = 0;
    get_frame();
    uint32_t end = DWT->CYCCNT;

    printf("#Frame grabbed - Time required:%ld -> Press button to transfer frame\n",end);
    get_button_press(BUTTON_SW1);
    transmit_UART_frame(1);

  }

}

/* System Clock Configuration */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  // Macro to configure the PLL multiplication factor //
  __HAL_RCC_PLL_PLLM_CONFIG(RCC_PLLM_DIV1);

  // Macro to configure the PLL clock source //
  __HAL_RCC_PLL_PLLSOURCE_CONFIG(RCC_PLLSOURCE_MSI);

  // Configure LSE Drive Capability //
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  // Configure the main internal regulator output voltage //
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  // Initializes the CPU, AHB and APB busses clocks //
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE
                              |RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    Error_Handler();

  // Configure the SYSCLKSource, HCLK, PCLK1 and PCLK2 clocks dividers //
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK4|RCC_CLOCKTYPE_HCLK2
                              |RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.AHBCLK2Divider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLK4Divider = RCC_SYSCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
    Error_Handler();

  //Initializes the peripherals clocks //
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SMPS|RCC_PERIPHCLK_USART1
                              |RCC_PERIPHCLK_I2C1|RCC_PERIPHCLK_USB;
  PeriphClkInitStruct.PLLSAI1.PLLN = 24;
  PeriphClkInitStruct.PLLSAI1.PLLP = RCC_PLLP_DIV2;
  PeriphClkInitStruct.PLLSAI1.PLLQ = RCC_PLLQ_DIV2;
  PeriphClkInitStruct.PLLSAI1.PLLR = RCC_PLLR_DIV2;
  PeriphClkInitStruct.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_USBCLK;
  PeriphClkInitStruct.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInitStruct.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  PeriphClkInitStruct.UsbClockSelection = RCC_USBCLKSOURCE_PLLSAI1;
  PeriphClkInitStruct.SmpsClockSelection = RCC_SMPSCLKSOURCE_HSI;
  PeriphClkInitStruct.SmpsDivSelection = RCC_SMPSCLKDIV_RANGE0;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    Error_Handler();

  // Enable MSI Auto calibration //
  HAL_RCCEx_EnableMSIPLLMode();
}

/* USART1 Initialization Function
 * Speed -> 115200
 * ParityBit -> None
 * StopBit   -> One
 *
 * UART channel can receive messages too.
*/
void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 230400;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

  // Initialize UART //
  if (HAL_UART_Init(&huart1) != HAL_OK)
    Error_Handler();

  // Set Transmitter FIFO threshold //
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
    Error_Handler();

  // Set Receiver FIFO threshold //
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
    Error_Handler();

  // Disable FIFO //
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
    Error_Handler();

}

/* I2C (v1) Initialization Function
 * Speed -> 100000
 * 7bit Adrress
 * Everything else default, but won't be needed
 *
*/
void MX_I2C1_Init(void)
{
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 100000;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

  // Initialize I2C //
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  	Error_Handler();

  // Configure Analog filter //
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  	Error_Handler();

  // Configure Digital filter //
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  	Error_Handler();
}

/* GPIO Initialization Function
 * Initializes the needed GPIOs to their appropriate settings.
 * Function also initializes the peripheral's (UART, DMA,etc..)
 * GPIOs according to the alternate function used.
 * Sensor GPIOs are initialized in sensor_lib.c functions.
 * The same goes for the camera's GPIOs
*/
void MX_GPIO_Init(void)
{

  // USART1 GPIO Configuration //
  // PB6     ------> USART1_TX //
  // PB7     ------> USART1_RX //
  if (huart1.Instance == USART1)
  {
    // Peripheral clock enable //
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART1);
    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);

    LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_6, LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetPinOutputType(GPIOB, LL_GPIO_PIN_6, LL_GPIO_OUTPUT_PUSHPULL);
    LL_GPIO_SetPinSpeed(GPIOB, LL_GPIO_PIN_6, LL_GPIO_SPEED_FREQ_VERY_HIGH);
    LL_GPIO_SetPinPull(GPIOB, LL_GPIO_PIN_6, LL_GPIO_PULL_UP);
    LL_GPIO_SetAFPin_0_7(GPIOB, LL_GPIO_PIN_6, LL_GPIO_AF_7);

    LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_7, LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetPinOutputType(GPIOB, LL_GPIO_PIN_7, LL_GPIO_OUTPUT_PUSHPULL);
    LL_GPIO_SetPinSpeed(GPIOB, LL_GPIO_PIN_7, LL_GPIO_SPEED_FREQ_VERY_HIGH);
    LL_GPIO_SetPinPull(GPIOB, LL_GPIO_PIN_7, LL_GPIO_PULL_UP);
    LL_GPIO_SetAFPin_0_7(GPIOB, LL_GPIO_PIN_7, LL_GPIO_AF_7);
  }
}

/* ADC Initialization Function - Configures ADC (Analog to Digital Converter)
 * Doesn't configure the channel in which we take a measurement and
 * perform the conversion just initialize the peripheral
 * with the default values.
 *
*/
void MX_ADC_Init(void)
{

  // Set the ADC handler pointer //
  adc_handle = ADC1;

  // Enable clock and Set clock source //
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_ADC);
  LL_ADC_SetCommonClock(__LL_ADC_COMMON_INSTANCE(adc_handle), LL_ADC_CLOCK_SYNC_PCLK_DIV2);

  // Data related - Highest resolution/Normal alignment //
  LL_ADC_SetResolution(adc_handle, LL_ADC_RESOLUTION_12B);
  LL_ADC_SetDataAlignment(adc_handle, LL_ADC_DATA_ALIGN_RIGHT);

  // Normal operation //
  if ((LL_ADC_IsDeepPowerDownEnabled(adc_handle) != PIN_LOW))
    LL_ADC_DisableDeepPowerDown(adc_handle);

  LL_ADC_SetLowPowerMode(adc_handle, LL_ADC_LP_MODE_NONE);
  LL_ADC_EnableInternalRegulator(adc_handle);

  // Perfrom ADC calibration - Help with better accuracy //
  LL_ADC_StartCalibration(adc_handle, LL_ADC_SINGLE_ENDED);
  while ((LL_ADC_IsCalibrationOnGoing(adc_handle)) != PIN_LOW);

  // We enable the ADC only by software //
  LL_ADC_REG_SetTriggerSource(adc_handle, LL_ADC_REG_TRIG_SOFTWARE);

  // We perform only one conversion on only one channel //
  LL_ADC_REG_SetSequencerLength(adc_handle, LL_ADC_REG_SEQ_SCAN_DISABLE);
  LL_ADC_REG_SetContinuousMode(adc_handle, LL_ADC_REG_CONV_SINGLE);

  // We perform oversampling, to adjust for ADC and environmental errors //
  // For each conversion sample continuously till all samples are obtained and the average is set //
  // We sample 8 times and then shift the result right by 3 //
  LL_ADC_SetOverSamplingScope(adc_handle, LL_ADC_OVS_GRP_REGULAR_RESUMED);
  LL_ADC_ConfigOverSamplingRatioShift(adc_handle, LL_ADC_OVS_RATIO_8, LL_ADC_OVS_SHIFT_RIGHT_3);
  LL_ADC_SetOverSamplingDiscont(adc_handle, LL_ADC_OVS_REG_CONT);

}

/* Function that waits for a button press */
void get_button_press(Button_TypeDef button)
{
  while (BSP_PB_GetState(button) != GPIO_PIN_RESET);
  HAL_Delay(50);
  while (BSP_PB_GetState(button) != GPIO_PIN_SET);
}

/* Wrapper Function that initializes all peripherals
 * By calling the appropriate functions,it initializes the following:
 * 1.System and system clock (proceeded by a global reset)
 * 2.The I2C1 interface
 * 3.The UART interface
 * 4.The GPIOs
 * 4a.The peripheral's GPIOs and their settings
 * 4b.Signal type GPIOs for the handling of the sensors
 * 5.LED and pushbuttons
 * 6.Camera register's initialization
 * 7.Sensors (KY015,KY028, etc...)
 *
 * Has 2 modes:
 * Mode 1 (Debug)  -> Uses only the non-blocking functions (flag = 0)
 * Mode 2 (Normal) -> Full functionality (flag = 1)
 * Finally transmits a message to the UART channel that
 * the initialization finished
*/
void initialize_peripherals(uint8_t flag)
{

  #ifdef DEVELOP_MODE
    uint8_t message_buf [26] = "Initializing Board..\n";
  #endif

  // Reset of all peripherals, Initializes the Flash interface and the Systick. //
  HAL_Init();

  // Configure the system clock //
  SystemClock_Config();

  // Peripherals initialization //
  MX_USART1_UART_Init();
  MX_GPIO_Init();

  // Push-buttons Initialization //
  BSP_PB_Init(BUTTON_SW1, BUTTON_MODE_GPIO);
  //BSP_PB_Init(BUTTON_SW2, BUTTON_MODE_EXTI);

  // Transmit to UART current progress //
  // Entering connected peripherals initialization //
  #ifdef DEVELOP_MODE
    transmit_UART_message(message_buf);
    strcpy((char*) message_buf, "Finished initialization\n");
  #endif

  // Set by user in main (debug_flag)
  if (flag)
  {
    // Initialize I2C interface needed for SCCB //
    MX_I2C1_Init();

    // Initialize the camera's required peripherals //
    OV7670_init_peripherals();

    // Set PA8 Clock speed -> 16MHz (Half of system clock) //
    HAL_RCC_MCOConfig (RCC_MCO1, RCC_MCO1SOURCE_SYSCLK, RCC_MCODIV_2);

    // Camera module reseting, not necessary //
    OV7670_reset();

    // Camera module registers initialization //
    OV7670_init();
  }

  // Activate KY-002 if specified //
  #ifdef KY_002_SENSOR
    ky002_init();
  #endif

  // Activate KY-015 if specified //
  #ifdef KY_015_SENSOR
    ky015_init();
  #endif

  // Activate KY-018 if specified //
  #ifdef KY_018_SENSOR
    MX_ADC_Init();
    ky018_init();
  #endif

  // Activate KY-020 if specified //
  #ifdef KY_020_SENSOR
    ky020_init();
  #endif

  // Activate KY-028 if specified //
  #ifdef KY_028_SENSOR
    // Initialize the ADC peripheral only once //
    #ifndef KY_018_SENSOR
      MX_ADC_Init();
    #endif
    ky028_init();
  #endif

  // Send to UART that initialization is finished //
  #ifdef DEVELOP_MODE
    transmit_UART_message(message_buf);
  #endif
}

/* This function is executed in case of error occurrence. */
void Error_Handler(void)
{
  while (1);
}

/* Helper function
 * Performs sanity checks when reading from a whole GPIO port.
 * Basically read from the gpio port in different ways:
 * 1.Using masks (just like HAL_Read_pin())
 * 2.Using pointers
 * Used only for debugging data pins of the camera's data GPIOs.
*/
void read_gpio_port(void)
{
  // Mask is 0x00FF //
  uint16_t gpio_mask = GPIO_PIN_0 | GPIO_PIN_1 |GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 |GPIO_PIN_6 | GPIO_PIN_7 ;

  // The base address of the GPIO port //
  // We test port A here only //
  __IO uint32_t *data_pins = &(GPIOA->IDR);

  // Check value //
  uint8_t check;

  // Read using a uint32_t pointer //
  // Overflow happens but we keep the values we want //
  check = (*data_pins);
  printf("Test 1 - Read using uint32_t pointer -> Value: %d\nPress button to continue\n", check);
  get_button_press(BUTTON_SW1);

  // Read using a mask //
  check = (*data_pins) & gpio_mask;
  printf("Test 2 - Read using bitmask -> Value: %d\nPress button to continue\n", check);
  get_button_press(BUTTON_SW1);

  // Read using a uint32_t pointer but with typecast//
  // Overflow happens but we keep the values we want //
  check = (uint32_t) (*data_pins);
  printf("Test 3 - Read using uint32_t pointer (uint8_t typecast) -> Value: %d\nPress button to continue\n", check);
  get_button_press(BUTTON_SW1);

}

/* Wrapper Function that performs reads data from every sensor that is defined
 * Function is used for debugging mostly and to test proper connectivity with
 * every sensor.
 * Prints results to terminal usign printf (due to the need for formating)
 *
*/
void sensor_test(void)
{
  uint32_t end;

  // KY-028 TESTS //
  #ifdef KY_028_SENSOR

    uint8_t threshold;
    float temperature;

    DWT->CYCCNT = 0;
    threshold = ky028_read(&temperature);
    end = DWT->CYCCNT;

    printf("Channell SQR1 is: %lu\nTime required for function call %ld\n", ADC1->SQR1, end);

    if (threshold)
      printf("KY028:Threshold reached and voltage is %f\nPress button to continue\n", temperature);
    else
      printf("KY028:Threshold not reached and voltage is %f\nPress button to continue\n",temperature);

    get_button_press(BUTTON_SW1);
  #endif

  // KY-002 TESTS //
  #ifdef KY_002_SENSOR
    uint8_t vibration;

    DWT->CYCCNT = 0;
    vibration = ky002_read();
    end = DWT->CYCCNT;

    printf("Time required for function call %ld\n", end);

    if (vibration)
      printf("KY002:Vibration detected\nPress button to continue\n");
    else
      printf("KY002:No Vibration detected\nPress button to continue\n");

    get_button_press(BUTTON_SW1);
  #endif

  // KY-020 TESTS //
  #ifdef KY_020_SENSOR
    uint8_t tilt;

    DWT->CYCCNT = 0;
    tilt = ky020_read();
    end = DWT->CYCCNT;

    printf("Time required for function call %ld\n", end);

    if (tilt)
      printf("KY020:Tilt detection\nPress button to continue\n");
    else
      printf("KY020:No tilt detected\nPress button to continue\n");

    get_button_press(BUTTON_SW1);
  #endif

  // KY-018 TESTS //
  #ifdef KY_018_SENSOR
    uint8_t light = 0;

    DWT->CYCCNT = 0;
    light = ky018_read();
    end = DWT->CYCCNT;

    printf("Channell SQR1 is: %lu\nTime required for function call %ld\n", ADC1->SQR1, end);
    printf("KY018:Light value is %d\nPress button to continue\n", light);

    get_button_press(BUTTON_SW1);
  #endif
}

/* Function that performs every test for the project
 * Only used for the demo
*/
void demo_test(void)
{

  printf("Proceeding to KY-028 test...\nPress button 1 get sensor data\n");
  do
  {
    // KY-028 TESTS //
    get_button_press(BUTTON_SW1);

    #ifdef KY_028_SENSOR
      uint8_t threshold;
      float temperature;

      threshold = ky028_read(&temperature);

      if (threshold)
	printf("KY028:Threshold reached and voltage is %f\nPress button to continue\n", temperature);
      else
	printf("KY028:Threshold not reached and voltage is %f\nPress button to continue\n",temperature);

    #endif
  }
  while(demo_flag == 0);

  // Reset the flag //
  demo_flag = 0;
  printf("Proceeding to KY-002 test...\nPress button 1 get sensor data\n");

  do
  {
    // KY-002 TESTS //
    get_button_press(BUTTON_SW1);

    #ifdef KY_002_SENSOR
      uint8_t vibration;

      vibration = ky002_read();

      if (vibration)
	printf("KY002:Vibration detected\nPress button to continue\n");
      else
	printf("KY002:No Vibration detected\nPress button to continue\n");

      #endif
  }
  while(demo_flag == 0);

  // Reset the flag //
  demo_flag = 0;
  printf("Proceeding to KY-020 test...\nPress button 1 get sensor data\n");

  do
  {
    // KY-020 TESTS //
    get_button_press(BUTTON_SW1);

    #ifdef KY_020_SENSOR
      uint8_t tilt;

      tilt = ky020_read();

      if (tilt)
	printf("KY020:Tilt detection\nPress button to continue\n");
      else
	printf("KY020:No tilt detected\nPress button to continue\n");

    #endif
  }
  while (demo_flag == 0);

  // Reset the flag //
  demo_flag = 0;
  printf("Proceeding to KY-018 test...\nPress button 1 get sensor data\n");

  do
  {
    // KY-018 TESTS //
    get_button_press(BUTTON_SW1);

    #ifdef KY_018_SENSOR
      uint8_t light = 0;

      light = ky018_read();

      printf("KY018:Light value is %d\nPress button to continue\n", light);

    #endif
  }
  while (demo_flag == 0);

  // Reset the flag //
  demo_flag = 0;
  printf("Proceeding to camera test...\nPress button 1 to capture an image\n");

  do
  {
    get_button_press(BUTTON_SW1);
    HAL_Delay(2000);

    get_frame();
    printf("Press button 1 to transfer frame\n");
    get_button_press(BUTTON_SW1);

    transmit_UART_frame(0);
    printf("#Press button 2 to repeat demo or 1 to repeat the image capture\n");
  }
  while (demo_flag == 0);

  // Reset the test //
  demo_flag = 0;

}

/* Callback function executed in case of interrupt
 * For now used only for the demo of the whole project
 *
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == BUTTON_SW2)
    demo_flag = 1;
}
