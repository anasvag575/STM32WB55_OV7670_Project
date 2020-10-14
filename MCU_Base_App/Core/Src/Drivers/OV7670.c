/**
  ******************************************************************************
  * @file           : OV7670.c
  * @brief          : OV7670 driver
  * @author	    : Anastasis Vagenas
  ******************************************************************************
  * Source code for the OV7670 camera module.
  * Module has functionalities that can perform a multitude of tasks:
  * 1.Reset the camera or get it in standby mode
  * 2.Write and Read the camera's registers
  * 3.Testbenches that perform basic testing for connectivity
  * 4.Initialize the camera with preloaded settings
  * 5.Frame capture
  * 6.Sending the frame via UART in PPM format (type P3 and P6)
  *
  * Driver works in a range of 40-400KHz for SIO_C or SCCB clock signal.
  * The driver assumes that the I2C communication is set up (hi2c1 handle)
  * and the GPIOs needed for the data pins (D0-D7), reset, VS, HS and PWDN
  *
  * Also has a debug mode needed when more info is needed, in the form of UART
  * messages transmitted via the helper UART channel or using printf when more
  * complex formating is required.
  * Constraints are saved in the corresponding header file.
  *
  ******************************************************************************
*/
#include "OV7670.h"

/* Constant matrix that contains 2 things:
 * In the first col  -> Camera register Address
 * In the second col -> Data to be writen to that register
 *
 * The matrix is used to initialize the camera parameters.
 * The contents of it are sourced from the OV7670 manual
 * and OV7670 user guide
 *
 * Important regs are defined in the header file along with the
 * default values for the application
*/
const uint8_t OV7670_reg[OV7670_REG_NUM][2] =

    // 3 resets just to be sure //
   {{ 0x12, 0x80 },
    { 0x12, 0x80 },
    { 0x12, 0x80 },

    // Image format //
    { 0x12, 0x14 },		// 0x14 = QVGA size - RGB mode //
    { 0x40, 0xD0 }, 		// Format RGB 565 and Full range //

    // Scaling numbers - Clock related //
    { 0x0C, 0x09 },  		// DCW enable //
    { 0x70, 0x3a },		// X_SCALING //
    { 0x71, 0x35 },		// Y_SCALING //
    { 0x72, 0x11 },		// DCW_SCALING //
    { 0x73, 0xf1 },		// PCLK_DIV_SCALING //
    { 0xa2, 0x02 },		// PCLK_DELAY_SCALING //
    { 0x3a, 0x01 }, 		// Auto update VSYNC AND HSYNC //

    // Hardware window //
    // VSTART = 10 | VSTOP = 490 | HSTART = 88 | HSTOP = 16 ?? //
    // Numbers don't make sense but they work anyway //
    { CLK_DIV_REG, CLK_DIV_VAL },	// XCLK Divider , Divide by the value //
    { CLK_PLL_REG, CLK_PLL_VAL },	// PLL Multiplier (for XCLK), Multiply by the value //
    { 0x32, 0x80 },			// HREF //
    { 0x17, 0x16 },			// HSTART //
    { 0x18, 0x04 },			// HSTOP //
    { 0x03, 0x0a },			// VREF //
    { 0x19, 0x02 },			// VSTART //
    { 0x1a, 0x7a },			// VSTOP //

    // Matrix coefficients  ->  Used for color correction/hue/saturation //
    // 	  Matrix Form	  //
    // |MTX1  MTX2  MTX3| //
    // |MTX4  MTX5  MTX6| //
    { 0x4f, 0xb3 }, 		// MTX1 //
    { 0x50, 0xb3 }, 		// MTX2	//
    { 0x51, 0x00 }, 		// MTX3 //
    { 0x52, 0x3d }, 		// MTX4 //
    { 0x53, 0xa7 }, 		// MTX5 //
    { 0x54, 0xe4 }, 		// MTX6 //
    { 0x58, 0x9e },		// Matrix sign register - enables auto center contrast //

    // --Gamma curve values-- //
    // These are the 16 points that create the gamma curve  //
    // Points are located at the xPos that is specified .   //
    // We specify the y coordinate of each point every time //
    // The gamma in terms of photography is close to 1/2.2  //
    // Close to what a RAW file from a camera should be.    //
    { 0x7b, 16  },	// G_P1 - xPos = 4 //
    { 0x7c, 30  },	// G_P2 - xPos = 8 //
    { 0x7d, 53  },	// G_P3 - xPos = 16 //
    { 0x7e, 90  },	// G_P4 - xPos = 32 //
    { 0x7f, 105 },	// G_P5 - xPos = 40 //
    { 0x80, 118 },	// G_P6 - xPos = 48 //
    { 0x81, 128 },	// G_P7 - xPos = 56 //
    { 0x82, 136 },	// G_P8 - xPos = 64 //
    { 0x83, 143 },	// G_P9 - xPos = 72 //
    { 0x84, 150 },	// G_P10 - xPos = 80 //
    { 0x85, 163 },	// G_P11 - xPos = 96 //
    { 0x86, 175 },	// G_P12 - xPos = 112 //
    { 0x87, 196 },	// G_P13 - xPos = 144 //
    { 0x88, 215 },	// G_P14 - xPos = 176 //
    { 0x89, 232 },	// G_P15 - xPos = 208 //
    { 0x7a, 32  },	// Slope of the last segment //
			// Calculated using -> slope = (256 - G_15) x 40/30 //

    // AWB parameters //
    { 0x6c, 0x0a }, 				// AWB Control 3 - Basic Control ? //
    { 0x6d, 0x55 }, 				// AWB Control 2 - Basic Control ? //
    { 0x6e, 0x11 }, 				// AWB Control 1 - Basic Control ? //
    { 0x6f, 0x9f },				// AWB Control 0 - Basic Control ? //
    { 0x41, 0x18 },				// AWB Gain Enable - Also enables auto De-noise //
    { AWB_RED_GAIN_REG, AWB_RED_GAIN },		// Red channel AWB gain //
    { AWB_GREEN_GAIN_REG, AWB_GREEN_GAIN }, 	// Green channel AWB gain //
    { AWB_BLUE_GAIN_REG, AWB_BLUE_GAIN }, 	// Blue channel AWB gain //

    // AWB Advanced parameters //
    { 0x43, 0x0a }, 		// AWB Control 1 - Reserved Magic //
    { 0x44, 0xf0 }, 		// AWB Control 2 - Reserved Magic //
    { 0x45, 0x34 }, 		// AWB Control 3 - Reserved Magic //
    { 0x46, 0x58 }, 		// AWB Control 4 - Reserved Magic //
    { 0x47, 0x28 }, 		// AWB Control 5 - Reserved Magic //
    { 0x48, 0x3a }, 		// AWB Control 6 - Reserved Magic //
    { 0x59, 0x88 }, 		// AWB Control 7 - Reserved Magic //
    { 0x5a, 0x88 }, 		// AWB Control 8 - Reserved Magic //
    { 0x5b, 0x44 }, 		// AWB Control 9 - Reserved Magic //
    { 0x5c, 0x67 },		// AWB Control 10 - Reserved Magic //
    { 0x5d, 0x49 }, 		// AWB Control 11 - Reserved Magic //
    { 0x5e, 0x0e },	 	// AWB Control 12 - Reserved Magic //

    // AGC and AEC parameters //
    { 0x24, 0x95 }, 		// AGC/AEC - Stable Operating Region (Upper Limit) //
    { 0x25, 0x33 }, 		// AGC/AEC - Stable Operating Region (Lower Limit) //
    { 0x26, 0xe3 }, 		// AGC/AEC Fast Mode Operating Region //
    { 0x9f, 0x78 }, 		// Histogram-based AEC/AGC Control 1 //
    { 0xa0, 0x68 }, 		// Histogram-based AEC/AGC Control 2 //
    { 0xa6, 0xd8 }, 		// Histogram-based AEC/AGC Control 3 //
    { 0xa7, 0xd8 }, 		// Histogram-based AEC/AGC Control 4 //
    { 0xa8, 0xf0 }, 		// Histogram-based AEC/AGC Control 5 //
    { 0xa9, 0x90 }, 		// Histogram-based AEC/AGC Control 6 //
    { 0xaa, 0x94 }, 		// Histogram-based AEC/AGC Control 7 //
    { 0x07, 0x00 },		// AEC Value //
    { 0x14, 0x18 },		// 16x AGC gain + Reserved Magic //
    { 0xa5, 0x05 },		// Banding step limit 50Hz //
    { 0xab, 0x07 },		// Banding step limit 60Hz //
    { 0x00, 0x0a },		// Gain Control - AGC //
    { 0x0d, 0x40 },		// Reserved magic //
    { 0xa1, 0x03 },		// Reserved magic //

    // Sharpness  //
    { SHARPNESS_REG, SHARPNESS + 0x05},	// Edge enhancement Factor - We set to 0, no need //
    { 0x76, 0xe7 },			// Pixel correction Enable //

    // Brightness and contrast //
    { BRIGHTNESS_REG, BRIGHTNESS },	// Brightness bias - Brightness slider (MSB sign - Rest value -> DEF = 0x00) //
    { CONTRAST_REG, CONTRAST + 0x05}, 	// Contrast bias - Contrast slider  (Increase or decrease -> DEF = 0x40) //
    { 0x3b, 0x16 },			// Banding filter enable - Fixes in exposure //

    // Fix Color Swap  //
    // These are "magic" reserved registers that perform color correction 		 	      //
    // In our case without them red is shown as green and every other color is off by a large margin. //
    // The fix is for THIS specific application, in your case you might not need them 		      //
    { 0x0E, 0x61 },
    { 0x0F, 0x4B },
    { 0x16, 0x02 },
    { 0x1E, 0x07 },
    { 0x21, 0x02 },
    { 0x22, 0x91 },
    { 0x29, 0x07 },
    { 0x33, 0x0B },
    { 0x35, 0x0B },
    { 0x37, 0x1D },
    { 0x38, 0x71 },
    { 0x39, 0x2A },
    { 0x3C, 0x78 },
    { 0x4D, 0x40 },
    { 0x4E, 0x20 },
    { 0x69, 0x00 },
    { 0x74, 0x10 },
    { 0x8D, 0x4F },
    { 0x8E, 0x00 },
    { 0x8F, 0x00 },
    { 0x90, 0x00 },
    { 0x91, 0x00 },
    { 0x96, 0x00 },
    { 0x9A, 0x00 },
    { 0xB0, 0x84 },
    { 0xB1, 0x0C },
    { 0xB2, 0x0E },
    { 0xB3, 0x82 },
    { 0xB8, 0x0A }};

/* Helper function that writes the specified data in the I2C channel.
 * We need to perform a 3-phase write operation:
 * 1.Send the Camera address ID (0x42)
 * 2.Send the register address we want to write to (reg_address)
 * 3.Send the data we want to write to the register (data)
 *
 * HAL_I2C_Mem_Write perform all of the above and is an I2C supported
 * function so we use that one.
 *
 * Function returns HAL_STATUS
 */
uint8_t write_camera_reg(uint8_t reg_address, uint8_t data)
{
  uint8_t check = 1;	// Check I2C operation status //
  uint8_t counter = 0;	// Counter used for maximum numbers of retries //

  // Continue until we reach max number of tries is reached or success //
  while ((check != HAL_OK) && (counter != MAX_I2C_RETRIES))
  {
    check = HAL_I2C_Mem_Write(&hi2c1, OV7670_WRITE_ADDR, reg_address, I2C_MEMADD_SIZE_8BIT, &data, 1, MAX_I2C_TIMEOUT);
    counter++;
  }

  // Delay needed based on manual //
  HAL_Delay(100);

  // Check if we succeeded or not //
  if (counter == MAX_I2C_RETRIES)
    return HAL_ERROR;
  else
    return check;
}

/* Helper function that reads the specified address data from the I2C channel
 * Function performs 2-phase write followed by a 2-phase read:
 * -----> 2-phase write:
 * 1.Send the camera ID address (0x42) -> (Write address)
 * 2.Send the address of the register we want to read from
 *
 * -----> 2-phase read:
 * 1.Master (MCU) will send the camera ID address (0x43) -> (Read address)
 * 2.Slave will send the contents of the register back
 *
 * It is used only to confirm that the write operation
 * was successful, other than that it has no other use
 *
 * Function returns HAL_STATUS
*/
uint8_t read_camera_reg(uint8_t reg_address, uint8_t *data)
{
  uint8_t check1 = 1, check2 = 1;
  uint8_t counter = 0;

  // User defined mode //
  #ifdef DEBUG_MODE
    // Entered function -> Step 1 //
    uint8_t err_mess [] = "Trying to read camera register\n";
    transmit_UART_message(err_mess);
  #endif

  // Continue until we reach max number of tries is reached or success //
  while ((check1 != HAL_OK) && (check2 != HAL_OK) && (counter != MAX_I2C_RETRIES))
  {
    check1 = HAL_I2C_Master_Transmit(&hi2c1, OV7670_WRITE_ADDR, &reg_address, 1, MAX_I2C_TIMEOUT);
    check2 = HAL_I2C_Master_Receive(&hi2c1, OV7670_READ_ADDR, data, 1, MAX_I2C_TIMEOUT);
    counter++;
  }

  // Return if we reached threshold //
  if(counter == MAX_I2C_RETRIES)
    return HAL_ERROR;

  // User defined mode //
  #ifdef DEBUG_MODE
  // Confirm Success/Failure of each phase -> Step 2 //

    // 2-phase write success //
    if(check1 == HAL_OK)
    {
      strcpy((char *)err_mess,"Phase 1:OK\n");
      transmit_UART_message(err_mess);
    }

    // 2-phase read success //
    if(check2 == HAL_OK)
    {
      strcpy((char *)err_mess,"Phase 2:OK\n");
      transmit_UART_message(err_mess);
    }
  #endif

  return (check1 | check2);
}

/* Helper function that initializes the camera's internal registers.
 * We use the global variable OV7670_reg which has on col:
 * 1.Register's address
 * 2.The data we want to write to that register
 *
 * We iterate for OV7670_REG_NUM and write to each register the data
 * we want using transmit_I2C_message
 *
 * Function returns HAL_STATUS
*/
uint8_t OV7670_init(void)
{

  uint8_t i = 0, err1 = HAL_OK, err2 = HAL_OK;
  uint8_t data;

  #ifdef DEVELOP_MODE
    uint8_t err_mess[50] = "Entered camera init\n";

    // Confirmation message //
    transmit_UART_message(err_mess);
  #endif

  // Configure camera registers //
  for (i = 0; i < OV7670_REG_NUM; i++)
  {
    // Helper function call over register matrix //
    err1 = write_camera_reg(OV7670_reg[i][0], OV7670_reg[i][1]);
    err2 = read_camera_reg(OV7670_reg[i][0], &data);

    // User defined debug mode prints formated messages via UART //
    #ifdef DEBUG_MODE
      // Print the status of the initialization //
      printf("Write Register: %x|Iter: %d|Status:",OV7670_reg[i][0],i);

      // Success or not and why //
      if (data == OV7670_reg[i][1])
	printf("OK\n");
      else
	printf("Not OK\n---Data:%x |Reg:%x\n",data,OV7670_reg[i][1]);
    #endif

    // Normal operation //
    #ifndef DEBUG_MODE
      // Transmit to UART in case of failure and return //
      if ((i > 3) && ((err1 != HAL_OK) || (err2 != HAL_OK )) && (data == OV7670_reg[i][1]))
      {

	#ifdef DEVELOP_MODE
	  strcpy((char*)err_mess, "Failure in updating camera's registers\n");
	  transmit_UART_message(err_mess);
	#endif

	return (err1||err2);
      }
    #endif
  }

  // Otherwise transmit to UART in case of success //
  #ifdef DEVELOP_MODE
    strcpy((char*)err_mess, "Updated camera's registers successfully\n");
    transmit_UART_message(err_mess);
  #endif

  return HAL_OK;
}

/* Helper function that initializes the camera's required GPIOs
 * and sets the I2C peripheral
*/
void OV7670_init_peripherals(void)
{

  // Enable the GPIO port clocks needed //
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOC);

  // I2C1 GPIO Configuration //
  // PB8     ------> I2C1_SCL //
  // PB9     ------> I2C1_SDA //
  if (hi2c1.Instance == I2C1)
  {
    // Peripheral clock enable //
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C1);

    LL_GPIO_SetPinMode(SIO_C_PORT, SIO_C_LL_PIN, LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetPinOutputType(SIO_C_PORT, SIO_C_LL_PIN, LL_GPIO_OUTPUT_OPENDRAIN);
    LL_GPIO_SetPinSpeed(SIO_C_PORT, SIO_C_LL_PIN, LL_GPIO_SPEED_FREQ_VERY_HIGH);
    LL_GPIO_SetPinPull(SIO_C_PORT, SIO_C_LL_PIN, LL_GPIO_PULL_UP);
    LL_GPIO_SetAFPin_8_15(SIO_C_PORT, SIO_C_LL_PIN, LL_GPIO_AF_4);

    LL_GPIO_SetPinMode(SIO_D_PORT, SIO_D_LL_PIN, LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetPinOutputType(SIO_D_PORT, SIO_D_LL_PIN, LL_GPIO_OUTPUT_OPENDRAIN);
    LL_GPIO_SetPinSpeed(SIO_D_PORT, SIO_D_LL_PIN, LL_GPIO_SPEED_FREQ_VERY_HIGH);
    LL_GPIO_SetPinPull(SIO_D_PORT, SIO_D_LL_PIN, LL_GPIO_PULL_UP);
    LL_GPIO_SetAFPin_8_15(SIO_D_PORT, SIO_D_LL_PIN, LL_GPIO_AF_4);
  }

  // Camera CLK GPIO Configuration //
  // PA8     ------> XCLK //
  // PC10    ------> PWDN //
  // PC12    ------> RESET //
  // XCLK Config //
  LL_GPIO_SetPinMode(XCLK_PORT, XCLK_LL_PIN, LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetPinOutputType(XCLK_PORT, XCLK_LL_PIN, LL_GPIO_OUTPUT_PUSHPULL);
  LL_GPIO_SetPinSpeed(XCLK_PORT, XCLK_LL_PIN, LL_GPIO_SPEED_FREQ_VERY_HIGH);
  LL_GPIO_SetPinPull(XCLK_PORT, XCLK_LL_PIN, LL_GPIO_PULL_NO);
  LL_GPIO_SetAFPin_8_15(XCLK_PORT, XCLK_LL_PIN, LL_GPIO_AF_6);

  // PWDN Config //
  LL_GPIO_SetPinMode(PWDN_PORT, PWDN_LL_PIN, LL_GPIO_MODE_OUTPUT);
  LL_GPIO_SetPinOutputType(PWDN_PORT, PWDN_LL_PIN, LL_GPIO_OUTPUT_PUSHPULL);
  LL_GPIO_SetPinSpeed(PWDN_PORT, PWDN_LL_PIN, LL_GPIO_SPEED_FREQ_VERY_HIGH);
  LL_GPIO_SetPinPull(PWDN_PORT, PWDN_LL_PIN, LL_GPIO_PULL_NO);

  // Reset Config //
  LL_GPIO_SetPinMode(RET_PORT, RET_LL_PIN, LL_GPIO_MODE_OUTPUT);
  LL_GPIO_SetPinOutputType(RET_PORT, RET_LL_PIN, LL_GPIO_OUTPUT_PUSHPULL);
  LL_GPIO_SetPinSpeed(RET_PORT, RET_LL_PIN, LL_GPIO_SPEED_FREQ_VERY_HIGH);
  LL_GPIO_SetPinPull(RET_PORT, RET_LL_PIN, LL_GPIO_PULL_NO);

  // Output sync signals //
  // PB4     ------> VSYNC //
  // PC3     ------> HSYNC //
  // PC1     ------> PXCLK //
  // VSYNC Config //
  LL_GPIO_SetPinMode(VSYNC_PORT, VSYNC_LL_PIN, LL_GPIO_MODE_INPUT);
  LL_GPIO_SetPinPull(VSYNC_PORT, VSYNC_LL_PIN, LL_GPIO_PULL_NO);

  // HSYNC Config //
  LL_GPIO_SetPinMode(HSYNC_PORT, HSYNC_LL_PIN, LL_GPIO_MODE_INPUT);
  LL_GPIO_SetPinPull(HSYNC_PORT, HSYNC_LL_PIN, LL_GPIO_PULL_NO);

  // PXCLK Config //
  LL_GPIO_SetPinMode(PXCLK_PORT, PXCLK_LL_PIN, LL_GPIO_MODE_INPUT);
  LL_GPIO_SetPinPull(PXCLK_PORT, PXCLK_LL_PIN, LL_GPIO_PULL_NO);

  // Output data signals //
  // Data0 - Data7 -> PA0-PA7 //
  LL_GPIO_SetPinMode(DATA_PORT, DATA0_LL_PIN, LL_GPIO_MODE_INPUT);
  LL_GPIO_SetPinPull(DATA_PORT, DATA0_LL_PIN, LL_GPIO_PULL_NO);

  LL_GPIO_SetPinMode(DATA_PORT, DATA1_LL_PIN, LL_GPIO_MODE_INPUT);
  LL_GPIO_SetPinPull(DATA_PORT, DATA1_LL_PIN, LL_GPIO_PULL_NO);

  LL_GPIO_SetPinMode(DATA_PORT, DATA2_LL_PIN, LL_GPIO_MODE_INPUT);
  LL_GPIO_SetPinPull(DATA_PORT, DATA2_LL_PIN, LL_GPIO_PULL_NO);

  LL_GPIO_SetPinMode(DATA_PORT, DATA3_LL_PIN, LL_GPIO_MODE_INPUT);
  LL_GPIO_SetPinPull(DATA_PORT, DATA3_LL_PIN, LL_GPIO_PULL_NO);

  LL_GPIO_SetPinMode(DATA_PORT, DATA4_LL_PIN, LL_GPIO_MODE_INPUT);
  LL_GPIO_SetPinPull(DATA_PORT, DATA4_LL_PIN, LL_GPIO_PULL_NO);

  LL_GPIO_SetPinMode(DATA_PORT, DATA5_LL_PIN, LL_GPIO_MODE_INPUT);
  LL_GPIO_SetPinPull(DATA_PORT, DATA5_LL_PIN, LL_GPIO_PULL_NO);

  LL_GPIO_SetPinMode(DATA_PORT, DATA6_LL_PIN, LL_GPIO_MODE_INPUT);
  LL_GPIO_SetPinPull(DATA_PORT, DATA6_LL_PIN, LL_GPIO_PULL_NO);

  LL_GPIO_SetPinMode(DATA_PORT, DATA7_LL_PIN, LL_GPIO_MODE_INPUT);
  LL_GPIO_SetPinPull(DATA_PORT, DATA7_LL_PIN, LL_GPIO_PULL_NO);
}

/* Helper function that resets the camera's internal registers.
 * Camera's reset is active low
 * So, we send a reset 3 times with a delay between each setting
 * Recommended to use before initializing camera (OV7670)
*/
void OV7670_reset(void)
{
  uint8_t i;
  #ifdef DEVELOP_MODE
  uint8_t message_buf [] = "Finished Camera Reset\n";
  #endif

  // PWDN Active High //
  PWDN_PORT->BRR = (uint32_t) PWDN_PIN;
  HAL_Delay(50);

  // RET Active Low //
  for(i = 0; i < 3; i++){
    RET_PORT->BRR = (uint32_t) RET_PIN;
    HAL_Delay(1000);

    RET_PORT->BSRR = (uint32_t) RET_PIN;
    HAL_Delay(1000);
  }

  // Transmit to UART the end of reset operation //
  #ifdef DEVELOP_MODE
    transmit_UART_message(message_buf);
  #endif
}

/* Helper function that set the camera's power mode
 * Set the PWDN pin level.
 * PWDN is an active high signal so:
 * For enabling camera -> PWDN = 0
 * For disabling camera -> PWDN = 1
 *
 * So based on mode we write the the specified value to the register and
 * wait for the specified amount of time
 */
void OV7670_set_power_mode(uint8_t mode)
{
  if (mode)
  {
    // Power Down //
    PWDN_PORT->BSRR = (uint32_t) PWDN_PIN;
  }
  else
  {
    // Power up //
    PWDN_PORT->BRR = (uint32_t) PWDN_PIN;
    HAL_Delay(4000); // Neeeded Delay //
  }
}

/* Helper function that testes whether the write and read operations work correctly.
 * Resets the camera using OV7670_reset and then resets by writing to reset register.
 * Performs a series of reads and writes by comparing the default values with
 * the new values we have written and then prints the output to terminal.
*/
uint8_t OV7670_testbench(void)
{
  uint8_t data;
  uint8_t check = 0;
  #ifdef DEVELOP_MODE
    uint8_t def_mes[] = "Ok default val\n";
    uint8_t new_mes[] = "Ok new val\n";
  #endif

  // Harware reset followed by software reset //
//  OV7670_reset();
//  write_camera_reg(0x12,0x80);
//  write_camera_reg(0x12,0x80);
//  write_camera_reg(0x12,0x80);

  // 1st test - Read Check //
  read_camera_reg(0x52, &data);
  if (data == 0x17)
  {
    #ifdef DEVELOP_MODE
      transmit_UART_message(def_mes);
    #endif
    check++;
  }

  // 1st test - Write Check //
  write_camera_reg(0x52,0x22);
  read_camera_reg(0x52, &data);
  if (data == 0x22)
  {
    #ifdef DEVELOP_MODE
      transmit_UART_message(new_mes);
    #endif
    check++;
  }

  // 2nd test - Read Check //
  read_camera_reg(0x53, &data);
  if (data == 0x29)
  {
    #ifdef DEVELOP_MODE
      transmit_UART_message(def_mes);
    #endif
    check++;
  }

  // 2nd test - Write Check //
  write_camera_reg(0x53, 0x5e);
  read_camera_reg(0x53, &data);
  if (data == 0x5e)
  {
    #ifdef DEVELOP_MODE
      transmit_UART_message(new_mes);
    #endif
    check++;
  }

  // 3rd test - Read Check //
  read_camera_reg(0x54, &data);
  if (data == 0x40)
  {
    #ifdef DEVELOP_MODE
      transmit_UART_message(def_mes);
    #endif
    check++;
  }

  // 3rd test - Write Check //
  write_camera_reg(0x54,0xa2);
  read_camera_reg(0x54, &data);
  if (data == 0xa2)
  {
    #ifdef DEVELOP_MODE
      transmit_UART_message(new_mes);
    #endif
    check++;
  }

  // 4th test - Read Check //
  read_camera_reg(0x51, &data);
  if (data == 0x0c)
  {
    #ifdef DEVELOP_MODE
      transmit_UART_message(def_mes);
    #endif
    check++;
  }

  // 4th test - Write Check //
  write_camera_reg(0x51, 0x34);
  read_camera_reg(0x51, &data);
  if (data == 0x34)
  {
    #ifdef DEVELOP_MODE
      transmit_UART_message(new_mes);
    #endif
    check++;
  }

  if(check == 8)
    return 1;
  else
    return 0;

}

/* Function that get frames from camera.
 * The function is the routine that gets frames from the camera:
 * 1.First we synchronise with VSYNC, which is active low
 * 2.Then we synchronise with HSYNC, which is active high
 * 3.Then we synchronise with the PIXCLK, where we want to detect positive edges
 * 4.Then we read the whole GPIO port where the data pins are
 * 5.Repeat until end of VSYNC
 *
 * In order to be correct we have to count 240 HSYNC pulses and 153600 clock ticks
 * Our resolution is 320x240 (QVGA) and every pixel has 2 bytes of data (RGB565)
 * So according to camera's manual PXCLK_ticks = 2 * 320 * 240 = 153600
 *
 * Function works at the lowest level due to speed constraints, meaning we only read
 * directly the memory without using any HAL or LL function and without any
 * verification.
 *
 * Some optimizations that we performed, to increase our signal sampling rate:
 * 1.Data pins are PA0 - PA7, continuous in memory so no need to apply a bitmask
 * 2.Use a pointer to the frame buffer that we increment after each read
 * 3.Critical code is the PXCLK detection where we have to decrease the number of
 * conditional execution (jumps caused from while() loops)
 * 4.A step to increase speed is to read the PXCLK and HSYNC together (have them in the
 * same port)
 * 5.Use of posedge and negedge detection would be another idea
 *
 * Speed at which we can read safely is about (XCLK input) / (PXLCK_DIV) = 1 Mhz max
 * But it could also go to about 4MHz if one of the while loops was gone,
 * meaning detecting the edge of the pixel clock, not tracking every change.
 * That might save us about 3-4 cycles and sample at about double the rate.
 *
 * Frame is stored in the global variable frame_buffer which then can be processed further
 * if needed
 * */
void get_frame(void)
{

  // Every signal's port base address //
  __IO uint32_t *vsync = &(VSYNC_PORT->IDR);
  __IO uint32_t *hsync = &(HSYNC_PORT->IDR);
  __IO uint32_t *pixclk = &(PXCLK_PORT->IDR);
  __IO uint32_t *data_pins = &(DATA_PORT->IDR);

  // Pointer to frame buffer //
  uint8_t *temp = frame_buffer;

  // Time critical section start //
  // Disabling interrupts //
  __disable_irq();

  // Sync with VSYNC //
  while (((*vsync) & (VSYNC_PIN)) == PIN_LOW);
  while (((*vsync) & (VSYNC_PIN)) != PIN_LOW);
  while (((*vsync) & (VSYNC_PIN)) == PIN_LOW)
  {
    // Sync with HSYNC //
    while (((*hsync) & (HSYNC_PIN)) != PIN_LOW)
    {
      // Sync with PXCLK - Detect the positive edge //
      // We do it 4 times to improve sampling speed //
      // We will enter the loop 80 times (4 * 80 = 320) //
      while (((*pixclk) & (PXCLK_PIN)) == PIN_LOW);
      *temp = *data_pins;
      temp++;
      while (((*pixclk) & (PXCLK_PIN)) != PIN_LOW);

      while (((*pixclk) & (PXCLK_PIN)) == PIN_LOW);
      *temp = *data_pins;
      temp++;
      while (((*pixclk) & (PXCLK_PIN)) != PIN_LOW);

      while (((*pixclk) & (PXCLK_PIN)) == PIN_LOW);
      *temp = *data_pins;
      temp++;
      while (((*pixclk) & (PXCLK_PIN)) != PIN_LOW);

      while (((*pixclk) & (PXCLK_PIN)) == PIN_LOW);
      *temp = *data_pins;
      temp++;
      while (((*pixclk) & (PXCLK_PIN)) != PIN_LOW);

    }
  }

  // Time critical section end //
  // Enabling interrupts again //
  __enable_irq();

}

/* Function that transmits the frame_buffer via UART.
 * The function converts the frame captured into RGB888 from RGB565 and then
 * transmits the image via UART into PPM format.
 * PPM Format ->Px (Either P3 or P6)
 * Width Height (Ascii Numbers both)
 * Color depth  (Ascii Number)
 * [Red0 Green0 Blue0] [Red1 Green1 Blue1] ....
 *
 * In our case Width is 320 Height is 240 and Color depth 8-bit so 255.
 * We can choose which format to use based on function argument we use:
 * 1.P3 means we use plain PPM which means the pixels are stored in ASCII format,
 *   meaning every channel from every pixel will be an ASCII character.
 *   Generally inefficient but easily readable
 * 2.P6 means we use binary PPM where the pixels are stored in binary format,
 *   meaning every channel from every pixel is a byte and they are stored serially
 *   in the file.
 *   More efficient but less readable.
 *   Needs a buffer to transmit the data in small chunks, in order not to overwhelm
 *   the Termite terminal
 *
 * So argument format = 0  ---> P3
 * 	       format = 1  ---> P6
 *
 */
void transmit_UART_frame(uint8_t format)
{

  // [0] -> Red   //
  // [1] -> Green //
  // [2] -> Blue  //
  uint8_t pixel[3];
  uint32_t j;
  uint8_t buf [960];	// Used for P6 format //
  uint16_t count = 0;
  uint8_t message_buf [] = "P6\n320 240\n255\n";

  // Only difference is the magic number //
  if (format)
    transmit_UART_message(message_buf);
  else
  {
    message_buf[1] = '3';
    transmit_UART_message(message_buf);
  }

  // Run for whole frame buffer //
  // Steps for every pixel aka 2 bytes //
  for (j = 0; j < 2 * IMG_ROW * IMG_COL; j = j+2)
  {

    // Get each channel's value using bitmasks //
    // 0xF8 -> 11111000 - Red	        //
    // 0x07 -> 00000111 - Green Upper	//
    // 0xe0 -> 11100000 - Green Lower	//
    // 0x1f -> 00011111 - Blue          //
    pixel[0] = (frame_buffer[j] & (0xF8)) >> 3;
    pixel[1] = ((frame_buffer[j]) & (0x07)) << 3;
    pixel[1] = pixel[1] | (((frame_buffer[j+1]) & (0xE0)) >> 5);
    pixel[2] = (frame_buffer[j+1]) & (0x1f);

    // Map to an 8-bit value //
    pixel[0] = ( pixel[0] * 255 + 15 ) >> 5;
    pixel[1] = ( pixel[1] * 255 + 31 ) >> 6;
    pixel[2] = ( pixel[2] * 255 + 13 ) >> 5;

    // Based on choice, transmit a P6 or P3 type of image	//
    // P6 needs a buffer to that is flushed every 3 image lines //
    if (format)
    {
      // Add the pixels in the buffer and increment the counter //
      buf[count] = pixel[0];
      buf[count+1] = pixel[1];
      buf[count+2] = pixel[2];
      count = count + 3;

      // When counter is full, flush and reset counter //
      if (count == 960)
      {
	HAL_UART_Transmit(&huart1, buf, 960, HAL_MAX_DELAY);
	HAL_Delay(25);	// Delay needed not to overwhelm Termite terminal //
	count = 0;
      }
    }
    else
    {
      // Flush the printf buffer every half line //
      if (j % IMG_COL == 0)
      {
	printf("\n");
	HAL_Delay(15); //Delay needed not to overwhelm Termite terminal //
      }

      // Printf required due to formatting //
      // We need integers in ASCII format //
      printf("%d %d %d ", pixel[0], pixel[1], pixel[2]);
    }
  }

  // Image Delimiter - Helps us find the image end in the Termite log //
  strcpy((char*) message_buf,"\n#IMAGE END\n" );
  transmit_UART_message(message_buf);

}
