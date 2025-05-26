#include "main.h"
#include "usb_host.h"

#include "stdint.h"

I2C_HandleTypeDef hi2c1;

I2C_HandleTypeDef hi2c2;

I2S_HandleTypeDef hi2s3;
SPI_HandleTypeDef hspi1;


void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2S3_Init(void);
static void MX_SPI1_Init(void);
static void MX_I2C2_Init(void);
void MX_USB_HOST_Process(void);

void prepareBuffer( uint8_t* buffer_ptr, uint8_t data, uint8_t rs, uint8_t rw );
void pollingMasterSend(uint8_t LCDaddr, uint8_t* tx_buffer, uint8_t data_size, uint8_t delay_ms);
void pollingMasterRead(uint8_t LCDaddr, uint8_t* buffer, uint8_t delay_ms);
void waitBFgoLow(uint8_t LCDaddr);
void LCD_Init (uint8_t* tx_buffer_ptr, uint8_t LCDaddr);


int main(void)
{

  /*
  * Setup:
  * PB10 - SCL
  * PB11 - SDA
  */

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();



  /* Configure the system clock */
  SystemClock_Config();


  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_I2S3_Init();
  MX_SPI1_Init();
  MX_USB_HOST_Init();
  MX_I2C2_Init();

  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */


  uint8_t tx_buffer[4];
  uint8_t data;
  uint8_t LCDaddr = 0x27 << 1;

  // TODO: perhaps the 4th bit doesn't need to be high?

  // -- Initialization --
  LCD_Init(tx_buffer, LCDaddr);


  // -- Write data --
  data = 0x48; // ASCII = H
  prepareBuffer(tx_buffer, data, 1, 0);
  pollingMasterSend(LCDaddr, tx_buffer, (uint8_t) 4, (uint8_t) 500);


  // -- Infinite loop --
  while (1)
  {
    /* USER CODE END WHILE */
    MX_USB_HOST_Process();

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

void waitBFgoLow(uint8_t LCDaddr){
	uint8_t rx_buffer = 0;

	do{
		pollingMasterRead(LCDaddr, &rx_buffer, (uint8_t) 0);
	} while(rx_buffer & 0b1 << 7);
}

void pollingMasterSend(uint8_t LCDaddr, uint8_t* tx_buffer, uint8_t data_size, uint8_t delay_ms){
	// Wait until the bus is ready
	while (HAL_I2C_GetState(&hi2c2) != HAL_I2C_STATE_READY)
	{
	}

	// TODO: delete
	// ! TODO: check if any error is returned in the first place
	int hal_status = HAL_I2C_Master_Transmit(&hi2c2, LCDaddr, tx_buffer, data_size, HAL_MAX_DELAY);
	if (hal_status != HAL_OK)
	{
		// TODO: delete
	  int dummy = 1;
	}

	HAL_Delay(delay_ms);
}

void pollingMasterRead(uint8_t LCDaddr, uint8_t* tx_buffer, uint8_t delay_ms){
	// Wait until the bus is ready
	while (HAL_I2C_GetState(&hi2c2) != HAL_I2C_STATE_READY)
	{
	}

	// TODO: delete
	// ! TODO: check if any error is returned in the first place
	int hal_status = HAL_I2C_Master_Receive(&hi2c2, LCDaddr, tx_buffer, 1, HAL_MAX_DELAY);
	if (hal_status != HAL_OK)
	{
	  int dummy = 1;
	}

	HAL_Delay(delay_ms);
}

void prepareBuffer( uint8_t* tx_buffer_ptr, uint8_t data, uint8_t rs, uint8_t rw ){
	char upper_nibble, lower_nibble;

	// The data is split in halves and shifter all the way to the left
	// The right 4 bits = 0x0 or Ob0000
	upper_nibble = (data & 0xF0);            // Extract upper nibble
	lower_nibble = ((data << 4) & 0xF0);     // Extract lower nibble

	// Get the values of RS and RW
	uint8_t rs_rw;
	rs_rw = rs + rw;

	// The right 4 bits are adjusted to have correct RS, RW, and E values
	tx_buffer_ptr[0] = upper_nibble | (0x0C + rs_rw);  // en=1, pin4=1
	tx_buffer_ptr[1] = upper_nibble | (0x08 + rs_rw);  // en=0, pin4=1
	tx_buffer_ptr[2] = lower_nibble | (0x0C + rs_rw);  // en=1, pin4=1
	tx_buffer_ptr[3] = lower_nibble | (0x08 + rs_rw);  // en=0, pin4=1
}



void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hi2c);

  if (hi2c->ErrorCode == HAL_I2C_ERROR_AF){
	  int dummy = 1;
  }
}

// TODO: delete
void HAL_I2C_MasterTxCpltCallback (I2C_HandleTypeDef * hi2c)
{
  // TX Done .. Do Something!
  // TODO: delete
  int dummy = 1;
}

void LCD_Init (uint8_t* tx_buffer_ptr, uint8_t LCDaddr){
	// TODO: potentially add BF checks


	// -- Wake up calls --
	HAL_Delay(100);  // wait for >40ms

	prepareBuffer(tx_buffer_ptr, 0x30, 0, 0);
	pollingMasterSend(LCDaddr, tx_buffer_ptr, (uint8_t) 4, (uint8_t) 5); // wait for >4.1ms

	prepareBuffer(tx_buffer_ptr, 0x30, 0, 0);
	pollingMasterSend(LCDaddr, tx_buffer_ptr, (uint8_t) 4, (uint8_t) 1); // wait for >100us

	prepareBuffer(tx_buffer_ptr, 0x30, 0, 0);
	pollingMasterSend(LCDaddr, tx_buffer_ptr, (uint8_t) 4, (uint8_t) 10); // wait for 10ms

	// -- Switch to 4bit mode --
	// Function set instruction with E = 1, E = 0
	// While in 8 bit mode, only two instances are needed
	uint8_t functionSet_buffer[2] = {0b00101100, 0b00101000};
	pollingMasterSend(LCDaddr, functionSet_buffer, (uint8_t) 2, (uint8_t) 10);

	// -- Display initialization --
	// Function set --> DL=0 (4 bit mode), N = 0 (1 line display) F = 1 (5x10 characters)
	prepareBuffer(tx_buffer_ptr, 0x24, 0, 0);
	pollingMasterSend(LCDaddr, tx_buffer_ptr, (uint8_t) 4, (uint8_t) 1); // wait for 1ms

	// -- Display on/off control --> D=0,C=0, B=0  ---> display off --
	prepareBuffer(tx_buffer_ptr, 0x08, 0, 0);
	pollingMasterSend(LCDaddr, tx_buffer_ptr, (uint8_t) 4, (uint8_t) 1); // wait for 1ms

    // -- Clear display --
	prepareBuffer(tx_buffer_ptr, 0x01, 0, 0);
	pollingMasterSend(LCDaddr, tx_buffer_ptr, (uint8_t) 4, (uint8_t) 2); // wait for 2ms

	// -- Entry mode set --> I/D = 1 (increment cursor) & S = 0 (no shift) --
	prepareBuffer(tx_buffer_ptr, 0x06, 0, 0);
	pollingMasterSend(LCDaddr, tx_buffer_ptr, (uint8_t) 4, (uint8_t) 1); // wait for 1ms

	// -- Display on/off control --> D = 1, C and B = 0. (Cursor and blink, last two bits) --
	prepareBuffer(tx_buffer_ptr, 0x0C, 0, 0);
	pollingMasterSend(LCDaddr, tx_buffer_ptr, (uint8_t) 4, (uint8_t) 1); // wait for 1ms

	// -- Set cursor at: row 0, column 0 --
	prepareBuffer(tx_buffer_ptr, 0x80, 0, 0);
	pollingMasterSend(LCDaddr, tx_buffer_ptr, (uint8_t) 4, (uint8_t) 2); // wait for 2ms
}





/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */



  /* USER CODE END I2C2_Init 2 */

}


/**
  * @brief I2S3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S3_Init(void)
{

  /* USER CODE BEGIN I2S3_Init 0 */

  /* USER CODE END I2S3_Init 0 */

  /* USER CODE BEGIN I2S3_Init 1 */

  /* USER CODE END I2S3_Init 1 */
  hi2s3.Instance = SPI3;
  hi2s3.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s3.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s3.Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s3.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;
  hi2s3.Init.AudioFreq = I2S_AUDIOFREQ_96K;
  hi2s3.Init.CPOL = I2S_CPOL_LOW;
  hi2s3.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s3.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S3_Init 2 */

  /* USER CODE END I2S3_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : CS_I2C_SPI_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_I2C_SPI_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PDM_OUT_Pin */
  GPIO_InitStruct.Pin = PDM_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(PDM_OUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT1_Pin */
  GPIO_InitStruct.Pin = BOOT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin
                           Audio_RST_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MEMS_INT2_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
