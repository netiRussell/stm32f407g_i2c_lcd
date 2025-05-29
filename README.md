# stm32f407g_i2c_lcd
 Sending data to LCD1602A from STM32F407G via I2C

# How to use:
-- Polling logic use --
 * Globally initialize:
    I2C_HandleTypeDef hi2c2;

 * In the main function:
 	// Initialization
 	LCD_Init(LCDaddr, tx_buffer);
 	uint8_t tx_buffer[4];
 	const uint8_t LCDaddr = 0x27 << 1;

 	// Write data
 	char data[] = "Ruslan A";
 	writeDataLCD(LCDaddr, tx_buffer, data);


-- Interrupt logic use --
 *  Globally initialize
 	I2C_HandleTypeDef hi2c2;
	LCD_HandleTypeDef lcd;

 * In the main function:
	// Initialization
	LCD_Init(LCDaddr, tx_buffer);
	lcd.LCD_addr = 0x27 << 1;

	// Write data
	char data[] = "Hello World!";
	writeDataLCD_IT(data);
