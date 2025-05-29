# stm32f407g_i2c_lcd
 Sending data to LCD1602A from STM32F407G via I2C

# How to use:
-- Polling logic use --
 * Globally initialize:<br />
    I2C_HandleTypeDef hi2c2;

 * In the main function:<br />
 	// Initialization<br />
	LCD_Init(LCDaddr, tx_buffer);<br />
 	uint8_t tx_buffer[4];<br />
 	const uint8_t LCDaddr = 0x27 << 1;<br />

 	// Write data<br />
 	char data[] = "Ruslan A";<br />
 	writeDataLCD(LCDaddr, tx_buffer, data);

<br />

-- Interrupt logic use --
 *  Globally initialize<br />
 	I2C_HandleTypeDef hi2c2;<br />
	LCD_HandleTypeDef lcd;

 * In the main function:<br />
	// Initialization<br />
	LCD_Init(LCDaddr, tx_buffer);<br />
	lcd.LCD_addr = 0x27 << 1;

	// Write data<br />
	char data[] = "Hello World!";<br />
	writeDataLCD_IT(data);
