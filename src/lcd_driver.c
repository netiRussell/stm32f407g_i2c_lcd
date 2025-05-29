#include "lcd_driver.h"

extern I2C_HandleTypeDef hi2c2;

/*
 * -- Polling logic use --
 * Globally initialize:
    I2C_HandleTypeDef hi2c2;

 * In the main function:
 	// Initialization
 	LCD_Init(tx_buffer, LCDaddr);
 	uint8_t tx_buffer[4];
 	const uint8_t LCDaddr = 0x27 << 1;

 	// Write data
 	char data[] = "Ruslan A";
 	writeDataLCD(LCDaddr, tx_buffer, data);


 * -- Interrupt logic use --
 *  Globally initialize
 	I2C_HandleTypeDef hi2c2;
	LCD_HandleTypeDef lcd;

 * In the main function:
	// Initialization
	LCD_Init(tx_buffer, LCDaddr);
	lcd.LCD_addr = 0x27 << 1;

	// Write data
	char data[] = "Hello World!";
	writeDataLCD_IT(data);
 */



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

void pollingMasterRead(uint8_t LCDaddr, uint8_t* rx_buffer, uint8_t delay_ms){
	// Wait until the bus is ready
	while (HAL_I2C_GetState(&hi2c2) != HAL_I2C_STATE_READY)
	{
	}

	// TODO: delete
	// ! TODO: check if any error is returned in the first place
	int hal_status = HAL_I2C_Master_Receive(&hi2c2, LCDaddr, rx_buffer, 1, HAL_MAX_DELAY);
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

void LCD_Init (uint8_t* tx_buffer_ptr, uint8_t LCDaddr){
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
	waitBFgoLow(LCDaddr);

	// -- Display initialization --
	// Function set --> DL=0 (4 bit mode), N = 0 (1 line display) F = 1 (5x10 characters)
	prepareBuffer(tx_buffer_ptr, 0x24, 0, 0);
	pollingMasterSend(LCDaddr, tx_buffer_ptr, (uint8_t) 4, (uint8_t) 1); // wait for 1ms
	waitBFgoLow(LCDaddr);

	// -- Display on/off control --> D=0,C=0, B=0  ---> display off --
	prepareBuffer(tx_buffer_ptr, 0x08, 0, 0);
	pollingMasterSend(LCDaddr, tx_buffer_ptr, (uint8_t) 4, (uint8_t) 1); // wait for 1ms
	waitBFgoLow(LCDaddr);

    // -- Clear display --
	prepareBuffer(tx_buffer_ptr, 0x01, 0, 0);
	pollingMasterSend(LCDaddr, tx_buffer_ptr, (uint8_t) 4, (uint8_t) 2); // wait for 2ms
	waitBFgoLow(LCDaddr);

	// -- Entry mode set --> I/D = 1 (increment cursor) & S = 0 (no shift) --
	prepareBuffer(tx_buffer_ptr, 0x06, 0, 0);
	pollingMasterSend(LCDaddr, tx_buffer_ptr, (uint8_t) 4, (uint8_t) 1); // wait for 1ms
	waitBFgoLow(LCDaddr);

	// -- Display on/off control --> D = 1, C and B = 0. (Cursor and blink, last two bits) --
	prepareBuffer(tx_buffer_ptr, 0x0C, 0, 0);
	pollingMasterSend(LCDaddr, tx_buffer_ptr, (uint8_t) 4, (uint8_t) 1); // wait for 1ms
	waitBFgoLow(LCDaddr);

	// -- Set cursor at: row 0, column 0 --
	prepareBuffer(tx_buffer_ptr, 0x80, 0, 0);
	pollingMasterSend(LCDaddr, tx_buffer_ptr, (uint8_t) 4, (uint8_t) 2); // wait for 2ms
	waitBFgoLow(LCDaddr);
}

void writeDataLCD(uint8_t LCDaddr, uint8_t* tx_buffer, char* data){

	while(*data != '\0'){
		prepareBuffer(tx_buffer, *data, 1, 0);
		pollingMasterSend(LCDaddr, tx_buffer, (uint8_t) 4, (uint8_t) 500);

		waitBFgoLow(LCDaddr);

		data++;
	}

}

void waitBFgoLow(uint8_t LCDaddr){
	uint8_t enableRead = 0b00001010;
	pollingMasterSend(LCDaddr, &enableRead, (uint8_t) 1, (uint8_t) 1);

	uint8_t rx_buffer = 0;

	do{
		pollingMasterRead(LCDaddr, &rx_buffer, (uint8_t) 0);
	} while(rx_buffer & 0b1 << 7);
}


/* -- Interrupt-based logic -- */
extern LCD_HandleTypeDef lcd;

void writeDataLCD_IT(char* data){
	// Make sure data is not empty
	if(*data == '\0'){
		return;
	}

	// Reset the LCD state machine
	lcd.state = SEND_CHAR;
	lcd.data = data;
	lcd.rx_buffer = 0;

	// Prepare data
	prepareBuffer(lcd.tx_buffer, *lcd.data, 1, 0);

	// Run the state machine. Initial data transfer
	HAL_I2C_Master_Transmit_IT(&hi2c2, lcd.LCD_addr, lcd.tx_buffer, (uint8_t) 4);
}

