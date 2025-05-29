#include "lcd_driver.h"

extern I2C_HandleTypeDef hi2c2;

/*
 * -- Polling logic use --
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


 * -- Interrupt logic use --
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
 */


/*
  Sends data to LCD by makeing sure that I2C bus is ready
  and polling I2C master send. Then, delayes for "delay_ms" milliseconds

  Parameters:
  ----------
  LCDaddr: 8 bit unsigned integer
    I2C slave address of LCD

  rx_buffer_ptr: 8 bit unsigned integer pointer
    Pointer to an array where the data received will be stored

  data_size: 8 bit unsigned integer
    Number of data bytes to be sent

  delay_ms: 8 bit unsigned integer
    Number of milliseconds to wait for after the I2C master receive function


  Returns:
  -------
  Void
*/
void pollingMasterSend(uint8_t LCDaddr, uint8_t* tx_buffer_ptr, uint8_t data_size, uint8_t delay_ms){
	// Wait until the bus is ready
	while (HAL_I2C_GetState(&hi2c2) != HAL_I2C_STATE_READY)
	{
	}

	// TODO: delete
	// ! TODO: check if any error is returned in the first place
	int hal_status = HAL_I2C_Master_Transmit(&hi2c2, LCDaddr, tx_buffer_ptr, data_size, HAL_MAX_DELAY);
	if (hal_status != HAL_OK)
	{
		// TODO: delete
	  int dummy = 1;
	}

	HAL_Delay(delay_ms);
}


/*
  Reads data from LCD by makeing sure that I2C bus is ready
  and polling I2C master receive. Then, delayes for "delay_ms" milliseconds

  Parameters:
  ----------
  LCDaddr: 8 bit unsigned integer
    I2C slave address of LCD

  rx_buffer_ptr: 8 bit unsigned integer pointer
    Pointer to an array where the data received will be stored

  delay_ms: 8 bit unsigned integer
    Number of milliseconds to wait for after the I2C master receive function


  Returns:
  -------
  Void
*/
void pollingMasterRead(uint8_t LCDaddr, uint8_t* rx_buffer_ptr, uint8_t delay_ms){
	// Wait until the bus is ready
	while (HAL_I2C_GetState(&hi2c2) != HAL_I2C_STATE_READY)
	{
	}

	// TODO: delete
	// ! TODO: check if any error is returned in the first place
	int hal_status = HAL_I2C_Master_Receive(&hi2c2, LCDaddr, rx_buffer_ptr, 1, HAL_MAX_DELAY);
	if (hal_status != HAL_OK)
	{
	  int dummy = 1;
	}

	HAL_Delay(delay_ms);
}


/*
  Formats 1 byte of data into 4 bytes for proper transmission

  Parameters:
  ----------
  tx_buffer_ptr: 8 bit unsigned integer pointer
    Pointer to an array where the next data byte is stored in
    a structure required by LCD (4 bytes where 4 msb bits are
    4 bits of data while 4 lsb bits are logic info to manage
    pins like R/W, E, R/S)

  data: 8 bit unsigned integer
    data byte that will be restructured

  rs: 8 bit unsigned integer
    value for R/S pin; 0 = Command related, 1 = Data related

  rw: 8 bit unsigned integer
    values for R/W pin; 0 = Write, 1 = Read

  Returns:
  -------
  Void
*/
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


/*
  Initializes LCD in accordance with the datasheet for LCD 1602

  Parameters:
  ----------
  LCDaddr: 8 bit unsigned integer
    I2C slave address of LCD

  tx_buffer_ptr: 8 bit unsigned integer pointer
    Pointer to an array where the next data byte is stored in
    a structure required by LCD (4 bytes where 4 msb bits are
    4 bits of data while 4 lsb bits are logic info to manage
    pins like R/W, E, R/S)

  Returns:
  -------
  Void
*/
void LCD_Init (uint8_t LCDaddr, uint8_t* tx_buffer_ptr){
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


/*
  Writes data to LCD by polling. First, data is restructured,
  then, it is sent via polling I2C and finally polling read
  takes place to make sure LCD is ready for the next data reception.

  Parameters:
  ----------
  LCDaddr: 8 bit unsigned integer
    I2C slave address of LCD

  tx_buffer_ptr: 8 bit unsigned integer pointer
    Pointer to an array where the next data byte is stored in
    a structure required by LCD (4 bytes where 4 msb bits are
    4 bits of data while 4 lsb bits are logic info to manage
    pins like R/W, E, R/S)

  data: char pointer
    pointer to a char array (string) content of which to be displayed on LCD


  Returns:
  -------
  Void
*/
void writeDataLCD(uint8_t LCDaddr, uint8_t* tx_buffer_ptr, char* data){

	while(*data != '\0'){
		prepareBuffer(tx_buffer_ptr, *data, 1, 0);
		pollingMasterSend(LCDaddr, tx_buffer_ptr, (uint8_t) 4, (uint8_t) 500);

		waitBFgoLow(LCDaddr);

		data++;
	}

}


/*
  Waits until LCD is ready to receive more data by enabling the read pin and
  constantly polling the MSB bit until it is equal to logic low(0)

  Parameters:
  ----------
  LCDaddr: 8 bit unsigned integer
    I2C slave address of LCD


  Returns:
  -------
  Void
*/
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

/*
  Writes data to LCD in an interrupt based manner with the use of a state machine.

  - It starts by sending first character under the SEND_CHAR state,
  in the HAL_I2C_MasterTxCpltCallback fuction in the SEND_CHAR state
  another interrupt master i2c send command takes place to enable read pin on the LCD;
  also, the state is transitioned to ENABLE_READ.
  - In the HAL_I2C_MasterTxCpltCallback fuction in the ENABLE_READ state
  the state is transitioned to POLL_BF and first master read takes place.
  - In the HAL_I2C_MasterRxCpltCallback function in the POLL_BF state,
  the program checks if LCD is ready to accept next string.
  If not => BF flag is polled again
  If yes => next data byte is transmitted and the state is transitioned to SEND_CHAR.
  As soon as the final data byte arrives = '\0', the state is transitioned to FINISHED
  and the data transimission is considered complete.


  Parameters:
  ----------
  data: char pointer
    pointer to a char array (string) content of which to be displayed on LCD


  Returns:
  -------
  Void
*/
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

