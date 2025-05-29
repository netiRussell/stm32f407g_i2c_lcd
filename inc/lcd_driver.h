#ifndef __LCD_DRIVER_H
#define __LCD_DRIVER_H

#include "main.h"
#include <stdint.h>

// Polling
void prepareBuffer( uint8_t* buffer_ptr, uint8_t data, uint8_t rs, uint8_t rw );
void pollingMasterSend(uint8_t LCDaddr, uint8_t* tx_buffer_ptr, uint8_t data_size, uint8_t delay_ms);
void pollingMasterRead(uint8_t LCDaddr, uint8_t* rx_buffer_ptr, uint8_t delay_ms);
void waitBFgoLow(uint8_t LCDaddr);
void LCD_Init (uint8_t LCDaddr, uint8_t* tx_buffer_ptr);
void writeDataLCD(uint8_t LCDaddr, uint8_t* tx_buffer_ptr, char* data);

// Interrupt-based
typedef enum {
	SEND_CHAR,
	ENABLE_READ,
	POLL_BF,
	FINISHED
} LCD_State;

typedef struct {
	LCD_State state;
	uint8_t LCD_addr;
	uint8_t tx_buffer[4];
	uint8_t rx_buffer;
	char* data;
} LCD_HandleTypeDef;

void writeDataLCD_IT(char* data);

#endif
