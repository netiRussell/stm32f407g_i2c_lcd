#ifndef __LCD_DRIVER_H
#define __LCD_DRIVER_H

#include "main.h"
#include <stdint.h>

void prepareBuffer( uint8_t* buffer_ptr, uint8_t data, uint8_t rs, uint8_t rw );
void pollingMasterSend(uint8_t LCDaddr, uint8_t* tx_buffer, uint8_t data_size, uint8_t delay_ms);
void pollingMasterRead(uint8_t LCDaddr, uint8_t* buffer, uint8_t delay_ms);
void waitBFgoLow(uint8_t LCDaddr);
void LCD_Init (uint8_t* tx_buffer_ptr, uint8_t LCDaddr);
void writeDataLCD(uint8_t LCDaddr, uint8_t* tx_buffer, char* data);

#endif
