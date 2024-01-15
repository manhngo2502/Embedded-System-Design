/*
 * lcd1602_i2c.h
 *
 *  Created on: Jan 4, 2024
 *      Author: admin
 */




#ifndef INC_LCD1602_I2C_H_
#define INC_LCD1602_I2C_H_
#include <stdbool.h>
#include<stdint.h>
#include<stdlib.h>
#include"stm32f1xx_hal.h"
/* Function prototypes */
/**
 * @brief Initialise LCD16x2
 * @param[in] *pI2cHandle - pointer to HAL I2C handle
 */
bool lcd1602_i2c_init(I2C_HandleTypeDef *pI2cHandle);
void lcd1602_i2c_setCursor(uint8_t row, uint8_t col);
void lcd1602_i2c_1stLine(void);
void lcd1602_i2c_2ndLine(void);
void lcd1602_i2c_TwoLines(void);
void lcd1602_i2c_OneLine(void);
void lcd1602_i2c_cursorShow(bool state);
void lcd1602_i2c_clear(void);
void lcd1602_i2c_sendData(uint8_t data);
void lcd1602_i2c_display(bool state);
void lcd1602_i2c_shiftRight(uint8_t offset);
void lcd1602_i2c_shiftLeft(uint8_t offset);
void lcd1602_i2c_printf(const char* str, ...);



#endif /* INC_LCD1602_I2C_H_ */
