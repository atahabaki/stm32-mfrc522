/*
 *  STM32 MFRC522 Driver - STM32 HAL Library for MFRC522 RFID Card Reader
 *  Copyright (C) 2022 A. Taha Baki
 *
 *  This file is part of STM32 MFRC522 Driver.
 *
 *  STM32 MFRC522 Driver is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  STM32 MFRC522 Driver is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with STM32 MFRC522 Driver.  If not, see <https://www.gnu.org/licenses/>.
 */

#include <hal_mfrc522.h>

MFRC522_Status HAL_MFRC522_WriteRegister(MFRC522 *rfid, MFRC522_Reg addr, uint8_t data) {
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET); // Select the MFRC522 chip..
  uint8_t _addr = addr << 1;
	HAL_SPI_Transmit(rfid->hspi, &_addr, 1, HAL_MFRC522_DEFAULT_TIMEOUT); // Send address first...
	HAL_SPI_Transmit(rfid->hspi, &data, 1, HAL_MFRC522_DEFAULT_TIMEOUT); // Then, send the data...
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET); // Disable the MFRC522 chip..
	return RC522_OK;
}

uint8_t HAL_MFRC522_ReadRegister(MFRC522 *rfid, MFRC522_Reg addr) {
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET); // Select the MFRC522 chip..
	//HAL_SPI_Transmit(rfid->hspi, &addr, 1, HAL_MFRC522_DEFAULT_TIMEOUT); // Send address first...
	uint8_t data[2];
  uint8_t _addr = (addr << 1 )| 0x80;
	data[0] = _addr;
	HAL_SPI_Receive(rfid->hspi, data, 2, HAL_MFRC522_DEFAULT_TIMEOUT); // Then, send the data...
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET); // Disable the MFRC522 chip..
	return data[1];
}