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

MFRC522_Status HAL_MFRC522_Reset(MFRC522 *rfid) {
	// 1. Write "SoftReset" into CommandReg register...
	HAL_MFRC522_WriteRegister(rfid, CommandReg, SoftReset);
	// 2. check PowerDownBit is set to 1...
	uint8_t data;
	uint8_t countTries = 0;
	do {
		HAL_Delay(50);
		data = HAL_MFRC522_ReadRegister(rfid, READ_CommandReg);
	} while((data & (1 << 4)) && ((++countTries) < 3 /* Timeout after 3 tries. */));

	// Try 2. step 3 times... otherwise return Timeout...
	return RC522_OK;
}

MFRC522_Status HAL_MFRC522_SetBitMask(MFRC522 *rfid, MFRC522_Reg addr, uint8_t mask) {
  uint8_t w_addr = addr << 1;
  uint8_t r_addr = _addr | 0x80;
	uint8_t tmp = HAL_MFRC522_ReadRegister(rfid, r_addr);
	return HAL_MFRC522_WriteRegister(rfid, w_addr, tmp | mask);
}

MFRC522_Status HAL_MFRC522_ClearBitMask(MFRC522 *rfid, MFRC522_Reg addr, uint8_t mask) {
  uint8_t w_addr = addr << 1;
  uint8_t r_addr = _addr | 0x80;
	uint8_t tmp = HAL_MFRC522_ReadRegister(rfid, r_addr);
	return HAL_MFRC522_WriteRegister(rfid, w_addr, tmp & (~mask));
}

MFRC522_Status HAL_MFRC522_Open_Antenna(MFRC522 *rfid) {
	#if TEST_WRITE
		uint8_t _prev = HAL_MFRC522_ReadRegister(rfid, TxControlReg);
	#endif
	MFRC522_Status status = HAL_MFRC522_SetBitMask(rfid, TxControlReg, 0x03);
	#if TEST_WRITE
		uint8_t _after = HAL_MFRC522_ReadRegister(rfid, TxControlReg);
		uint8_t _should = _prev | 0x03;
		if (_after != _should)
			return RC522_ERR;
	#endif
	return status;
}

MFRC522_Status HAL_MFRC522_Close_Antenna(MFRC522 *rfid) {
	#if TEST_WRITE
		uint8_t _prev = HAL_MFRC522_ReadRegister(rfid, TxControlReg);
	#endif
	MFRC522_Status status = HAL_MFRC522_ClearBitMask(rfid, TxControlReg, 0x03);
	#if TEST_WRITE
		uint8_t _after = HAL_MFRC522_ReadRegister(rfid, TxControlReg);
		uint8_t _should = _prev & ~(0x03);
		if (_after != _should)
			return RC522_ERR;
	#endif
	return status;
}