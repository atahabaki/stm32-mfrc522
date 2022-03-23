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

MFRC522_Status HAL_MFRC522_WriteRegister_Multi(MFRC522 *rfid, MFRC522_Reg addr, uint16_t count, uint8_t *data) {
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET); // Select the MFRC522 chip..
  uint8_t _addr = addr << 1;
	HAL_SPI_Transmit(rfid->hspi, &_addr, 1, HAL_MFRC522_DEFAULT_TIMEOUT); // Send address first...
	HAL_SPI_Transmit(rfid->hspi, data, count, HAL_MFRC522_DEFAULT_TIMEOUT); // Then, send the data...
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
	HAL_MFRC522_WriteRegister(rfid, CommandReg, SoftReset|0x10);
	// 2. check PowerDownBit is set to 1...
	uint8_t data;
	uint8_t countTries = 0;
  while (countTries < 3) {
		data = HAL_MFRC522_ReadRegister(rfid, CommandReg);
    if (data == 16)
      return RC522_OK;
		HAL_Delay(50);
    countTries++;
  }

	// Try 2. step 3 times... otherwise return Timeout...
	return RC522_TIMEOUT;
}

MFRC522_Status HAL_MFRC522_SoftPowerDown(MFRC522 *rfid) {
  HAL_MFRC522_WriteRegister(rfid, CommandReg, 0x10);
  #if TEST_WRITE
    uint8_t is_powered_down = HAL_MFRC522_ReadRegister(rfid, CommandReg);
    if ((is_powered_down & 0x10) != 0x10)
      return RC522_ERR;
  #endif
  return RC522_OK;
}

MFRC522_Status HAL_MFRC522_SoftPowerUp(MFRC522 *rfid) {
  uint8_t val = HAL_MFRC522_ReadRegister(rfid, CommandReg);
  val &= ~0x10;
  HAL_MFRC522_WriteRegister(rfid, CommandReg, val);
  uint8_t is_powered_down = HAL_MFRC522_ReadRegister(rfid, CommandReg);
  uint32_t timeout = HAL_GetTick() + HAL_MFRC522_DEFAULT_TIMEOUT;
  while (HAL_GetTick() <= timeout) {
    if (!(is_powered_down & 0x10)) {
      return RC522_OK;
    }
  }
  return RC522_TIMEOUT;
}

MFRC522_Status HAL_MFRC522_SetBitMask(MFRC522 *rfid, MFRC522_Reg addr, uint8_t mask) {
	uint8_t tmp = HAL_MFRC522_ReadRegister(rfid, addr);
	return HAL_MFRC522_WriteRegister(rfid, addr, tmp | mask);
}

MFRC522_Status HAL_MFRC522_ClearBitMask(MFRC522 *rfid, MFRC522_Reg addr, uint8_t mask) {
	uint8_t tmp = HAL_MFRC522_ReadRegister(rfid, addr);
	return HAL_MFRC522_WriteRegister(rfid, addr, tmp & (~mask));
}

uint8_t HAL_MFRC522_GetAntennaGain(MFRC522 *rfid) {
  return HAL_MFRC522_ReadRegister(rfid, RFCfgReg) & (0x07<<4);
}

MFRC522_Status HAL_MFRC522_SetAntennaGain(MFRC522 *rfid, uint8_t mask) {
  if (HAL_MFRC522_GetAntennaGain(rfid) != mask) {
    HAL_MFRC522_ClearBitMask(rfid, RFCfgReg, (0x07<<4));
    HAL_MFRC522_SetBitMask(rfid, RFCfgReg, mask & (0x07<<4));
  }
  return RC522_OK;
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

MFRC522_Status HAL_MFRC522_Init(MFRC522 *rfid) {
	// First Reset...
	HAL_MFRC522_Reset(rfid);
	// Reset baudrates...
	HAL_MFRC522_WriteRegister(rfid, TxModeReg, 0x00);
	#if TEST_WRITE
		if (HAL_MFRC522_ReadRegister(rfid, TxModeReg) != 0x00)
			return RC522_ERR;
	#endif
	HAL_MFRC522_WriteRegister(rfid, RxModeReg, 0x00);
	#if TEST_WRITE
		if (HAL_MFRC522_ReadRegister(rfid, RxModeReg) != 0x00)
			return RC522_ERR;
	#endif
	// Reset ModWidthReg...
	HAL_MFRC522_WriteRegister(rfid, ModWidthReg, 0x26);
	#if TEST_WRITE
		if (HAL_MFRC522_ReadRegister(rfid, ModWidthReg) != 0x26)
			return RC522_ERR;
	#endif
	// Timer...
	HAL_MFRC522_WriteRegister(rfid, TModeReg, 0x80);
	#if TEST_WRITE
		if (HAL_MFRC522_ReadRegister(rfid, TModeReg) != 0x80)
			return RC522_ERR;
	#endif
	HAL_MFRC522_WriteRegister(rfid, TPrescalerReg, 0xA9);
	#if TEST_WRITE
		if (HAL_MFRC522_ReadRegister(rfid, TPrescalerReg) != 0xA9)
			return RC522_ERR;
	#endif
	HAL_MFRC522_WriteRegister(rfid, TReloadRegH, 0x03);
	#if TEST_WRITE
		if (HAL_MFRC522_ReadRegister(rfid, TReloadRegH) != 0x03)
			return RC522_ERR;
	#endif
	HAL_MFRC522_WriteRegister(rfid, TReloadRegL, 0xE8);
	#if TEST_WRITE
		if (HAL_MFRC522_ReadRegister(rfid, TReloadRegL) != 0xE8)
			return RC522_ERR;
	#endif
	HAL_MFRC522_WriteRegister(rfid, TxASKReg, 0x40);
	#if TEST_WRITE
		if (HAL_MFRC522_ReadRegister(rfid, TxASKReg) != 0x40)
			return RC522_ERR;
	#endif
	HAL_MFRC522_WriteRegister(rfid, ModeReg, 0x3D);
	#if TEST_WRITE
		if (HAL_MFRC522_ReadRegister(rfid, ModeReg) != 0x3D)
			return RC522_ERR;
	#endif

	HAL_MFRC522_Open_Antenna(rfid);
	HAL_Delay(4);

	uint8_t version = HAL_MFRC522_ReadRegister(rfid, VersionReg);
	if (version == 0x91 || version == 0x92)
		return RC522_OK;
	else return RC522_UNKNOWN_BOARD;
}
