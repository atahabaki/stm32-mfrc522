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

MFRC522_Status HAL_MFRC522_WriteRegister(MFRC522 *rfid, MFRC522_Reg addr, u8 data) {
	HAL_GPIO_WritePin(rfid->ss_pin.Port, rfid->ss_pin.Pin, GPIO_PIN_RESET); // Select the MFRC522 chip..
  u8 _addr = addr << 1;
	HAL_SPI_Transmit(rfid->hspi, &_addr, 1, HAL_MFRC522_DEFAULT_TIMEOUT); // Send address first...
	HAL_SPI_Transmit(rfid->hspi, &data, 1, HAL_MFRC522_DEFAULT_TIMEOUT); // Then, send the data...
	HAL_GPIO_WritePin(rfid->ss_pin.Port, rfid->ss_pin.Pin, GPIO_PIN_SET); // Disable the MFRC522 chip..
	return RC522_OK;
}

MFRC522_Status HAL_MFRC522_WriteRegister_Multi(MFRC522 *rfid, MFRC522_Reg addr, uint16_t count, u8 *data) {
	HAL_GPIO_WritePin(rfid->ss_pin.Port, rfid->ss_pin.Pin, GPIO_PIN_RESET); // Select the MFRC522 chip..
  u8 _addr = addr << 1;
	HAL_SPI_Transmit(rfid->hspi, &_addr, 1, HAL_MFRC522_DEFAULT_TIMEOUT); // Send address first...
	HAL_SPI_Transmit(rfid->hspi, data, count, HAL_MFRC522_DEFAULT_TIMEOUT); // Then, send the data...
	HAL_GPIO_WritePin(rfid->ss_pin.Port, rfid->ss_pin.Pin, GPIO_PIN_SET); // Disable the MFRC522 chip..
	return RC522_OK;
}

u8 HAL_MFRC522_ReadRegister(MFRC522 *rfid, MFRC522_Reg addr) {
  u8 value;
	HAL_GPIO_WritePin(rfid->ss_pin.Port, rfid->ss_pin.Pin, GPIO_PIN_RESET); // Select the MFRC522 chip..
  u8 _addr = (addr << 1) | 0x80;
  HAL_SPI_Transmit(rfid->hspi, &_addr, 1, HAL_MFRC522_DEFAULT_TIMEOUT);
  // Read the value back. Send 0 to stop reading.
  HAL_SPI_TransmitReceive(rfid->hspi, &zero, &value, 1, HAL_MFRC522_DEFAULT_TIMEOUT);
	HAL_GPIO_WritePin(rfid->ss_pin.Port, rfid->ss_pin.Pin, GPIO_PIN_SET); // Disable the MFRC522 chip..
	return value;
}

void HAL_MFRC522_ReadRegister_Multi(MFRC522 *rfid, MFRC522_Reg addr, u8 count, u8 *values, u8 rxAlign) {
  if (count == 0) { return; }
  u8 address = (addr << 1) | 0x80;  // MSB == 1 is for reading. LSB is not used in address. Datasheet section 8.1.2.3.
  u8 index = 0;             // Index in values array.
  u8 zero = 0;
	HAL_GPIO_WritePin(rfid->ss_pin.Port, rfid->ss_pin.Pin, GPIO_PIN_RESET); // Select the MFRC522 chip..
  count--;                       // One read is performed outside of the loop
  //  (void) m_SPI.write(address);   // Tell MFRC522 which address we want to read
  HAL_SPI_Transmit(rfid->hspi, &address, 1, HAL_MFRC522_DEFAULT_TIMEOUT);
  while (index < count)
  {
    if ((index == 0) && rxAlign) // Only update bit positions rxAlign..7 in values[0]
    {
      // Create bit mask for bit positions rxAlign..7
      u8 mask = 0;
      for (u8 i = rxAlign; i <= 7; i++)
        mask |= (1 << i);
      // Read value and tell that we want to read the same address again.
      // u8 value = m_SPI.write(address);
      u8 value;
      HAL_SPI_TransmitReceive(rfid->hspi, &address, &value, 1, HAL_MFRC522_DEFAULT_TIMEOUT);
      // Apply mask to both current value of values[0] and the new data in value.
      values[0] = (values[index] & ~mask) | (value & mask);
    }
    // Read value and tell that we want to read the same address again.
    // values[index] = m_SPI.write(address);
    else
      HAL_SPI_TransmitReceive(rfid->hspi, &address, &(values[index]), 1, HAL_MFRC522_DEFAULT_TIMEOUT);
    index++;
  }
  //  values[index] = m_SPI.write(0); // Read the final byte. Send 0 to stop reading.
  HAL_SPI_TransmitReceive(rfid->hspi, &zero, &(values[index]), 1, HAL_MFRC522_DEFAULT_TIMEOUT);
	HAL_GPIO_WritePin(rfid->ss_pin.Port, rfid->ss_pin.Pin, GPIO_PIN_SET); // Disable the MFRC522 chip..
}

MFRC522_Status HAL_MFRC522_Reset(MFRC522 *rfid) {
	// 1. Write "SoftReset" into CommandReg register...
	HAL_MFRC522_WriteRegister(rfid, CommandReg, SoftReset|0x10);
	// 2. check PowerDownBit is set to 1...
	u8 data;
	u8 countTries = 0;
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
    u8 is_powered_down = HAL_MFRC522_ReadRegister(rfid, CommandReg);
    if ((is_powered_down & 0x10) != 0x10)
      return RC522_ERR;
  #endif
  return RC522_OK;
}

MFRC522_Status HAL_MFRC522_SoftPowerUp(MFRC522 *rfid) {
  u8 val = HAL_MFRC522_ReadRegister(rfid, CommandReg);
  val &= ~0x10;
  HAL_MFRC522_WriteRegister(rfid, CommandReg, val);
  u8 is_powered_down = HAL_MFRC522_ReadRegister(rfid, CommandReg);
  uint32_t timeout = HAL_GetTick() + HAL_MFRC522_DEFAULT_TIMEOUT;
  while (HAL_GetTick() <= timeout) {
    if (!(is_powered_down & 0x10)) {
      return RC522_OK;
    }
  }
  return RC522_TIMEOUT;
}

MFRC522_Status HAL_MFRC522_SetBitMask(MFRC522 *rfid, MFRC522_Reg addr, u8 mask) {
	u8 tmp = HAL_MFRC522_ReadRegister(rfid, addr);
	return HAL_MFRC522_WriteRegister(rfid, addr, tmp | mask);
}

MFRC522_Status HAL_MFRC522_ClearBitMask(MFRC522 *rfid, MFRC522_Reg addr, u8 mask) {
	u8 tmp = HAL_MFRC522_ReadRegister(rfid, addr);
	return HAL_MFRC522_WriteRegister(rfid, addr, tmp & (~mask));
}

u8 HAL_MFRC522_GetAntennaGain(MFRC522 *rfid) {
  return HAL_MFRC522_ReadRegister(rfid, RFCfgReg) & (0x07<<4);
}

MFRC522_Status HAL_MFRC522_SetAntennaGain(MFRC522 *rfid, u8 mask) {
  if (HAL_MFRC522_GetAntennaGain(rfid) != mask) {
    HAL_MFRC522_ClearBitMask(rfid, RFCfgReg, (0x07<<4));
    HAL_MFRC522_SetBitMask(rfid, RFCfgReg, mask & (0x07<<4));
  }
  return RC522_OK;
}

MFRC522_Status HAL_MFRC522_Open_Antenna(MFRC522 *rfid) {
	#if TEST_WRITE
		u8 _prev = HAL_MFRC522_ReadRegister(rfid, TxControlReg);
	#endif
	MFRC522_Status status = HAL_MFRC522_SetBitMask(rfid, TxControlReg, 0x03);
	#if TEST_WRITE
		u8 _after = HAL_MFRC522_ReadRegister(rfid, TxControlReg);
		u8 _should = _prev | 0x03;
		if (_after != _should)
			return RC522_ERR;
	#endif
	return status;
}

MFRC522_Status HAL_MFRC522_Close_Antenna(MFRC522 *rfid) {
	#if TEST_WRITE
		u8 _prev = HAL_MFRC522_ReadRegister(rfid, TxControlReg);
	#endif
	MFRC522_Status status = HAL_MFRC522_ClearBitMask(rfid, TxControlReg, 0x03);
	#if TEST_WRITE
		u8 _after = HAL_MFRC522_ReadRegister(rfid, TxControlReg);
		u8 _should = _prev & ~(0x03);
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

	u8 version = HAL_MFRC522_ReadRegister(rfid, VersionReg);
	if (version == 0x91 || version == 0x92)
		return RC522_OK;
	else return RC522_UNKNOWN_BOARD;
}

MFRC522_Status HAL_MFRC522_CalculateCRC(MFRC522 *rfid, u8 *data, u8 length, u8 *result) {
  HAL_MFRC522_WriteRegister(rfid, CommandReg, Idle);
  HAL_MFRC522_WriteRegister(rfid, DivIrqReg, 0x04);
  HAL_MFRC522_WriteRegister(rfid, FIFOLevelReg, 0x80);
  HAL_MFRC522_WriteRegister_Multi(rfid, FIFODataReg, length, data);
  HAL_MFRC522_WriteRegister(rfid, CommandReg, CalcCRC);
  for (uint16_t i = 5000; i > 0; i--) {
    u8 n = HAL_MFRC522_ReadRegister(rfid, DivIrqReg);
    if (n & 0x04) {
      // Stop calculation...
      HAL_MFRC522_WriteRegister(rfid, CommandReg, Idle);
      result[0] = HAL_MFRC522_ReadRegister(rfid, CRCResultRegL);
      result[1] = HAL_MFRC522_ReadRegister(rfid, CRCResultRegH);
      return RC522_OK;
    }
  }
  return RC522_TIMEOUT;
}

MFRC522_Status HAL_MFRC522_CommunicatePICC(MFRC522 *rfid, u8 command, u8 waitIRq, u8 *sendData, u8 sendLen, u8 *backData, u8 *backLen, u8 *validBits, u8 rxAlign, bool checkCRC) {
  u8 txLastBits = validBits ? *validBits : 0;
  u8 bitFraming = (rxAlign << 4)+txLastBits;

  HAL_MFRC522_WriteRegister(rfid, CommandReg, Idle);      // Stop any active command.
  HAL_MFRC522_WriteRegister(rfid, ComIrqReg, 0x7F);          // Clear all seven interrupt request bits
  HAL_MFRC522_WriteRegister(rfid, FIFOLevelReg, 0x80);        // FlushBuffer = 1, FIFO initialization
  HAL_MFRC522_WriteRegister_Multi(rfid, FIFODataReg, sendLen, sendData);  // Write sendData to the FIFO
  HAL_MFRC522_WriteRegister(rfid, BitFramingReg, bitFraming);    // Bit adjustments
  HAL_MFRC522_WriteRegister(rfid, CommandReg, command);        // Execute the command
  if(command == Transceive)
    HAL_MFRC522_SetBitMask(rfid, BitFramingReg, 0x80);  // StartSend=1, transmission of data starts

  uint16_t i;
  for(i = 2000; i > 0; i--) {
    u8 n = HAL_MFRC522_ReadRegister(rfid, ComIrqReg);  // ComIrqReg[7..0] bits are: Set1 TxIRq RxIRq IdleIRq HiAlertIRq LoAlertIRq ErrIRq TimerIRq
    if(n & waitIRq)          // One of the interrupts that signal success has been set.
      break;
    if(n & 0x01)            // Timer interrupt - nothing received in 25ms
      return RC522_TIMEOUT;
  }
  // 35.7ms and nothing happened. Communication with the MFRC522 might be down.
  if(i == 0)
    return RC522_TIMEOUT;
  
  // Stop now if any errors except collisions were detected.
  u8 errorRegValue = HAL_MFRC522_ReadRegister(rfid, ErrorReg); // ErrorReg[7..0] bits are: WrErr TempErr reserved BufferOvfl CollErr CRCErr ParityErr ProtocolErr
  if(errorRegValue & 0x13)   // BufferOvfl ParityErr ProtocolErr
    return RC522_ERR;
  
  u8 _validBits = 0;
  
  // If the caller wants data back, get it from the MFRC522.
  if(backData && backLen) {
    u8 n = HAL_MFRC522_ReadRegister(rfid, FIFOLevelReg);  // Number of bytes in the FIFO
    if(n > *backLen) {
      return RC522_NO_ROOM;
    }
    *backLen = n;                      // Number of bytes returned
    HAL_MFRC522_ReadRegister_Multi(rfid, FIFODataReg, n, backData, rxAlign);  // Get received data from FIFO
    _validBits = HAL_MFRC522_ReadRegister(rfid, ControlReg) & 0x07;    // RxLastBits[2:0] indicates the number of valid bits in the last received byte. If this value is 000b, the whole byte is valid.
    if(validBits)
      *validBits = _validBits;
  }
  
  // Tell about collisions
  if(errorRegValue & 0x08)    // CollErr
    return RC522_COLLISION;
  
  // Perform CRC_A validation if requested.
  if(backData && backLen && checkCRC) {
    // In this case a MIFARE Classic NAK is not OK.
    if(*backLen == 1 && _validBits == 4)
      return RC522_MIFARE_NACK;
    // We need at least the CRC_A value and all 8 bits of the last byte must be received.
    if(*backLen < 2 || _validBits != 0)
      return RC522_CRC_WRONG;
    // Verify CRC_A - do our own calculation and store the control in controlBuffer.
    u8 controlBuffer[2];
    MFRC522_Status status = HAL_MFRC522_CalculateCRC(rfid, &backData[0], *backLen-2, &controlBuffer[0]);
    if(status != RC522_OK)
      return status;
    if((backData[*backLen-2] != controlBuffer[0]) || (backData[*backLen-1] != controlBuffer[1])) 
      return RC522_CRC_WRONG;
  }
  
  return RC522_OK;
}

MFRC522_Status HAL_MFRC522_TransceiveData(MFRC522 *rfid, u8 *sendData, u8 sendLen, u8 *backData, u8 *backLen, u8 *validBits, u8 rxAlign, bool checkCRC) {
  u8 waitIRq = 0x30;
  return HAL_MFRC522_CommunicatePICC(rfid, Transceive, waitIRq, sendData, sendLen, backData, backLen, validBits, rxAlign, checkCRC);
}
