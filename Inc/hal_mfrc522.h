/*
 *  STM32 MFRC522 Driver - An android application for those who aims to learn a new language.
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

#ifndef HAL_MFRC522_H
#define HAL_MFRC522_H

#include <stdint.h>
// Change this hal include according to your device...
#include <stm32f7xx_hal.h>

typedef struct {
	GPIO_TypeDef* Port;
	uint16_t Pin;
} GPIO_Pin;

typedef struct {
	GPIO_Pin ss_pin;
	SPI_HandleTypeDef *hspi;
} MFRC522;

typedef enum {
  // Page 0...
  CommandReg = 0x02,
  ComlEnReg = 0x04,
  DivlEnReg = 0x06,
  ComIrqReg = 0x08,
  DivIrqReg = 0x0A,
  ErrorReg = 0x0C,
  Status1Reg = 0x0E,
  Status2Reg = 0x10,
  FIFODataReg = 0x12,
  FIFOLevelReg = 0x14,
  WaterLevelReg = 0x16,
  ControlReg = 0x18,
  BitFramingReg = 0x1A,
  CollReg = 0x1C,

  // Page 1...
  ModeReg = 0x22,
  TxModeReg = 0x24,
  RxModeReg = 0x26,
  TxControlReg = 0x28,
  TxASKReg = 0x2A,
  TxSelReg = 0x2C,
  RxSelReg = 0x2E,
  RxThresholdReg = 0x30,
  DemodReg = 0x32,
  MfTxReg = 0x38,
  MfRxReg = 0x3A,
  SerialSpeedReg = 0x3E,

  // Page 2...
  CRCResultRegH = 0x42,
  CRCResultRegL = 0x44,
  ModWidthReg = 0x48,
  RFCfgReg = 0x4C,
  GsNReg = 0x4E,
  CWGsPReg = 0x50,
  ModGsPReg = 0x52,
  TModeReg = 0x54,
  TPrescalerReg = 0x56,
  TReloadRegH = 0x58,
  TReloadRegL = 0x5A,
  TCounterValReg = 0x5C,

  // Page 3...
  TestSel1Reg = 0x62,
  TestSel2Reg = 0x64,
  TestPinEnReg = 0x66,
  TestPinValueReg = 0x68,
  TestBusReg = 0x6A,
  AutoTestReg = 0x6C,
  VersionReg = 0x6E,
  AnalogTestReg = 0x70,
  TestDAC1Reg = 0x72,
  TestDAC2Reg = 0x74,
  TestADCReg = 0x76,
} MFRC522_Reg;

#endif
