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

#ifndef HAL_MFRC522_H
#define HAL_MFRC522_H

#include <stdint.h>
#include <hal_mfrc522_config.h>
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

  // 00h Reserved reserved for future use Table 21 on page 38
  CommandReg = 0x01, // starts and stops command execution Table 23 on page 38
  ComlEnReg = 0x02,  // enable and disable interrupt request control bits Table
                     // 25 on page 38
  DivlEnReg = 0x03,  // enable and disable interrupt request control bits Table
                     // 27 on page 39
  ComIrqReg = 0x04,  // interrupt request bits Table 29 on page 39
  DivIrqReg = 0x05,  // interrupt request bits Table 31 on page 40
  ErrorReg = 0x06,   // error bits showing the error status of the last command
                     // executed Table 33 on page 41
  Status1Reg = 0x07, // communication status bits Table 35 on page 42
  Status2Reg = 0x08, // receiver and transmitter status bits Table 37 on page 43
  FIFODataReg =
      0x09, // input and output of 64 byte FIFO buffer Table 39 on page 44
  FIFOLevelReg =
      0x0A, // number of bytes stored in the FIFO buffer Table 41 on page 44
  WaterLevelReg =
      0x0B, // level for FIFO underflow and overflow warning Table 43 on page 44
  ControlReg = 0x0C, // miscellaneous control registers Table 45 on page 45
  BitFramingReg =
      0x0D,       // adjustments for bit-oriented frames Table 47 on page 46
  CollReg = 0x0E, // bit position of the first bit-collision detected on the RF
                  // interface Table 49 on page 46
                  // 0Fh Reserved reserved for future use Table 51 on page 47

  // Page 1...

  Reserved = 0x10, // reserved for future use Table 53 on page 47
  ModeReg = 0x11,  // defines general modes for transmitting and receiving Table
                   // 55 on page 48
  TxModeReg =
      0x12, // defines transmission data rate and framing Table 57 on page 48
  RxModeReg =
      0x13, // defines reception data rate and framing Table 59 on page 49
  TxControlReg = 0x14, // controls the logical behavior of the antenna driver
                       // pins TX1 and TX2 Table 61 on page 50
  TxASKReg = 0x15, // controls the setting of the transmission modulation Table
                   // 63 on page 51
  TxSelReg = 0x16, // selects the internal sources for the antenna driver Table
                   // 65 on page 51
  RxSelReg = 0x17, // selects internal receiver settings Table 67 on page 52
  RxThresholdReg =
      0x18,        // selects thresholds for the bit decoder Table 69 on page 53
  DemodReg = 0x19, // defines demodulator settings Table 71 on page 53
  // Reserved = 0x1A, // reserved for future use Table 73 on page 54
  // Reserved = 0x1B, // reserved for future use Table 75 on page 54
  MfTxReg = 0x1C, // controls some MIFARE communication transmit parameters
                  // Table 77 on page 55
  MfRxReg = 0x1D, // controls some MIFARE communication receive parameters Table
                  // 79 on page 55
  // Reserved = 0x1E, // reserved for future use Table 81 on page 55
  SerialSpeedReg = 0x1F, // selects the speed of the serial UART interface Table
                         // 83 on page 55

  // Page 2...

  // Reserved = 0x20, // reserved for future use Table 85 on page 57
  CRCResultRegH = 0x21, // shows the MSB and LSB values of the CRC calculation
                        // Table 87 on page 57
  CRCResultRegL = 0x22, // Table 89 on page 57
  // Reserved = 0x23, // reserved for future use Table 91 on page 58
  ModWidthReg = 0x24, // controls the ModWidth setting Table 93 on page 58
  // Reserved = 0x25, // reserved for future use Table 95 on page 58
  RFCfgReg = 0x26, // configures the receiver gain Table 97 on page 59
  GsNReg = 0x27,   // selects the conductance of the antenna driver pins TX1 and
                   // TX2 for modulation Table 99 on page 59
  CWGsPReg = 0x28, // defines the conductance of the p-driver output during
                   // periods of no modulation Table 101 on page 60
  ModGsPReg = 0x29, // defines the conductance of the p-driver output during
                    // periods of modulation Table 103 on page 60
  TModeReg =
      0x2A, // defines settings for the internal timer Table 105 on page 60
  TPrescalerReg = 0x2B, // Table 107 on page 61
  TReloadRegH =
      0x2C, // defines the 16-bit timer reload value Table 109 on page 62
  TReloadRegL = 0x2D,     // Table 111 on page 62
  TCounterValRegH = 0x2E, // shows the 16-bit timer value Table 113 on page 63
  TCounterValRegL = 0x2F, // Table 115 on page 63

  // Page 3...

  // 30h Reserved reserved for future use Table 117 on page 63
  TestSel1Reg = 0x31, // general test signal configuration Table 119 on page 63
  TestSel2Reg = 0x32, // general test signal configuration and PRBS control
                      // Table 121 on page 64
  TestPinEnReg =
      0x33, // enables pin output driver on pins D1 to D7 Table 123 on page 64
  TestPinValueReg = 0x34, // defines the values for D1 to D7 when it is used as
                          // an I/O bus Table 125 on page 65
  TestBusReg =
      0x35, // shows the status of the internal test bus Table 127 on page 65
  AutoTestReg = 0x36,   // controls the digital self test Table 129 on page 66
  VersionReg = 0x37,    // shows the software version Table 131 on page 66
  AnalogTestReg = 0x38, // controls the pins AUX1 and AUX2 Table 133 on page 67
  TestDAC1Reg =
      0x39, // defines the test value for TestDAC1 Table 135 on page 68
  TestDAC2Reg =
      0x3A, // defines the test value for TestDAC2 Table 137 on page 68
  TestADCReg =
      0x3B, // shows the value of ADC I and Q channels Table 139 on page 68
  // 3Ch to 3Fh Reserved reserved for production tests Table 141 to Table 147 on
  // page 69

} MFRC522_Reg;

typedef enum {
	Idle= 0x00,       // no action, cancels current command execution
	Mem=0x01,         // stores 25 bytes into the internal buffer
	GenRandomID=0x2,  // generates a 10-byte random ID number
	CalcCRC=0x3,      //activates the CRC coprocessor or performs a self test
	Transmit=0x04,    //transmits data from the FIFO buffer
	NoCmdChange=0x07, // no command change, can be used to modify the
		// CommandReg register bits without affecting the command,
		// for example, the PowerDown bit
	Receive=0x08,     // activates the receiver circuits
	Transceive=0xC,   // transmits data from FIFO buffer to antenna and automatically
		// activates the receiver after transmission
	//- 1101 reserved for future use
	MFAuthent=0x0E,   // performs the MIFARE standard authentication as a reader
	SoftReset=0x0F
} MFRC522_CMD;

typedef enum {
	RC522_OK,
	RC522_TIMEOUT,
	RC522_ERR,
	RC522_UNKNOWN_BOARD,
} MFRC522_Status;

MFRC522_Status HAL_MFRC522_WriteRegister(MFRC522 *rfid, MFRC522_Reg addr, uint8_t data);
uint8_t HAL_MFRC522_ReadRegister(MFRC522 *rfid, MFRC522_Reg addr);
MFRC522_Status HAL_MFRC522_Reset(MFRC522 *rfid);
MFRC522_Status HAL_MFRC522_SetBitMask(MFRC522 *rfid, MFRC522_Reg addr, uint8_t mask);
MFRC522_Status HAL_MFRC522_ClearBitMask(MFRC522 *rfid, MFRC522_Reg addr, uint8_t mask);
MFRC522_Status HAL_MFRC522_Open_Antenna(MFRC522 *rfid);
MFRC522_Status HAL_MFRC522_Close_Antenna(MFRC522 *rfid);
MFRC522_Status HAL_MFRC522_Init(MFRC522 *rfid);
MFRC522_Status HAL_MFRC522_SoftPowerDown(MFRC522 *rfid);
MFRC522_Status HAL_MFRC522_SoftPowerUp(MFRC522 *rfid);

#endif
