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

#endif
