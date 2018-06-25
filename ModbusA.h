//     Implementation of Modbus ASCII protocol for Arduino IDE
//     Copyright (C) 2018  Liqon
//
//     This program is free software: you can redistribute it and/or modify
//     it under the terms of the GNU General Public License as published by
//     the Free Software Foundation, either version 3 of the License, or
//     (at your option) any later version.
//
//     This program is distributed in the hope that it will be useful,
//     but WITHOUT ANY WARRANTY; without even the implied warranty of
//     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//     GNU General Public License for more details.
//
//     You should have received a copy of the GNU General Public License
//     along with this program.  If not, see <https://www.gnu.org/licenses/>.

#pragma once
#include <arduino.h>
#include <stdint.h>
#include <stdlib.h>
#include <HardwareSerial.h>

#define BYTE unsigned char
#define ASCII_BUFFER_Len 250u
#define RTU_BUFFER_LEN 127u
#define HOLDINGREG_LEN 64u

class ModbusA
{
  public:
	static int16_t holdingRegister[HOLDINGREG_LEN];
	ModbusA();
	~ModbusA();
	void loop();
	void begin(HardwareSerial *serial_stream, uint8_t device_address);

  private:
	static uint8_t deviceAddress;
	static char asciiBuffer[ASCII_BUFFER_LEN];
	static uint8_t rtuBuffer[RTU_BUFFER_LEN];
	static int rtu_buffer_stat;
	static int ascii_buffer_stat;
	static HardwareSerial *pSerial;

	void convertToAscii();
	void convertToRtu();
	int readHoldingRegisters();
	int readInputRegisters();
	int writeMultipleRegisters();
	int writeSingleRegister();
	uint8_t CheckLRC(uint8_t *data, uint8_t byte_count);
	void ReadSerialData();
	void FlushBuffers();
};