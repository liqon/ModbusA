#ifndef ModbusA_h
#define ModbusA_h

#include "ModbusA.h"

static const char toBase16[] = {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9',
							   'a', 'b', 'c', 'd', 'e', 'f'};
static const unsigned char fromBase16[] = {	
		0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
		0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
		0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
		0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
		0,  0,  0,  0,  0,  0,  0,  0,  0,  1,
		2,  3,  4,  5,  6,  7,  8,  9,  0,  0,
		0,  0,  0,  0,  0, 10, 11, 12, 13, 14,
		15, 0,  0,  0,  0,  0,  0,  0,  0,  0,
		0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
		0,  0,  0,  0,  0,  0,  0, 10, 11, 12,
		13, 14, 15, 0,  0,  0,  0,  0,  0,  0	};
int16_t ModbusA::holdingRegister[HOLDINGREGS];
int16_t ModbusA::ainputRegs[ANALOG_INPUTS];
uint8_t ModbusA::coils[DIGITAL_OUTPUTS];
uint8_t ModbusA::dinputRegs[DIGITAL_INPUTS];
/*
 * PRIVATE VARS
 */
char ModbusA::asciiBuffer[ASCII_BUFFER_LEN];
uint8_t ModbusA::rtuBuffer[RTU_BUFFER_LEN];
int ModbusA::rtuBuffer_stat;
int ModbusA::asciiBuffer_stat; //number of written chars
uint8_t ModbusA::deviceAddress;
HardwareSerial *ModbusA::pSerial;
void ModbusA::loop()
{
	char *ascii_buffer_ptr = asciiBuffer;
	uint8_t *rtu_buffer_ptr = rtuBuffer;
	readSerialPort();
	if (ascii_buffer_status > 5)
	{
		asciiToRtu();
		if (*rtuBuffer == deviceAddress)
		{
			switch (rtu_buffer[1])
			{
			case 0x03u:
				readHoldingRegisters();
				break;
			case 0x06u:
				writeSingleRegister(); //Single Register
				break;
			case 0x10:
				writeMultipleRegisters(); //Preset multiple registers
				break;
			}
		}
		flushBuffers();
	}
}
void ModbusA::begin(HardwareSerial *serial_stream, uint8_t device_address)
{
	asciiBuffer_stat = 0;
	rtuBuffer_stat = 0;
	flushBuffers();
	this->pSerial = serial_stream;
	ModbusA::deviceAddress = device_address;
}
//LITTLE-ENDIAN-TO-BIG-ENDIAN
static inline void int16ToUint8(int16_t *input, uint8_t *output)
{
	uint8_t *input_ptr = (uint8_t *)input;
	*output++ = input_ptr[1];
	*output = *input_ptr;
}
/**** FUNCTION 0x10 (16) Preset multiple registers **************/
int ModbusA::writeMultipleRegisters()
{
	int k;
	//char *ascii_buffer_ptr = asciiBuffer;
	uint8_t *rtu_buffer_ptr, byte_count;
	int16_t start_address, no_registers;
	//Register Address
	start_address = (rtu_buffer[2] << 8) | rtu_buffer[3];
	//No. of registers to read
	no_registers = (rtu_buffer[4] << 8) | rtu_buffer[5];
	//Bytes count in message
	byte_count = rtu_buffer[6];
	rtu_buffer_ptr = rtu_buffer + 7;
	//write data
	for (k = 0; k < no_registers; k++)
	{
		holdingRegister[start_address++] = (*rtu_buffer_ptr << 8) | rtu_buffer_ptr[1];
		rtu_buffer_ptr += 2;
	}
	/***  SEND NORMAL RESPONSE *************************************/
	rtu_buffer_status = 6;
	rtuToAscii();
	pSerial->write(asciiBuffer, asciiBuffer_stat);
	return 0;
}
/**** FUNCTION 0x06 (06) Preset single registers ****************/
int ModbusA::writeSingleRegister()
{
	int k;
	uint8_t *rtu_buffer_ptr, byte_count;
	int16_t register_address;
	/**************************************************************/
	//Register Address
	register_address = (rtu_buffer[2] << 8) | rtu_buffer[3];
	holdingRegister[register_address] = (rtu_buffer[4] << 8) | rtu_buffer[5];
	/***  SEND NORMAL RESPONSE *************************************/
	rtu_buffer_status = 6;
	rtuToAscii();
	_modbus_serial->write(ascii_buffer, ascii_buffer_status);
	return 0;
}
BYTE ModbusA::CheckLRC(BYTE *data, uint8_t byte_count)
{
	BYTE ret_LRC = 0, *p_data = data;
	int k = 0;
	for (; k < byte_count; k++)
	{
		ret_LRC += *p_data++;
	}
	ret_LRC = 0xff & (-ret_LRC);
	return ret_LRC;
}
/**** FUNCTION 0x03 (02) Read holding registers ****************/
int ModbusA::readHoldingRegisters()
{
	int k;
	uint8_t *rtu_buffer_ptr, byte_count;
	int16_t start_address, no_registers, *registerValue;
	/**************************************************************/
	//Register Address
	start_address = (rtu_buffer[2] << 8) | rtu_buffer[3];
	//No. of registers to read
	no_registers = (rtu_buffer[4] << 8) | rtu_buffer[5];
	/********************* RESPONSE *******************************/
	rtu_buffer_ptr = rtu_buffer + 6;
	//Byte count
	byte_count = (no_registers << 1);
	*rtu_buffer_ptr++ = byte_count;
	for (k = 0; k < no_registers; k++)
	{
		registerValue = holdingRegister + start_address++;
		int16ToUint8(registerValue, rtu_buffer_ptr);
		rtu_buffer_ptr += 2;
	}
	rtu_buffer_status = 3 + byte_count;
	rtuToAscii();
	pSerial->write(asciiBuffer, asciiBuffer_stat);
	return 0;
}
void ModbusA::readSerialPort()
{
	int r = 0;
	bool rx_on = false, rx_off = false;
	unsigned long _loop_starttime = 0;
	if (pSerial->available() > 0)
	{
		r = pSerial->read();
		if (r == 58)
		{
			rx_on = true;
		}
	}
	_loop_starttime = millis();
	while (rx_on == true)
	{
		if (pSerial->available() > 0)
		{
			r = pSerial->read();
			ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
			{
				if (rx_off == true)
				{
					rx_on = false;
					if (r != '\n')
					{
						flushBuffers();
					}
					break;
				}
				if (r == '\r')
				{
					rx_off = true;
				}
				if (rx_on == true && rx_off == false)
				{
					asciiBuffer[asciiBuffer_stat] = (char)r;
					asciiBuffer_stat++;
				}
			}
		}
		if ((millis() - _loop_starttime) > 2000)
		{
			rx_on = false;
			break;
		}
	}
}
void ModbusA::flushBuffers()
{
	int k;
	char *buffer1 = asciiBuffer;
	uint8_t *buffer2 = rtuBuffer;
	for (k = 0; k < RTU_BUFFER_LEN; k++)
	{
		*buffer1++ = 0;
		*buffer2++ = 0;
	}
	for (k = RTU_BUFFER_LEN; k < ASCII_BUFFER_LEN; k++)
	{
		*buffer1++ = 0;
	}
	asciiBuffer_stat = 0;
	rtuBuffer_stat = 0;
}
ModbusA::ModbusA()
{
}

ModbusA::~ModbusA()
{
}
void ModbusA::rtuToAscii()
{
	//Convert to Modbus ASCII
	int i;
	char *ascii_buffer_ptr = asciiBuffer;
	uint8_t *rtu_buffer_ptr = rtuBuffer;
	*ascii_buffer_ptr++ = ':';
	asciiBuffer_stat = 1;
	for (i = 0; i < rtuBuffer_stat; i++)
	{
		*ascii_buffer_ptr++ = toBase16[(*rtu_buffer_ptr >> 4)];
		*ascii_buffer_ptr++ = toBase16[*rtu_buffer_ptr++ & 0xf];
		asciiBuffer_stat += 2;
	}
	BYTE cLRC = ModbusA::computeLRC(rtuBuffer, rtuBuffer_stat);
	*ascii_buffer_ptr++ = toBase16[(cLRC >> 4)];
	*ascii_buffer_ptr++ = toBase16[(cLRC & 0xf)];

	*ascii_buffer_ptr++ = '\r';
	*ascii_buffer_ptr = '\n';
	asciiBuffer_stat += 4;
}
void ModbusA::asciiToRtu()
{
	//Convert to Modbus ASCII
	int i;
	char *ascii_buffer_ptr = asciiBuffer;
	uint8_t *rtu_buffer_ptr = rtuBuffer;
	rtuBuffer_stat = 0;
	for (i = 0; i < asciiBuffer_stat; i += 2)
	{
		*rtu_buffer_ptr++ = (fromBase16[*ascii_buffer_ptr] << 4) | hexTable[ascii_buffer_ptr[1]];
		rtuBuffer_stat++;
		ascii_buffer_ptr += 2;
	}
}
#endif