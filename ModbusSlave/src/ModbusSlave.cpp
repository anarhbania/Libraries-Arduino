#include "ModbusSlave.h"

ModbusSlave::ModbusSlave(HardwareSerial *port, uint32_t baud, uint8_t slaveID, uint16_t registersAddress, uint16_t *registers, uint16_t registersSize, uint64_t timeout)
{
	_port = port;
	_slaveID = slaveID;
	_registersAddress = registersAddress;
	_registers = registers;
	_registersSize = registersSize;
	_timeout = timeout;
	
	(*_port).begin(baud, MODE);
	
	if(baud > 19200)
	{
		_t1_5 = 750; 
		_t3_5 = 1750; 
	}
	else 
	{
		_t1_5 = 15000000 / baud;
		_t3_5 = 35000000 / baud;
	}
} 

void ModbusSlave::REDE(uint8_t pinREDE)
{
	_pinREDE = pinREDE;
		
	pinMode(_pinREDE, OUTPUT);
	digitalWrite(_pinREDE, LOW);
} 

uint8_t ModbusSlave::Update(void)
{	
	if((*_port).available())
	{
		_lastTimeout = millis();
		
		uint8_t frameQuantity = 0;
	
		while((*_port).available())
		{
			if(frameQuantity == FRAME_SIZE)
			{
				frameQuantity -= frameQuantity;
			}
		  
			_frame[frameQuantity++] = (*_port).read();
			delayMicroseconds(_t1_5);
		}
	
		if(frameQuantity > 7)
		{
			if(_frame[0] == _slaveID)
			{
				uint16_t calculateCRC = ModbusSlave::CalculateCRC16(frameQuantity - 2);
				
				if(calculateCRC == (((_frame[frameQuantity - 1] << 8) | _frame[frameQuantity - 2])))
				{
					uint16_t nextFrame = 0;
					uint16_t startingAddress = ((_frame[2] << 8) | _frame[3]);
					uint16_t quantityRegisters = ((_frame[4] << 8) | _frame[5]);
					uint16_t quantityData = 2 * quantityRegisters;

					if(_frame[1] == READ_HOLDING_REGISTERS)
					{
						if(startingAddress >= _registersAddress)
						{
							if(quantityRegisters <= _registersSize)
							{
								_frame[2] = quantityData;

								for(uint16_t i = startingAddress - _registersAddress; i < startingAddress - _registersAddress + quantityRegisters; i++)
								{
									_frame[3 + nextFrame] = _registers[i] >> 8;
									_frame[4 + nextFrame] = _registers[i] & 0xFF;

									nextFrame += 2;
								}

								calculateCRC = ModbusSlave::CalculateCRC16(quantityData + 3);

								_frame[3 + quantityData] = calculateCRC & 0xFF;
								_frame[4 + quantityData] = calculateCRC >> 8;

								ModbusSlave::SendAnswer(5 + quantityData);
								
								_alarm = 0;
							}
							else
							{
								ModbusSlave::SendException(READ_HOLDING_REGISTERS, ILLEGAL_DATA_VALUE);
							}
						}
						else
						{
							ModbusSlave::SendException(READ_HOLDING_REGISTERS, ILLEGAL_DATA_ADDRESS);
						}
					}
					else if(_frame[1] == PRESET_SINGLE_REGISTER)
					{
						if(startingAddress >= _registersAddress)
						{
							_registers[startingAddress - _registersAddress] = ((_frame[4] << 8) | _frame[5]);

							calculateCRC = ModbusSlave::CalculateCRC16(6);

							_frame[6] = calculateCRC & 0xFF;
							_frame[7] = calculateCRC >> 8;

							ModbusSlave::SendAnswer(8);
							
							_alarm = 0;
						}
						else
						{
							ModbusSlave::SendException(PRESET_SINGLE_REGISTER, ILLEGAL_DATA_ADDRESS);
						}
					}
					else if(_frame[1] == PRESET_MULTIPLE_REGISTERS)
					{
						if(_frame[6] == (frameQuantity - 9))
						{
							if(startingAddress >= _registersAddress)
							{
								if(quantityRegisters <= _registersSize)
								{
									for(uint16_t i = startingAddress - _registersAddress; i < startingAddress - _registersAddress + quantityRegisters; i++)
									{
										_registers[i] = ((_frame[7 + nextFrame] << 8) | _frame[8 + nextFrame]);

										nextFrame += 2;
									}

									calculateCRC = ModbusSlave::CalculateCRC16(6);

									_frame[6] = calculateCRC & 0xFF;
									_frame[7] = calculateCRC >> 8;

									ModbusSlave::SendAnswer(8);
									
									_alarm = 0;
								}
								else
								{
									ModbusSlave::SendException(PRESET_MULTIPLE_REGISTERS, ILLEGAL_DATA_VALUE);
								}
							}
							else
							{
								ModbusSlave::SendException(PRESET_MULTIPLE_REGISTERS, ILLEGAL_DATA_ADDRESS);
							}
						}
					}
					else
					{
						ModbusSlave::SendException(_frame[1], ILLEGAL_DATA_FUNCTION);
					}
				}
			}
		}
	}
	else if(millis() - _lastTimeout > _timeout)
	{
		_alarm = ALARM_COMMUNICATION;
	}
	
	return _alarm;
}

float ModbusSlave::ConversionToFloat(uint16_t variable1, uint16_t variable0)
{
	uint32_t variableInt = ((variable1 << 16) | variable0);
	float variableFloat = *(float*)&variableInt;
	
	return variableFloat;
}

void ModbusSlave::SendAnswer(uint8_t length)
{	
	if(_pinREDE != -1)
	{
		digitalWrite(_pinREDE, HIGH);
	}
	
	for(uint8_t i = 0; i < length; i++)
	{
		(*_port).write(_frame[i]);
	}

	(*_port).flush();

	delayMicroseconds(_t3_5);
	
	if(_pinREDE != -1)
	{
		digitalWrite(_pinREDE, LOW);
	}
}

void ModbusSlave::SendException(uint8_t function, uint8_t exception)
{
	_frame[0] = _slaveID;
	_frame[1] = (0x80 | function);
	_frame[2] = exception;

	uint16_t calculateCRC = ModbusSlave::CalculateCRC16(3);
	_frame[3] = calculateCRC >> 8;
	_frame[4] = calculateCRC & 0xFF;

	ModbusSlave::SendAnswer(5);
}

uint16_t ModbusSlave::CalculateCRC16(uint8_t length)
{
	uint16_t crc16 = 0xFFFF;

	for(uint8_t i = 0; i < length; i++)
	{
		crc16 = crc16 ^ _frame[i];

		for(uint8_t j = 0; j < 8; j++)
		{
			if((crc16 & 1) == 1)
			{
				crc16 = (crc16 >> 1) ^ 0xA001;
			}
			else
			{
				crc16 >>= 1;
			}
		}
	}

	return crc16;
}
