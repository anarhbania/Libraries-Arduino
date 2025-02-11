#ifndef MODBUS_SLAVE_H
#define MODBUS_SLAVE_H

#include "Arduino.h"

#define MODE SERIAL_8N1 // data bits | (O) odd, (E) even, (N) no parity | stop bits

#define FRAME_SIZE  128

#define READ_HOLDING_REGISTERS    0x03
#define PRESET_SINGLE_REGISTER    0x06
#define PRESET_MULTIPLE_REGISTERS 0x10

#define ILLEGAL_DATA_FUNCTION     0x01
#define ILLEGAL_DATA_ADDRESS      0x02
#define ILLEGAL_DATA_VALUE        0x03

#define ALARM_COMMUNICATION       0x01

class ModbusSlave
{
	public:

	ModbusSlave(HardwareSerial *_port, uint32_t _baud, uint8_t _slaveID, uint16_t _registersAddress, uint16_t *_registers, uint16_t _registersSize, uint64_t _timeout);
	void REDE(uint8_t _pinREDE);
	uint8_t Update(void);
	float ConversionToFloat(uint16_t variable1, uint16_t variable0);

	private:

	HardwareSerial *port;

	uint8_t alarm;
	uint8_t pinREDE = -1;
	uint8_t slaveID;
	uint8_t frame[FRAME_SIZE];

	uint16_t *registers;
	uint16_t registersAddress;
	uint16_t registersSize;

	uint16_t t1_5;
	uint16_t t3_5;
	
	uint64_t timeout;
	uint64_t lastTimeout;

	void SendAnswer(uint8_t length);
	void SendException(uint8_t function, uint8_t exception);
	uint16_t CalculateCRC16(uint8_t length);
};

#endif
