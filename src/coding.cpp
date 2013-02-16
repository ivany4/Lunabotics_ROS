#include "coding.h"

uint8_t decodeByte(char buffer[], int &pointer)
{
	union BytesToUint8 converter;
	converter.bytes[0] = buffer[pointer++];
	return converter.uint8Value;
}

float decodeFloat(char buffer[], int &pointer)
{
	union BytesToIntFloat converter;
	for (unsigned int i = 0; i < sizeof(float); i++) {
		converter.bytes[i] = buffer[pointer++];
	}
	return converter.floatValue;
}

void encodeDouble(char buffer[], int &pointer, double value)
{
	union BytesToDouble converter;
	converter.doubleValue = value;
	for (unsigned int i = 0; i < sizeof(value); i++) {
		buffer[pointer++] = converter.bytes[i];
	}
}

void encodeInt(char buffer[], int &pointer, int value)
{
	union BytesToIntFloat converter;
	converter.intValue = value;
	for (unsigned int i = 0; i < sizeof(value); i++) {
		buffer[pointer++] = converter.bytes[i];
	}
}

void encodeByte(char buffer[], int &pointer, uint8_t value)
{
	union BytesToUint8 converter;
	converter.uint8Value = value;
	buffer[pointer++] = converter.bytes[0];
}

void encodeFloat(char buffer[], int &pointer, float value)
{
	union BytesToIntFloat converter;
	for (unsigned int i = 0; i < sizeof(value); i++) {
		buffer[pointer++] = converter.bytes[i];
	}
}
