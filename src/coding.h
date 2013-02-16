#include "ros/ros.h"
union BytesToDouble {
    char    bytes[8];
    double	doubleValue;
};

union BytesToUint8 {
    char bytes[1];
    uint8_t uint8Value;
};

union BytesToIntFloat {
	int intValue;
	float floatValue;
	char bytes[4];
};

uint8_t decodeByte(char buffer[], int &pointer);
float decodeFloat(char buffer[], int &pointer);
void encodeDouble(char buffer[], int &pointer, double value);
void encodeInt(char buffer[], int &pointer, int value);
void encodeByte(char buffer[], int &pointer, uint8_t value);
void encodeFloat(char buffer[], int &pointer, float value);
