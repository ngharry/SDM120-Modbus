#include "SDM120.h"

void setup() {
	Serial.begin(9600);
}

void loop() {

	byte reg[4] = {0x43, 0x66, 0x33, 0x34}; //< 4 bytes to be converted to float
	float f = bytesToFloat(reg);

	Serial.print("IEEE 754 FP (Single Precision) of {");
	for (byte i = 0; i < 4; i++) {
		Serial.print(reg[i], HEX);
		Serial.print(" ");
	}
	Serial.print("} is ");
	Serial.println(f);
	delay(2000);
}
