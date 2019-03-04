#include "SDM120.h"

void setup() {
	Serial.begin(9600);
}

void loop() {

    byte hexArrIeee754[sizeof(float)] = {};
    float f = 2.8f;

    floatToBytes(hexArrIeee754, f);

	Serial.print("Float ");
	Serial.print(f);
	Serial.print(" is converted to { ");
	for (byte i = 0; i < 4; i++) {
		Serial.print(hexArrIeee754[i], HEX);
		Serial.print(" ");
	}
	Serial.println("}");
	Serial.println();
	delay(2000);
}
