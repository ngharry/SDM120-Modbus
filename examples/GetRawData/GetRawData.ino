#include "SDM120.h"

#define LED              17
#define BAUD_RATE        9600
#define RS485_CONTROL    4

#define TICTAG_SDM_ID    0x02

SDM TictagSDM(Serial1, BAUD_RATE, RS485_CONTROL);

void setup() {
    Serial.begin(BAUD_RATE);
    TictagSDM.begin(TICTAG_SDM_ID);
    flash();
}

void loop() {
	byte reData[SDM::RESPONSE_FRAME_SIZE] = {};

    SDM::RCommand rCmd = TictagSDM._setReadCommand(SDM::FUNC_READ_INPUT_REG,
                                                   SDM::VOLTAGE_HI_BYTE,
                                                   SDM::VOLTAGE_LO_BYTE);

    if (TictagSDM.getRawData(reData, rCmd.cmd, rCmd.size) == SDM::ERR_READ_RS485) {
        return ;
    }

    Serial.print("Read Voltage Command: { ");
    TictagSDM._debugViewCommand(rCmd);
    Serial.println("}");

    Serial.print("Response: { ");
    for (byte i = 0; i < SDM::RESPONSE_FRAME_SIZE; i++) {
        Serial.print(reData[i], HEX);
        Serial.print(" ");
    }
    Serial.println("}");
    Serial.println();
    delay(2000);
}

void flash()
{
    digitalWrite(LED, HIGH);
    delay(50);
    digitalWrite(LED, LOW);
}
