#include "SDM120.h"

#define LED              17
#define BAUD_RATE        9600
#define RS485_CONTROL    4
#define TICTAG_SDM_ID    0x02

SDM TictagSDM(Serial1, BAUD_RATE, RS485_CONTROL);

void setup() {
    Serial.begin(BAUD_RATE);
    pinMode(LED, OUTPUT);
    TictagSDM.begin(TICTAG_SDM_ID);
    flash();
}

void loop() {
    byte fpPrecision = 2; //< floating point precision
    SDM::Type type = SDM::TOTAL_ACTIVE_ENERGY;

    String info = "";
    String unit = "";

    switch (type) {
        case SDM::VOLTAGE:
            info = "Voltage: ";
            unit = "V";
            break;
        case SDM::CURRENT:
            info = "Current: ";
            unit = "A";
            break;

        case SDM::FREQUENCY:
            info = "Frequency: ";
            unit = "Hz";
            break;

        case SDM::TOTAL_ACTIVE_ENERGY:
            info = "Total active energy: ";
            unit = "KWh";
            break;

        case SDM::BAUDRATE:
            info = "Baud rate: ";
            unit = "";
            break;

        case SDM::NODE_ID:
            info = "Node ID: ";
            unit = "";
            break;

        default:
            info = "N/A";
            unit = "N/A";
            break;
    }

    float value = TictagSDM.getValue(type);

    Serial.print(info);
    Serial.print(value, fpPrecision);
    Serial.println(unit);

    delay(2000);
}

void flash()
{
    digitalWrite(LED, HIGH);
    delay(50);
    digitalWrite(LED, LOW);
}
