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
    Serial.print("SDM Node ID: ");
    Serial.println(TICTAG_SDM_ID, HEX);
    SDM::RCommand rcmd; 
    SDM::WCommand wcmd;
    

    Serial.print("Read Voltage Command {");
    rcmd = TictagSDM._setReadCommand(SDM::FUNC_READ_INPUT_REG,
                                    SDM::VOLTAGE_HI_BYTE,
                                    SDM::VOLTAGE_LO_BYTE);
    TictagSDM._debugViewCommand(rcmd);
    Serial.println("}");

    Serial.print("Read Total Active Energy Command {");
    rcmd = TictagSDM._setReadCommand(SDM::FUNC_READ_INPUT_REG,
                                    SDM::TOTAL_ACTIVE_ENERGY_HI_BYTE,
                                    SDM::TOTAL_ACTIVE_ENERGY_LO_BYTE);
    TictagSDM._debugViewCommand(rcmd);
    Serial.println("}");

    Serial.print("Read Baud Rate Command {");
    rcmd = TictagSDM._setReadCommand(SDM::FUNC_READ_HOLDING_REG,
                                    SDM::BAUDRATE_HI_BYTE,
                                    SDM::BAUDRATE_LO_BYTE);
    TictagSDM._debugViewCommand(rcmd);
    Serial.println("}");
    Serial.println();

    byte hexArrIeee754[sizeof(float)] = {};
    float fBaud = 0.0f;


    Serial.print("Write to Baudrate Register Command {"); //< Command to change baudrate
    // 2400 bps -> 0.0f
    // 4800 bps -> 1.0f
    // 9600 bps -> 2.0f
    // 19200 bps -> 3.0f
    floatToBytes(hexArrIeee754, 2.0f); //< 9600 bps
    wcmd = TictagSDM._setWriteCommand(SDM::BAUDRATE_HI_BYTE, SDM::BAUDRATE_LO_BYTE,
                                      hexArrIeee754[0], hexArrIeee754[1],
                                      hexArrIeee754[2], hexArrIeee754[3]);
    TictagSDM._debugViewCommand(wcmd);
    Serial.println("}");


    Serial.print("Write to Node ID Register Command {"); //< Command to change Node ID
    // 2400 bps -> 0.0f
    // 4800 bps -> 1.0f
    // 9600 bps -> 2.0f
    // 19200 bps -> 3.0f
    floatToBytes(hexArrIeee754, 3); //< Change Node ID to 3
    wcmd = TictagSDM._setWriteCommand(SDM::NODE_ID_HI_BYTE, SDM::NODE_ID_LO_BYTE,
                                      hexArrIeee754[0], hexArrIeee754[1],
                                      hexArrIeee754[2], hexArrIeee754[3]);
    TictagSDM._debugViewCommand(wcmd);
    Serial.println("}");
    Serial.println();
    Serial.println();
    delay(2000);
}

void flash()
{
    digitalWrite(LED, HIGH);
    delay(50);
    digitalWrite(LED, LOW);
}
