#include "SDM120.h"

/** 
* @brief Check the system endianess
*/
bool isLittleEndian()
{
	byte endianess = 0x0001;
	return (bool)(*(byte *)&(endianess)); //< True if the system is little endian
}

/**
* @brief Convert 4 bytes to float
*/
float bytesToFloat(byte* regData)
{
    float result = NAN;

    for (int8_t i = 3; i >= 0; i--) { //< [WARNING] Changing int8_t to byte 
                                      //  can crash the program
        ((uint8_t*)&result)[i] = regData[3 - i];
    }

    return result;
}

/**
* @brief Convert IEEE 754 floating point to array of bytes
*/
void floatToBytes(byte* bytes, float f)
{
    int32_t num;
    memcpy(&num, &f, sizeof (num));
    
    bytes[0] = (num >> 24) & 0xFF;
    bytes[1] = (num >> 16) & 0xFF;
    bytes[2] = (num >> 8) & 0xFF;
    bytes[3] = num & 0xFF;
}

/* Constructor */
SDM::SDM(): _sdmSerial(Serial1)
{
	_nodeId = DEFAULT_NODE_ID;
    _baudRate = DEFAULT_BAUD_RATE;
    _protocol = DEFAULT_PROTOCOL;
    _ctlPin = DEFAULT_RS485_CTL_PIN;
}

/* Copy constructor */
SDM::SDM(HardwareSerial& serial, long baudRate, 
         byte protocol, byte ctlPin): _sdmSerial(serial)
{
    _baudRate = baudRate;
    _protocol = protocol;
    _ctlPin = ctlPin;
}

/* Copy constructor */
SDM::SDM(HardwareSerial& serial, long baudRate,  byte ctlPin): _sdmSerial(serial)
{
    _baudRate = baudRate;
    _protocol = DEFAULT_PROTOCOL;
    _ctlPin = ctlPin;
}

/**
* @public
* @brief Initialise SDM-120 Serial
*/
void SDM::begin(const byte nodeID)
{
    _nodeId = nodeID;
    pinMode(_ctlPin, OUTPUT);
    _sdmSerial.begin(_baudRate, _protocol);
    digitalWrite(_ctlPin, RS485_RECEIVE);
} 

/**
* @public
* @brief Set node ID of SDM-120
*/
void SDM::setNodeId(const byte nodeId)
{
    _nodeId = nodeId;
}

/**
* @public
* @brief Ensure the preservation of data.
*/
bool SDM::isDataPreserved(byte* receivedData, size_t size)
{
    SDM::Crc reCalCrc = SDM::_calculateCrc(receivedData, size - 2);
    
    SDM::Crc receivedCrc = 0x00;

    // Merge 2 checksum bytes into 16-bit CRC-16
    byte lowByte = receivedData[size - 2];
    byte highByte = receivedData[size - 1];
    if (isLittleEndian())
        receivedCrc = (lowByte << 8) | highByte;
    else 
        receivedCrc = (highByte << 8) | lowByte;

    /* 
    * If the calculated CRC-16 and the received CRC-16 are not the same, 
    * then the data from reader to RS485 are not preserved.
    */
    return (receivedCrc != reCalCrc) ? false : true;
}

/**
* @public
* @brief Get bytes of data from SDM120
*/
SDM::Status SDM::getRawData(byte* reData, byte* request, size_t size)
{
    digitalWrite(_ctlPin, RS485_TRANSMIT);
    _sdmSerial.write(request, size);
    delay(10); //< [WARNING] This delay is VERY IMPORTANT
    digitalWrite(_ctlPin, RS485_RECEIVE);

    byte i = 0;
    while (_sdmSerial.available() > 0) {
        reData[i] = _sdmSerial.read();
        if (reData[i] < 0) 
            return ERR_READ_RS485;
        i++;
    }
    delay(2);
    
    return STATUS_SUCCESS;
}

/**
* @public
* @brief Get float value based on the requested type of data
*/
float SDM::getValue(SDM::Type type)
{
    COMMAND function = FUNC_READ_INPUT_REG;

    if (type == SDM::VOLTAGE) {
        _highAddrByte = SDM::VOLTAGE_HI_BYTE;
        _lowAddrByte = SDM::VOLTAGE_LO_BYTE;
    } 
    else if (type == SDM::CURRENT) {
        _highAddrByte = SDM::CURRENT_HI_BYTE;
        _lowAddrByte = SDM::CURRENT_LO_BYTE;    
    } 
    else if (type == SDM::FREQUENCY) {
        _highAddrByte = SDM::FREQ_HI_BYTE;
        _lowAddrByte = SDM::FREQ_LO_BYTE;
    }
    else if (type == SDM::TOTAL_ACTIVE_ENERGY) {
        _highAddrByte = SDM::TOTAL_ACTIVE_ENERGY_HI_BYTE;
        _lowAddrByte = SDM::TOTAL_ACTIVE_ENERGY_LO_BYTE;
    }
    else if (type == SDM::BAUDRATE) {
        _highAddrByte = SDM::BAUDRATE_HI_BYTE;
        _lowAddrByte = SDM::BAUDRATE_LO_BYTE;
        function = FUNC_READ_HOLDING_REG;
    } 
    else if (type == SDM::NODE_ID) {
        _highAddrByte = SDM::NODE_ID_HI_BYTE;
        _lowAddrByte = SDM::NODE_ID_LO_BYTE;
        function = FUNC_READ_HOLDING_REG;
    }

    byte reData[RESPONSE_FRAME_SIZE] = {};

    RCommand rCmd = SDM::_setReadCommand(function, _highAddrByte, _lowAddrByte);

    if (getRawData(reData, rCmd.cmd, rCmd.size) == ERR_READ_RS485) {
        return ;
    }

    /* Debug view */
    #ifdef DEBUG
        SDM::_debugViewCommand(rCmd);
        for (byte i = 0; i < RESPONSE_FRAME_SIZE; i++) {
            Serial.print(reData[i]);
            Serial.print(" ");
        }
        Serial.println();
    #endif
    
	return bytesToFloat(reData + RE_HIGH_REG_HIGH_BYTE_INDEX);
}

/**
* @protected
* @brief Debug function: view read command to be sent
*/
void SDM::_debugViewCommand(RCommand rCmd)
{
    for (byte i = 0; i < rCmd.size; i++) {
        Serial.print(rCmd.cmd[i], HEX);
        Serial.print(" ");
    }
}

/**
* @protected
* @brief Debug function: wiew write command to be sent
*/
void SDM::_debugViewCommand(WCommand wCmd)
{
    for (byte i = 0; i < wCmd.size; i++) {
        Serial.print(wCmd.cmd[i], HEX);
        Serial.print(" ");
    }
}

/**
* @protected
* @brief Set baud rate for SDM120 (in Setting mode)
*/
SDM::Status SDM::_setSDMBaudRate(const long baudRate)
{
    byte hexArrIeee754[sizeof(float)] = {};
    float fBaud = 0.0f;

    switch (baudRate) {
        case 2400:
            fBaud = 0.0f;
            break;
        case 4800:
            fBaud = 1.0f;
            break;
        case 9600:
            fBaud = 2.0f;
            break;
        case 19200:
            fBaud = 3.0f;
            break;
        case 38400:
            fBaud = 4.0f;
            break;
        default:
            fBaud = 2.0f;
            break;
    }
    
    floatToBytes(hexArrIeee754, (float)fBaud);

    byte dataHighRegHighByte = hexArrIeee754[0];
    byte dataHighRegLowByte = hexArrIeee754[1];
    byte dataLowRegHighByte = hexArrIeee754[2];
    byte dataLowRegLowByte = hexArrIeee754[3];

    WCommand wCmd = SDM::_setWriteCommand(SDM::BAUDRATE_HI_BYTE, SDM::BAUDRATE_LO_BYTE,
                                          dataHighRegHighByte, dataHighRegLowByte,
                                          dataLowRegHighByte, dataLowRegLowByte);
    SDM::_debugViewCommand(wCmd);

    return STATUS_SUCCESS;
}

/**
* @protected
* @brief Set Node ID of SDM120 (in Setting mode)
*/
SDM::Status SDM::_setSDMNodeID(const float nodeID)
{
    byte hexArrIeee754[sizeof(float)] = {};

    floatToBytes(hexArrIeee754, (float)nodeID);

    byte dataHighRegHighByte = hexArrIeee754[0];
    byte dataHighRegLowByte = hexArrIeee754[1];
    byte dataLowRegHighByte = hexArrIeee754[2];
    byte dataLowRegLowByte = hexArrIeee754[3];

    WCommand wCmd = SDM::_setWriteCommand(SDM::NODE_ID_HI_BYTE, SDM::NODE_ID_LO_BYTE,
                                          dataHighRegHighByte, dataHighRegLowByte,
                                          dataLowRegHighByte, dataLowRegLowByte);
    SDM::_debugViewCommand(wCmd);

    return STATUS_SUCCESS;
}

/**
* @protected
* @brief Construct command sending to SDM120 to read register values
*/
const SDM::RCommand SDM::_setReadCommand(byte func, 
                                        const byte highAddrByte,
                                        const byte lowAddrByte)
{
    memset(&_rdCmd, 0, sizeof(_rdCmd));
    _rdCmd.size = READ_COMMAND_LENGTH;
    _rdCmd.cmd[READ_COMMAND_LENGTH] = {};
    _rdCmd.cmd[CMD_RD_SLAVE_ADDR_INDEX] = _nodeId;
    _rdCmd.cmd[CMD_RD_FUNC_INDEX] = func;
    _rdCmd.cmd[CMD_RD_START_ADDR_HIGH_INDEX] = highAddrByte;
    _rdCmd.cmd[CMD_RD_START_ADDR_LOW_INDEX] = lowAddrByte;
    _rdCmd.cmd[CMD_RD_NUM_REG_HIGH_INDEX] = 0x00; //< fixed value
    _rdCmd.cmd[CMD_RD_NUM_REG_LOW_INDEX] = 0x02; //< fixed value
    Crc crc = _calculateCrc(_rdCmd.cmd, _rdCmd.size - 2);
    _rdCmd.cmd[CMD_RD_ERROR_CHECK_LOW_INDEX] = highByte(crc);
    _rdCmd.cmd[CMD_RD_ERROR_CHECK_HIGH_INDEX] = lowByte(crc);
    return _rdCmd;
}

/**
* @protected
* @brief Construct command sending to SDM120 to write register values
*/
const SDM::WCommand SDM::_setWriteCommand(const byte highAddrByte, 
                                          const byte lowAddrByte, 
                                          byte dataHighRegHighByte,
                                          byte dataHighRegLowByte, 
                                          byte dataLowRegHighByte,
                                          byte dataLowRegLowByte)
{
    memset(&_wrtCmd, 0, sizeof(_wrtCmd));
    _wrtCmd.size = WRITE_COMMAND_LENGTH;
    _wrtCmd.cmd[WRITE_COMMAND_LENGTH] = {};
    _wrtCmd.cmd[CMD_WRT_SLAVE_ADDR_INDEX] = _nodeId;
    _wrtCmd.cmd[CMD_WRT_FUNC_INDEX] = FUNC_WRITE_HOLDING_REG;
    _wrtCmd.cmd[CMD_WRT_START_ADDR_HIGH_INDEX] = highAddrByte;
    _wrtCmd.cmd[CMD_WRT_START_ADDR_LOW_INDEX] = lowAddrByte;
    _wrtCmd.cmd[CMD_WRT_NUM_REG_HIGH_INDEX] = 0x00; //< fixed value
    _wrtCmd.cmd[CMD_WRT_NUM_REG_LOW_INDEX] = 0x02; //< fixed value
    _wrtCmd.cmd[CMD_WRT_BYTE_COUNT_INDEX] = 0x04; //< Write 4 bytes reg (fixed value)
    _wrtCmd.cmd[CMD_WRT_DAT_HIGH_REG_HIGH_BYTE_INDEX] = dataHighRegHighByte;
    _wrtCmd.cmd[CMD_WRT_DAT_HIGH_REG_LOW_BYTE_INDEX] = dataHighRegLowByte;
    _wrtCmd.cmd[CMD_WRT_DAT_LOW_REG_HIGH_BYTE_INDEX] = dataLowRegHighByte;
    _wrtCmd.cmd[CMD_WRT_DAT_LOW_REG_LOW_BYTE_INDEX] = dataLowRegLowByte;
    Crc crc = _calculateCrc(_wrtCmd.cmd, _wrtCmd.size - 2);
    _wrtCmd.cmd[CMD_WRT_ERROR_CHECK_LOW_INDEX] = highByte(crc);
    _wrtCmd.cmd[CMD_WRT_ERROR_CHECK_HIGH_INDEX] = lowByte(crc);
    return _wrtCmd;
}

/**
* @private
* @brief CRC-16 Calculator (polynomial of 0xA001)
*/
SDM::Crc SDM::_calculateCrc(byte* data, size_t size)
{
    _crc = 0xFFFF;
    for (size_t i = 0; i < size; i++) {
        _crc ^= data[i];
        for (byte j = 0; j < 8; j++) {
            if (_crc & 0x0001)
                _crc = (_crc >> 1) ^ 0xA001; //< fixed polynomial
            else
                _crc = _crc >> 1;
        }
    }
    
    if (isLittleEndian())
        return _crc >> 8 | _crc << 8;
    else
        return _crc;
}
