#ifndef TICTAG_SDM_120_H
#define TICTAG_SDM_120_H

#include <Arduino.h>
#include <stdint.h>

#define RS485_TRANSMIT           HIGH
#define RS485_RECEIVE            LOW

#define DEFAULT_PROTOCOL         SERIAL_8N1
#define DEFAULT_BAUD_RATE        2400
#define DEFAULT_NODE_ID          0x01
#define DEFAULT_RS485_CTL_PIN    4    

#define DEBUG
#undef DEBUG

class SDM
{
public:
    typedef uint16_t Crc;
    typedef uint32_t SerialProtocol;

    enum Type: byte {
        VOLTAGE,
        CURRENT, 
        FREQUENCY,
        TOTAL_ACTIVE_ENERGY,
        BAUDRATE,
        NODE_ID
    };

    /* See `doc/SDM120-PROTOCOL.pdf */
    enum COMMAND: byte {
        READ_COMMAND_LENGTH         = 8, //< length of read command 
        WRITE_COMMAND_LENGTH        = 13, //< Length of write command (including data)
        
        /*
        * Reading configuration
        */
        FUNC_READ_INPUT_REG         = 0x04,
        FUNC_READ_HOLDING_REG       = 0x03,

        TOTAL_ACTIVE_ENERGY_HI_BYTE = 0x01,
        TOTAL_ACTIVE_ENERGY_LO_BYTE = 0x56,

        VOLTAGE_HI_BYTE             = 0x00,
        VOLTAGE_LO_BYTE             = 0x00,

        CURRENT_HI_BYTE             = 0x00,
        CURRENT_LO_BYTE             = 0x06,

        FREQ_HI_BYTE                = 0x00,
        FREQ_LO_BYTE                = 0x46,

        /*
        * Writing configuration
        */
        FUNC_WRITE_HOLDING_REG      = 0x10,

        BAUDRATE_HI_BYTE            = 0x00,
        BAUDRATE_LO_BYTE            = 0x1C,

        NODE_ID_HI_BYTE             = 0x00,
        NODE_ID_LO_BYTE             = 0x14
    };

    /* 
    * Read Command Frame: 
    +---------+----------+-----------+-----------+----------+----------+--------+--------+
    | Slave   | Function |   Start   |   Start   |   Num    |   Num    | CRC    | CRC    |
    | Address |   Code   | Addr (Hi) | Addr (Lo) | Reg (Hi) | Reg (Lo) | (Lo)   | (Hi)   | 
    +---------+----------+-----------+-----------+----------+----------+--------+--------+
    | 1 byte  |  1 byte  |   1 byte  |   1 byte  |  1 byte  |  1 byte  | 1 byte | 1 byte |
    +---------+----------+-----------+-----------+----------+----------+--------+--------+
    * @explaination
    * - Slave Address: 8-bit value representing the slave being addressed (1 to 247).
    *
    * - Function Code: 8-bit value telling the addressed slave what action is to
    * be performed (3, 4, 8 or 16 are valid for Digital meter).
    *
    * - Start Addr (Hi): The top (most significant) eight bits of a 16-bit number
    * specifying the start address of the data being requested. 
    *
    * - Start Addr (Lo): The bottom (least significant) eight bits of a 16-bit 
    * number specifying the start address of the data being requested. As registers
    * are used in pairs and start at zero, then this must be an even number.
    *
    * - Num Reg (Hi): The top (most significant) eight bits of a 16-bit number 
    * specifying the number of registers being requested. 
    *
    * - Num Reg (Lo): The bottom (least significant) eight bits of a 16-bit 
    * number specifying the number of registers being requested. As registers 
    * are used in pairs, then this must be an even number. 
    *
    * - Error Check (Lo): The bottom (least significant) eight bits of a 16-bit
    * number representing the error check value. 
    *
    * - Error Check (Hi): The top (most significant) eight bits of a 16-bit 
    * number representing the error check value. 
    */
    enum COMMAND_READ_INDEX: byte {
        CMD_RD_SLAVE_ADDR_INDEX,
        CMD_RD_FUNC_INDEX,
        CMD_RD_START_ADDR_HIGH_INDEX,
        CMD_RD_START_ADDR_LOW_INDEX,
        CMD_RD_NUM_REG_HIGH_INDEX,
        CMD_RD_NUM_REG_LOW_INDEX,
        CMD_RD_ERROR_CHECK_LOW_INDEX,
        CMD_RD_ERROR_CHECK_HIGH_INDEX
    };

    /* 
    * Write Command Frame: 
    +---------+----------+-----------+-----------+----------+----------+--------+---------+--------+--------+
    | Slave   | Function |   Start   |   Start   |   Num    |   Num    | Byte   |   Data  | CRC    | CRC    |
    | Address |   Code   | Addr (Hi) | Addr (Lo) | Reg (Hi) | Reg (Lo) | Count  |    []   | (Lo)   | (Hi)   | 
    +---------+----------+-----------+-----------+----------+----------+--------+---------+--------+--------+
    | 1 byte  |  1 byte  |   1 byte  |   1 byte  |  1 byte  |  1 byte  | 1 byte | 4 bytes | 1 byte | 1 byte |
    +---------+----------+-----------+-----------+----------+----------+--------+---------+--------+--------+
    * @explaination
    * - Slave Address, Function Code, Start Addr, Num Reg: see Read Command Frame.
    *
    * - Byte Count: Number of bytes need to be written to SDM120
    *
    * - Data[]: 4 bytes of data writtem to SDM120 represent IEEE 754 floating 
    * point single presision.
    *
    * CRC: see Read Command Frame.
    */
    enum COMMAND_WRITE_INDEX: byte {
        CMD_WRT_SLAVE_ADDR_INDEX,
        CMD_WRT_FUNC_INDEX,
        CMD_WRT_START_ADDR_HIGH_INDEX,
        CMD_WRT_START_ADDR_LOW_INDEX,
        CMD_WRT_NUM_REG_HIGH_INDEX,
        CMD_WRT_NUM_REG_LOW_INDEX,
        CMD_WRT_BYTE_COUNT_INDEX,
        CMD_WRT_DAT_HIGH_REG_HIGH_BYTE_INDEX,
        CMD_WRT_DAT_HIGH_REG_LOW_BYTE_INDEX,
        CMD_WRT_DAT_LOW_REG_HIGH_BYTE_INDEX,
        CMD_WRT_DAT_LOW_REG_LOW_BYTE_INDEX,
        CMD_WRT_ERROR_CHECK_LOW_INDEX,
        CMD_WRT_ERROR_CHECK_HIGH_INDEX
    };

    /* 
    * Response Frame: 
    +---------+----------+--------+----------+----------+----------+----------+--------+--------+ 
    | Slave   | Function | Byte   |  First   |  First   | Second   | Second   | CRC    | CRC    |
    | Address |   Code   | Count  | Reg (Hi) | Reg (Lo) | Reg (Hi) | Reg (Lo) | (Lo)   | (Hi)   |
    +---------+----------+--------+----------+----------+----------+----------+--------+--------+
    | 1 byte  |  1 byte  | 1 byte |  1 byte  |  1 byte  |  1 byte  |  1 byte  | 1 byte | 1 byte |
    +---------+----------+--------+----------+----------+----------+----------+--------+--------+
    * @explaination
    *
    * - Slave Address: 8-bit value representing the address of the responding slave
    *
    * - Function Code: 8-bit value which, when a copy of the function code in 
    * the query, indicates that the slave recognized the query and has responded.
    *
    * - Byte Count: 8-bit value indicating the number of data bytes contained 
    * within this response.
    *
    * - First Reg (Hi): The top (most significant) eight bits of a 16-bit number
    * representing the first register requested in the query. 
    *
    * - First Register (Lo)*: The bottom (least significant) eight bits of a 
    * 16-bit number representing the first register requested in the query. 
    *
    * - Second Register (Hi): The top (most significant) eight bits of a 16-bit
    * number representing the second register requested in the query. 
    *
    * - Second Register (Lo): The bottom (least significant) eight bits of a 
    * 16-bit number representing the second register requested in the query.
    *
    * - Error Check (Lo): The bottom (least significant) eight bits of a 16-bit 
    * number representing the error check value. 
    *
    * - Error Check (Hi): The top (most significant) eight bits of a 16-bit 
    * number representing the error check value. 
    */
    enum RESPOND_INDEX: byte {
        RE_SLAVE_ADDR_INDEX,
        RE_FUNC_INDEX,
        RE_BYTE_COUNT_INDEX,
        RE_HIGH_REG_HIGH_BYTE_INDEX,
        RE_HIGH_REG_LOW_BYTE_INDEX,
        RE_LOW_REG_HIGH_BYTE_INDEX,
        RE_LOW_REG_LOW_BYTE_INDEX,
        RE_ERROR_CHECK_LOW_INDEX,
        RE_ERROR_CHECK_HIGH_INDEX,

        RESPONSE_FRAME_SIZE
    };

    enum Status: byte {
        STATUS_ERROR          = 0x00,
        STATUS_SUCCESS        = 0x01,
        ERR_READ_RS485        = 0x02,
        ERR_FUNC              = 0xFA,
    };

    /* struct of write command */
    typedef struct {
        size_t size;
        byte cmd[WRITE_COMMAND_LENGTH];
    } WCommand;

    /* struct of read command */
    typedef struct {
        size_t size;
        byte cmd[READ_COMMAND_LENGTH];
    } RCommand;

    /**
    * Default constructor
    * - Baud Rate: 2400 bps
    * - RS485 Control Pin: 4
    * - SDM 120 Node ID: 0x01
    * - Hardware Serial: Serial1
    */
    SDM();

    /**
    * @brief Copy Constructor
    * @param
    * - serial: HardwareSerial used in the system (e.g. Serial1)
    * - baudRate: baud rate of SDM-120 Modbus
    * - protocol: default protocol (SERIAL_8N1: 8 data bits, no parity, 1 stop bit)
    * - rxPin: RX
    * - txPin; TX
    * - ctlPin: RS485 control pin
    */
    SDM(HardwareSerial& serial, long baudRate, byte ctlPin);

    /**
    * @brief Copy Constructor
    * @param
    * - serial: HardwareSerial used in the system (e.g. Serial1)
    * - baudRate: baud rate of SDM-120 Modbus
    * - protocol: protocol RS485
    * - rxPin: RX
    * - txPin; TX
    * - ctlPin: RS485 control pin
    */
    SDM(HardwareSerial& serial, long baudRate, byte protocol, byte ctlPin);

    /**
    * @brief Initialise SDM-120 Serial
    * @param none
    * @return none
    */
    void begin(const byte _nodeId);

    /**
    * @brief Set node ID of SDM-120
    * @param 
    * - node: constant node ID
    * @return none
    */
    void setNodeId(const byte node);

    /**
    * @brief Ensure the preservation of data.
    *
    * @param
    * - receivedData: pointer to array of bytes of received data from RS485
    * - size: number of elements in array.
    *
    * @return true if the data are preserved, otherwise false.
    */
    bool isDataPreserved(byte* receivedData, size_t size);

    /**
    * @brief Get bytes of data from SDM120
    *
    * @param[in]
    * - request: array of request to be sent to SDM120
    * - size: size of request array
    * @param[out]
    * - reData: response array is stored in reData
    *
    * @return
    * - STATUS_SUCCESS: successfully get bytes of data from SDM120
    * - ERR_READ_RS485: no read data from RS485. 
    */
    Status getRawData(byte* reData, byte* request, size_t size);

    /**
    * @brief Get float value based on the requested type of data
    *
    * @param
    * - type:
    *    + SDM::VOLTAGE: Get voltage (Unit: Volts)
    *    + SDM::CURRENT: Get current (Unit: Amps)
    *    + SDM::FREQUENCY: Get frequency (Unit: Hz)
    *    + SDM::TOTAL_ACTIVE_ENERGT: Get total energy (Unit: kWh)
    *
    * @return float
    */
    float getValue(Type type);

// protected:
    /**
    * @brief Construct command sending to SDM120 to read register values
    *
    * @param
    * - func:
    *    + FUNC_READ_INPUT_REG: This function allows to read values of input 
    *    register (variable registers)(e.g. voltages, current, frequency, etc.)
    *    + FUNC_READ_HOLDING_REG: This function allows to read values of holding
    *    register (preset registers).
    *    (e.g. baud rate, node id, network parity stop, etc.) 
    *    + FUNC_WRITE_HOLDING_REG: This function allows to write holding registers
    *    (e.g. change baud rate, or change node id, etc.)
    * - highAddrByte: high byte of the address needs to be read (see TictagSdm120.h)
    * - lowAddrByte: low byte of the address needs to be read (see TictagSdm120.h)
    *
    * @return struct of read command (including size of read command (8), array of command)
    */
    const RCommand _setReadCommand(byte func, 
                                   const byte highAddrByte, 
                                   const byte lowAddrByte);
    /**
    * @brief Construct command sending to SDM120 to change register values
    *
    * @param
    * - highAddrByte: high byte of the address needs to be written (see TictagSdm120.h)
    * - lowAddrByte: low byte of the address needs to be written (see TictagSdm120.h)
    * - dataHighRegHighByte: data to be written (high register, high byte)
    * - dataHighRegLowByte: data to be written (high register, low byte)
    * - dataLowRegHighByte: data to be written (low register, high byte)
    * - dataLowRegLowByte: data to be written (low register, low byte)
    *
    * @return struct of write command (including size of write command (13), 
    * array of command)
    */
    const WCommand _setWriteCommand(const byte highAddrByte, const byte lowAddrByte, 
                                    byte dataHighRegHighByte, byte dataHighRegLowByte, 
                                    byte dataLowRegHighByte, byte dataLowRegLowByte);
    /**
    * @brief Set baud rate for SDM120 (in Setting mode)
    * @param 
    * - baudRate: baud rate needs to be set to SDM120
    * @return
    * - STATUS_SUCCESS: set baud rate successfully
    */
    Status _setSDMBaudRate(const long baudRate);

    /**
    * @brief Set Node ID of SDM120 (in Setting mode)
    * @param
    * - nodeID: ID needs to be set to SDM120
    * @return
    * - STATUS_SUCCESS: set Node ID successfully.
    */
    Status _setSDMNodeID(const float nodeID);

    /**
    * @brief Debug function: view read command to be sent
    * @param 
    * - rCmd: struct (RCommand) of read command to be sent
    * @return none
    */
    void _debugViewCommand(RCommand rCmd);

    /**
    * @brief Debug function: wiew write command to be sent
    * @param 
    * - wCmd: struct (WCommand) of write command to be sent
    * @return none
    */
    void _debugViewCommand(WCommand wCmd);
    
private:
    /**
    * @brief CRC-16 Calculator (polynomial of 0xA001)
    *
    * @param
    * - data: array of bytes of data needed to calculate checksum.
    * - size: number of elements in array.
    *
    * @return 16 bits of CRC-16 checksum
    */
    Crc _calculateCrc(byte* data, size_t size);
    
    Crc _crc;

    byte _nodeId;
    byte _highAddrByte;
    byte _lowAddrByte;
    
    WCommand _wrtCmd;
    RCommand _rdCmd;

    // Config
    long _baudRate;
    SerialProtocol _protocol;
    byte _ctlPin;
    HardwareSerial& _sdmSerial;
};

/** 
* @brief Check the system endianess
* @param none
* @return true if the system is little endian, false otherwise.
*/
bool isLittleEndian();

/**
* @brief Convert 4 bytes to float
* @param
* - regData: 4 bytes need to be converted to float (array)
* @return converted float
*/
float bytesToFloat(byte* regData);

/**
* @brief Convert IEEE 754 floating point to array of bytes
*
* @param[in]
* - f: float to be converted
* @param[out]
* - bytes: array of converted bytes
*
* @return none
*/
void floatToBytes(byte* bytes, float f);

#endif
