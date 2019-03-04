# SDM120 Modbus Library #
This is an Arduino Library for the multi-functional energy meter SDM120 Modbus.

## Index ##
- [Pre-requisite](#pre-requisite)
- [Installation](#installation)
  * [Windows](#windows)
  * [Linux](#linux)
- [How-to](#how-to)
  * [Convert Float to Array of Bytes (IEEE 754)](#)
  * [Convert Array of Bytes to Float (IEEE 754)](#)
  * [Construct Commands](#construct-commands)
  * [Get Raw Data from SDM120 Modbus](#get-raw-data-from-sdm120-modbus)
  * [Get all Processed Data from SDM120 Modbus](#get-all-processed-data-from-sdm120-modbus)
- [Error Codes](#error-codes)
- [Bugs Reporting](#bugs-reporting)
- [TODO](#todo)

## Pre-requisite ##
- Installed [the lastest version of Arduino](https://www.arduino.cc/en/Main/Software "Download Arduino")
- git

## Installation ##
### Windows ###
- Open `cmd`, enter

```
cd %userprofile%\Documents\Arduino\libraries &&
git clone https://github.com/ngharry/sdm120
```
   to download SDM120 Modbus library.
- Wait until the downloading process finishes.

### Linux ###
[TODO] Add tutorial for Linux here. 

## How to ##
### Convert Float to Array of Bytes (IEEE 754) ###
See [examples/FloatToBytes](examples/FloatToBytes/FloatToBytes.ino).

### Convert Array of Bytes to Float (IEEE 754) ###
See [examples/BytesToFloat](examples/BytesToFloat/BytesToFloat.ino).

### Construct Commands ###
See [examples/ConstructCommands](examples/ConstructCommands/ConstructCommands.ino).

### Get Raw Data from SDM120 Modbus ###
See [examples/GetRawData](examples/GetRawData/GetRawData.ino).

### Get all Processed Data from SDM120 Modbus ###
See [examples/GetData](examples/GetData/GetData.ino).

## Error Codes ##

| HEX | DEC | Name | Description |
|:----------:|:------------:|:--------|:-------|
|0x00 | 0 | STATUS_ERROR | General errors |
|0x01 | 1 | STATUS_SUCCESS| Function runs successfully, no errors |
|0x02 | 2 | ERR_READ_RS485 | No data read from RS485 communication |
|0xFA | 250 | ERR_FUNC | Unrecognised function sent to SDM120 |

## Bugs Reporting ## 

## TODO ##
- [ ] Add terminology table.
- [ ] Add more error codes (as specified in `doc/SDM120 PROTOCOL.pdf`).
- [ ] Add intalling process for Linux (I love Linux).
