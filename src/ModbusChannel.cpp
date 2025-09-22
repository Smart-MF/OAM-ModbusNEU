#include "ModbusChannel.h"
#include "Arduino.h"

uint32_t timer1sec;

modbusChannel::modbusChannel(uint8_t index, uint8_t id, uint8_t baud_value, uint8_t parity_value, HardwareSerial &serial) : _serial(serial)
{
    _channelIndex = index;
    _modbus_ID = id;
    _baud_value = baud_value;
    _parity_value = parity_value;
    //_serial = serial;
}

const std::string modbusChannel::name()
{
    return "modbusChannel";
}

void modbusChannel::setup()
{
    logInfoP("setup");
    logIndentUp();
    logDebugP("debug setup");
    logTraceP("trace setup");
    logIndentDown();
}

void modbusChannel::loop()
{
    if (delayCheck(timer1sec, 1000))
    {
        timer1sec = millis();
        logInfoP("loop");
    }
}

bool modbusChannel::modbusParitySerial(uint32_t baud, HardwareSerial &serial)
{
    switch (_parity_value)
    {
    case 0: // Even (1 stop bit)
        serial.begin(baud, SERIAL_8E1);
        logDebugP.println("Parity: Even (1 stop bit)");
        return true;
        break;
    case 1: // Odd (1 stop bit)
        serial.begin(baud, SERIAL_8O1);
        logDebugP.println("Parity: Odd (1 stop bit)");
        return true;
        break;
    case 2: // None (2 stop bits)
        serial.begin(baud, SERIAL_8N2);
        logDebugP.println("Parity: None (2 stop bits)");
        return true;
        break;
    case 3: // None (1 stop bit)
        serial.begin(baud, SERIAL_8N1);
        logDebugP.println("Parity: None (1 stop bit)");
        return true;
        break;

    default:
        logDebugP.print("Parity: Error: ");
        logDebugP.println(_parity_value);
        return false;
        break;
    }
}

bool modbusChannel::modbusInitSerial(HardwareSerial &serial)
{
    // Set Modbus communication baudrate
    switch (_baud_value)
    {
    case 0:
        logDebugP.println("Baudrate: 1200kBit/s");
        return modbusParitySerial(1200, serial);

        break;
    case 1:
        logDebugP.println("Baudrate: 2400kBit/s");
        return modbusParitySerial(2400, serial);
        break;
    case 2:
        logDebugP.println("Baudrate: 4800kBit/s");
        return modbusParitySerial(4800, serial);
        break;
    case 3:
        logDebugP.println("Baudrate: 9600kBit/s");
        return modbusParitySerial(9600, serial);
        break;
    case 4:
        logDebugP.println("Baudrate: 19200kBit/s");
        return modbusParitySerial(19200, serial);
        break;
    case 5:
        logDebugP.println("Baudrate: 38400kBit/s");
        return modbusParitySerial(38400, serial);
        break;
    case 6:
        logDebugP.println("Baudrate: 56000kBit/s");
        return modbusParitySerial(56000, serial);
        break;
    case 7:
        logDebugP.println("Baudrate: 115200kBit/s");
        return modbusParitySerial(115200, serial);
        break;
    default:
        logDebugP.print("Baudrate: Error: ");
        logDebugP.println(_baud_value);
        return false;
        break;
    }
}
