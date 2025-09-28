#include "ModbusModule.h"
#include "HardwareConfig.h"
#include "ModBusMaster.h"

// #define DEVICE_SMARTMF_MODBUS_RTU_3BE
#define SMARTMF_MODBUS_SERIAL Serial2

uint32_t timer_time_between_Reg_Reads;
uint32_t timer_time_between_Cycle_Reads;

uint32_t idle_Count_Test = 0;

bool modbusModule::idle_processing = false;

modbusModule::modbusModule()
{
    idle(modbusModule::idleCallback);
}

const std::string modbusModule::name()
{
    return "modbus";
}

const std::string modbusModule::version()
{
    // hides the module in the version output on the console, because the firmware version is sufficient.
    return "";
}

void modbusModule::setup(bool configured)
{
    // delay(1000);
    logDebugP("Setup0");
    logIndentUp();

    // setup Pins
#ifdef DEVICE_SMARTMF_1TE_MODBUS
    pinMode(SMARTMF_LED, OUTPUT);
    digitalWrite(SMARTMF_LED, LOW);
#endif
    pinMode(SMARTMF_MODBUS_DIR_PIN, OUTPUT);
    SMARTMF_MODBUS_SERIAL.setRX(SMARTMF_MODBUS_RX_PIN);
    SMARTMF_MODBUS_SERIAL.setTX(SMARTMF_MODBUS_TX_PIN);

    modbusInitSerial(SMARTMF_MODBUS_SERIAL);

    if (configured)
    {
        // setupCustomFlash();                               // ********************************* anpassen wenn notwendig *********************
        setupChannels();
    }
    logIndentDown();
}

void modbusModule::setupChannels()
{
    pinMode(SMARTMF_MODBUS_DIR_PIN, OUTPUT);
    digitalWrite(SMARTMF_MODBUS_DIR_PIN, 0);

    // for (uint8_t i = 0; i < ParamMOD_VisibleChannels; i++)
    for (uint8_t i = 0; i < 10; i++)
    {
        _channels[i] = new modbusChannel(i, 3, 3, SMARTMF_MODBUS_SERIAL);
        _channels[i]->setup();
        _channels[i]->idle(idleCallback);
        _channels[i]->preTransmission(preTransmission);
        _channels[i]->postTransmission(postTransmission);
    }
}

void modbusModule::setupCustomFlash()
{
    logDebugP("initialize modbus flash");
    OpenKNX::Flash::Driver _modbusStorage;
#ifdef ARDUINO_ARCH_ESP32
    _modbusStorage.init("modbus");
#else
    _modbusStorage.init("modbus", modbus_FLASH_OFFSET, modbus_FLASH_SIZE);
#endif

    logTraceP("write modbus data");
    // _modbusStorage.writeByte(0, 0x11);
    // _modbusStorage.writeWord(1, 0xFFFF);
    // _modbusStorage.writeInt(3, 6666666);
    // for (size_t i = 0; i < 4095; i++)
    // {
    //     _modbusStorage.writeByte(i, i);
    // }
    // _modbusStorage.commit();

    logDebugP("read modbus data");
    logIndentUp();
    // logHexDebugP(_modbusStorage.flashAddress(), 4095);
    // logDebugP("byte: %02X", _modbusStorage.readByte(0)); // UINT8
    // logDebugP("word: %i", _modbusStorage.readWord(1));   // UINT16
    // logDebugP("int: %i", _modbusStorage.readInt(3));     // UINT32

    logIndentDown();
}

void modbusModule::idleCallback()
{
    idle_processing = true;
    openknx.loop();
    idle_processing = false;
}

void modbusModule::preTransmission()
{
    digitalWrite(SMARTMF_MODBUS_DIR_PIN, 1);
}

void modbusModule::postTransmission()
{
    digitalWrite(SMARTMF_MODBUS_DIR_PIN, 0);
}

void modbusModule::loop(bool configured)
{

    if (delayCheck(_timer1, 1000))
    {
        logDebugP("LoopModule");
        _timer1 = millis();
        logDebugP("CH%i:", _channel);
    }

    if (configured)
    {
        if (ParamMOD_VisibleChannels == 0)
            return;

        uint8_t processed = 0;
        uint8_t result;
        do
        {
            _channels[_currentChannel]->loop();
            if (!idle_processing)
            {
                result = _channels[_channel]->readModbus(true);

                switch (result)
                {
                case 0:
                    // logDebugP("ERROR");
                    _channel++;
                    break;
                case 1:
                    _channel++;
                    break;
                case 0x02: // ku8MBIllegalDataAddress
                    _channel++;
                    break;
                case 0x03: // ku8MBIllegalDataValue
                    _channel++;
                    break;
                case 0x04: // ku8MBSlaveDeviceFailure
                    _channel++;
                    break;
                case 0xE0: // ku8MBInvalidSlaveID
                    _channel++;
                    break;
                case 0xE1: // ku8MBInvalidFunction
                    _channel++;
                    break;
                case 0xE2: // ku8MBResponseTimedOut
                    logDebugP("ERROR: Response Timed Out");
                    _channel++;
                    break;

                default:
                    break;
                }
                if (_channel >= ParamMOD_VisibleChannels)
                    _channel = 0;

                digitalWrite(15, LOW);
                logDebugP("cout: %i", idle_Count_Test);
                idle_Count_Test = 0;
            }
        } while (openknx.freeLoopIterate(ParamMOD_VisibleChannels, _currentChannel, processed));
    }
}

#ifdef OPENKNX_DUALCORE

void modbusModule::setup1(bool configured)
{
    delay(1000);
    // logDebugP("Setup1");
}

void modbusModule::loop1(bool configured)
{
    if (delayCheck(_timer2, 7200))
    {
        // logDebugP("Loop1");
        _timer2 = millis();
    }
}
#endif

void modbusModule::processInputKo(GroupObject &ko)
{
    // logDebugP("processInputKo GA%04X", ko.asap());
    // logHexDebugP(ko.valueRef(), ko.valueSize());
}

void modbusModule::showHelp()
{
    openknx.console.printHelpLine("modbus", "Print a modbus text");
}

bool modbusModule::processCommand(const std::string cmd, bool diagnoseKo)
{
    if (cmd.substr(0, 5) == "modbus")
    {
        logInfoP("modbus Info");
        logIndentUp();
        logInfoP("Info 1");
        logInfoP("Info 2");
        logIndentUp();
        logInfoP("Info 2a");
        logInfoP("Info 2b");
        logIndentDown();
        logInfoP("Info 3");
        logIndentDown();
        return true;
    }

    return false;
}

#ifdef ARDUINO_ARCH_RP2040
#ifndef OPENKNX_USB_EXCHANGE_IGNORE
void modbusModule::registerUsbExchangeCallbacks()
{
    // Sample
    openknxUsbExchangeModule.onLoad("modbus.txt", [](UsbExchangeFile *file) -> void
                                    { file->write("Demo"); });
    openknxUsbExchangeModule.onEject("modbus.txt", [](UsbExchangeFile *file) -> bool
                                     {
        // File is required
        if (file == nullptr)
        {
            logError("modbusModule", "File modbus.txt was deleted but is mandatory");
            return false;
        }
        return true; });
}
#endif
#endif

bool modbusModule::modbusParitySerial(uint32_t baud, HardwareSerial &serial)
{
    switch (ParamMOD_BusParitySelection)
    {
    case 0: // Even (1 stop bit)
        serial.begin(baud, SERIAL_8E1);
        logInfoP("Parity: Even (1 stop bit)");
        return true;
        break;
    case 1: // Odd (1 stop bit)
        serial.begin(baud, SERIAL_8O1);
        logInfoP("Parity: Odd (1 stop bit)");
        return true;
        break;
    case 2: // None (2 stop bits)
        serial.begin(baud, SERIAL_8N2);
        logInfoP("Parity: None (2 stop bits)");
        return true;
        break;
    case 3: // None (1 stop bit)
        serial.begin(baud, SERIAL_8N1);
        logInfoP("Parity: None (1 stop bit)");
        return true;
        break;

    default:
        logInfoP("Parity: Error: %i", ParamMOD_BusParitySelection);
        return false;
        break;
    }
}

bool modbusModule::modbusInitSerial(HardwareSerial &serial)
{
    // Set Modbus communication baudrate
    switch (ParamMOD_BusBaudrateSelection)
    {
    case 0:
        logInfoP("Baudrate: 1200kBit/s");
        return modbusParitySerial(1200, serial);

        break;
    case 1:
        logInfoP("Baudrate: 2400kBit/s");
        return modbusParitySerial(2400, serial);
        break;
    case 2:
        logInfoP("Baudrate: 4800kBit/s");
        return modbusParitySerial(4800, serial);
        break;
    case 3:
        logInfoP("Baudrate: 9600kBit/s");
        return modbusParitySerial(9600, serial);
        break;
    case 4:
        logInfoP("Baudrate: 19200kBit/s");
        return modbusParitySerial(19200, serial);
        break;
    case 5:
        logInfoP("Baudrate: 38400kBit/s");
        return modbusParitySerial(38400, serial);
        break;
    case 6:
        logInfoP("Baudrate: 56000kBit/s");
        return modbusParitySerial(56000, serial);
        break;
    case 7:
        logInfoP("Baudrate: 115200kBit/s");
        return modbusParitySerial(115200, serial);
        break;
    default:
        logInfoP("Baudrate: Error: %i", ParamMOD_BusBaudrateSelection);
        return false;
        break;
    }
}

modbusModule openknxmodbusModule;