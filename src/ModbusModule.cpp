#include "ModbusModule.h"
#include "HardwareConfig.h"

// #define DEVICE_SMARTMF_MODBUS_RTU_3BE
#define SMARTMF_MODBUS_SERIAL Serial2

bool modbusModule::idle_processing = false;

modbusModule::modbusModule() {}

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
    logInfoP("Setup0");
    logIndentUp();

// setup Pins
#ifdef DEVICE_SMARTMF_MODBUS_RTU_3BE
    pinMode(SMARTMF_MODBUS_DIR_PIN, OUTPUT);
    SMARTMF_MODBUS_SERIAL.setRX(SMARTMF_MODBUS_RX_PIN);
    SMARTMF_MODBUS_SERIAL.setTX(SMARTMF_MODBUS_TX_PIN);
#endif
    SMARTMF_MODBUS_SERIAL.begin(9600);

    if (configured)
    {
        // setupCustomFlash();                               // ********************************* anpassen wenn notwendig *********************
        setupChannels();
    }
    logIndentDown();
}

void modbusModule::setupChannels()
{
    uint8_t slave_id = 1;

    // for (uint8_t i = 0; i < ParamMOD_VisibleChannels; i++)
    for (uint8_t i = 0; i < 10; i++)
    {
        _channels[i] = new modbusChannel(i, slave_id, 3, 3, SMARTMF_MODBUS_SERIAL);
        _channels[i]->setup();
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

void modbusModule::modbus_idle()
{
    idle_processing = true;
    openknx.loop();
    idle_processing = false;
}

void modbusModule::loop(bool configured)
{
    if (idle_processing)
    {
        return;
    }

    if (delayCheck(_timer1, 5100))
    {
        logInfoP("Loop0");
        _timer1 = millis();
    }

    if (configured)
    {
        if (ParamMOD_VisibleChannels == 0) return;

        uint8_t processed = 0;
        do
            _channels[_currentChannel]->loop();

        while (openknx.freeLoopIterate(ParamMOD_VisibleChannels, _currentChannel, processed));
    }
}

#ifdef OPENKNX_DUALCORE

void modbusModule::setup1(bool configured)
{
    delay(1000);
    // logInfoP("Setup1");
}

void modbusModule::loop1(bool configured)
{
    if (delayCheck(_timer2, 7200))
    {
        // logInfoP("Loop1");
        _timer2 = millis();
    }
}
#endif

void modbusModule::processInputKo(GroupObject& ko)
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
    openknxUsbExchangeModule.onLoad("modbus.txt", [](UsbExchangeFile* file) -> void {
        file->write("Demo");
    });
    openknxUsbExchangeModule.onEject("modbus.txt", [](UsbExchangeFile* file) -> bool {
        // File is required
        if (file == nullptr)
        {
            logError("modbusModule", "File modbus.txt was deleted but is mandatory");
            return false;
        }
        return true;
    });
}
    #endif
#endif

modbusModule openknxmodbusModule;