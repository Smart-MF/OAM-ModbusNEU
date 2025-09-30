#include "ModbusChannel.h"
#include "Arduino.h"

#define Serial_Debug_Modbus_Min
#define Serial_Debug_Modbus




// bool modbusChannel::idle_processing = false;

modbusChannel::modbusChannel(uint8_t index, uint8_t baud_value, uint8_t parity_value, HardwareSerial &serial) : _serial(serial)
{
    _channelIndex = index;
    _baud_value = baud_value;
    _parity_value = parity_value;

    // idle(modbusChannel::idleCallback);
}

const std::string modbusChannel::name()
{
    return "modbusChannel";
}

void modbusChannel::setup()
{
    logInfoP("setup ");
    logIndentUp();
    logDebugP("debug setup");
    logTraceP("trace setup");
    logIndentDown();

    // Set Counter
    _readCyclecounter = ParamMOD_CHModBusReadCycle;

    switch (ParamMOD_CHModbusSlaveSelection)
    {
    case 1:
        _modbus_ID = ParamMOD_BusID_Slave1;
        break;
    case 2:
        _modbus_ID = ParamMOD_BusID_Slave2;
        break;
    case 3:
        _modbus_ID = ParamMOD_BusID_Slave3;
        break;
    case 4:
        _modbus_ID = ParamMOD_BusID_Slave4;
        break;
    case 5:
        _modbus_ID = ParamMOD_BusID_Slave5;
        break;
    case 6:
        _modbus_ID = ParamMOD_BusID_Slave6;
        break;
    case 7:
        _modbus_ID = ParamMOD_BusID_Slave7;
        break;
    case 8:
        _modbus_ID = ParamMOD_BusID_Slave8;
        break;
    case 9:
        _modbus_ID = ParamMOD_BusID_Slave9;
        break;
    case 10:
        _modbus_ID = ParamMOD_BusID_Slave10;
        break;

    default:
        logInfoP("MODBBUS wrong Slave ");
        logInfoP("Slave %i:", MOD_CHModbusSlaveSelection);
        break;
    }

    begin(_modbus_ID, _serial);
}

void modbusChannel::loop()
{
    sendKNX();
 
    // if (delayCheck(timer1sec, 10000))
    //{
    //     timer1sec = millis();
    //     logDebugP("loopChannel");
    // }
}


bool modbusChannel::isActiveCH()
{
    if (ParamMOD_CHModbusSlaveSelection == 0)
        return false;
    else
        return true;
}

bool modbusChannel::isReadyCH()
{
    if (ParamMOD_CHModbusSlaveSelection != 0 && _readCyclecounter == 1)
        return true;
    else
    {
        _readCyclecounter--;
        return false;
    }
}

uint8_t modbusChannel::readModbus(bool readRequest)
{
    // 1. DPT auslesen: bei 0 abbrechen
    // 2. Richtung bestimmen: KNX - Modbus / Modbus - KNX
    // 3. Funktion bestimmen: 0x03 Lese holding Reg, ...
    // 4. Register bestimmen
    // 5. Registertyp wenn notwendig
    // 6. Register Position wenn notwendig
    // 7. Modbus Wert abfragen
    // 8. KNX Botschaft senden

    if (ParamMOD_CHModbusSlaveSelection == 0 || _modbus_ID == 0)
        return 0; // Kein Slave ausgewählt = Channel inaktiv, daher abbruch

    if (ParamMOD_CHModBusDptSelection == 0 || ParamMOD_CHModBusDptSelection > 14)
        return 0; // Kein DPT ausgewählt, oder dpt >14, daher abbruch

    // wird in Funktion isReadyCH() bei jeder Abfrage um 1 dekrementiert
    _readCyclecounter = ParamMOD_CHModBusReadCycle; // setzt Abfragezähler wieder zurück

    ErrorHandlingLED();

    // Richtungsauswahl: KNX - Modbus oder Modbus - KNX
    switch (ParamMOD_CHModBusBusDirection)
    {
    case 0: // KNX -> modbus
#ifdef Serial_Debug_Modbus_Min
        if (readRequest)
        {

            logDebugP("KNX->Modbus ");
        }
#endif
        if (_readyToSend)
        {
            _readyToSend = false;
            return sendModbus();
        }
        return false;
        break;

    case 1: // modbus -> KNX
        return modbusToKnx(ParamMOD_CHModBusDptSelection, readRequest);
        break;

    default:
        return false;
        break;
    }
}

bool modbusChannel::sendModbus()
{
    uint8_t dpt = ParamMOD_CHModBusDptSelection;
    if (dpt == 0) // Kein DPT ausgewählt, daher abbruch
        return false;
    else if (dpt > 14) // >14 dann ist der dpt nicht in Spec, damit abbruch
        return false;

    return knxToModbus(dpt, true);
}

void modbusChannel::ErrorHandling()
{
    if (errorState[0] == false && errorState[1] == false)
    {
        errorState[0] = true;
    }
    // set Fehler Stufe 2
    else if (errorState[0] == true && errorState[1] == false)
    {
        errorState[1] = true;
    }
}

void modbusChannel::ErrorHandlingLED()
{
    bool error = false;
    for (int i = 0; i < MOD_ChannelCount; i++)
    {
        if (errorState[0])
        {
            error = true;
        }
    }
    //    if (error)
    //        setLED_ERROR(HIGH);
    //    else
    //        setLED_ERROR(LOW);
}

inline uint16_t modbusChannel::adjustRegisterAddress(uint16_t u16ReadAddress, uint8_t RegisterStart)
{
    return u16ReadAddress && RegisterStart ? u16ReadAddress - 1 : u16ReadAddress;
}

void modbusChannel::sendKNX()
{
    uint32_t lCycle = ParamMOD_CHModBusSendDelay * 1000;
    // if cyclic sending is requested, send the last value if one is available
    if (lCycle && delayCheck(sendDelay, lCycle))
    {
        logInfoP("CH%i",_channelIndex);
        KoMOD_GO_BASE_.objectWritten();
        sendDelay = millis();
    }
}

/**********************************************************************************************************
 **********************************************************************************************************
 *  Modbus to KNX
 *
 *
 ***********************************************************************************************************
 **********************************************************************************************************/

// Routine zum Einlesen des ModBus-Register mit senden auf KNX-Bus
uint8_t modbusChannel::modbusToKnx(uint8_t dpt, bool readRequest)
{

    bool lSend = 0; // = readRequest; // && !valueValid; // Flag if value should be send on KNX
    uint8_t result = 0;
    uint16_t registerAddr = ParamMOD_CHModbusRegister; // adjustRegisterAddress(ParamMOD_CHModbusRegister, ParamMOD);

#ifdef Serial_Debug_Modbus
    logDebugP("Modbus->KNX %i, ID:%i, Slave:%i", registerAddr, _modbus_ID, ParamMOD_CHModbusSlaveSelection);
#endif

    // uint32_t lCycle = ParamMOD_CHModBusSendDelay * 1000;

    // if cyclic sending is requested, send the last value if one is available
    // lSend = (lCycle && delayCheck(sendDelay, lCycle)); //  && valueValid);

    // wählt den passenden DPT
    switch (dpt)
    {
    //*****************************************************************************************************************************************
    //*****************************************  DPT 1.001 ************************************************************************************
    //*****************************************************************************************************************************************
    case 1:
        if (readRequest)
        {
#ifdef Serial_Debug_Modbus
            logDebugP("DPT1 |");
#endif
            // clear Responsebuffer before revicing a new message
            clearResponseBuffer();

            bool v = false;

            // Bit Register
            if (ParamMOD_CHModBusInputTypDpt1 == 0)
            {
                switch (ParamMOD_CHModBusReadBitFunktion) // Choose Modbus Funktion (0x01 readHoldingRegisters ODER 0x02 readInputRegisters)
                {
                case 1: // 0x01 Lese Coils
#ifdef Serial_Debug_Modbus
                    logDebugP(" 0x01 ");
#endif
                    result = readCoils(registerAddr, 1);
                    break;

                case 2:
#ifdef Serial_Debug_Modbus
                    logDebugP(" 0x02 ");
#endif
                    result = readDiscreteInputs(registerAddr, 1);
                    break;
                default: // default Switch(ModBusReadBitFunktion)
                    return result;
                }
                if (result == ku8MBSuccess)
                {
                    v = getResponseBuffer(0);
                }
            }
            // Bit in Word
            else if (ParamMOD_CHModBusInputTypDpt1 == 1)
            {
                switch (ParamMOD_CHModBusReadWordFunktion)
                {
                case 3: // 0x03 Lese holding registers
#ifdef Serial_Debug_Modbus
                    logDebugP(" 0x03 ");
#endif
                    result = readHoldingRegisters(registerAddr, 1);
                    break; // Ende 0x03

                case 4:
#ifdef Serial_Debug_Modbus
                    logDebugP(" 0x04 ");
#endif
                    result = readInputRegisters(registerAddr, 1);
                    break;
                default: // Error Switch (0x03 & 0x04)
                    return result;
                } // Ende Switch (0x03 & 0x04)
                if (result == ku8MBSuccess)
                {
                    v = (getResponseBuffer(0) >> ParamMOD_CHModBusBitPosDpt1 & 1);
                }
            }
            else
            {
                return result;
            }

            if (result == ku8MBSuccess)
            {
                // Invertiert
                if (ParamMOD_CHModBusInputTypInvDpt1 > 0)
                {
                    v = !v;
                }
                // senden bei Wertänderung
                bool lAbsoluteBool = ParamMOD_CHModBusValueChange;
                lSend |= (lAbsoluteBool && v != lastSentValue.lValueBool);

                // we always store the new value in KO, even it it is not sent (to satisfy potential read request)
                KoMOD_GO_BASE_.valueNoSend(v, DPT_Bool);

                if (lSend)
                {
                    lastSentValue.lValueBool = v;
                }

#ifdef Serial_Debug_Modbus_Min
                logDebugP("%i", v);
#endif

                // Löscht Fehlerspeicher
                errorState[0] = false;
                errorState[1] = false;
            }
            else // Fehler bei der Übertragung
            {
#ifdef Serial_Debug_Modbus_Min
                logDebugP("ERROR: %u", result, HEX);
#endif
                ErrorHandling();

                return result;
            }
        }
        break;
    //*****************************************************************************************************************************************
    //*****************************************  DPT 5.001 ************************************************************************************
    //*****************************************************************************************************************************************
    case 4:
        if (readRequest)
        {
#ifdef Serial_Debug_Modbus
            logDebugP("DPT5.001 |");
#endif

            // clear Responsebuffer before revicing a new message
            clearResponseBuffer();

            uint8_t v;

            // Choose Modbus Funktion (0x03 readHoldingRegisters ODER 0x04 readInputRegisters)
            switch (ParamMOD_CHModBusReadWordFunktion)
            {
            case 3: // 0x03 Lese holding registers
#ifdef Serial_Debug_Modbus
                logDebugP(" 0x03 ");
#endif
                result = readHoldingRegisters(registerAddr, 1);
                break; // Ende 0x03

            case 4:
#ifdef Serial_Debug_Modbus
                logDebugP(" 0x04 ");
#endif
                result = readInputRegisters(registerAddr, 1);
                break;
            default: // Error Switch (0x03 & 0x04)
                return result;
            } // Ende Switch (0x03 & 0x04)

            if (result == ku8MBSuccess)
            {
                switch (ParamMOD_CHModBusRegisterPosDPT5)
                {
                case 1: // High Byte
                    v = (getResponseBuffer(0) >> 8);
                    break;
                case 2: // Low Byte
                    v = getResponseBuffer(0);
                    break;
                case 3: // frei Wählbar
                    v = (getResponseBuffer(0) >> (ParamMOD_CHModBusOffsetRight5));
                    v = v & (ParamMOD_CHModbusCountBitsDPT56);
#ifdef Serial_Debug_Modbus
                    logDebugP("%i", v, BIN);

#endif
                    break;
                default:
                    return result;
                } // Ende Register Pos

                // senden bei Wertänderung
                uint16_t lAbsolute = ParamMOD_CHModBusValueChange;
                uint8_t lDiff = abs(v - lastSentValue.lValueUint8_t);
                if (lAbsolute > 0 && lDiff >= lAbsolute)
                    lSend = true;

                // we always store the new value in KO, even it it is not sent (to satisfy potential read request)
                KoMOD_GO_BASE_.valueNoSend(v, DPT_Scaling);
                if (lSend)
                {
                    lastSentValue.lValueUint8_t = v;
                }

#ifdef Serial_Debug_Modbus_Min
                logDebugP("%i", v);
#endif

                // Löscht Fehlerspeicher
                errorState[0] = false;
                errorState[1] = false;
            }
            else // Fehler in der Übtragung
            {
#ifdef Serial_Debug_Modbus_Min
                logInfoP("ERROR: %i", result, HEX);
#endif
                ErrorHandling();

                return result;
            }
        }
        break;
    //*****************************************************************************************************************************************
    //*****************************************  DPT 5.010 ************************************************************************************
    //*****************************************************************************************************************************************
    case 5:
        if (readRequest)
        {
#ifdef Serial_Debug_Modbus
            logDebugP("DPT5 |");
#endif

            // clear Responsebuffer before revicing a new message
            clearResponseBuffer();

            uint8_t v;

            // Choose Modbus Funktion (0x03 readHoldingRegisters ODER 0x04 readInputRegisters)
            switch (ParamMOD_CHModBusReadWordFunktion)
            {
            case 3: // 0x03 Lese holding registers
#ifdef Serial_Debug_Modbus
                logDebugP(" 0x03 ");
#endif
                result = readHoldingRegisters(registerAddr, 1);
                break; // Ende 0x03

            case 4:
#ifdef Serial_Debug_Modbus
                logDebugP(" 0x04 ");
#endif
                result = readInputRegisters(registerAddr, 1);
                break;
            default: // Error Switch (0x03 & 0x04)
                return result;
            } // Ende Switch (0x03 & 0x04)

            if (result == ku8MBSuccess)
            {
                switch (ParamMOD_CHModBusRegisterPosDPT5)
                {
                case 1: // High Byte
                    v = (getResponseBuffer(0) >> 8);
                    break;
                case 2: // Low Byte
                    v = getResponseBuffer(0);
                    break;
                case 3: // frei Wählbar
                    v = (getResponseBuffer(0) >> (ParamMOD_CHModBusOffsetRight5));
                    v = v & (ParamMOD_CHModbusCountBitsDPT56);
                    logDebugP("%u", v, BIN);
                    break;
                default:
                    return result;
                } // Ende Register Pos

                // senden bei Wertänderung
                uint16_t lAbsolute = ParamMOD_CHModBusValueChange;
                uint8_t lDiff = abs(v - lastSentValue.lValueUint8_t);
                if (lAbsolute > 0 && lDiff >= lAbsolute)
                    lSend = true;

                // we always store the new value in KO, even it it is not sent (to satisfy potential read request)
                KoMOD_GO_BASE_.valueNoSend(v, DPT_Value_1_Ucount);
                if (lSend)
                {
                    lastSentValue.lValueUint8_t = v;
                }

#ifdef Serial_Debug_Modbus_Min
                logDebugP("%u", v);
#endif

                // Löscht Fehlerspeicher
                errorState[0] = false;
                errorState[1] = false;
            }
            else // Fehler in der Übtragung
            {
#ifdef Serial_Debug_Modbus_Min
                logInfoP("ERROR: %i", result, HEX);
#endif

                ErrorHandling();

                return result;
            }
        }
        break;
    //*****************************************************************************************************************************************
    //*****************************************  DPT 7 ***************************************************************************************
    //*****************************************************************************************************************************************
    case 7:
        if (readRequest)
        {
            // clear Responsebuffer before revicing a new message
            clearResponseBuffer();

            uint16_t v;

            // Choose Modbus Funktion (0x03 readHoldingRegisters ODER 0x04 readInputRegisters)
            switch (ParamMOD_CHModBusReadWordFunktion)
            {
            case 3: // 0x03 Lese holding registers
                logDebugP("DPT7 | 0x03 | Reg: %u", registerAddr);
                result = readHoldingRegisters(registerAddr, 1);
                break; // Ende 0x03

            case 4:
                logDebugP("DPT7 | 0x04 | Reg: %u", registerAddr);
                result = readInputRegisters(registerAddr, 1);
                break;
            default: // Error Switch (0x03 & 0x04)
                return result;
            } // Ende Switch (0x03 & 0x04)

            if (result == ku8MBSuccess)
            {
                switch (ParamMOD_CHModBusRegisterPosDPT7)
                {
                case 1: // High/LOW Byte
                    v = getResponseBuffer(0);
                    break;
                case 2: // frei Wählbar
                    v = (getResponseBuffer(0) >> (ParamMOD_CHModBusOffsetRight7));
                    v = v & ((1 << ParamMOD_CHModbusCountBitsDPT7) - 1);
                    break;
                default:
                    return result;
                } // Ende Register Pos

                // senden bei Wertänderung
                uint16_t lAbsolute = ParamMOD_CHModBusValueChange;
                uint16_t lDiff = abs(v - lastSentValue.lValueUint16_t);
                if (lAbsolute > 0 && lDiff >= lAbsolute)
                    lSend = true;

                // we always store the new value in KO, even it it is not sent (to satisfy potential read request)
                KoMOD_GO_BASE_.valueNoSend(v, DPT_Value_2_Ucount);
                if (lSend)
                {
                    lastSentValue.lValueUint16_t = v;
                }

#ifdef Serial_Debug_Modbus_Min
                logDebugP("Value: %u", v);
#endif

                // Löscht Fehlerspeicher
                errorState[0] = false;
                errorState[1] = false;
            }
            else // Fehler in der Übtragung
            {
#ifdef Serial_Debug_Modbus_Min
                logInfoP("ERROR: %i", result, HEX);
#endif
                ErrorHandling();

                return result;
            }
        }
        break;
    //*****************************************************************************************************************************************
    //*****************************************  DPT 8 signed 16Bit ***************************************************************************
    //*****************************************************************************************************************************************
    case 8:
        if (readRequest)
        {
#ifdef Serial_Debug_Modbus
            logDebugP("DPT8 |");
#endif
            // clear Responsebuffer before revicing a new message
            clearResponseBuffer();

            int16_t v;

            // Choose Modbus Funktion (0x03 readHoldingRegisters ODER 0x04 readInputRegisters)
            switch (ParamMOD_CHModBusReadWordFunktion)
            {
            case 3: // 0x03 Lese holding registers
                logDebugP(" 0x03 ");
                result = readHoldingRegisters(registerAddr, 1);
                break; // Ende 0x03

            case 4:
                logDebugP(" 0x04 ");
                result = readInputRegisters(registerAddr, 1);
                break;
            default: // Error Switch (0x03 & 0x04)
                return result;
            } // Ende Switch (0x03 & 0x04)

            if (result == ku8MBSuccess)
            {

                v = (int16_t)getResponseBuffer(0);

                // senden bei Wertänderung
                uint16_t lAbsolute = ParamMOD_CHModBusValueChange;
                uint16_t lDiff = abs(v - lastSentValue.lValueInt16_t);
                if (lAbsolute > 0 && lDiff >= lAbsolute)
                    lSend = true;

                // we always store the new value in KO, even it it is not sent (to satisfy potential read request)
                KoMOD_GO_BASE_.valueNoSend(v, DPT_Value_2_Count);
                if (lSend)
                {
                    lastSentValue.lValueInt16_t = v;
                }

// Serial Output
#ifdef Serial_Debug_Modbus_Min
                logDebugP("%u", v);
#endif

                // Löscht Fehlerspeicher
                errorState[0] = false;
                errorState[1] = false;
            }
            else // Fehler in der Übtragung
            {
#ifdef Serial_Debug_Modbus_Min
                logInfoP("ERROR: %i", result, HEX);
#endif
                ErrorHandling();

                return result;
            }
        }
        break;
    //*****************************************************************************************************************************************
    //*****************************************  DPT 9 ***************************************************************************************
    //*****************************************************************************************************************************************
    case 9:
        if (readRequest)
        {
#ifdef Serial_Debug_Modbus
            logDebugP("DPT9 |");
#endif

            // clear Responsebuffer before revicing a new message
            clearResponseBuffer();

            float v;

            // Choose Modbus Funktion (0x03 readHoldingRegisters ODER 0x04 readInputRegisters)
            switch (ParamMOD_CHModBusReadWordFunktion)
            {
            case 3: // 0x03 Lese holding registers
#ifdef Serial_Debug_Modbus
                logDebugP(" 0x03 ");
#endif
                result = readHoldingRegisters(registerAddr, 1);
                break; // Ende 0x03

            case 4:
#ifdef Serial_Debug_Modbus
                logDebugP(" 0x04 ");
#endif
                result = readInputRegisters(registerAddr, 1);
                break;
            default: // Error Switch (0x03 & 0x04)
                return result;
            } // Ende Switch (0x03 & 0x04)

            if (result == ku8MBSuccess)
            {
                if (ParamMOD_CHModBusRegisterPosDPT9 <= 3)
                {
                    uint16_t uraw;
                    // adapt input value (Low Byte / High Byte / High&Low Byte / .... )
                    switch (ParamMOD_CHModBusRegisterPosDPT9)
                    {
                    case 1: // Low Byte unsigned
                        uraw = (uint8_t)(getResponseBuffer(0) & 0xff);
                        break;
                    case 2: // High Byte unsigned
                        uraw = (uint8_t)((getResponseBuffer(0) >> 8) & 0xff);
                        break;
                    case 3: // High/Low Byte unsigned
                        uraw = getResponseBuffer(0);
                        break;
                    default:
                        return result;
                    }

#ifdef Serial_Debug_Modbus
                    logDebugP("%u", uraw);
#endif
                    //  ************************ MUSS NOCH GEPRÜFT WERDEN !!!!!!!!!!!!!!!!!!!!!!!!!!!
                    v = uraw / (float)ParamMOD_CHModBuscalculationValueDiff;
                    v = v + (int16_t)ParamMOD_CHModBuscalculationValueAdd;
                }
                else
                {
                    int16_t sraw;
                    // adapt input value (Low Byte / High Byte / High&Low Byte / .... )
                    switch (ParamMOD_CHModBusRegisterPosDPT9)
                    {
                    case 4:                                           // Low Byte signed
                        sraw = (int8_t)(getResponseBuffer(0) & 0xff); // muss noch bearbeitet werden !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                        break;
                    case 5:                                                  // High Byte signed
                        sraw = (int8_t)((getResponseBuffer(0) >> 8) & 0xff); // muss noch bearbeitet werden !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                        break;
                    case 6:                                   // High/Low Byte signed
                        sraw = (int16_t)getResponseBuffer(0); // muss noch bearbeitet werden !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                        break;
                    default:
                        return result;
                    }
#ifdef Serial_Debug_Modbus
                    logDebugP("%i", sraw);
#endif
                    //  ************************ MUSS NOCH GEPRÜFT WERDEN !!!!!!!!!!!!!!!!!!!!!!!!!!!
                    v = sraw / (float)ParamMOD_CHModBuscalculationValueDiff;
                    v = v + (int16_t)ParamMOD_CHModBuscalculationValueAdd;
                }

                // senden bei Wertänderung
                float lAbsolute = ParamMOD_CHModBusValueChange / 10.0;
                float lDiff = abs(v - lastSentValue.lValue);
                if (lAbsolute > 0.0f && lDiff >= lAbsolute)
                    lSend = true;

                // lSend |= (lAbsolute && abs(v - lastSentValue.lValue) * 10.0 >= lAbsolute);

                // we always store the new value in KO, even it it is not sent (to satisfy potential read request)
                KoMOD_GO_BASE_.valueNoSend(v, DPT_Value_Temp); //  ************************ MUSS NOCH GEPRÜFT WERDEN Float mit 2 Bytes !!!!!!!!!!!!!!!!!!!!!!!!!!!
                if (lSend)
                {
                    lastSentValue.lValue = v;
                }

// Serial Output
#ifdef Serial_Debug_Modbus_Min
                logDebugP("%f", v, 2);
#endif

                // Löscht Fehlerspeicher
                errorState[0] = false;
                errorState[1] = false;
            }
            else // Fehler in der Übtragung
            {
#ifdef Serial_Debug_Modbus_Min
                logInfoP("ERROR: %i", result, HEX);
#endif
                ErrorHandling();

                return result;
            }
        }
        break;
    //*****************************************************************************************************************************************
    //*****************************************  DPT 12 ***************************************************************************************
    //*****************************************************************************************************************************************
    case 12:
        if (readRequest)
        {
#ifdef Serial_Debug_Modbus
            logDebugP("DPT12 ");
#endif

            // clear Responsebuffer before revicing a new message
            clearResponseBuffer();

            uint32_t v;

            // Bestimmt ob Register-Typ: Word oder Double Word
            switch (ParamMOD_CHModBusWordTyp12) // Choose Word Register OR Double Word Register
            {
            case 0: // Word Register
#ifdef Serial_Debug_Modbus
                logDebugP("| Word ");
#endif
                // Choose Modbus Funktion (0x03 readHoldingRegisters ODER 0x04 readInputRegisters)
                switch (ParamMOD_CHModBusReadWordFunktion)
                {
                case 3: // 0x03 Lese holding registers
#ifdef Serial_Debug_Modbus
                    logDebugP(" 0x03 ");
#endif
                    result = readHoldingRegisters(registerAddr, 1);
                    break;

                case 4:

#ifdef Serial_Debug_Modbus
                    logDebugP(" 0x04 ");
#endif
                    result = readInputRegisters(registerAddr, 1);
                    break;
                default:
                    return result;
                }

                if (result == ku8MBSuccess)
                {
                    // adapt input value (Low Byte / High Byte / High&Low Byte / .... )
                    switch (ParamMOD_CHModBusRegisterPosDPT12)
                    {
                    case 1:                                         // Low Byte signed
                        v = (uint8_t)(getResponseBuffer(0) & 0xff); // muss noch bearbeitet werden !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                        break;
                    case 2:                                                // High Byte signed
                        v = (uint8_t)((getResponseBuffer(0) >> 8) & 0xff); // muss noch bearbeitet werden !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                        break;
                    case 3:                                 // High/Low Byte signed
                        v = (uint16_t)getResponseBuffer(0); // muss noch bearbeitet werden !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                        break;
                    default:
                        return result;
                    }

                    // Löscht Fehlerspeicher
                    errorState[0] = false;
                    errorState[1] = false;
                }
                else // Fehler
                {
#ifdef Serial_Debug_Modbus_Min
                    logInfoP("ERROR: %i", result, HEX);
#endif
                    ErrorHandling();

                    return result;
                }

                break;
            case 1: // Double Word Register
#ifdef Serial_Debug_Modbus
                logDebugP("| Double Word ");
#endif
                // Choose Modbus Funktion (0x03 readHoldingRegisters ODER 0x04 readInputRegisters)
                switch (ParamMOD_CHModBusReadWordFunktion)
                {
                case 3: // 0x03 Lese holding registers
#ifdef Serial_Debug_Modbus
                    logDebugP(" 0x03 ");
#endif
                    result = readHoldingRegisters(registerAddr, 2);
                    break;

                case 4:
#ifdef Serial_Debug_Modbus
                    logDebugP(" 0x04 ");
#endif
                    result = readInputRegisters(registerAddr, 2);
                    break;
                default:
                    return result;
                }

                if (result == ku8MBSuccess)
                {
                    // check HI / LO   OR   LO / Hi  order
                    switch (ParamMOD_CHModBusWordPosDpt12)
                    {
                        //  ************************************************************************** MUSS NOCH GEPRÜFT WERDEN !!!!!!!!!!!!!!!!!!!!!!!!!!!
                    case 0: // HI Word / LO Word
                        v = (uint32_t)(getResponseBuffer(0) << 16 | getResponseBuffer(1));
                        break;
                    case 1: // LO Word / HI Word
                        v = (uint32_t)(getResponseBuffer(0) | getResponseBuffer(1) << 16);
                        //  ************************************************************************** MUSS NOCH GEPRÜFT WERDEN !!!!!!!!!!!!!!!!!!!!!!!!!!!
                        break;
                    default:
                        return result;
                    } // Ende // HI / LO Word
                }
                else // Fehler
                {
#ifdef Serial_Debug_Modbus_Min
                    logInfoP("ERROR: %I", result, HEX);
#endif

                    ErrorHandling();

                    return result;
                }
                break; // Ende Case 1 Double Register
            default:
                return result;
            } // ENDE ENDE Word / Double Word Register

            if (result == ku8MBSuccess)
            {
                // senden bei Wertänderung
                uint32_t lAbsolute = ParamMOD_CHModBusValueChange;
                int absVAlue = (v - lastSentValue.lValueUint32_t);
                uint32_t lDiff = abs(absVAlue);
                if (lAbsolute > 0 && lDiff >= lAbsolute)
                    lSend = true;

                // we always store the new value in KO, even it it is not sent (to satisfy potential read request)
                KoMOD_GO_BASE_.valueNoSend(v, DPT_Value_4_Count);
                if (lSend)
                {
                    lastSentValue.lValueUint32_t = v;
                }

#ifdef Serial_Debug_Modbus_Min
                logDebugP("%u", v);
#endif

                // Löscht Fehlerspeicher
                errorState[0] = false;
                errorState[1] = false;
            }

        } // ENDE
        break; // Ende PDT12
    //*****************************************************************************************************************************************
    //*****************************************  DPT 13 ***************************************************************************************
    //*****************************************************************************************************************************************
    case 13:
        if (readRequest)
        {
#ifdef Serial_Debug_Modbus
            logDebugP("DPT13 ");
#endif

            // clear Responsebuffer before revicing a new message
            clearResponseBuffer();

            int32_t v;

            // Bestimmt ob Register-Typ: Word oder Double Word
            switch (ParamMOD_CHModBusWordTyp13) // Choose Word Register OR Double Word Register
            {
            case 0: // Word Register
#ifdef Serial_Debug_Modbus
                logDebugP("| Word ");
#endif
                // Choose Modbus Funktion (0x03 readHoldingRegisters ODER 0x04 readInputRegisters)
                switch (ParamMOD_CHModBusReadWordFunktion)
                {
                case 3: // 0x03 Lese holding registers
#ifdef Serial_Debug_Modbus
                    logDebugP(" 0x03 ");
#endif
                    result = readHoldingRegisters(registerAddr, 1);
                    break;

                case 4:

#ifdef Serial_Debug_Modbus
                    logDebugP(" 0x04 ");
#endif
                    result = readInputRegisters(registerAddr, 1);
                    break;
                default:
                    return result;
                }

                if (result == ku8MBSuccess)
                {
                    // adapt input value (Low Byte / High Byte / High&Low Byte / .... )
                    switch (ParamMOD_CHModBusRegisterPosDPT13)
                    {
                    case 1:                                        // Low Byte signed
                        v = (int8_t)(getResponseBuffer(0) & 0xff); // muss noch bearbeitet werden !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                        break;
                    case 2:                                               // High Byte signed
                        v = (int8_t)((getResponseBuffer(0) >> 8) & 0xff); // muss noch bearbeitet werden !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                        break;
                    case 3:                                // High/Low Byte signed
                        v = (int16_t)getResponseBuffer(0); // muss noch bearbeitet werden !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                        break;
                    default:
                        return result;
                    }
                }
                else
                {
#ifdef Serial_Debug_Modbus_Min
                    logInfoP("ERROR: %i", result, HEX);
#endif
                    ErrorHandling();

                    return result;
                }

                break;
            case 1: // Double Word Register
#ifdef Serial_Debug_Modbus
                logDebugP("| Double Word ");
#endif
                // Choose Modbus Funktion (0x03 readHoldingRegisters ODER 0x04 readInputRegisters)
                switch (ParamMOD_CHModBusReadWordFunktion)
                {
                case 3: // 0x03 Lese holding registers
#ifdef Serial_Debug_Modbus
                    logDebugP(" 0x03 ");
#endif
                    result = readHoldingRegisters(registerAddr, 2);
                    break;

                case 4:
#ifdef Serial_Debug_Modbus
                    logDebugP(" 0x04 ");
#endif
                    result = readInputRegisters(registerAddr, 2);
                    break;
                default:
                    return result;
                }

                if (result == ku8MBSuccess)
                {
                    // check HI / LO   OR   LO / Hi  order
                    switch (ParamMOD_CHModBusWordPosDpt13)
                    {
                        //  ************************************************************************** MUSS NOCH GEPRÜFT WERDEN !!!!!!!!!!!!!!!!!!!!!!!!!!!
                    case 0: // HI Word / LO Word
                        v = (int32_t)(getResponseBuffer(0) << 16 | getResponseBuffer(1));
                        break;
                    case 1: // LO Word / HI Word
                        v = (int32_t)(getResponseBuffer(0) | getResponseBuffer(1) << 16);
                        //  ************************************************************************** MUSS NOCH GEPRÜFT WERDEN !!!!!!!!!!!!!!!!!!!!!!!!!!!
                        break;
                    default:
                        return result;
                    } // Ende // HI / LO Word
                }
                else
                {
#ifdef Serial_Debug_Modbus_Min
                    logInfoP("ERROR: %i", result, HEX);
#endif
                    ErrorHandling();

                    return result;
                }
                break; // Ende Case 1 Double Register
            default:
                return result;
            } // ENDE ENDE Word / Double Word Register

            if (result == ku8MBSuccess)
            {
                // senden bei Wertänderung
                uint32_t lAbsolute = ParamMOD_CHModBusValueChange;
                uint32_t lDiff = abs(v - lastSentValue.lValueInt32_t);
                if (lAbsolute > 0 && lDiff >= lAbsolute)
                    lSend = true;

                // we always store the new value in KO, even it it is not sent (to satisfy potential read request)
                KoMOD_GO_BASE_.valueNoSend(v, DPT_Value_4_Count);
                if (lSend)
                {
                    lastSentValue.lValueInt32_t = v;
                }

#ifdef Serial_Debug_Modbus_Min
                logDebugP("%i", v);
#endif

                // Löscht Fehlerspeicher
                errorState[0] = false;
                errorState[1] = false;
            }

        } // ENDE
        break; // Ende PDT13

    //*****************************************************************************************************************************************
    //*****************************************  DPT 14 ***************************************************************************************
    //*****************************************************************************************************************************************
    case 14:

        if (readRequest)
        {
            // clear Responsebuffer before receiving a new message
            clearResponseBuffer();

            float v;

            // Bestimmt ob Register-Typ: Word oder Double Word
            switch (ParamMOD_CHModBusWordTyp14) // Choose Word Register OR Double Word Register
            {
            case 0: // Word Register
                // Choose Modbus Funktion (0x03 readHoldingRegisters ODER 0x04 readInputRegisters)
                switch (ParamMOD_CHModBusReadWordFunktion)
                {
                case 3: // 0x03 Lese holding registers
#ifdef Serial_Debug_Modbus
                    logDebugP("DPT14| Word 0x03 ");
#endif
                    result = readHoldingRegisters(registerAddr, 1);
                    break;

                case 4:

#ifdef Serial_Debug_Modbus
                    logDebugP("DPT14| Word 0x04 ");
#endif
                    result = readInputRegisters(registerAddr, 1);
                    break;
                default:
                    return result;
                }

                if (result == ku8MBSuccess)
                {
                    if (ParamMOD_CHModBusRegisterPosDPT14 <= 3)
                    {
                        uint16_t uraw;
                        // adapt input value (Low Byte / High Byte / High&Low Byte / .... )
                        switch (ParamMOD_CHModBusRegisterPosDPT14)
                        {
                        case 1: // Low Byte unsigned
                            uraw = (uint8_t)(getResponseBuffer(0) & 0xff);
                            break;
                        case 2: // High Byte unsigned
                            uraw = (uint8_t)((getResponseBuffer(0) >> 8) & 0xff);
                            break;
                        case 3: // High/Low Byte unsigned
                            uraw = getResponseBuffer(0);
                            break;
                        default:
                            return result;
                        }

                        //  ************************ MUSS NOCH GEPRÜFT WERDEN !!!!!!!!!!!!!!!!!!!!!!!!!!!
                        v = (float)uraw / ParamMOD_CHModBuscalculationValueDiff;
                        v = v + ParamMOD_CHModBuscalculationValueAdd;

                        logDebugP("value: %f | raw: %u", v, uraw);
                        logDebugP("Diff: %u | add: %i", ParamMOD_CHModBuscalculationValueDiff, ParamMOD_CHModBuscalculationValueAdd);
                    }
                    else
                    {
                        int16_t sraw;
                        // adapt input value (Low Byte / High Byte / High&Low Byte / .... )
                        switch (ParamMOD_CHModBusRegisterPosDPT14)
                        {
                        case 4:                                           // Low Byte signed
                            sraw = (int8_t)(getResponseBuffer(0) & 0xff); // muss noch bearbeitet werden !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                            break;
                        case 5:                                                  // High Byte signed
                            sraw = (int8_t)((getResponseBuffer(0) >> 8) & 0xff); // muss noch bearbeitet werden !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                            break;
                        case 6:                                   // High/Low Byte signed
                            sraw = (int16_t)getResponseBuffer(0); // muss noch bearbeitet werden !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                            break;
                        default:
                            return result;
                        }

                        //  ************************ MUSS NOCH GEPRÜFT WERDEN !!!!!!!!!!!!!!!!!!!!!!!!!!!
                        v = sraw / (float)ParamMOD_CHModBuscalculationValueDiff;
                        v = v + (int16_t)ParamMOD_CHModBuscalculationValueAdd;

                        logDebugP("value: %f", v);
                    }
                }
                else
                {
#ifdef Serial_Debug_Modbus_Min
                    logInfoP("ERROR: %u", result, HEX);
#endif
                    ErrorHandling();

                    return result;
                }

                break;
            case 1: // Double Word Register
                // Choose Modbus Funktion (0x03 readHoldingRegisters ODER 0x04 readInputRegisters)
                switch (ParamMOD_CHModBusReadWordFunktion)
                {
                case 3: // 0x03 Lese holding registers
#ifdef Serial_Debug_Modbus
                    logDebugP("DPT14| Double Word | 0x03 ");
#endif
                    result = readHoldingRegisters(registerAddr, 2);
                    break;

                case 4:
#ifdef Serial_Debug_Modbus
                    logDebugP("DPT14| Double Word | 0x04 ");
#endif
                    result = readInputRegisters(registerAddr, 2);
                    break;
                default:
                    return result;
                }

                if (result == ku8MBSuccess)
                {
                    uint32_t raw;

                    // check HI / LO   OR   LO / Hi  order
                    switch (ParamMOD_CHModBusWordPosDpt14)
                    {
                        //  ************************************************************************** MUSS NOCH GEPRÜFT WERDEN !!!!!!!!!!!!!!!!!!!!!!!!!!!
                    case 0: // HI Word / LO Word
                        raw = getResponseBuffer(0) << 16 | getResponseBuffer(1);
                        break;
                    case 1: // LO Word / HI Word
                        raw = getResponseBuffer(0) | getResponseBuffer(1) << 16;
                        //  ************************************************************************** MUSS NOCH GEPRÜFT WERDEN !!!!!!!!!!!!!!!!!!!!!!!!!!!
                        break;
                    default:
                        return result;
                    } // Ende // HI / LO Word
                    // check receive input datatype ( signed / unsgined / Float)
                    switch (ParamMOD_CHModBusRegisterValueTypDpt14)
                    {
                    case 1: // unsigned
                    {
                        uint32_t lValueu32bit = raw;
                        //                                                         ************************ MUSS NOCH GEPRÜFT WERDEN !!!!!!!!!!!!!!!!!!!!!!!!!!!
                        v = lValueu32bit / (float)ParamMOD_CHModBuscalculationValueDiff;
                        v = v + (int16_t)ParamMOD_CHModBuscalculationValueAdd;
                    }
                    break;
                    case 2: // signed
                    {
                        int32_t lValuei32bit = (int32_t)raw;
                        //                                                         ************************ MUSS NOCH GEPRÜFT WERDEN !!!!!!!!!!!!!!!!!!!!!!!!!!!
                        v = lValuei32bit / (float)ParamMOD_CHModBuscalculationValueDiff;
                        v = v + (int16_t)ParamMOD_CHModBuscalculationValueAdd;
                    }
                    break;
                    case 3: // float
                    {
                        // going via union allows the compiler to be sure about alignment
                        union intfloat
                        {
                            uint32_t intVal;
                            float floatVal;
                        };
                        float lValueFloat = ((intfloat *)&raw)->floatVal;
                        //                                                         ************************ MUSS NOCH GEPRÜFT WERDEN !!!!!!!!!!!!!!!!!!!!!!!!!!!
                        v = lValueFloat / (float)ParamMOD_CHModBuscalculationValueDiff;
                        v = v + (int16_t)ParamMOD_CHModBuscalculationValueAdd;
                    }
                    break;
                    default:
                        return result;
                    }
                }
                else
                {
#ifdef Serial_Debug_Modbus_Min
                    logInfoP("ERROR: %u", result, HEX);
#endif

                    ErrorHandling();

                    return result;
                }
                break; // Ende Case 1 Double Register
            default:
                return result;
            } // ENDE ENDE Word / Double Word Register

            if (result == ku8MBSuccess)
            {
                // send on first value or value change
                float lAbsolute = ParamMOD_CHModBusValueChange / 10.0;
                float lDiff = abs(v - lastSentValue.lValue);
                if (lAbsolute > 0.0f && lDiff >= lAbsolute)
                    lSend = true;

                // we always store the new value in KO, even it it is not sent (to satisfy potential read request)
                KoMOD_GO_BASE_.valueNoSend(v, DPT_Value_Acceleration_Angular);
                if (lSend)
                {
                    lastSentValue.lValue = v;
                }

                logDebugP("%f", v, 2);

                // Löscht Fehlerspeicher
                errorState[0] = false;
                errorState[1] = false;
            }

        } // ENDE
        break; // Ende PDT14

    default: // all other dpts
        logInfoP("Falscher DPT: %i", dpt);
        break;
    } // Ende DPT Wahl Wahl

    if (lSend && !errorState[0] && !errorState[1])
    {
        KoMOD_GO_BASE_.objectWritten();
        valueValid = true;
        sendDelay = millis();
        lSend = false;
    }

    return true;
}

//*****************************************************************************************************************************************
//*****************************************************************************************************************************************
//
//*****************************************  KNX TO Modbus ********************************************************************************
//
//*****************************************************************************************************************************************
//*****************************************************************************************************************************************

uint8_t modbusChannel::knxToModbus(uint8_t dpt, bool readRequest)
{
    uint8_t result = 0;
    uint16_t registerAddr = ParamMOD_CHModbusRegister; // adjustRegisterAddress(ParamMOD_CHModbusRegister);

#ifdef Serial_Debug_Modbus_Min
    logDebugP("KNX: CH%i", (_channelIndex + 1));
#endif

    //*****************************************************************************************************************************************
    //*****************************************  DPT 1.001 ************************************************************************************
    //*****************************************************************************************************************************************
    if (dpt == 1)
    {
        if (readRequest)
        {
            bool v = (bool)KoMOD_GO_BASE_.value(DPT_Switch) ^ (ParamMOD_CHModBusInputTypInvDpt1);

            // Bit Register
            if (ParamMOD_CHModBusInputTypDpt1 == 0)
            {
#ifdef Serial_Debug_Modbus
                SERIAL_DEBUG.print(" 0x05 ");
#endif
                result = writeSingleCoil(registerAddr, v);
            }
            // Bit in Word
            else if (ParamMOD_CHModBusInputTypDpt1 == 1)
            {
                uint16_t value = v << ParamMOD_CHModBusBitPosDpt1;
                result = sendProtocol(registerAddr, value);
            }
            else
            {
                return result;
            }
            printDebugResult("1.001", registerAddr, result);
        }
    }
    //*****************************************************************************************************************************************
    //*****************************************  DPT 5.004 ************************************************************************************
    //*****************************************************************************************************************************************
    else if (4 == dpt)
    {
        if (readRequest)
        {
            result = sendProtocol(registerAddr, KoMOD_GO_BASE_.value(DPT_Percent_U8));
            printDebugResult("5.004", registerAddr, result);
        }
    }
    //*****************************************************************************************************************************************
    //*****************************************  DPT 5.010 ************************************************************************************
    //*****************************************************************************************************************************************
    else if (5 == dpt)
    {
        if (readRequest)
        {
            uint16_t v = KoMOD_GO_BASE_.value(DPT_Value_1_Ucount);

            switch (ParamMOD_CHModBusRegisterPosDPT5)
            {
            case 1: // High Byte
                v <<= 8;
                break;
            case 2: // Low Byte
                // already at correct position
                break;
            case 3: // frei Wählbar
                v &= ParamMOD_CHModbusCountBitsDPT56;
                v <<= ParamMOD_CHModBusOffsetRight5;
                break;
            default:
                return result;
            } // Ende Register Pos

            result = sendProtocol(registerAddr, v);
            printDebugResult("5.001", registerAddr, result);
        }
    }
    //*****************************************************************************************************************************************
    //*****************************************  DPT 7 ***************************************************************************************
    //*****************************************************************************************************************************************
    else if (7 == dpt)
    {
        if (readRequest)
        {
            uint16_t v = KoMOD_GO_BASE_.value(DPT_Value_2_Ucount);

            switch (ParamMOD_CHModBusRegisterPosDPT7)
            {
            case 1: // High/LOW Byte
                // already at correct position
                break;
            case 2: // frei Wählbar
                v &= ((1 << ParamMOD_CHModbusCountBitsDPT7) - 1);
                v <<= (ParamMOD_CHModBusOffsetRight7);
                break;
            default:
                return result;
            } // Ende Register Pos

            result = sendProtocol(registerAddr, v);
            printDebugResult("5", registerAddr, result);
        }
    }
    //*****************************************************************************************************************************************
    //*****************************************  DPT 8 signed 16Bit ***************************************************************************
    //*****************************************************************************************************************************************
    else if (8 == dpt)
    {
        if (readRequest)
        {
            result = sendProtocol(registerAddr, KoMOD_GO_BASE_.value(DPT_Value_2_Count));
            printDebugResult("8", registerAddr, result);
        }
    }
    //*****************************************************************************************************************************************
    //*****************************************  DPT 9 ***************************************************************************************
    //*****************************************************************************************************************************************
    else if (9 == dpt)
    {
        if (readRequest)
        {
            float raw = KoMOD_GO_BASE_.value(DPT_Value_Temp);
            uint16_t v;
            // adapt input value (Low Byte / High Byte / High&Low Byte / .... )
            switch (ParamMOD_CHModBusRegisterPosDPT9)
            {
            case 1: // Low Byte unsigned
                v = ((uint16_t)roundf(raw)) & 0xff;
                break;
            case 2: // High Byte unsigned
                v = (((uint16_t)roundf(raw)) >> 8) & 0xff;
                break;
            case 3: // High/Low Byte unsigned
                v = (uint16_t)roundf(raw);
                break;
            case 4: // Low Byte signed
                v = ((int)roundf(raw)) & 0xff;
                break;
            case 5: // High Byte signed
                v = (((int)roundf(raw)) >> 8) & 0xff;
                break;
            case 6: // High/Low Byte signed
                v = (int)roundf(raw);
                break;
            default:
                return result;
            }
            result = sendProtocol(registerAddr, v);
            printDebugResult("9", registerAddr, result);
        }
    }
    //*****************************************************************************************************************************************
    //*****************************************  DPT 12 ***************************************************************************************
    //*****************************************************************************************************************************************
    else if (12 == dpt)
    {
        if (readRequest)
        {
            uint32_t v = KoMOD_GO_BASE_.value(DPT_Value_4_Ucount);
            setTransmitBuffer(0, v >> 16);
            setTransmitBuffer(1, v & 0xffff);
            result = writeMultipleRegisters(registerAddr, 2);
            printDebugResult("12 0x10", registerAddr, result);
        }
    }
    //*****************************************************************************************************************************************
    //*****************************************  DPT 13 ***************************************************************************************
    //*****************************************************************************************************************************************
    else if (13 == dpt)
    {
        if (readRequest)
        {
            int32_t v = KoMOD_GO_BASE_.value(DPT_Value_4_Count);
            setTransmitBuffer(0, v >> 16);
            setTransmitBuffer(1, v);
            result = writeMultipleRegisters(registerAddr, 2);
            printDebugResult("13 0x10", registerAddr, result);
        }
    }
    //*****************************************************************************************************************************************
    //*****************************************  DPT 14 ***************************************************************************************
    //*****************************************************************************************************************************************
    else if (14 == dpt)
    {
        if (readRequest)
        {
            float raw = KoMOD_GO_BASE_.value(DPT_Value_Acceleration_Angular);
            union floatint
            {
                float floatVal;
                uint32_t intVal;
            };
            uint32_t v = ((floatint *)&raw)->intVal;
            // HI / LO   OR   LO / Hi  order
            if (ParamMOD_CHModBusWordPosDpt14 == 0)
            { // HI / LO
                setTransmitBuffer(0, v >> 16);
                setTransmitBuffer(1, v);
            }
            else
            { // LO / HI
                setTransmitBuffer(0, v);
                setTransmitBuffer(1, v >> 16);
            }
            result = writeMultipleRegisters(registerAddr, 2);
            printDebugResult("14 0x10", registerAddr, result);
        }
    }

    return true;
}

uint8_t modbusChannel::sendProtocol(uint16_t registerAddr, uint16_t u16value)
{
    if (0x06 == ParamMOD_CHModBusWriteWordFunktion)
    {
#ifdef Serial_Debug_Modbus
        SERIAL_DEBUG.print(" 0x06 ");
#endif
        return writeSingleRegister(registerAddr, u16value);
    }
    else if (0x10 == ParamMOD_CHModBusWriteWordFunktion)
    {
#ifdef Serial_Debug_Modbus
        SERIAL_DEBUG.print(" 0x10 ");
#endif
        setTransmitBuffer(0, u16value);
        return writeMultipleRegisters(registerAddr, 1);
    }
    return ku8MBIllegalFunction;
}

void modbusChannel::printDebugResult(const char *dpt, uint16_t registerAddr, uint8_t result)
{
#ifdef Serial_Debug_Modbus
    logDebugP("DPT:%s Reg:%u", dpt, registerAddr);
#endif
#ifdef Serial_Debug_Modbus_Min
    switch (result)
    {
    case ku8MBSuccess:
        logDebugP("DONE");
        break;
#ifdef Serial_Debug_Modbus
    case ku8MBInvalidSlaveID:
        logDebugP("ERROR: Invalid Slave ID");
        break;
    case ku8MBInvalidFunction:
        logDebugP("ERROR: Invalid Function");
        break;
    case ku8MBResponseTimedOut:
        logDebugP("ERROR: Response Timed Out");
        break;
    case ku8MBInvalidCRC:
        logDebugP("ERROR: Invalid CRC");
        break;
#endif
    default:
        logDebugP("ERROR");
        break;
    }
#endif
}

bool modbusChannel::modbusParitySerial(uint32_t baud, HardwareSerial &serial)
{
    switch (_parity_value)
    {
    case 0: // Even (1 stop bit)
        serial.begin(baud, SERIAL_8E1);
        // logDebugP.println("Parity: Even (1 stop bit)");
        return true;
        break;
    case 1: // Odd (1 stop bit)
        serial.begin(baud, SERIAL_8O1);
        // logDebugP.println("Parity: Odd (1 stop bit)");
        return true;
        break;
    case 2: // None (2 stop bits)
        serial.begin(baud, SERIAL_8N2);
        // logDebugP.println("Parity: None (2 stop bits)");
        return true;
        break;
    case 3: // None (1 stop bit)
        serial.begin(baud, SERIAL_8N1);
        // logDebugP.println("Parity: None (1 stop bit)");
        return true;
        break;

    default:
        // logDebugP.print("Parity: Error: ");
        // logDebugP.println(_parity_value);
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
        // logDebugP.println("Baudrate: 1200kBit/s");
        return modbusParitySerial(1200, serial);

        break;
    case 1:
        // logDebugP.println("Baudrate: 2400kBit/s");
        return modbusParitySerial(2400, serial);
        break;
    case 2:
        // logDebugP.println("Baudrate: 4800kBit/s");
        return modbusParitySerial(4800, serial);
        break;
    case 3:
        // logDebugP.println("Baudrate: 9600kBit/s");
        return modbusParitySerial(9600, serial);
        break;
    case 4:
        // logDebugP.println("Baudrate: 19200kBit/s");
        return modbusParitySerial(19200, serial);
        break;
    case 5:
        // logDebugP.println("Baudrate: 38400kBit/s");
        return modbusParitySerial(38400, serial);
        break;
    case 6:
        // logDebugP.println("Baudrate: 56000kBit/s");
        return modbusParitySerial(56000, serial);
        break;
    case 7:
        // logDebugP.println("Baudrate: 115200kBit/s");
        return modbusParitySerial(115200, serial);
        break;
    default:
        // logDebugP.print("Baudrate: Error: ");
        // logDebugP.println(_baud_value);
        return false;
        break;
    }
}
