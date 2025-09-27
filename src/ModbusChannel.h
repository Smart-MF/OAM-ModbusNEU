#pragma once
#include "OpenKNX.h"
#include "ModBusMaster.h"

class modbusChannel : public OpenKNX::Channel, public ModbusMaster
{
private:
  uint8_t _modbus_ID;
  uint8_t _baud_value;
  uint8_t _parity_value;
  HardwareSerial &_serial;
  static bool idle_processing;

  bool errorState[2];
  bool valueValid;
  bool _readyToSend;
  uint8_t _skipCounter;

  typedef union Values
  {
    uint8_t lValueUint8_t;
    uint16_t lValueUint16_t;
    int16_t lValueInt16_t;
    uint32_t lValueUint32_t;
    int lValueUint;
    int32_t lValueInt32_t;
    float lValue;
    bool lValueBool;
  } values_t;
  values_t lastSentValue;

  bool modbusParitySerial(uint32_t baud, HardwareSerial &serial);
  bool modbusInitSerial(HardwareSerial &serial);
  void ErrorHandling();
  void ErrorHandlingLED();

public:
  modbusChannel(uint8_t index, uint8_t _baud_value, uint8_t _parity_value, HardwareSerial &serial);
  inline uint16_t adjustRegisterAddress(uint16_t u16ReadAddress, uint8_t RegisterStart);

  bool readModbus(bool readRequest);
  bool sendModbus();
  bool modbusToKnx(uint8_t dpt, bool readRequest);
  bool knxToModbus(uint8_t dpt, bool readRequest);
  void printDebugResult(const char *dpt, uint16_t registerAddr, uint8_t result);
  uint8_t sendProtocol(uint16_t registerAddr, uint16_t u16value);
  const std::string name() override;
  void setup() override;
  void loop() override;
};