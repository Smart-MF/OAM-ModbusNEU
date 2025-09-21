#pragma once
#include "OpenKNX.h"

class modbusChannel : public OpenKNX::Channel
{
  private:
    uint8_t _modbus_ID;
    uint8_t _baud_value;
    uint8_t _parity_value;
    HardwareSerial &_serial;

    bool modbusParitySerial(uint32_t baud, HardwareSerial &serial);
    bool modbusInitSerial(HardwareSerial &serial);

  public:
    modbusChannel(uint8_t index, uint8_t id, uint8_t _baud_value, uint8_t _parity_value, HardwareSerial &serial);
    const std::string name() override;
    void setup() override;
    void loop() override;
};