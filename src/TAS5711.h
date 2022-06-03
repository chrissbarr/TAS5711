/*************************************************** 
 This is a library for the TAS5711 I2S DAC/AMP.

 Written by Chris Barr, 2022.
 ****************************************************/

#ifndef _TAS5711_H_
#define _TAS5711_H_

#include <stdint.h>
#include <Arduino.h>

namespace TAS5711 {

enum class Register : uint8_t {
  ClockControl = 0x00,
  DeviceID = 0x01,
  ErrorStatus = 0x02,
  SystemControl1 = 0x03,
  SerialDataInterface = 0x04,
  SystemControl2 = 0x05,
  SoftMute = 0x06,
  VolumeMaster = 0x07,
  VolumeCh1 = 0x08,
  VolumeCh2 = 0x09,
  VolumeCh3 = 0x0A,
  VolumeConfiguration = 0x0E,
  ModulationLimit = 0x10,
  ICDelayCh1 = 0x11,
  ICDelayCh2 = 0x12,
  ICDelayCh3 = 0x13,
  ICDelayCh4 = 0x14,
  PWMGroupShutdown = 0x19,
  StartStopPeriod = 0x1A,
  OscillatorTrim = 0x1B,
  BackendError = 0x1C,
  InputMux = 0x20,
  Ch4SourceSelect = 0x21,
  PWMMux = 0x25
};

enum class SerialDataInterfaceSetting : uint8_t {
  RightJustified_16bits = 0x00,
  RightJustified_20bits = 0x01,
  RightJustified_24bits = 0x02,
  I2S_16bits = 0x03,
  I2S_20bits = 0x04,
  I2S_24bits = 0x05,
  LeftJustified_16bits = 0x06,
  LeftJustified_20bits = 0x07,
  LeftJustified_24bits = 0x08
};

template <typename WIRE>
class TAS5711 {
public:
  explicit TAS5711(WIRE& wire) : mWire(wire) {}

  // address should be either 0x34 or 0x36
  bool begin(uint8_t address, int16_t resetPin = -1, int16_t pdnPin = -1, int16_t powerPin = -1)
  {
    _i2caddr = address >> 1;
    mWire.begin();

    if (resetPin != -1 && pdnPin != -1) {
      pinMode(resetPin, OUTPUT);
      pinMode(pdnPin, OUTPUT);
      digitalWrite(resetPin, LOW);
      digitalWrite(pdnPin, LOW);

      if (powerPin != -1) {
        pinMode(powerPin, OUTPUT);
        digitalWrite(powerPin, LOW);
      }

      delay(10);
      digitalWrite(pdnPin, HIGH);
      delay(1);
      digitalWrite(resetPin, HIGH);
      delay(20);

      digitalWrite(powerPin, HIGH);
    }

    // trim oscillator
    writeRegister(Register::OscillatorTrim, 0x00);
    delay(60);

    if (readRegister(Register::ErrorStatus) != 0x00) {
      return false;
    }

    if (readRegister(Register::DeviceID) != 0x00) {
      return false;
    }

    // unmute all
    writeRegister(Register::VolumeMaster, 0x00);

    return true;
  }

  void exitShutdown()
  {
    writeRegister(Register::SystemControl2, 0x00);
  }

  void enterShutdown()
  {
    writeRegister(Register::SystemControl2, 0x40);
  }

  void setFormat(SerialDataInterfaceSetting setting)
  {
    writeRegister(Register::SerialDataInterface, static_cast<uint8_t>(setting));
  }

  void setSoftMute(bool enabled)
  {
    uint8_t val = 0x00;
    if (enabled) { val = 0x07; }
    writeRegister(Register::SoftMute, val);
  }

  void writeRegister(Register reg, uint8_t value)
  {
    mWire.beginTransmission(_i2caddr);
    mWire.write(static_cast<uint8_t>(reg));
    mWire.write(value);
    mWire.endTransmission();
  }

  uint8_t readRegister(Register reg)
  {
    mWire.beginTransmission(_i2caddr);
    mWire.write(static_cast<uint8_t>(reg));
    mWire.endTransmission();
    mWire.requestFrom(_i2caddr, uint8_t(1));
    return mWire.read();;
  }

private:
  uint8_t _i2caddr;
  WIRE& mWire;


};


} // namespace _TAS5711_H_

#endif