#include <Arduino.h>
#include <Wire.h>
#include "TAS5711.h"

TAS5711::TAS5711<TwoWire> tas5711(Wire);
  
void setup() {  
  tas5711.begin(0x34); 
  tas5711.setFormat(TAS5711::SerialDataInterfaceSetting::I2S_16bits);
  tas5711.exitShutdown();
  tas5711.setSoftMute(false);
}


void loop() {}