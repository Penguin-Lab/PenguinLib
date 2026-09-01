#ifndef PENGUINUTILS_H
#define PENGUINUTILS_H

#include <Arduino.h>
#include "RobotLeg.h"
#include <Wire.h>
#include <HardwareSerial.h>
#include <SparkFun_BNO08x_Arduino_Library.h>

// DFPlayer mp3 audio
class dfPlayer {
private:
  HardwareSerial mySerial;

public:
  dfPlayer(int rx, int tx);
  
  void sendCommand(byte command, byte param1, byte param2);
  void playTrack(int track);
  void setVolume(int volume);
};

// Touch sensor
class touchSensor {
private:
  int touchChannel;
  int touchValue;
  int touches;
  unsigned long touchTime;
  int measuredValue;
  int touchState;

public:
  touchSensor(int channel, int touchValue, int touches);
  
  bool touched();
  bool affection();
};

// BNO08X IMU
class bnoImu {
private:
  BNO08x myImu;

  uint8_t address;
  float maxAngle;

public:
  bnoImu(uint8_t address, float maxAngle);

  void start();
  bool getEvent();
  float3 getPose(float3 lastPose);
};

#endif