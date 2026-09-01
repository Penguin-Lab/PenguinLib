#include "penguinUtils.h"

// DFPlayer mp3 audio
dfPlayer::dfPlayer(int rx, int tx) : mySerial(2){
    mySerial.begin(9600, SERIAL_8N1, rx, tx);
}
  
void dfPlayer::sendCommand(byte command, byte param1, byte param2){
    byte commandBuffer[10] = { 0x7E, 0xFF, 0x06, command, 0x00, param1, param2, 0x00, 0x00, 0xEF };
    // Checksum
    int16_t checksum = -(commandBuffer[1] + commandBuffer[2] + commandBuffer[3] + commandBuffer[4] + commandBuffer[5] + commandBuffer[6]);
    commandBuffer[7] = (checksum >> 8) & 0xFF;
    commandBuffer[8] = checksum & 0xFF;
    mySerial.write(commandBuffer, 10);
}

void dfPlayer::playTrack(int track){
    sendCommand(0x03, 0x00, track);
}

void dfPlayer::setVolume(int volume){
    sendCommand(0x06, 0x00, constrain(volume, 0, 30));
}

// Touch sensor
touchSensor::touchSensor(int channel, int touchValue, int touches) : touchChannel(channel), touchValue(touchValue), touches(touches){
    touchTime = millis();
    measuredValue = 60;
    touchState = 0;
}
  
bool touchSensor::touched(){
    measuredValue = touchRead(touchChannel);
    return measuredValue < touchValue;
}

bool touchSensor::affection(){
    bool petted = false;
    bool touching = touched();
    if (touchState < touches*2){
      if (touchState == 0){
        if (touching){
          touchState++;
          touchTime = millis();
        }
      }
      else if (touchState % 2 == 0){
        if ((millis()-touchTime)>5000){
          touchState = 0;
        }
        else{
          if (touching){
            touchState++;
            touchTime = millis();
          }
        }
      }
      else{
        if ((millis()-touchTime)>5000){
          touchState = 0;
        }
        else{
          if (!touching){
            touchState++;
            touchTime = millis();
          }
        }
      }
    }
    else if (touchState == touches*2){
      touchState++;
      petted = true;
    }
    else{
      if ((millis()-touchTime)>5000){
        if (!touching){
          touchState = 0;
        }
        touchTime = millis();
      }
    }
    return petted;
}

// BNO08X IMU
bnoImu::bnoImu(uint8_t address, float maxAngle) : address(address), maxAngle(maxAngle) {
}

void bnoImu::start(){
    if (!myImu.begin(address, Wire)) {
        Serial.println("BNO08X not found!");
        while (1) delay(10);
    }
    myImu.enableRotationVector(50);
}

bool bnoImu::getEvent(){
    return myImu.getSensorEvent() && myImu.getSensorEventID() == SENSOR_REPORTID_ROTATION_VECTOR;
}

float3 bnoImu::getPose(float3 lastPose){
    float roll_deg  = roundf(myImu.getRoll()*1800.0f/PI)/10.0f;
    roll_deg = 0.95*roll_deg + 0.05*lastPose.coxa;
    if (roll_deg > maxAngle) roll_deg = maxAngle;
    if (roll_deg < -maxAngle) roll_deg = -maxAngle;
    
    float pitch_deg = roundf(myImu.getPitch()*1800.0f/PI)/10.0f;
    pitch_deg = 0.95*pitch_deg + 0.05*lastPose.femur;
    if (pitch_deg > maxAngle) pitch_deg = maxAngle;
    if (pitch_deg < -maxAngle) pitch_deg = -maxAngle;
    
    return {roll_deg, pitch_deg, 0.0f};
}