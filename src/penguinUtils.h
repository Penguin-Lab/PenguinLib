#ifndef PENGUINUTILS_H
#define PENGUINUTILS_H

#include <Arduino.h>
#include <Wire.h>
#include <HardwareSerial.h>
#include "MathTypes.h"
#include <SparkFun_BNO08x_Arduino_Library.h>

#ifdef USE_MQTT
#include <WiFi.h>
#include <PubSubClient.h>
#endif

#ifdef USE_DABBLE
#define CUSTOM_SETTINGS
#define INCLUDE_GAMEPAD_MODULE
#include <DabbleESP32.h>
#endif

// Definitions
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

// Communication
struct JoystickData {
    float velocity;
    int angle;
    int mode;
    int state;
};

#ifdef USE_MQTT
// Communication: Rabbit-MQTT
class comMqtt {
private:
    const char* ssid;
    const char* wifiPassword;

    const char* mqttServer;
    int mqttPort;
    const char* mqttUser;
    const char* mqttPassword;

    const char* topicVel;

    WiFiClient espClient;
    PubSubClient mqttClient;

    String message;
    
    void callback(char* topic, byte* payload, unsigned int length);
    void reconnect();
    
    static comMqtt* instance;
    static void mqttCallback(char* topic, byte* payload, unsigned int length);
    
    public:
    comMqtt(
      const char* ssid,
      const char* wifiPassword,
      const char* mqttServer,
      int mqttPort,
      const char* mqttUser,
      const char* mqttPassword,
      const char* topicVel
    );
    
    JoystickData joystickData;

    void begin();
    bool connected();
    JoystickData getJoystick();
};
#endif

#ifdef USE_DABBLE
// Communication: Dabble
struct GamepadState {
    bool square, circle, triangle, cross, up, down, left, right, start, select;
    float radius;
    int angle;
};

class comDabble {
private:
    const char* name;
    
    public:
    comDabble(const char* name);
    
    JoystickData joystickData;
    
    void begin();
    JoystickData getJoystick();

    void update();
    GamepadState readGamepad();
};
#endif

// Functions
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

#ifdef USE_MQTT
// Communication: Rabbit-MQTT
comMqtt* comMqtt::instance = nullptr;

comMqtt::comMqtt(
    const char* ssid,
    const char* wifiPassword,
    const char* mqttServer,
    int mqttPort,
    const char* mqttUser,
    const char* mqttPassword,
    const char* topicVel
)
    : ssid(ssid),
      wifiPassword(wifiPassword),
      mqttServer(mqttServer),
      mqttPort(mqttPort),
      mqttUser(mqttUser),
      mqttPassword(mqttPassword),
      topicVel(topicVel),
      mqttClient(espClient)
{
    instance = this;
    joystickData = {0.0, 0, 0, 11};
}

void comMqtt::begin(){
    WiFi.begin(ssid, wifiPassword);

    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }

    Serial.println();
    Serial.println("WiFi connected...");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());

    mqttClient.setServer(mqttServer, mqttPort);
    mqttClient.setCallback(mqttCallback);

    reconnect();
}

bool comMqtt::connected(){
    return mqttClient.connected();
}

JoystickData comMqtt::getJoystick(){
    if (!mqttClient.connected()) {
        reconnect();
    }
    mqttClient.loop();
    return joystickData;
}

void comMqtt::mqttCallback(char* topic, byte* payload, unsigned int length){
    if (instance != nullptr) {
        instance->callback(topic, payload, length);
    }
}

void comMqtt::callback(char* topic, byte* payload, unsigned int length){
    char messageChar[length + 1];
    memcpy(messageChar, payload, length);
    messageChar[length] = '\0';

    if (strcmp(topic, topicVel) == 0) {
        message = String(messageChar);
        Serial.print("Received message: ");
        Serial.println(message);

        int separator = message.indexOf(',');
        if (separator != -1) {
          String velStr = message.substring(0, separator);
          String angStr = message.substring(separator + 1);

          joystickData.velocity = velStr.toFloat();
          joystickData.angle = angStr.toInt();

          if (joystickData.state != 11){
            if (joystickData.velocity > 2) {
              joystickData.mode = 0;
              joystickData.state = 3;
            }
            else if ((joystickData.velocity == 2)&&(joystickData.angle == 180)) {
              joystickData.mode = 1;
              joystickData.state = 3;
            }
            else if ((joystickData.velocity == 2)&&(joystickData.angle == 0)) {
              joystickData.mode = 1;
              joystickData.state = 3;
            }
            else if ((joystickData.velocity == -10) && (joystickData.angle == 180)) {
              joystickData.mode = 0;
              joystickData.state = 10;
            }
            else{
              joystickData.mode = 0;
              joystickData.state = 0;
            }
          }
          else{
            if ((joystickData.velocity == -10) && (joystickData.angle == 0)) {
              joystickData.mode = 0;
              joystickData.state = 9;
            }
          }
        }
    }
}

void comMqtt::reconnect(){
    while (!mqttClient.connected()) {
        Serial.print("Reconnecting to MQTT Broker...");
        if (mqttClient.connect("ESP32Client", mqttUser, mqttPassword)){
            Serial.println("Connected!");
            mqttClient.subscribe(topicVel);

            Serial.println("Subscribed at topic:");
            Serial.println(topicVel);
        }
        else {
            Serial.print("Failure, rc=");
            Serial.print(mqttClient.state());
            Serial.println(" Trying again in 5 seconds...");
            delay(5000);
        }
    }
}
#endif

#ifdef USE_DABBLE
// Communication: Dabble
comDabble::comDabble(const char* name) : name(name){
    joystickData = {0.0, 0, 0, 11};
}

void comDabble::begin(){
    Dabble.begin(name);
}

JoystickData comDabble::getJoystick(){
    Dabble.processInput();
    joystickData.velocity = GamePad.getRadius();
    // Commands after start
    if (joystickData.state != 11){
      // Circular walk mode
      if (GamePad.isRightPressed()||GamePad.isLeftPressed()||GamePad.isCrossPressed()){
        joystickData.mode = 1;
      }
      // Omnidirectional walk mode
      if (GamePad.isUpPressed()||GamePad.isDownPressed()){
        joystickData.mode = 0;
      }
      // Tilt reaction with IMU mode
      if (GamePad.isSquarePressed()){
        joystickData.mode = 2;
      }
      // High handshake while pressing
      if (GamePad.isCirclePressed()){
        joystickData.state = 1;
      }
      // Wiggle while pressing
      else if(GamePad.isTrianglePressed()){
        joystickData.state = 2;
      }
      // Walk
      else if(joystickData.velocity > 2){
        joystickData.angle = GamePad.getAngle();
        joystickData.state = 3;
      }
      // Shutdown robot
      else if(GamePad.isSelectPressed()){
        joystickData.state = 10;
      }
      // Idle
      else{
        joystickData.state = 0;
      }
    }
    // Before start
    else{
      // Start command
      if(GamePad.isStartPressed()){
        joystickData.state = 9;
      }
    }
    return joystickData;
}

void comDabble::update(){
    Dabble.processInput();
}

GamepadState comDabble::readGamepad(){
    return {
        GamePad.isSquarePressed(), GamePad.isCirclePressed(),
        GamePad.isTrianglePressed(), GamePad.isCrossPressed(),
        GamePad.isUpPressed(), GamePad.isDownPressed(),
        GamePad.isLeftPressed(), GamePad.isRightPressed(),
        GamePad.isStartPressed(), GamePad.isSelectPressed(),
        GamePad.getRadius(), GamePad.getAngle()
    };
}
#endif

#endif