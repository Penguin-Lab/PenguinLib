#define CUSTOM_SETTINGS
#define INCLUDE_GAMEPAD_MODULE
#include <DabbleESP32.h>
#include <Adafruit_PWMServoDriver.h>
#include <Wire.h>

// Define the PCA9685 channel connected to the servo motor
#define MOTOR 0
// Define the PCA9685 address
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

#define TIME 20
#define LEFT 0
#define RIGHT 4

int PULSE_MIN = 129;
int PULSE_MAX = 559;
int pulse = (PULSE_MAX+PULSE_MIN)/2;
int littleStep = 5;
 
void setup() {
  Serial.begin(115200);
  Dabble.begin("Motor");
  Serial.println("Motor calibration!");

  pinMode(RIGHT,INPUT_PULLUP);
  pinMode(LEFT,INPUT_PULLUP);

  pwm.begin(); pwm.setPWMFreq(50);
  pwm.setPWM(MOTOR, 0, pulse);
  Serial.println("Pulse: " + String(pulse));
  delay(2000);
}

bool rightFlag = 0;
bool leftFlag = 0;
void loop(){
  Dabble.processInput();
  
  // Manual
  if (GamePad.isUpPressed()&&!rightFlag){
    pulse+=littleStep;
    pwm.setPWM(MOTOR, 0, pulse);
    Serial.println("Pulse: " + String(pulse));
    rightFlag = !rightFlag;
  }

  if (GamePad.isRightPressed()&&rightFlag){
    pulse+=littleStep;
    pwm.setPWM(MOTOR, 0, pulse);
    Serial.println("Pulse: " + String(pulse));
    rightFlag = !rightFlag;
  }

  if (GamePad.isDownPressed()&&!leftFlag){
    pulse-=littleStep;
    pwm.setPWM(MOTOR, 0, pulse);
    Serial.println("Pulse: " + String(pulse));
    leftFlag = !leftFlag;
  }

  if (GamePad.isLeftPressed()&&leftFlag){
    pulse-=littleStep;
    pwm.setPWM(MOTOR, 0, pulse);
    Serial.println("Pulse: " + String(pulse));
    leftFlag = !leftFlag;
  }

  // Automatic
  if (GamePad.isSquarePressed()){
    while(true){
      pulse++;
      pwm.setPWM(MOTOR, 0, pulse);
      Serial.println("Pulse: " + String(pulse));
      delay(TIME);
      if (!digitalRead(LEFT)){
        break;
      }
    }

    pwm.setPWM(MOTOR, 0, pulse);
    PULSE_MAX = pulse;
    Serial.println("Maximum pulse: " + String(PULSE_MAX));
  }

  if (GamePad.isCirclePressed()){
    while(true){
      pulse--;
      pwm.setPWM(MOTOR, 0, pulse);
      Serial.println("Pulse: " + String(pulse));
      delay(TIME);
      if (!digitalRead(RIGHT)){
        break;
      }
    }

    pwm.setPWM(MOTOR, 0, pulse);
    PULSE_MIN = pulse;
    Serial.println("Minimum pulse: " + String(PULSE_MIN));
  }

  if (GamePad.isTrianglePressed()){
    Serial.println("Range: " + String(PULSE_MIN) + " to " + String(PULSE_MAX));
  }
  
  if (GamePad.isCrossPressed()){
    pulse = int((PULSE_MAX+PULSE_MIN)/2);
    pwm.setPWM(MOTOR, 0, pulse);
    Serial.println("Value: " + String(pulse));
  }

  if (GamePad.isStartPressed()){
    PULSE_MIN = 100;
    PULSE_MAX = 540;
    pulse = (PULSE_MAX+PULSE_MIN)/2;
    pwm.setPWM(MOTOR, 0, pulse);
    Serial.println("Value: " + String(pulse));
  }
  delay(200);
}