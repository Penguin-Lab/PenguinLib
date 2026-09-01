#define CUSTOM_SETTINGS
#define INCLUDE_GAMEPAD_MODULE
#include <DabbleESP32.h>
#include <Wire.h>
#include <RobotLeg.h>

const float L1 = 5.325, L2 = 10.4, L3 = 14.565;

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x41);

// Leg creation with calibrations values
RobotLeg Right   = {&pwm,  0,  1,  2, 315, 747, 332, -96, 495, 923, L1, L2, L3};
int3 angles = {0,0,0};

void setup() {
  Serial.begin(38400);
  Serial.println("Robot leg started!");

  // Config PCA9685
  pwm.begin();
  pwm.setPWMFreq(50);

  delay(2000);

  floatxyz xyzStart = Right.initializeLeg(angles);
}


void loop(){
  Right.move(0,0,0);
  delay(4000);
  Right.move(0,45,0);
  delay(4000);
  Right.move(0,45,-70);
  delay(4000);
}