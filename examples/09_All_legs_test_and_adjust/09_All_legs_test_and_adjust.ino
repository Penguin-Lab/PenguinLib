#define CUSTOM_SETTINGS
#define INCLUDE_GAMEPAD_MODULE
#include <DabbleESP32.h>
#include <Wire.h>
#include <RobotLeg.h>

const float L1 = 5.325, L2 = 10.4, L3 = 14.565;

Adafruit_PWMServoDriver pwmL = Adafruit_PWMServoDriver(0x40);
Adafruit_PWMServoDriver pwmR = Adafruit_PWMServoDriver(0x41);

// Leg creation with calibrations values
RobotLeg LeftFoward    = {&pwmL, 12, 13, 14, 341, 819, 336, 795, 157, -320, L1, L2, L3};
RobotLeg LeftMiddle    = {&pwmL,  4,  5,  6, 356, 844, 335, 814, 147, -313, L1, L2, L3};
RobotLeg LeftBackward  = {&pwmL,  0,  1,  2, 346, 816, 349, 796, 162, -320, L1, L2, L3};
RobotLeg RightFoward   = {&pwmR,  3,  2,  1, 315, 747, 326, -104, 496, 920, L1, L2, L3};
RobotLeg RightMiddle   = {&pwmR, 11, 10,  9, 327, 761, 348, -86, 522, 934, L1, L2, L3};
RobotLeg RightBackward = {&pwmR, 15, 14, 13, 335, 769, 343, -83, 530, 960, L1, L2, L3};
RobotLeg* hexapodLegs[6] = {&LeftFoward, &LeftMiddle, &LeftBackward, &RightFoward, &RightMiddle, &RightBackward};

int LEG = 5;
int MOTOR = 1;
int pulse0;
int pulse90, pulsem90;
int pulse180;

int littleStepUp = 5;
int littleStepDown = 2;
bool rightFlag = 0;
bool leftFlag = 0;

int3 angles = {0,0,0};

void move(){
  int3 anglesLeg;
  if(LEG > 2){
    anglesLeg = {-angles.coxa,angles.femur,angles.tibia};
  }
  else{
    anglesLeg = {angles.coxa,angles.femur,angles.tibia};
  }
  // hexapodLegs[LEG]->initializeLeg(angles);
  hexapodLegs[LEG]->angles = {-1, -1, -1};
  hexapodLegs[LEG]->move(angles.coxa,angles.femur,angles.tibia);
}

int3 changeAngle(int3 angles, int motor, int angle){
  int3 anglesOut = angles;
  if(motor == 0){
    anglesOut.coxa = angle;
  }
  else if(motor == 1){
    anglesOut.femur = angle;
  }
  else{
    anglesOut.tibia = angle;
  }
  return anglesOut;
}

void changePulse(int motor){
  if(motor == 0){
    pulse0 = hexapodLegs[LEG]->LIMMIN.coxa;
    pulse180 = hexapodLegs[LEG]->LIMMAX.coxa;
  }
  else if(motor == 1){
    pulse0 = hexapodLegs[LEG]->LIMMIN.femur;
    pulse180 = hexapodLegs[LEG]->LIMMAX.femur;
    pulse90 = map(90,0,180,pulse0,pulse180);
  }
  else{
    pulse0 = hexapodLegs[LEG]->LIMMIN.tibia;
    pulse180 = hexapodLegs[LEG]->LIMMAX.tibia;
    pulsem90 = map(-90,0,180,pulse0,pulse180);
  }
}

void updateLIM(int motor){
  if(motor == 1){
    hexapodLegs[LEG]->LIMMIN.femur = pulse0;
    hexapodLegs[LEG]->LIMMAX.femur = pulse180;
  }
  else{
    hexapodLegs[LEG]->LIMMIN.tibia = pulse0;
    hexapodLegs[LEG]->LIMMAX.tibia = pulse180;
  }
}

void setup() {
  Serial.begin(38400);
  Dabble.begin("Legs");
  Serial.println("Robot leg" + String(LEG) + " started!");

  // Config PCA9685
  pwmL.begin();
  pwmL.setPWMFreq(50);
  pwmR.begin();
  pwmR.setPWMFreq(50);

  delay(2000);

  for(int k=0; k<6; k++){
    floatxyz xyzStart = hexapodLegs[k]->initializeLeg(angles);
  }
  // floatxyz xyzStart = hexapodLegs[LEG]->initializeLeg(angles);
  changePulse(MOTOR);
  Serial.println("Range: " + String(pulse0) + " to " + String(pulse180));
}


void loop(){
  Dabble.processInput();
  // Selection
  if (GamePad.isCrossPressed()){
    MOTOR = 1;
    changePulse(MOTOR); // pulse0, pulse90 e pulse180 recebem os valores da perna
  }
  else if (GamePad.isSquarePressed()){
    MOTOR = 2;
    changePulse(MOTOR); // pulse0, pulsem90 e pulse180 recebem os valores da perna
  }

  // Manual
  if (GamePad.isUpPressed()&&!rightFlag){
    if(MOTOR == 1){
      if(angles.femur == 0){
        pulse0 += littleStepUp;
      }
      else{
        pulse90 += littleStepUp;
        pulse180 = map(180,0,90,pulse0,pulse90);
      }
    }
    else{
      if(angles.tibia == 0){
        pulse0 += littleStepUp;
      }
      else{
        pulsem90 += littleStepUp;
        pulse180 = map(180,0,-90,pulse0,pulsem90);
      }
    }
    updateLIM(MOTOR);
    move();
    Serial.println("Range: " + String(pulse0) + " to " + String(pulse180));
    rightFlag = !rightFlag;
  }

  if (GamePad.isRightPressed()&&rightFlag){
    if(MOTOR == 1){
      if(angles.femur == 0){
        pulse0 += littleStepUp;
      }
      else{
        pulse90 += littleStepUp;
        pulse180 = map(180,0,90,pulse0,pulse90);
      }
    }
    else{
      if(angles.tibia == 0){
        pulse0 += littleStepUp;
      }
      else{
        pulsem90 += littleStepUp;
        pulse180 = map(180,0,-90,pulse0,pulsem90);
      }
    }
    updateLIM(MOTOR);
    move();
    Serial.println("Range: " + String(pulse0) + " to " + String(pulse180));
    rightFlag = !rightFlag;
  }

  if (GamePad.isDownPressed()&&!leftFlag){
    if(MOTOR == 1){
      if(angles.femur == 0){
        pulse0 -= littleStepDown;
      }
      else{
        pulse90 -= littleStepDown;
        pulse180 = map(180,0,90,pulse0,pulse90);
      }
    }
    else{
      if(angles.tibia == 0){
        pulse0 -= littleStepDown;
      }
      else{
        pulsem90 -= littleStepDown;
        pulse180 = map(180,0,-90,pulse0,pulsem90);
      }
    }
    updateLIM(MOTOR);
    move();
    Serial.println("Range: " + String(pulse0) + " to " + String(pulse180));
    leftFlag = !leftFlag;
  }

  if (GamePad.isLeftPressed()&&leftFlag){
    if(MOTOR == 1){
      if(angles.femur == 0){
        pulse0 -= littleStepDown;
      }
      else{
        pulse90 -= littleStepDown;
        pulse180 = map(180,0,90,pulse0,pulse90);
      }
    }
    else{
      if(angles.tibia == 0){
        pulse0 -= littleStepDown;
      }
      else{
        pulsem90 -= littleStepDown;
        pulse180 = map(180,0,-90,pulse0,pulsem90);
      }
    }
    updateLIM(MOTOR);
    move();
    Serial.println("Range: " + String(pulse0) + " to " + String(pulse180));
    leftFlag = !leftFlag;
  }

  if (GamePad.isTrianglePressed()){
    Serial.println("Leg: " + String(LEG));
    Serial.println("Range coxa: " + String(hexapodLegs[LEG]->LIMMIN.coxa) + " to " + String(hexapodLegs[LEG]->LIMMAX.coxa));
    Serial.println("Range femur: " + String(hexapodLegs[LEG]->LIMMIN.femur) + " to " + String(hexapodLegs[LEG]->LIMMAX.femur));
    Serial.println("Range tibia: " + String(hexapodLegs[LEG]->LIMMIN.tibia) + " to " + String(hexapodLegs[LEG]->LIMMAX.tibia));
  }

  if (GamePad.isStartPressed()){
    angles = changeAngle(angles, MOTOR, 0);
    move();
  }
  else if (GamePad.isSelectPressed()){
    if(MOTOR == 1){
      angles = changeAngle(angles, MOTOR, 90);
    }
    else{
      angles = changeAngle(angles, MOTOR, -90);
    }
    move();
  }

  delay(200);
}