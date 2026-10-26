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

int LEG = 0;
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
}


void loop(){
  Dabble.processInput();
  // Selection
  if (GamePad.isCrossPressed()){
    pwmL.sleep();
  }

  if (GamePad.isDownPressed()){
    move();
  }

  delay(200);
}