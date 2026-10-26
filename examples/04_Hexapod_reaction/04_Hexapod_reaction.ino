#include <LeggedRobot.h>
#define USE_DABBLE
#include <penguinUtils.h>

// Imu
bnoImu imu(0x4B,15.0);

const float L1 = 5.325, L2 = 10.4, L3 = 14.565;

Adafruit_PWMServoDriver pwmL = Adafruit_PWMServoDriver(0x40);
Adafruit_PWMServoDriver pwmR = Adafruit_PWMServoDriver(0x41);

// Criacao das patas e do hexapod
RobotLeg LeftFoward    = {&pwmL, 12, 13, 14, 341, 819, 336, 795, 157, -320, L1, L2, L3};
RobotLeg LeftMiddle    = {&pwmL,  4,  5,  6, 356, 844, 335, 814, 147, -313, L1, L2, L3};
RobotLeg LeftBackward  = {&pwmL,  0,  1,  2, 346, 816, 349, 796, 162, -320, L1, L2, L3};

RobotLeg RightFoward   = {&pwmR,  3,  2,  1, 315, 747, 326, -104, 496, 920, L1, L2, L3};
RobotLeg RightMiddle   = {&pwmR, 11, 10,  9, 327, 761, 348, -86, 522, 934, L1, L2, L3};
RobotLeg RightBackward = {&pwmR, 15, 14, 13, 335, 769, 343, -83, 530, 960, L1, L2, L3};

LegConfig LeftFowardConfig    = {floatxyz{9.30,7.05,0.00}, int3{-45,26,-100}, 0, 0};
LegConfig LeftMiddleConfig    = {floatxyz{0.00,8.00,0.00}, int3{  0,26,-100}, RobotLeg::HALF_POINTS, 0};
LegConfig LeftBackwardConfig  = {floatxyz{-9.5,7.00,0.00}, int3{ 45,26,-100}, 0, 0};

LegConfig RightFowardConfig   = {floatxyz{9.30,-7.05,0.0}, int3{225,26,-100}, RobotLeg::HALF_POINTS, M_PI};
LegConfig RightMiddleConfig   = {floatxyz{0.00,-8.00,0.0}, int3{180,26,-100}, 0, M_PI};
LegConfig RightBackwardConfig = {floatxyz{-9.5,-7.00,0.0}, int3{135,26,-100}, RobotLeg::HALF_POINTS, M_PI};

RobotLeg* hexapodLegs[6] = {&LeftFoward, &LeftMiddle, &LeftBackward, &RightFoward, &RightMiddle, &RightBackward};
LegConfig hexapodConfigs[6] = {LeftFowardConfig, LeftMiddleConfig, LeftBackwardConfig, RightFowardConfig, RightMiddleConfig, RightBackwardConfig};

LeggedRobot robot(hexapodLegs, hexapodConfigs, 6);
comDabble com("Robot");

void TaskRobot(void *pvParameters);
void TaskCommunication(void *pvParameters);
void TaskIMU(void *pvParameters);
 
void setup() {
  Serial.begin(38400);
  com.begin();
  Serial.println("Robot started!");

  Wire.begin(21, 22);

  // Config PCA9685
  pwmL.begin(); pwmL.setPWMFreq(50);
  pwmR.begin(); pwmR.setPWMFreq(50);

  // Start BNO080 at 0x4B
  imu.start();
  delay(300);
  imu.calibrate();

  delay(2000);
  xTaskCreate(TaskRobot, "robot", 4096, NULL, 1, NULL);
  xTaskCreate(TaskCommunication, "communication", 4096, NULL, 1, NULL);
  xTaskCreate(TaskIMU, "imu", 4096, NULL, 1, NULL);
}

void loop(){}

void TaskRobot(void *pvParameters) {
  int k = 0;
  int totalPoints = RobotLeg::TOTAL_POINTS;
  for (;;) {
    // High handshake
    if(com.joystickData.state == 1){
        totalPoints = 50;
        int3 angles = {-10,-10,0};
        // robot.handShake(k,3,totalPoints);
        robot.highHandShake(k,3,angles,totalPoints);
        #if DEBUG_SIMULADOR
          robot.sendAngles(k);
        #endif
        if (k < (totalPoints/2)){
          k++;
        }
        vTaskDelay(pdMS_TO_TICKS(2));
    }
    // Wiggle
    else if(com.joystickData.state == 2){
        int3 angles = robot.circularRollPitchYaw(k, 10);
        robot.bodyTilt(angles);
        #if DEBUG_SIMULADOR
          robot.sendAngles(k);
        #endif         
        if (k == LeggedRobot::TOTAL_CIRCULAR_POINTS - 1){
          k = 0;
        }
        else{
          k++;
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    // mode 0 - Omnidirectional walk, mode 1 - Circular walk, mode 2 - Circular walk with IMU reaction
    else if(com.joystickData.state == 3){
      float angle = joystickToInt(com.joystickData.angle,5);
      if (com.joystickData.mode == 0){
        float angle_rad = angle*M_PI/180.0;
        k = robot.walk(k,angle_rad);
      }
      else if (com.joystickData.mode == 1){
        k = robot.circularWalk(k,angle);
      }
      else{
        int3 anglesint3 = int3(robot.poseAngles);
        k = robot.reactCircularWalk(k,angle,anglesint3);
      }
      #if DEBUG_SIMULADOR
        robot.sendAngles(k);
      #endif         
      vTaskDelay(pdMS_TO_TICKS(20));
    }
    // Start robot
    else if(com.joystickData.state == 9){
        robot.start();
        com.joystickData.state = 0;
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    // Shutdown robot
    else if(com.joystickData.state == 10){
        robot.shutdown();
        com.joystickData.state = 11;
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    // Idle position
    else if(com.joystickData.state == 0){
      k = 0;
      // Idle reacting to IMU
      if (com.joystickData.mode == 2){
        int3 anglesint3 = int3(robot.poseAngles);
        robot.idle(anglesint3);
      }
      // Idle straight position
      else{
        robot.idle();
      }
      #if DEBUG_SIMULADOR
        robot.sendAngles(k);
      #endif     
      vTaskDelay(pdMS_TO_TICKS(15));
    }
    else{
      vTaskDelay(pdMS_TO_TICKS(10));
    }
  }
}

void TaskCommunication(void *pvParameters) {
  for (;;) {
    com.getJoystick();
    vTaskDelay(pdMS_TO_TICKS(20));   
  }
}

void TaskIMU(void *pvParameters) {
  for (;;) {
    // Modes without IMU reaction
    if (com.joystickData.mode == 0 || com.joystickData.mode == 1){
        robot.poseAngles = {0.0,0.0,0.0};
    }
    else{
      // int3 angles = {roll,pitch,yaw};
      if (imu.getEvent()) {
        robot.poseAngles = imu.getPose(robot.poseAngles);
      }
    }
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}