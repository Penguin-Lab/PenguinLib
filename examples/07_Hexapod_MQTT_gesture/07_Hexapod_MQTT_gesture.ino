#include <LeggedRobot.h>
#define USE_MQTT
#include <penguinUtils.h>

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

// Atualizei esses valores!
LegConfig LeftFowardConfig    = {floatxyz{9.30,7.05,0.00}, int3{-45,26,-100}, 0, 0};
LegConfig LeftMiddleConfig    = {floatxyz{0.00,8.00,0.00}, int3{  0,26,-100}, RobotLeg::HALF_POINTS, 0};
LegConfig LeftBackwardConfig  = {floatxyz{-9.5,7.00,0.00}, int3{ 45,26,-100}, 0, 0};

LegConfig RightFowardConfig   = {floatxyz{9.30,-7.05,0.0}, int3{225,26,-100}, RobotLeg::HALF_POINTS, M_PI};
LegConfig RightMiddleConfig   = {floatxyz{0.00,-8.00,0.0}, int3{180,26,-100}, 0, M_PI};
LegConfig RightBackwardConfig = {floatxyz{-9.5,-7.00,0.0}, int3{135,26,-100}, RobotLeg::HALF_POINTS, M_PI};

RobotLeg* hexapodLegs[6] = {&LeftFoward, &LeftMiddle, &LeftBackward, &RightFoward, &RightMiddle, &RightBackward};
LegConfig hexapodConfigs[6] = {LeftFowardConfig, LeftMiddleConfig, LeftBackwardConfig, RightFowardConfig, RightMiddleConfig, RightBackwardConfig};

LeggedRobot robot(hexapodLegs, hexapodConfigs, 6);
comMqtt com(
    "SEU-WIFI",
    "SUA-SENHA",
    "10.10.2.211",
    30001,
    "guest",
    "guest",
    "leggedrobot/is-gesture"
);

void TaskRobot(void *pvParameters);
void TaskCommunication(void *pvParameters);
 
void setup() {
  Serial.begin(38400);
  com.begin();
  Serial.println("Robot started!");

  // Config PCA9685
  pwmL.begin(); pwmL.setPWMFreq(50);
  pwmR.begin(); pwmR.setPWMFreq(50);

  delay(2000);
  xTaskCreate(TaskRobot, "robot", 4096, NULL, 1, NULL);
  xTaskCreate(TaskCommunication, "communication", 4096, NULL, 1, NULL);
}

void loop(){}

int3 bowing(int3 finalAngles, int3 currentAngles, int step){
    int3 nextAngles = currentAngles;
    if (currentAngles.coxa > finalAngles.coxa)
        nextAngles.coxa -= step;
    else if (currentAngles.coxa < finalAngles.coxa)
        nextAngles.coxa += step;

    if (currentAngles.femur > finalAngles.femur)
        nextAngles.femur -= step;
    else if (currentAngles.femur < finalAngles.femur)
        nextAngles.femur += step;

    if (currentAngles.tibia > finalAngles.tibia)
        nextAngles.tibia -= step;
    else if (currentAngles.tibia < finalAngles.tibia)
        nextAngles.tibia += step;
    
    return nextAngles;
}

floatxyz legTrajetory(floatxyz footPosition, int k, int offset, int totalPoints){
    floatxyz xyz;
    int halfPoints = totalPoints/2;
    int kn = (k + offset) % totalPoints;
    if (kn < halfPoints){
      float t = float(kn)/(halfPoints-1);
      float u = 1 - t;
      xyz.x = footPosition.x + cos(angle_rad)*(-footPosition.x + u*u*u*this->P0[0] + 3*u*u*t*this->P1[0] + 3*u*t*t*this->P2[0] + t*t*t*this->P3[0]);
      xyz.y = footPosition.y + sin(angle_rad)*(-footPosition.x + u * u * u * this->P0[0] + 3*u*u*t*this->P1[0] + 3*u*t*t*this->P2[0] + t*t*t*this->P3[0]);
      xyz.z = u*u*u*this->P0[1] + 3*u*u*t*this->P1[1] + 3*u*t*t*this->P2[1] + t*t*t*this->P3[1];
    }
    else{
      xyz.x = footPosition.x + cos(angle_rad)*(-footPosition.x + this->P3[0] + (this->P0[0] - this->P3[0])*(float(kn - HALF_POINTS)/(HALF_POINTS - 1)));
      xyz.y = footPosition.y + sin(angle_rad)*(-footPosition.x + this->P3[0] + (this->P0[0] - this->P3[0])*(float(kn - HALF_POINTS)/(HALF_POINTS - 1)));
      xyz.z = footPosition.z;
    }
    return xyz;

    int3 angles[MAX_LEGS];
    for (int i = 0; i < numLegs; i++) {
        floatxyz xyz = robotlegs[i]->linearTrajetory(robotlegs[i]->footPosition, k, config[i].offset, angle_rad - config[i].installedAngle_rad);
        angles[i] = robotlegs[i]->inverseKinematics(xyz);
    }
    for (int i = 0; i < numLegs; i++) {
        robotlegs[i]->move(angles[i].coxa, angles[i].femur, angles[i].tibia);
    }

    if (k == robotlegs[0]->TOTAL_POINTS - 1){
        return 0;
    }
    else{
        return k+1;
    }
}

void TaskRobot(void *pvParameters) {
  int k = 0;
  int totalPoints = RobotLeg::TOTAL_POINTS;
  int last_joystick_velocity = -1;
  for (;;) {
    if(last_joystick_velocity != com.joystickData.velocity){
        last_joystick_velocity = com.joystickData.velocity;
        if(com.joystickData.state == 3){
            // Bowing
            if(last_joystick_velocity == 3){
                int3 angles = {0,10,0};
                while(angles != robot.poseAngles){
                    robot.poseAngles = bowing(angles, robot.poseAngles, 1);
                    robot.bodyTilt(robot.poseAngles);
                    vTaskDelay(pdMS_TO_TICKS(500));
                }
                angles = {0,0,0};
                while(angles != robot.poseAngles){
                    robot.poseAngles = bowing(angles, robot.poseAngles, 1);
                    robot.bodyTilt(robot.poseAngles);
                    vTaskDelay(pdMS_TO_TICKS(500));
                }
                vTaskDelay(pdMS_TO_TICKS(20));
            }
            // Typing on a keyboard
            else if(last_joystick_velocity == 4){
                int3 anglesLeftMiddle = robot.robotlegs[1]->inverseKinematics(robot.robotlegs[1]->footPosition);
                int3 anglesRightMiddle = robot.robotlegs[4]->inverseKinematics(robot.robotlegs[4]->footPosition);
                while(!robot.robotlegs[1]->moveSoft(anglesLeftMiddle.coxa-10, anglesLeftMiddle.femur+10, anglesLeftMiddle.tibia, 1)){
                    vTaskDelay(pdMS_TO_TICKS(20));
                }
                while(!robot.robotlegs[1]->moveSoft(anglesLeftMiddle.coxa-10, anglesLeftMiddle.femur, anglesLeftMiddle.tibia, 1)){
                    vTaskDelay(pdMS_TO_TICKS(20));
                }
                while(!robot.robotlegs[4]->moveSoft(anglesRightMiddle.coxa+10, anglesRightMiddle.femur+10, anglesRightMiddle.tibia, 1)){
                    vTaskDelay(pdMS_TO_TICKS(20));
                }
                while(!robot.robotlegs[4]->moveSoft(anglesRightMiddle.coxa+10, anglesRightMiddle.femur, anglesRightMiddle.tibia, 1)){
                    vTaskDelay(pdMS_TO_TICKS(20));
                }
                int3 anglesLeftFoward = robot.robotlegs[0]->inverseKinematics(robot.robotlegs[0]->footPosition);
                int3 anglesRightFoward = robot.robotlegs[3]->inverseKinematics(robot.robotlegs[3]->footPosition);
                bool done = false;
                while(!done){
                    done = robot.robotlegs[0]->moveSoft(anglesLeftFoward.coxa-80, anglesLeftFoward.femur+30, anglesLeftFoward.tibia, 1);
                    done &= robot.robotlegs[3]->moveSoft(anglesRightFoward.coxa+80, anglesRightFoward.femur+30, anglesRightFoward.tibia, 1);
                    vTaskDelay(pdMS_TO_TICKS(20));
                }
                for(int j=0; j<3; j++){
                    done = false;
                    while(!done){
                        done = robot.robotlegs[0]->moveSoft(anglesLeftFoward.coxa-80, anglesLeftFoward.femur+40, anglesLeftFoward.tibia, 1);
                        done &= robot.robotlegs[3]->moveSoft(anglesRightFoward.coxa+80, anglesRightFoward.femur+20, anglesRightFoward.tibia, 1);
                        vTaskDelay(pdMS_TO_TICKS(20));
                    }
                    done = false;
                    while(!done){
                        done = robot.robotlegs[0]->moveSoft(anglesLeftFoward.coxa-80, anglesLeftFoward.femur+20, anglesLeftFoward.tibia, 1);
                        done &= robot.robotlegs[3]->moveSoft(anglesRightFoward.coxa+80, anglesRightFoward.femur+40, anglesRightFoward.tibia, 1);
                        vTaskDelay(pdMS_TO_TICKS(20));
                    }
                }
                done = false;
                while(!done){
                    done = robot.robotlegs[0]->moveSoft(anglesLeftFoward.coxa, anglesLeftFoward.femur+30, anglesLeftFoward.tibia, 1);
                    done &= robot.robotlegs[3]->moveSoft(anglesRightFoward.coxa, anglesRightFoward.femur+30, anglesRightFoward.tibia, 1);
                    vTaskDelay(pdMS_TO_TICKS(20));
                }
                done = false;
                while(!done){
                    done = robot.robotlegs[0]->moveSoft(anglesLeftFoward.coxa, anglesLeftFoward.femur, anglesLeftFoward.tibia, 1);
                    done &= robot.robotlegs[3]->moveSoft(anglesRightFoward.coxa, anglesRightFoward.femur, anglesRightFoward.tibia, 1);
                    vTaskDelay(pdMS_TO_TICKS(20));
                }
                while(!robot.robotlegs[1]->moveSoft(anglesLeftMiddle.coxa, anglesLeftMiddle.femur+10, anglesLeftMiddle.tibia, 1)){
                    vTaskDelay(pdMS_TO_TICKS(20));
                }
                while(!robot.robotlegs[1]->moveSoft(anglesLeftMiddle.coxa, anglesLeftMiddle.femur, anglesLeftMiddle.tibia, 1)){
                    vTaskDelay(pdMS_TO_TICKS(20));
                }
                while(!robot.robotlegs[4]->moveSoft(anglesRightMiddle.coxa, anglesRightMiddle.femur+10, anglesRightMiddle.tibia, 1)){
                    vTaskDelay(pdMS_TO_TICKS(20));
                }
                while(!robot.robotlegs[4]->moveSoft(anglesRightMiddle.coxa, anglesRightMiddle.femur, anglesRightMiddle.tibia, 1)){
                    vTaskDelay(pdMS_TO_TICKS(20));
                }
                vTaskDelay(pdMS_TO_TICKS(20));
            }
            // Cheer up
            else if(last_joystick_velocity == 5){
                for(int j=0; j<3; j++){
                    int3 angles = robot.circularRollPitchYaw(k, 10);
                    robot.bodyTilt(angles);       
                    if (k == LeggedRobot::TOTAL_CIRCULAR_POINTS - 1){
                        k = 0;
                    }
                    else{
                        k++;
                    }
                    vTaskDelay(pdMS_TO_TICKS(20));
                }
                k = 0;
            }
            // Handshaking
            else if(last_joystick_velocity == 6){
                totalPoints = 50;
                int3 angles = {-10,-10,0};
                // robot.handShake(k,3,totalPoints);
                unsigned long handTime = millis();
                while((millis()-handTime)<6000){
                    robot.highHandShake(k,3,angles,totalPoints);
                    if (k < (totalPoints/2)){
                        k++;
                    }
                    vTaskDelay(pdMS_TO_TICKS(2));
                }
            }
            else{
                vTaskDelay(pdMS_TO_TICKS(10));
            }
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
            robot.idle();
            vTaskDelay(pdMS_TO_TICKS(15));
        }
        else{
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
  }
}


void TaskCommunication(void *pvParameters) {
  for (;;) {
    // You can use the Joystick data as a payload with 2 values (access data with: com.joystickData.velocity and com.joystickData.angle)
    com.getJoystick();
    vTaskDelay(pdMS_TO_TICKS(20));   
  }
}