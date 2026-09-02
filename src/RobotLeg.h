#ifndef ROBOTLEG_H
#define ROBOTLEG_H

#include <Arduino.h>
#include <Adafruit_PWMServoDriver.h>
#include "MathTypes.h"


// =====================================================
// Robot Leg Class
// =====================================================

class RobotLeg {

public:

    RobotLeg(
        Adafruit_PWMServoDriver* pwm,
        int pinCoxa,
        int pinFemur,
        int pinTibia,
        int CMIN,
        int CMAX,
        int FMIN,
        int FMAX,
        int TMIN,
        int TMAX,
        float L1,
        float L2,
        float L3
    );

    // Classes constants
    static constexpr int TOTAL_POINTS = 25;
    static constexpr int HALF_POINTS = TOTAL_POINTS / 2;

    // Servo control
    void move(int angCoxa, int angFemur, int angTibia);
    bool moveSoft(int angCoxa, int angFemur, int angTibia, int step);
    void moveToInitialPosition();
    void shutdown();

    // Kinematics
    floatxyz directKinematics(int3 angles);
    int3 inverseKinematics(floatxyz xyz);

    // Trajectories
    floatxyz linearTrajetory(floatxyz footPosition, int k, int offset, float angle_rad);
    floatxyz legBezierTrajetory(floatxyz footPosition, int k, int offset, float dx, float dy, float dz, int totalPoints);
    void updateBezier(float stepLength);

    // Leg initialization
    void calculateFootPosition(int3 anglesIni);
    floatxyz initializeLeg(int3 anglesIni);

    // Leg data
    int3 angles;
    floatxyz footPosition;

    float L1;
    float L2;
    float L3;

    float P0[2];
    float P1[2];
    float P2[2];
    float P3[2];

private:

    Adafruit_PWMServoDriver* pwm;

    int3 pins;
    int3 LIMMIN;
    int3 LIMMAX;
};

#endif