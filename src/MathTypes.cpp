#include "MathTypes.h"

// =====================================================
// float3 / int3
// =====================================================

int3::int3(const float3& values) {
    coxa = (int)values.coxa;
    femur = (int)values.femur;
    tibia = (int)values.tibia;
}

float3::float3(const int3& values) {
    coxa = (float)values.coxa;
    femur = (float)values.femur;
    tibia = (float)values.tibia;
}


// =====================================================
// Conversions
// =====================================================

int radToDegree(float angles_rad) {
    return int(angles_rad * 180.0 / M_PI);
}

int3 radToDegree(float3 angles_rad) {
    return {
        int(angles_rad.coxa * 180.0 / M_PI),
        int(angles_rad.femur * 180.0 / M_PI),
        int(angles_rad.tibia * 180.0 / M_PI)
    };
}

float3 degreeToRad(int3 angles) {
    return {
        float(angles.coxa) * M_PI / 180.0,
        float(angles.femur) * M_PI / 180.0,
        float(angles.tibia) * M_PI / 180.0
    };
}


// =====================================================
// Joystick
// =====================================================

float joystickToRad(int joystickAngle) {
    float angle_rad = (float(joystickAngle) - 90.0) * M_PI / 180.0;

    while (angle_rad > M_PI)
        angle_rad -= 2 * M_PI;

    while (angle_rad < -M_PI)
        angle_rad += 2 * M_PI;

    return angle_rad;
}


int joystickToInt(int joystickAngle, int deltaAngle) {
    int angle = joystickAngle - 90;

    while (angle > 180)
        angle -= 360;

    while (angle < -180)
        angle += 360;

    if (abs(angle - 180) <= deltaAngle || abs(angle + 180) <= deltaAngle) {
        return 180;
    }

    if (abs(angle - 90) <= deltaAngle * 4) {
        return 90;
    }

    if (abs(angle + 90) <= deltaAngle * 4) {
        return -90;
    }

    if (abs(angle) <= deltaAngle) {
        return 0;
    }

    return angle;
}