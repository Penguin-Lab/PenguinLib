#ifndef MATHTYPES_H
#define MATHTYPES_H

#include <Arduino.h>
#include <math.h>

// =====================================================
// Math types
// =====================================================

struct int3;
struct float3;

struct int3 {
    int coxa;
    int femur;
    int tibia;

    int3() = default;
    int3(const float3& values);
    int3(int c, int f, int t) : coxa(c), femur(f), tibia(t) {}

    int3 operator+(const int3& rhs) const {
        return {
            coxa + rhs.coxa,
            femur + rhs.femur,
            tibia + rhs.tibia
        };
    }

    int3 operator-(const int3& rhs) const {
        return {
            coxa - rhs.coxa,
            femur - rhs.femur,
            tibia - rhs.tibia
        };
    }

    int3 operator*(int scalar) const {
        return {
            coxa * scalar,
            femur * scalar,
            tibia * scalar
        };
    }

    int3 operator/(int scalar) const {
        return {
            coxa / scalar,
            femur / scalar,
            tibia / scalar
        };
    }

    bool operator==(const int3& rhs) const {
        return coxa == rhs.coxa &&
               femur == rhs.femur &&
               tibia == rhs.tibia;
    }

    bool operator!=(const int3& rhs) const {
        return !(*this == rhs);
    }
};

struct float3 {
    float coxa;
    float femur;
    float tibia;

    float3() = default;
    float3(const int3& values);
    float3(float c, float f, float t) : coxa(c), femur(f), tibia(t) {}

    float3 operator+(const float3& rhs) const {
        return {
            coxa + rhs.coxa,
            femur + rhs.femur,
            tibia + rhs.tibia
        };
    }

    float3 operator-(const float3& rhs) const {
        return {
            coxa - rhs.coxa,
            femur - rhs.femur,
            tibia - rhs.tibia
        };
    }

    float3 operator*(float scalar) const {
        return {
            coxa * scalar,
            femur * scalar,
            tibia * scalar
        };
    }

    float3 operator/(float scalar) const {
        return {
            coxa / scalar,
            femur / scalar,
            tibia / scalar
        };
    }

    bool operator==(const float3& rhs) const {
        return coxa == rhs.coxa &&
               femur == rhs.femur &&
               tibia == rhs.tibia;
    }

    bool operator!=(const float3& rhs) const {
        return !(*this == rhs);
    }
};

struct floatxyz {
    float x;
    float y;
    float z;

    floatxyz operator+(const floatxyz& rhs) const {
        return {
            x + rhs.x,
            y + rhs.y,
            z + rhs.z
        };
    }

    floatxyz operator-(const floatxyz& rhs) const {
        return {
            x - rhs.x,
            y - rhs.y,
            z - rhs.z
        };
    }

    floatxyz operator*(float scalar) const {
        return {
            x * scalar,
            y * scalar,
            z * scalar
        };
    }

    floatxyz operator/(float scalar) const {
        return {
            x / scalar,
            y / scalar,
            z / scalar
        };
    }

    floatxyz operator*(const floatxyz& rhs) const {
        return {
            x * rhs.x,
            y * rhs.y,
            z * rhs.z
        };
    }
};


// =====================================================
// Math functions
// =====================================================

int radToDegree(float angles_rad);
int3 radToDegree(float3 angles_rad);
float3 degreeToRad(int3 angles);

float joystickToRad(int joystickAngle);
int joystickToInt(int joystickAngle, int deltaAngle);

#endif