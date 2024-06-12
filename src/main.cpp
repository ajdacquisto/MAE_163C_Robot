#include "ControlSystem.h"
#include <Arduino.h>

// Define your pins and constants here
constexpr int SD_CS_PIN = 10;
constexpr int FRONT_SERVO_PIN = 9;
constexpr int RIGHT_SERVO_PIN = 8;
constexpr int REAR_SERVO_PIN = 7;
constexpr int LEFT_SERVO_PIN = 6;
constexpr int PWM_PIN = 5;
constexpr float FRONT_REAR_ARM_LENGTH = 0.1f; // Example length
constexpr float LEFT_RIGHT_ARM_LENGTH = 0.1f; // Example length
constexpr float MAX_SERVO_ANGLE = 30.0f;      // Example max angle
constexpr float MAX_THRUST = 10.0f;           // Example max thrust
constexpr float MAX_PROP_SPEED = 2000.0f;     // Example max prop speed

BLA::Matrix<3, 3> Kp = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};

BLA::Matrix<3, 3> Kd = {0.1f, 0.0f, 0.0f, 0.0f, 0.1f, 0.0f, 0.0f, 0.0f, 0.1f};

ControlSystem controlSystem(0.01f, Kp, Kd, 0.1f, 1.0f, SD_CS_PIN,
                            FRONT_SERVO_PIN, RIGHT_SERVO_PIN, REAR_SERVO_PIN,
                            LEFT_SERVO_PIN, FRONT_REAR_ARM_LENGTH,
                            LEFT_RIGHT_ARM_LENGTH, MAX_SERVO_ANGLE, MAX_THRUST,
                            MAX_PROP_SPEED, PWM_PIN);

void setup() { controlSystem.initialize(); }

void loop() { controlSystem.controlLoop(); }