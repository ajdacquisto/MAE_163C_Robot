#include "ControlSystem.h"
#include <Arduino.h>

// Define your pins and constants here
const uint8_t SD_CS_PIN = 10;
const uint8_t FRONT_SERVO_PIN = 9;
const uint8_t RIGHT_SERVO_PIN = 8;
const uint8_t REAR_SERVO_PIN = 7;
const uint8_t LEFT_SERVO_PIN = 6;
const uint8_t PWM_PIN = 5;
const float FRONT_REAR_ARM_LENGTH = 0.1; // Example length
const float LEFT_RIGHT_ARM_LENGTH = 0.1; // Example length
const float MAX_SERVO_ANGLE = 30.0;      // Example max angle
const float MAX_THRUST = 10.0;           // Example max thrust
const float MAX_PROP_SPEED = 2000.0;     // Example max prop speed

float Kp_array[3][3] = {{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}};
float Kd_array[3][3] = {{0.1, 0.0, 0.0}, {0.0, 0.1, 0.0}, {0.0, 0.0, 0.1}};

Matrix3x3 Kp(Kp_array);
Matrix3x3 Kd(Kd_array);

ControlSystem controlSystem(0.01, Kp, Kd, 0.1, 1.0, SD_CS_PIN, FRONT_SERVO_PIN,
                            RIGHT_SERVO_PIN, REAR_SERVO_PIN, LEFT_SERVO_PIN,
                            FRONT_REAR_ARM_LENGTH, LEFT_RIGHT_ARM_LENGTH,
                            MAX_SERVO_ANGLE, MAX_THRUST, MAX_PROP_SPEED,
                            PWM_PIN);

void setup() { controlSystem.initialize(); }

void loop() { controlSystem.controlLoop(); }