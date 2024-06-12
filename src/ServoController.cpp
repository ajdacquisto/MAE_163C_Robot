/**
 * @file ServoController.cpp
 * @brief Implementation file for the ServoController class.
 */

#include "ServoController.h"
#include "config.h" // Ensure this include path is correct

/**
 * @brief Default constructor for the ServoController class.
 */
ServoController::ServoController() {
  // Constructor implementation
}

/**
 * @brief Destructor for the ServoController class.
 */
ServoController::~ServoController() {
  // Destructor implementation
}

/**
 * @brief Initializes the servos.
 *
 * This function attaches the servos to their respective pins.
 */
void ServoController::init() {
  // Initialize the servos
  servo1.attach(SERVO1_PIN);
  servo2.attach(SERVO2_PIN);
  servo3.attach(SERVO3_PIN);
  servo4.attach(SERVO4_PIN);
}