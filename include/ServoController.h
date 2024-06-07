#ifndef SERVOCONTROLLER_H
#define SERVOCONTROLLER_H

#include "config.h"
#include <Servo.h> // Update the include path to use angle brackets instead of quotes

/**
 * @class ServoController
 * @brief Controls multiple servos for a robot.
 * 
 * The ServoController class provides a way to control multiple servos for a robot.
 * It initializes the servos and provides member functions to control their movements.
 */
class ServoController {
public:
  /**
   * @brief Default constructor.
   */
  ServoController();

  /**
   * @brief Destructor.
   */
  ~ServoController();

  // Add your member functions here
  /**
   * @brief Initializes the servo controller.
   * 
   * This function initializes the servo controller by attaching the servos to their respective pins.
   */
  void init();

private:
  // Add your member variables here
  Servo servo1; /**< The first servo. */
  Servo servo2; /**< The second servo. */
  Servo servo3; /**< The third servo. */
  Servo servo4; /**< The fourth servo. */
};

#endif // SERVOCONTROLLER_H
