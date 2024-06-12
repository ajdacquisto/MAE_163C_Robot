#ifndef SPEEDCONTROLLER_H
#define SPEEDCONTROLLER_H

class SpeedController {
private:
  int pwm_pin;
  float propeller_speed;

public:
  SpeedController(int pwm_pin);
  void init();
  void setSpeed(float speed);
  float getSpeed() const;
};

#endif // SPEEDCONTROLLER_H