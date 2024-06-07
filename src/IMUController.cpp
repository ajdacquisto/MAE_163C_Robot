#include "IMUController.h"

IMUController::IMUController() {
  // Constructor implementation
}

IMUController::~IMUController() {
  // Destructor implementation
}

void IMUController::init() {
  // Initialize the IMU
  Wire.begin();
}

String IMUController::convert_int16_to_str(int16_t i) { // converts int16 to string. Moreover, resulting strings will have the same length in the debug monitor.
  sprintf(tmp_str, "%6d", i);
  return tmp_str;
}

String IMUController::read() {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x6B); // starting with register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false); // keep the connection active
    Wire.requestFrom(MPU_ADDR, 7*2, true); // request a total of 14 registers

    accelerometer_x = Wire.read() << 8 | Wire.read();
    accelerometer_y = Wire.read() << 8 | Wire.read();
    accelerometer_z = Wire.read() << 8 | Wire.read();
    temperature = Wire.read() << 8 | Wire.read();
    gyro_x = Wire.read() << 8 | Wire.read();
    gyro_y = Wire.read() << 8 | Wire.read();
    gyro_z = Wire.read() << 8 | Wire.read();

    String result = "aX = " + convert_int16_to_str(accelerometer_x);
    result += " | aY = " + convert_int16_to_str(accelerometer_y);
    result += " | aZ = " + convert_int16_to_str(accelerometer_z);
    result += " | tmp = " + String(temperature / 340.00 + 36.53);
    result += " | gX = " + convert_int16_to_str(gyro_x);
    result += " | gY = " + convert_int16_to_str(gyro_y);
    result += " | gZ = " + convert_int16_to_str(gyro_z);
    result += "\n";

    return result;
}