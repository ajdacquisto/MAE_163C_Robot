/**
 * @file SDController.cpp
 * @brief Implementation of the SDController class.
 */

#include "SDController.h"

/**
 * @brief Constructs an SDController object with the specified chip select (CS) pin.
 * @param csPin The chip select (CS) pin to be used for the SD card.
 */
SDController::SDController(int csPin) : _csPin(csPin), _initialized(false) {}

/**
 * @brief Initializes the SD card and opens the data file for writing.
 * @return True if the SD card is successfully initialized and the data file is opened, false otherwise.
 */
bool SDController::begin() {
  if (SD.begin(_csPin)) {
    _file = SD.open("data.txt", FILE_WRITE);
    _initialized = _file ? true : false;
  }
  return _initialized;
}

/**
 * @brief Writes the specified data to the data file.
 * @param data The data to be written.
 * @return True if the data is successfully written, false otherwise.
 */
bool SDController::writeData(const String &data) {
  if (_initialized && _file) {
    _file.println(data);
    return true;
  }
  return false;
}

/**
 * @brief Closes the data file and releases the SD card.
 * @return True if the data file is successfully closed and the SD card is released, false otherwise.
 */
bool SDController::close() {
  if (_initialized && _file) {
    _file.close();
    _initialized = false;
    return true;
  }
  return false;
}
