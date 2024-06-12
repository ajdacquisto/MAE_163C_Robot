#include "SDController.h"

SDController::SDController(uint8_t csPin)
    : _csPin(csPin), _initialized(false) {}

bool SDController::begin() {
  if (sd.begin(_csPin)) {
    if (file.open("data.txt", O_WRONLY | O_CREAT | O_APPEND)) {
      _initialized = true;
    } else {
      _initialized = false;
    }
  }
  return _initialized;
}

bool SDController::writeData(const char *data) {
  if (_initialized && file.isOpen()) {
    file.println(data);
    file.sync();
    return true;
  }
  return false;
}

bool SDController::close() {
  if (_initialized && file.isOpen()) {
    file.close();
    _initialized = false;
    return true;
  }
  return false;
}