#include "SDController.h"

SDController::SDController(int csPin) : _csPin(csPin), _initialized(false) {}

bool SDController::begin() {
    if (SD.begin(_csPin)) {
        _file = SD.open("data.txt", FILE_WRITE);
        _initialized = _file ? true : false;
    }
    return _initialized;
}

bool SDController::writeData(const String& data) {
    if (_initialized && _file) {
        _file.println(data);
        return true;
    }
    return false;
}

bool SDController::close() {
    if (_initialized && _file) {
        _file.close();
        _initialized = false;
        return true;
    }
    return false;
}
