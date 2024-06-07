#ifndef SDCONTROLLER_H
#define SDCONTROLLER_H

#include <SD.h>
#include <SPI.h>

class SDController {
public:
    SDController(int csPin);
    bool begin();
    bool writeData(const String& data);
    bool close();

private:
    int _csPin;
    File _file;
    bool _initialized;
};

#endif // SDCONTROLLER_H
