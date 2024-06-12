#ifndef SDCONTROLLER_H
#define SDCONTROLLER_H

#include <SPI.h>
#include <SdFat.h>

class SDController {
public:
  SDController(uint8_t csPin);

  bool begin();
  bool writeData(const char *data);
  bool close();

private:
  uint8_t _csPin;
  SdFat sd;
  SdFile file;
  bool _initialized;
};

#endif // SDCONTROLLER_H