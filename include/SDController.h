#ifndef SDCONTROLLER_H
#define SDCONTROLLER_H

#include <SD.h>
#include <SPI.h>

/**
 * @brief The SDController class provides an interface for controlling an SD card.
 */
class SDController {
public:
  /**
   * @brief Constructs an SDController object with the specified chip select (CS) pin.
   * @param csPin The chip select (CS) pin to be used for communication with the SD card.
   */
  SDController(int csPin);

  /**
   * @brief Initializes the SD card and prepares it for reading and writing.
   * @return True if the initialization is successful, false otherwise.
   */
  bool begin();

  /**
   * @brief Writes data to the SD card.
   * @param data The data to be written.
   * @return True if the write operation is successful, false otherwise.
   */
  bool writeData(const String &data);

  /**
   * @brief Closes the SD card file.
   * @return True if the file is closed successfully, false otherwise.
   */
  bool close();

private:
  int _csPin; /**< The chip select (CS) pin used for communication with the SD card. */
  File _file; /**< The file object representing the opened file on the SD card. */
  bool _initialized; /**< Flag indicating whether the SD card has been initialized. */
};

#endif // SDCONTROLLER_H
