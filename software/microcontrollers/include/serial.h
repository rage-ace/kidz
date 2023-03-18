#ifndef SERIAL_H
#define SERIAL_H

#include <Arduino.h>

class SerialManager {
  public:
    SerialManager(HardwareSerial &serial, const uint32_t baudRate,
                  const unsigned int rxPacketSize, const byte rxStartByte,
                  const byte rxEndByte, const unsigned int txPacketSize,
                  const byte txStartByte, const byte txEndByte)
        : _serial(serial), _baudRate(baudRate), _rxPacketSize(rxPacketSize),
          _rxStartByte(rxStartByte), _rxEndByte(rxEndByte),
          _txPacketSize(txPacketSize), _txStartByte(txStartByte),
          _txEndByte(txEndByte) {}

    // Setup serial
    void setup(const bool wait = false);
    void waitForPacket(void *writeTo = nullptr);

    // Send and receive packets
    bool readPacket(void *writeTo);
    void sendPacket(const void *readFrom);

    // Redirect serial
    void redirectBuffer(Stream &serial);

    // Print text
    template <typename... Args>
    void printf(const char *fmt, const Args &...args);

  private:
    HardwareSerial &_serial;
    uint32_t _baudRate;

    unsigned int _rxPacketSize;
    byte _rxStartByte;
    byte _rxEndByte;

    unsigned int _txPacketSize;
    byte _txStartByte;
    byte _txEndByte;
};

#endif
