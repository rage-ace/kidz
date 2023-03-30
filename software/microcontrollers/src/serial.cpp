#include "serial.h"

#include <cassert>

// Starts the serial port.
void SerialManager::setup(const bool wait) { _serial.begin(_baudRate); }

// Waits for the next packet.
void SerialManager::waitForPacket(void *writeTo) {
    while (!readPacket(writeTo)) delay(1);
}

// Reads the last packet from the serial port if there is any in the buffer.
// Return if a packet has been read.
bool SerialManager::readPacket(void *writeTo) {
    bool read = false;
    while (_serial.available() >= (signed int)_rxPacketSize) {
        const auto syncByte = _serial.read();
        if (syncByte == _rxStartByte) {
            // start of packet is valid, proceed to read rest of packet
            char buf[_rxPacketSize - 1];
            _serial.readBytes(buf, _rxPacketSize - 1);
            if (buf[_rxPacketSize - 2] == _rxEndByte) {
                // end of packet is valid, proceed to save payload
                if (writeTo != nullptr) memcpy(writeTo, buf, _rxPacketSize - 2);
                read = true;
            }
        }
    }
    return read;
}

// Sends a packet over the serial port.
void SerialManager::sendPacket(const void *readFrom) {
    assert(readFrom != nullptr);
    _serial.write(_txStartByte);
    _serial.write((const uint8_t *)readFrom, _txPacketSize - 2);
    _serial.write(_txEndByte);
}

// Prints the current buffer to a new serial port.
void SerialManager::redirectBuffer(Stream &serial) {
    if (_serial.available() > 0) serial.write(_serial.read());
}

// TODO: THIS DOES NOT WORK
// // Prints a formatted string to the serial port.
// template <typename... Args>
// void SerialManager::printf(const char *fmt, const Args &...args) {
//     _serial.printf(fmt, args...);
// }
