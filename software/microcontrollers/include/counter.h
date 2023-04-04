#ifndef TEENSY_COUNTER_H
#define TEENSY_COUNTER_H

#include <cstdint>

class Counter {
  public:
    Counter();
    bool countElapsed(const uint32_t count);
    bool millisElapsed(const uint32_t millis);
    bool microsElapsed(const uint32_t micros);
    void reset();

  private:
    bool started = false;
    uint32_t _count;
    uint32_t _lastTime;
};

#endif
