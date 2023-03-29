#include <stdint.h>

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
    uint32_t _lastDcount = UINT32_MAX;
    uint32_t _lastDmillis = UINT32_MAX;
    uint32_t _lastDmicros = UINT32_MAX;
};
