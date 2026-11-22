// Host stub of the bits of arduino-pico the ground station uses.
#ifndef GS_STUB_ARDUINO_H
#define GS_STUB_ARDUINO_H
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <stdarg.h>
#include <math.h>
#include <string>
#include <vector>
#include <deque>

#define HIGH 1
#define LOW 0
#define INPUT 0
extern uint32_t g_now;
extern int g_pin[32];
static inline uint32_t millis(void) { return g_now; }
static inline void delay(uint32_t) {}
static inline void pinMode(int, int) {}
static inline int digitalRead(int p) { return g_pin[p & 31]; }

class Print {
public:
    virtual ~Print() {}
    virtual size_t write(const uint8_t *b, size_t n) = 0;
    size_t write(uint8_t c) { return write(&c, 1); }
    size_t print(const char *s) { return write((const uint8_t *)s, strlen(s)); }
    size_t print(char c) { return write((const uint8_t *)&c, 1); }
    size_t print(int v) { char b[16]; snprintf(b, sizeof b, "%d", v); return print(b); }
    size_t print(long v) { char b[24]; snprintf(b, sizeof b, "%ld", v); return print(b); }
    size_t print(unsigned v) { char b[16]; snprintf(b, sizeof b, "%u", v); return print(b); }
    size_t print(unsigned long v) { char b[24]; snprintf(b, sizeof b, "%lu", v); return print(b); }
    size_t print(double v, int d = 2) { char b[32]; snprintf(b, sizeof b, "%.*f", d, v); return print(b); }
    size_t printf(const char *fmt, ...) __attribute__((format(printf, 2, 3))) {
        char b[256]; va_list ap; va_start(ap, fmt); vsnprintf(b, sizeof b, fmt, ap); va_end(ap); return print(b);
    }
};

/** USB serial: input is a queue the test fills, output is split into lines + raw bytes. */
class SerialStub : public Print {
public:
    std::string in;
    std::string raw;                    // everything written
    bool connected = true;
    void begin(unsigned long) {}
    explicit operator bool() const { return connected; }
    int available() { return (int)in.size(); }
    int read() { if (in.empty()) return -1; int c = (uint8_t)in[0]; in.erase(0, 1); return c; }
    int availableForWrite() { return 4096; }
    using Print::write;
    size_t write(const uint8_t *b, size_t n) override { raw.append((const char *)b, n); return n; }
};
extern SerialStub Serial;
#endif
