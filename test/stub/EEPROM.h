#ifndef GS_STUB_EEPROM_H
#define GS_STUB_EEPROM_H
#include <Arduino.h>
class EEPROMStub {
public:
    uint8_t mem[256]; int commits = 0;
    EEPROMStub() { memset(mem, 0xFF, sizeof mem); }
    void begin(size_t) {}
    uint8_t read(int a) { return mem[a & 255]; }
    void write(int a, uint8_t v) { mem[a & 255] = v; }
    bool commit() { commits++; return true; }
};
extern EEPROMStub EEPROM;
#endif
