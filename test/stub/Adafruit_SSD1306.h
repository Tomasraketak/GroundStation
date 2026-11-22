#ifndef GS_STUB_SSD1306_H
#define GS_STUB_SSD1306_H
#include <Arduino.h>
#include <Wire.h>
#define SSD1306_SWITCHCAPVCC 2
#define SSD1306_WHITE 1
#define SSD1306_BLACK 0
/** Keeps the text of the last frame so a test can look at the screen. */
class Adafruit_SSD1306 : public Print {
public:
    std::string screen, frame;
    Adafruit_SSD1306(int, int, TwoWireStub *, int) {}
    bool begin(int, int) { return true; }
    void clearDisplay() { frame.clear(); }
    void display() { screen = frame; }
    void setTextColor(int) {}
    void setTextSize(int) {}
    void setCursor(int, int) { frame += '|'; }
    void fillRect(int, int, int, int, int) {}
    using Print::write;
    size_t write(const uint8_t *b, size_t n) override { frame.append((const char *)b, n); return n; }
};
#endif
