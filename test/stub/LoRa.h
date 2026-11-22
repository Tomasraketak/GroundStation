// Host stub of sandeepmistry LoRa: the test queues received packets and
// reads back what the station transmitted (with the time it did).
#ifndef GS_STUB_LORA_H
#define GS_STUB_LORA_H
#include <Arduino.h>
#include <SPI.h>
struct StubPkt { std::vector<uint8_t> b; uint32_t t_ms; };
class LoRaStub {
public:
    bool begin_ok = true;
    int sync_word = 0x12, tx_dbm = 17;
    std::deque<StubPkt> rx;             // packets waiting to be "heard"
    std::vector<StubPkt> tx;            // packets sent
    std::vector<uint8_t> cur, txbuf;
    size_t pos = 0;
    int rssi = -70; float snr = 9.0f;
    void setSPI(SPIClassStub &) {}
    void setPins(int, int, int) {}
    int begin(long) { sync_word = 0x12; return begin_ok; }
    void idle() {}
    void receive() {}
    void setSpreadingFactor(int) {}
    void setSignalBandwidth(long) {}
    void setCodingRate4(int) {}
    void setPreambleLength(long) {}
    void setSyncWord(int w) { sync_word = w; }
    void setTxPower(int d) { tx_dbm = d; }
    void enableCrc() {}
    int parsePacket() {
        if (rx.empty() || (int32_t)(g_now - rx.front().t_ms) < 0) return 0;
        cur = rx.front().b; rx.pop_front(); pos = 0;
        return (int)cur.size();
    }
    int available() { return (int)(cur.size() - pos); }
    int read() { return pos < cur.size() ? cur[pos++] : -1; }
    int packetRssi() { return rssi; }
    float packetSnr() { return snr; }
    int beginPacket() { txbuf.clear(); return 1; }
    size_t write(const uint8_t *b, size_t n) { txbuf.insert(txbuf.end(), b, b + n); return n; }
    int endPacket(bool = false) { tx.push_back({ txbuf, g_now }); return 1; }
};
extern LoRaStub LoRa;
#endif
