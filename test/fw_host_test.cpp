// =====================================================================
//  Host test of firmware/Ground_Station/Ground_Station.ino
//
//  The sketch is compiled on the PC against small stubs (test/stub) of
//  arduino-pico, LoRa, EEPROM and the OLED. The test plays the rocket:
//  it puts downlink packets "on the air", sends USB lines like the GUI
//  does, steps the clock one millisecond at a time and checks
//
//    * mode switching (USB, legacy commands, button) and its persistence,
//      the LoRa sync word of each rocket,
//    * the USB lines ($T $B $AT $AS $U $G $MODE),
//    * uplink timing inside the rocket's RX window, both copies of ABORT /
//      COUNTDOWN, the packet contents (magic, token, CRC, arg),
//    * ACK, retries in the next windows, FAILED after 3 windows,
//    * the ARIADNA UTC fallback window and the binary frames for the old
//      Daidalos ground_visualizer.py.
//
//      make -C test
// =====================================================================
#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>
#include <EEPROM.h>

uint32_t g_now = 0;
int g_pin[32] = {};
SerialStub Serial;
SPIClassStub SPI1;
LoRaStub LoRa;
TwoWireStub Wire;
EEPROMStub EEPROM;

#include "../firmware/Ground_Station/Ground_Station.ino"

static int g_fail = 0, g_ok = 0;
#define CHECK(c, ...) do { if (c) { g_ok++; } else { g_fail++; printf("   FAIL %s:%d: ", __FILE__, __LINE__); printf(__VA_ARGS__); printf("\n"); } } while (0)

// ---------------------------------------------------------------- helpers
static size_t g_out_pos = 0;
/** Text lines written since the last call (binary frames skipped). */
static std::vector<std::string> lines()
{
    std::vector<std::string> v;
    std::string cur;
    for (size_t i = g_out_pos; i < Serial.raw.size(); i++) {
        const char c = Serial.raw[i];
        if (c == '\n') { if (!cur.empty()) v.push_back(cur); cur.clear(); }
        else if (c != '\r') cur += c;
    }
    g_out_pos = Serial.raw.size();
    return v;
}
static bool has(const std::vector<std::string> &v, const std::string &prefix)
{
    for (auto &s : v) if (s.compare(0, prefix.size(), prefix) == 0) return true;
    return false;
}
static bool contains(const std::vector<std::string> &v, const std::string &part)
{
    for (auto &s : v) if (s.find(part) != std::string::npos) return true;
    return false;
}
static std::string last(const std::vector<std::string> &v, const std::string &prefix)
{
    for (auto it = v.rbegin(); it != v.rend(); ++it) if (it->compare(0, prefix.size(), prefix) == 0) return *it;
    return "";
}
static std::string find(const std::vector<std::string> &v, const std::string &prefix)
{
    for (auto &s : v) if (s.compare(0, prefix.size(), prefix) == 0) return s;
    return "";
}
static std::vector<std::string> split(const std::string &s)
{
    std::vector<std::string> f; std::string c;
    for (char ch : s) { if (ch == ',') { f.push_back(c); c.clear(); } else c += ch; }
    f.push_back(c);
    return f;
}
static void usb(const char *l) { Serial.in += l; Serial.in += "\n"; }
static void step(uint32_t ms) { for (uint32_t i = 0; i < ms; i++) { g_now++; loop(); } }
static void air(const void *p, size_t n) { StubPkt k; k.b.assign((const uint8_t *)p, (const uint8_t *)p + n); k.t_ms = g_now; LoRa.rx.push_back(k); }

static zl_tlm_t ztlm(uint16_t next_rx, uint8_t seq = 0, uint8_t rc = 0, uint8_t state = ZS_IDLE)
{
    zl_tlm_t t = {};
    t.state = state; t.alt_cm = 12345; t.next_rx_in_ms = next_rx; t.last_ul_seq = seq; t.last_ul_rc = rc;
    t.vbat_mv = 8100; t.lat_1e7 = 497985411; t.lon_1e7 = 166912772;
    zl_lora_seal(&t, sizeof t, ZL_TLM_MAGIC);
    return t;
}
static ariadna_lora_sync_t async(uint16_t next_rx, uint8_t seq = 0, uint8_t rc = 0, uint8_t flags = 0, int32_t t_rem = -1)
{
    ariadna_lora_sync_t s;
    memset(&s, 0, sizeof s);
    s.utc_ms = ARIADNA_UTC_UNKNOWN; s.next_rx_in_ms = next_rx; s.rx_dur_ms = 300; s.t_launch_rem_ms = t_rem;
    s.last_cmd_seq = seq; s.last_cmd_rc = rc; s.rampa_state = 2; s.rampa_flags = 0x83; s.sync_flags = flags; s.rampa_vbat_mv = 12400;
    ariadna_lora_sync_seal(&s);
    return s;
}
static ariadna_tlm_t atlm(void)
{
    ariadna_tlm_t t;
    memset(&t, 0, sizeof t);
    t.version = ARIADNA_TLM_VERSION; t.state = ARIADNA_STATE_ARMED; t.alt_dm = 15; t.vbat_cv = 150; t.qx = 1000;
    t.t_event_ds = ARIADNA_TLM_T_UNKNOWN; t.pyro = 0x13; t.health = 0xFF; t.uwb = 0xC4;
    return t;
}
static void reset_tx() { LoRa.tx.clear(); }

// ---------------------------------------------------------------- tests
static void t_boot_and_mode()
{
    printf("== boot, HELLO, mode switching\n");
    g_now = 10;
    setup(); setup1();
    lines();
    CHECK(s_mode == MODE_ZENITH, "blank EEPROM boots in ZENITH");
    CHECK(LoRa.sync_word == ZL_LORA_SYNCWORD, "Zenith sync word 0x%X", LoRa.sync_word);
    usb("HELLO"); step(2);
    auto v = lines();
    CHECK(find(v, "$ID,") == "$ID,Ground Station,3.00,ZENITH", "ID line '%s'", find(v, "$ID,").c_str());
    CHECK(has(v, "$MODE,ZENITH"), "MODE line");

    usb("MODE ARIADNA"); step(2);
    v = lines();
    CHECK(has(v, "$MODE,ARIADNA") && has(v, "#OK MODE ARIADNA"), "switched to ARIADNA");
    CHECK(LoRa.sync_word == 0x12, "Ariadna sync word 0x%X", LoRa.sync_word);
    CHECK(EEPROM.mem[0] == EE_MAGIC && EEPROM.mem[1] == MODE_ARIADNA && EEPROM.mem[2] == (uint8_t)~MODE_ARIADNA, "persisted");
    usb("MODE sapphire"); step(2);
    CHECK(s_mode == MODE_ZENITH && LoRa.sync_word == ZL_LORA_SYNCWORD, "alias SAPPHIRE -> ZENITH");
    usb("MODE daidalos"); step(2);
    CHECK(s_mode == MODE_ARIADNA, "alias DAIDALOS -> ARIADNA");
    usb("MODE MARS"); step(2);
    v = lines();
    CHECK(has(v, "#ERR MODE"), "bad mode refused");
    usb("MODE"); step(2);
    CHECK(has(lines(), "$MODE,ARIADNA"), "MODE query");

    // a reboot keeps the mode
    s_mode = MODE_ZENITH;
    setup();
    CHECK(s_mode == MODE_ARIADNA && LoRa.sync_word == 0x12, "mode restored from EEPROM after reboot");

    // button: BL held 3 s toggles the rocket
    g_now = 5000; step(1); lines();
    g_pin[BTN_L] = HIGH;
    for (int i = 0; i < 3200; i++) { g_now++; loop1(); loop(); }
    g_pin[BTN_L] = LOW;
    for (int i = 0; i < 10; i++) { g_now++; loop1(); loop(); }
    CHECK(s_mode == MODE_STAND && LoRa.sync_word == SL_LORA_SYNCWORD, "BL hold 3 s: ARIADNA -> STAND");
    for (int i = 0; i < 300; i++) { g_now++; loop1(); loop(); }
    CHECK(s_oled.screen.find("STAND") != std::string::npos, "OLED shows the new mode: %s", s_oled.screen.c_str());
    g_pin[BTN_L] = HIGH;
    for (int i = 0; i < 3200; i++) { g_now++; loop1(); loop(); }
    g_pin[BTN_L] = LOW;
    for (int i = 0; i < 10; i++) { g_now++; loop1(); loop(); }
    CHECK(s_mode == MODE_ZENITH, "BL hold 3 s again: STAND -> ZENITH");
    for (int i = 0; i < 300; i++) { g_now++; loop1(); loop(); }
    CHECK(s_oled.screen.find("ZENITH") != std::string::npos, "OLED shows the new rocket: %s", s_oled.screen.c_str());
    lines();
}

static void t_zenith_uplink()
{
    printf("== ZENITH: telemetry, window lock, ABORT twice, ACK\n");
    usb("MODE ZENITH"); step(2); lines(); reset_tx();
    const uint32_t air52 = zl_lora_airtime_us(sizeof(zl_tlm_t)) / 1000u;
    zl_tlm_t t = ztlm(4000);
    air(&t, sizeof t);
    const uint32_t rx_at = g_now + 1;            // heard on the next loop pass
    step(1);
    auto v = lines();
    const std::string T = find(v, "$T,");
    CHECK(!T.empty(), "$T emitted");
    auto f = split(T);
    CHECK(f.size() == 28, "$T has 27 fields (%zu)", f.size() - 1);
    CHECK(f.size() > 8 && f[8] == "123.45", "alt field '%s'", f.size() > 8 ? f[8].c_str() : "");
    const uint32_t open = rx_at - air52 + 4000;

    usb("CMD ABORT"); step(1);
    v = lines();
    CHECK(has(v, "$U,1,ABORT,0,QUEUED"), "ABORT queued");
    step(open + 400 - g_now);
    CHECK(LoRa.tx.size() == 2, "two ABORT copies sent (%zu)", LoRa.tx.size());
    if (LoRa.tx.size() == 2) {
        const int32_t d1 = (int32_t)(LoRa.tx[0].t_ms - open), d2 = (int32_t)(LoRa.tx[1].t_ms - open);
        CHECK(d1 >= (int)ZL_UL_OFFSET1_MS && d1 <= (int)ZL_UL_OFFSET1_MS + 2, "1st copy at +%d ms", d1);
        CHECK(d2 >= (int)ZL_UL_OFFSET2_MS && d2 <= (int)ZL_UL_OFFSET2_MS + 2, "2nd copy at +%d ms", d2);
        const auto &b = LoRa.tx[0].b;
        CHECK(b.size() == sizeof(zl_uplink_t) && zl_lora_check(b.data(), b.size(), sizeof(zl_uplink_t), ZL_UL_MAGIC), "valid zl_uplink_t");
        zl_uplink_t u; memcpy(&u, b.data(), sizeof u);
        CHECK(u.cmd == ZL_UL_ABORT && u.token == ZL_TOK_ABORT && u.seq == 1 && u.dest == ZL_UL_DEST_ROCKET, "ABORT fields");
        zl_uplink_t u2; memcpy(&u2, LoRa.tx[1].b.data(), sizeof u2);
        CHECK(u2.flags & ZL_ULF_RETRY, "2nd copy flagged RETRY");
    }
    v = lines();
    CHECK(has(v, "$U,1,ABORT,0,SENT,") && has(v, "$U,1,ABORT,0,SENT2,"), "SENT / SENT2 reported");
    t = ztlm(9500, 1, ZL_RC_ABORTED, ZS_ABORT);
    air(&t, sizeof t); step(2);
    v = lines();
    CHECK(has(v, "$U,1,ABORT,0,ACK,6,1"), "ACK rc 6 reported");
    CHECK(!s_ul.active, "nothing pending after ACK");
}

static void t_zenith_retry_fail()
{
    printf("== ZENITH: no ACK -> retries in 3 windows -> FAILED\n");
    reset_tx(); lines();
    zl_tlm_t t = ztlm(1000);
    air(&t, sizeof t); step(1);
    usb("CMD PADREF"); step(1);
    step(35000);
    auto v = lines();
    CHECK(LoRa.tx.size() == 3, "PADREF sent once per window, 3 windows (%zu)", LoRa.tx.size());
    if (LoRa.tx.size() == 3) {
        const int32_t p1 = (int32_t)(LoRa.tx[1].t_ms - LoRa.tx[0].t_ms), p2 = (int32_t)(LoRa.tx[2].t_ms - LoRa.tx[1].t_ms);
        CHECK(p1 == 10000 && p2 == 10000, "windows 10 s apart (%d, %d)", p1, p2);
    }
    CHECK(has(v, "$U,2,PADREF,0,FAILED"), "FAILED after 3 windows");
}

static void t_zenith_rampa_and_misc()
{
    printf("== ZENITH: rampa beacon, ROCKETPWR to the rampa, COUNTDOWN range, CANCEL\n");
    reset_tx(); lines();
    g_now += 30000; step(1);                       // rocket long gone
    zl_rampa_beacon_t b = {};
    b.state = ZR_ST_IDLE; b.vbat_mv = 12100; b.next_rx_in_ms = 2000;
    zl_lora_seal(&b, sizeof b, ZL_BEACON_MAGIC);
    air(&b, sizeof b); step(1);
    auto v = lines();
    CHECK(has(v, "$B,"), "$B emitted");
    CHECK(s_win_dest == ZL_UL_DEST_RAMPA, "locked to the rampa");
    usb("CMD ROCKETPWR 1"); step(2500);
    CHECK(LoRa.tx.size() == 1, "ROCKETPWR sent to the rampa (%zu)", LoRa.tx.size());
    if (!LoRa.tx.empty()) {
        zl_uplink_t u; memcpy(&u, LoRa.tx[0].b.data(), sizeof u);
        CHECK(u.cmd == ZL_UL_ROCKET_PWR && u.dest == ZL_UL_DEST_RAMPA && u.arg == 1, "ROCKETPWR fields");
    }
    usb("CMD COUNTDOWN 5"); step(2);
    CHECK(has(lines(), "#ERR countdown 10..3600 s"), "Zenith countdown range");
    usb("CMD COUNTDOWN 30"); step(2);
    CHECK(s_ul.active && s_ul.arg == 30000, "Zenith countdown arg in ms (%ld)", (long)s_ul.arg);
    usb("CANCEL"); step(2);
    v = lines();
    CHECK(contains(v, ",CANCELLED,") && !s_ul.active, "CANCEL");
}

static void t_ariadna_uplink()
{
    printf("== ARIADNA: telemetry, sync lock, COUNTDOWN on channel 2, ACK\n");
    usb("MODE ARIADNA"); step(2); lines(); reset_tx();
    ariadna_tlm_t at = atlm();
    air(&at, sizeof at); step(1);
    auto v = lines();
    const std::string A = find(v, "$AT,");
    auto f = split(A);
    CHECK(f.size() == 33, "$AT has 32 fields (%zu)", f.size() - 1);
    CHECK(f.size() > 6 && f[6] == "1.5", "alt field '%s'", f.size() > 6 ? f[6].c_str() : "");
    CHECK(f.size() > 16 && f[16] == "-32768", "t_event sentinel kept raw '%s'", f.size() > 16 ? f[16].c_str() : "");
    CHECK(Serial.raw.find("\xA5\x1D") == std::string::npos, "no binary frames without LEGACY");

    usb("CMD MAINALT 300"); step(2);
    CHECK(has(lines(), "#ERR MAINALT: Ariadna does not have this command"), "Zenith-only command refused");
    usb("CMD COUNTDOWN 3"); step(2);
    CHECK(has(lines(), "#ERR countdown 5..3600 s"), "Ariadna countdown range");

    const uint32_t air28 = ariadna_lora_airtime_us(sizeof(ariadna_lora_sync_t)) / 1000u;
    ariadna_lora_sync_t s = async(5000);
    air(&s, sizeof s);
    const uint32_t rx_at = g_now + 1;
    step(1);
    v = lines();
    CHECK(has(v, "$AS,"), "$AS emitted");
    CHECK(split(find(v, "$AS,")).size() == 15, "$AS has 14 fields");
    const uint32_t open = rx_at - air28 + 5000;
    usb("CMD COUNTDOWN 30 2"); step(1);
    const uint32_t queued = g_now;
    step(open + 400 - g_now);
    CHECK(LoRa.tx.size() == 2, "two COUNTDOWN copies (%zu)", LoRa.tx.size());
    if (LoRa.tx.size() == 2) {
        const auto &b = LoRa.tx[0].b;
        ariadna_lora_uplink_t u;
        CHECK(b.size() == sizeof u && ariadna_lora_uplink_decode(b.data(), b.size(), &u), "valid ariadna uplink");
        memcpy(&u, b.data(), sizeof u);
        const int32_t d1 = (int32_t)(LoRa.tx[0].t_ms - open);
        CHECK(d1 >= (int)ARIADNA_LORA_UL_OFFSET1_MS && d1 <= (int)ARIADNA_LORA_UL_OFFSET1_MS + 2, "1st copy at +%d ms", d1);
        CHECK(u.cmd == ARIADNA_UL_COUNTDOWN && u.token == ARIADNA_UL_TOK_COUNTDOWN && u.target == 2, "COUNTDOWN fields (cmd %u tok %04X ch %u)", u.cmd, u.token, u.target);
        const uint32_t expect = 30000u - (LoRa.tx[0].t_ms - queued);
        CHECK(u.arg_ms == expect, "arg_ms = time left to the fixed T-0 (%lu vs %lu)", (unsigned long)u.arg_ms, (unsigned long)expect);
        ariadna_lora_uplink_t u2; memcpy(&u2, LoRa.tx[1].b.data(), sizeof u2);
        CHECK(u2.arg_ms == expect - (LoRa.tx[1].t_ms - LoRa.tx[0].t_ms), "2nd copy keeps the same T-0");
        CHECK(u2.flags & ARIADNA_ULF_RETRY, "2nd copy flagged RETRY");
    }
    s = async(9000, s_ul.seq, ARIADNA_ULRC_OK, ARIADNA_SYNCF_CD_ACTIVE, 20000);
    air(&s, sizeof s); step(2);
    v = lines();
    CHECK(last(v, "$U,").find(",COUNTDOWN,0,ACK,1,1") != std::string::npos, "ACK reported: %s", last(v, "$U,").c_str());
}

static void t_ariadna_final_window()
{
    printf("== ARIADNA: ABORT in the T-1 s final window\n");
    reset_tx(); lines();
    ariadna_lora_sync_t s = async(600, 0, 0, ARIADNA_SYNCF_FINAL_WIN | ARIADNA_SYNCF_CD_ACTIVE, 1750);
    air(&s, sizeof s); step(1);
    usb("CMD ABORT"); step(1);
    step(1200);
    CHECK(LoRa.tx.size() == 2, "ABORT twice in the final window (%zu)", LoRa.tx.size());
    if (!LoRa.tx.empty()) {
        ariadna_lora_uplink_t u; memcpy(&u, LoRa.tx[0].b.data(), sizeof u);
        CHECK(u.cmd == ARIADNA_UL_ABORT && u.token == ARIADNA_UL_TOK_ABORT && (u.flags & ARIADNA_ULF_FINAL_WIN), "ABORT flagged FINAL_WIN");
    }
    CHECK(!s_win_valid, "the passed final window is not extrapolated on the 10 s grid");
    usb("CANCEL"); step(2); lines();
}

static void t_ariadna_utc_fallback()
{
    printf("== ARIADNA: no sync for > 25 s -> window from PC UTC\n");
    reset_tx(); lines();
    g_now += 40000; step(1);
    usb("UTC 43200000"); step(1);                  // 12:00:00.000 now
    const uint32_t t_utc = g_now;                  // taken in this loop pass
    usb("CMD PING"); step(1);
    step(10500);
    // the 12:00:00 window is open right now, no ACK -> again at 12:00:10
    CHECK(LoRa.tx.size() == 2, "PING sent in two UTC windows (%zu)", LoRa.tx.size());
    if (LoRa.tx.size() == 2) CHECK(LoRa.tx[1].t_ms - LoRa.tx[0].t_ms == 10000, "10 s apart");
    if (!LoRa.tx.empty()) {
        const uint32_t utc = 43200000u + (LoRa.tx[0].t_ms - t_utc);
        CHECK(utc % 10000u >= ARIADNA_LORA_UL_OFFSET1_MS && utc % 10000u <= ARIADNA_LORA_UL_OFFSET1_MS + 2, "at UTC +%lu ms in the window", (unsigned long)(utc % 10000u));
        ariadna_lora_uplink_t u; memcpy(&u, LoRa.tx[0].b.data(), sizeof u);
        CHECK(u.utc_ms == utc, "uplink carries the PC UTC (%lu vs %lu)", (unsigned long)u.utc_ms, (unsigned long)utc);
    }
    usb("CANCEL"); step(2); lines();
}

static void t_legacy()
{
    printf("== old Daidalos ground_visualizer.py commands\n");
    usb("MODE ZENITH"); step(2); lines(); reset_tx();
    s_legacy = false;
    usb("UTC:3600000"); step(2);
    CHECK(s_mode == MODE_ARIADNA && s_legacy, "legacy command -> ARIADNA + binary frames");
    usb("CMD:CD:30:3"); step(2);
    CHECK(s_ul.active && s_ul.cmd == ZL_UL_COUNTDOWN && s_ul.arg == 3, "CMD:CD:30:3 -> COUNTDOWN on channel 3");
    usb("CMD:REBOOT:2"); step(2);
    CHECK(s_ul.active && s_ul.cmd == ZL_UL_REBOOT && s_ul.arg == 2, "CMD:REBOOT:2");
    usb("CMD:PREP"); step(2);
    CHECK(s_ul.cmd == ZL_UL_PREPARE, "CMD:PREP");
    usb("CMD:DISARM"); step(2);
    CHECK(s_ul.cmd == ZL_UL_SAFE, "CMD:DISARM -> SAFE/DISARM");
    usb("CANCEL"); step(2);
    const size_t before = Serial.raw.size();
    ariadna_tlm_t at = atlm();
    air(&at, sizeof at); step(1);
    const std::string out = Serial.raw.substr(before);
    const size_t k = out.find("\xA5\x1D");
    CHECK(k != std::string::npos, "binary frame emitted");
    if (k != std::string::npos && out.size() >= k + 4 + 52 + 2) {
        const uint8_t *p = (const uint8_t *)out.data() + k;
        CHECK(p[2] == ARIADNA_PKT_TELEMETRY && p[3] == 52, "type 0x40, 52 B payload");
        const uint16_t crc = (uint16_t)(p[56] | (p[57] << 8));
        CHECK(ariadna_crc16(p + 2, 54) == crc, "frame CRC");
    }
    usb("T:1"); step(2);
    CHECK(s_test, "T:1 test mode");
    usb("LEGACY 0"); step(2);
    lines();
}

static void t_silent_rocket()
{
    printf("== rocket silent: a command does not hang forever\n");
    usb("MODE ZENITH"); step(2); lines(); reset_tx();
    g_now += 60000; step(1);
    usb("CMD SAFE"); step(1);
    step(44000);
    CHECK(s_ul.active, "still waiting after 44 s");
    step(2000);
    CHECK(!s_ul.active && contains(lines(), ",SAFE,0,FAILED"), "FAILED after 45 s without any window");
    CHECK(LoRa.tx.empty(), "nothing transmitted blind");
}

static void t_test_mode_and_status()
{
    printf("== TEST mode and $G for both rockets\n");
    usb("MODE ZENITH"); step(2); lines();
    usb("TEST 1"); step(1100);
    auto v = lines();
    CHECK(has(v, "$T,"), "Zenith test telemetry");
    const auto g = split(find(v, "$G,"));
    CHECK(g.size() == 13 && g[9] == "ZENITH", "$G has 12 fields and the mode (%zu)", g.size() - 1);
    usb("MODE ARIADNA"); step(2);
    CHECK(!s_test, "mode switch stops the test generator");
    usb("TEST 1"); step(1100);
    v = lines();
    CHECK(has(v, "$AT,"), "Ariadna test telemetry");
    CHECK(split(find(v, "$G,")).size() == 13 && split(find(v, "$G,"))[9] == "ARIADNA", "$G mode ARIADNA");
    for (int i = 0; i < 800; i++) { g_now++; loop1(); }
    CHECK(s_oled.screen.find("[T]") != std::string::npos, "OLED marks test mode");
    usb("TEST 0"); step(2);
    usb("FOO"); step(2);
    CHECK(has(lines(), "#ERR HELLO"), "help on an unknown line");
}

static void t_abort_button()
{
    printf("== BC held 1.5 s queues ABORT\n");
    usb("MODE ZENITH"); step(2); lines();
    g_pin[BTN_C] = HIGH;
    for (int i = 0; i < 1600; i++) { g_now++; loop1(); loop(); }
    g_pin[BTN_C] = LOW;
    for (int i = 0; i < 5; i++) { g_now++; loop1(); loop(); }
    CHECK(s_ul.active && s_ul.cmd == ZL_UL_ABORT, "ABORT queued by the button");
    CHECK(contains(lines(), ",ABORT,0,QUEUED"), "reported to the GUI");
    usb("CMD PREPARE"); step(2);
    CHECK(has(lines(), "#ERR abort pending"), "a pending ABORT is not replaced");
}

static sl_tlm_t stlm(uint8_t state = SL_ST_SAFE, uint8_t ack_seq = 0, uint8_t ack_rc = 0)
{
    sl_tlm_t t = {};
    t.state = state; t.flags = SL_F_KEY | SL_F_ADS_OK | SL_F_LC_CAL; t.vbat_mv = 8000; t.v5_mv = 5000;
    t.fexp = 1; for (int i = 0; i < SL_SUBS; i++) t.f[i] = sl_fpack(123.4f, 1);
    t.fpeak = sl_fpack(130.0f, 1); t.p_dpsi = 2500; t.ppeak_dpsi = 2600; t.t1_dc = 251; t.t2_dc = SL_T_NONE;
    t.ack_seq = ack_seq; t.ack_rc = ack_rc; t.test_no = 3; t.sps = 2000; t.pyro = 0x21;
    sl_seal(&t, sizeof t, SL_TLM_MAGIC);
    return t;
}
static bool sent_cmd(size_t i, sl_cmd_pkt_t &c)
{
    if (i >= LoRa.tx.size()) return false;
    const auto &b = LoRa.tx[i].b;
    if (!sl_check(b.data(), b.size(), sizeof c, SL_CMD_MAGIC)) return false;
    memcpy(&c, b.data(), sizeof c);
    return true;
}

static void t_stand()
{
    printf("== STAND: thrust stand on the rampa\n");
    usb("MODE MOTOR"); step(2);
    auto v = lines();
    CHECK(s_mode == MODE_STAND && has(v, "$MODE,STAND") && LoRa.sync_word == SL_LORA_SYNCWORD, "MODE MOTOR -> STAND, sync 0x%X", LoRa.sync_word);
    CHECK(EEPROM.mem[1] == MODE_STAND, "STAND persisted");
    reset_tx();
    step(1600);
    sl_cmd_pkt_t c{};
    CHECK(LoRa.tx.size() >= 2 && sent_cmd(0, c) && c.cmd == SL_C_PING, "rampa not heard: blind PINGs wake it (%zu)", LoRa.tx.size());
    CHECK(LoRa.tx.size() >= 2 && LoRa.tx[1].t_ms - LoRa.tx[0].t_ms >= 690, "blind PING every 700 ms");
    v = lines();
    CHECK(!contains(v, "PING"), "PINGs do not flood the USB");

    // telemetry -> $ST, the station answers right after it
    step(400); reset_tx();
    sl_tlm_t t = stlm();
    air(&t, sizeof t); step(20);
    v = lines();
    const std::string ST = find(v, "$ST,");
    auto f = split(ST);
    CHECK(f.size() == 33, "$ST has 32 fields (%zu)", f.size() - 1);
    CHECK(f.size() > 11 && f[11] == "123.40" && f[22] == "250.0" && f[24] == "25.1" && f[25] == "nan", "$ST values: force %s psi %s T1 %s T2 %s",
          f.size() > 25 ? f[11].c_str() : "", f.size() > 25 ? f[22].c_str() : "", f.size() > 25 ? f[24].c_str() : "", f.size() > 25 ? f[25].c_str() : "");
    CHECK(LoRa.tx.size() == 1 && sent_cmd(0, c) && c.cmd == SL_C_PING, "heartbeat PING after the TLM");
    if (!LoRa.tx.empty()) CHECK(LoRa.tx[0].t_ms - (g_now - 19) >= 6 && LoRa.tx[0].t_ms - (g_now - 19) <= 9, "6 ms after the TLM");

    // ARM: validated, sent after the next TLM, ACKed by the TLM after that
    usb("CMD ARM 5"); step(1);
    CHECK(has(lines(), "#ERR ARM"), "ARM 5 refused (igniter 1-3)");
    usb("CMD ARM 2 1500"); step(1);
    v = lines();
    CHECK(has(v, "$U,") && contains(v, ",ARM,2,QUEUED"), "ARM queued");
    reset_tx();
    t = stlm(); air(&t, sizeof t); step(20);
    CHECK(sent_cmd(0, c) && c.cmd == SL_C_ARM && c.arg == 2 && c.arg2 == 1500 && c.token == SL_TOK_ARM, "ARM 2 / 1500 ms sent with token");
    v = lines();
    CHECK(contains(v, ",ARM,2,SENT,"), "SENT reported");
    const uint8_t arm_seq = c.seq;
    t = stlm(SL_ST_ARMING, arm_seq, SL_RC_OK); air(&t, sizeof t); step(20);
    v = lines();
    CHECK(contains(v, ",ARM,2,ACK,1,"), "ACK rc 1");
    CHECK(!s_sul.active, "nothing pending");

    // FIRE: countdown range, retried after every TLM until the ACK
    usb("CMD FIRE 99"); step(1);
    CHECK(has(lines(), "#ERR FIRE"), "FIRE 99 s refused");
    usb("CMD FIRE 10"); step(1); lines();
    reset_tx();
    for (int i = 0; i < 3; i++) { t = stlm(SL_ST_ARMED); air(&t, sizeof t); step(200); }
    int fires = 0; uint8_t fseq = 0;
    for (size_t i = 0; i < LoRa.tx.size(); i++) if (sent_cmd(i, c) && c.cmd == SL_C_FIRE) { fires++; fseq = c.seq; CHECK(c.arg == 10 && c.token == SL_TOK_FIRE, "FIRE 10 s, token"); }
    CHECK(fires == 3, "FIRE repeated after each TLM until ACKed (%d)", fires);
    t = stlm(SL_ST_COUNTDOWN, fseq, SL_RC_OK); t.t_rel_ms = -9000; sl_seal(&t, sizeof t, SL_TLM_MAGIC); air(&t, sizeof t); step(20);
    v = lines();
    CHECK(contains(v, ",FIRE,2,ACK,1,"), "FIRE acknowledged");

    // ABORT: rampa silent -> blind every 300 ms; button BC works in STAND
    reset_tx();
    step(1600);
    usb("CMD ABORT"); step(1500);
    int ab = 0;
    for (size_t i = 0; i < LoRa.tx.size(); i++) if (sent_cmd(i, c) && c.cmd == SL_C_ABORT) ab++;
    CHECK(ab >= 4, "rampa not heard: ABORT sent blind every 300 ms (%d)", ab);
    usb("CMD TARE"); step(1);
    CHECK(has(lines(), "#ERR abort pending"), "ABORT cannot be replaced");
    t = stlm(SL_ST_ABORT, s_sul.seq, SL_RC_ABORTED); air(&t, sizeof t); step(20);
    v = lines();
    CHECK(contains(v, ",ABORT,2,ACK,5,"), "ABORT ACK rc 5");
    g_pin[BTN_C] = HIGH;
    for (int i = 0; i < 1600; i++) { g_now++; loop1(); loop(); }
    g_pin[BTN_C] = LOW;
    v = lines();
    CHECK(contains(v, ",ABORT,2,QUEUED"), "BC held 1.5 s -> STAND ABORT");
    usb("CANCEL"); step(2); lines();

    // results and curve chunks
    sl_res_t r = {};
    r.test_no = 3; r.flags = SL_RF_FIRED | SL_RF_IGNITION | SL_RF_SAVED | SL_RF_CURVE; r.peak_n = 150.5f; r.impulse_ns = 210.25f;
    r.burn_s = 1.9f; r.curve_n = 400; r.curve_hz = 100; r.t0_idx = 100; r.sps = 2000;
    sl_seal(&r, sizeof r, SL_RES_MAGIC);
    air(&r, sizeof r); step(5);
    v = lines();
    const std::string SR = find(v, "$SR,");
    CHECK(SR.compare(0, 23, "$SR,3,27,0,150.500,210.") == 0, "$SR line '%s'", SR.c_str());
    usb("CMD CURVE 32"); step(1); lines(); reset_tx();
    t = stlm(); air(&t, sizeof t); step(20);
    CHECK(sent_cmd(0, c) && c.cmd == SL_C_CURVE && c.arg == 32, "CURVE 32 requested");
    sl_curve_t cv = {};
    cv.test_no = 3; cv.offset = 32; cv.n = 16; cv.fexp = 0;
    for (int i = 0; i < 16; i++) { cv.f[i] = (int16_t)(i * 100); cv.p[i] = (int16_t)(i * 10); }
    sl_seal(&cv, sizeof cv, SL_CRV_MAGIC);
    air(&cv, sizeof cv); step(3);
    v = lines();
    const std::string SC = find(v, "$SC,");
    CHECK(split(SC).size() == 4 + 32 && SC.compare(0, 19, "$SC,3,32,16,0.00,10") == 0, "$SC line with 16 + 16 values");
    CHECK(contains(v, ",CURVE,2,ACK,1,"), "the CURVE packet itself acknowledges the request");

    // CAL in N -> mN on the air
    usb("CMD CAL 98.0665"); step(1); lines(); reset_tx();
    t = stlm(); air(&t, sizeof t); step(20);
    CHECK(sent_cmd(0, c) && c.cmd == SL_C_CAL && c.arg == 98067 && c.token == SL_TOK_CAL, "CAL 98.0665 N -> 98067 mN, token");
    t = stlm(SL_ST_SAFE, c.seq, SL_RC_OK); air(&t, sizeof t); step(20); lines();

    // no ACK at all -> FAILED
    usb("CMD TARE"); step(1); lines();
    for (int i = 0; i < 14; i++) { t = stlm(); air(&t, sizeof t); step(200); }
    CHECK(contains(lines(), ",TARE,2,FAILED"), "no ACK after 12 sends -> FAILED");

    // the rampa still a launch pad: its Zenith beacon is shown
    zl_rampa_beacon_t b = {}; b.vbat_mv = 7900; zl_lora_seal(&b, sizeof b, ZL_BEACON_MAGIC);
    air(&b, sizeof b); step(2);
    CHECK(has(lines(), "$B,"), "rampa beacon ($B) in STAND mode");

    // test generator
    usb("TEST 1"); step(1000);
    v = lines();
    CHECK(has(v, "$ST,"), "TEST 1 generates $ST");
    usb("TEST 0"); step(2);
    usb("MODE ZENITH"); step(2); lines();
    CHECK(s_mode == MODE_ZENITH && LoRa.sync_word == ZL_LORA_SYNCWORD, "back to ZENITH");
}

int main()
{
    t_boot_and_mode();
    t_zenith_uplink();
    t_zenith_retry_fail();
    t_zenith_rampa_and_misc();
    t_ariadna_uplink();
    t_ariadna_final_window();
    t_ariadna_utc_fallback();
    t_legacy();
    t_silent_rocket();
    t_test_mode_and_status();
    t_abort_button();
    t_stand();
    printf("\n%s: %d checks, %d failed\n", g_fail ? "FW HOST TEST FAILED" : "FW HOST TEST PASSED", g_ok + g_fail, g_fail);
    return g_fail ? 1 : 0;
}
