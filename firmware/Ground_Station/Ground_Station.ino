// =====================================================================
//  GROUND STATION — one firmware for both rockets and the motor test stand
//
//      ZENITH   (Zenith v2.4 flight computer, "Sapphire" rocket)
//      ARIADNA  (Ariadna V2 avionics, "Daidalos" landing rocket)
//      STAND    (the rampa PCB alone as a static-fire thrust stand:
//                igniter, thrust, chamber pressure, 2 temperatures)
//
//  The mode is chosen at run time — from the GUI (USB "MODE ZENITH" /
//  "MODE ARIADNA" / "MODE STAND") or by holding the left button for 3 s
//  (cycles ZENITH -> ARIADNA -> STAND) — and remembered
//  in flash, so the station comes back in the same mode after a power
//  cycle and works without a laptop. No recompiling, no reflashing.
//
//  Target: Raspberry Pi Pico 2 W (rp2040:rp2040:rpipico2w), arduino-pico.
//          (rpipico2 builds and runs the same — nothing here uses Wi-Fi.)
//  Needs : LoRa (Sandeep Mistry), Adafruit GFX, Adafruit SSD1306.
//
//  Pinout (Netlist_PCBGroundStation_2026-11-19, same PCB for both rockets):
//    LoRa Ra-02  SPI1: MISO GP12, CS GP11, SCK GP14, MOSI GP15, RST GP20, DIO0 GP21
//    OLED SSD1306 0x3C  I2C0: SDA GP4, SCL GP5 (400 kHz)
//    Buttons (active HIGH, 10k pull-downs):
//      BL GP3   short: next OLED page        hold 3 s: next mode ZENITH -> ARIADNA -> STAND
//      BC GP6   hold 1.5 s: ABORT (works without the laptop)
//      BR GP10  hold 2 s: reboot the station
//
//  ---------------------------------------------------------------------
//  RADIO — both rockets use 433 MHz, SF7, BW 250 kHz, CR 4/5, preamble 8.
//  The only difference on the air is the LoRa sync word (Zenith 0x2B, the
//  Ariadna default 0x12), so the station hears exactly one rocket at a time
//  and switching is instant. Both use the same TDMA idea: the rocket opens
//  a 300 ms RX window every 10 s (+ a final abort window at T-1 s) and
//  every downlink packet says when. The station locks to it and transmits
//  one queued command inside it (ABORT / COUNTDOWN twice), then waits for
//  the rocket's acknowledgement (seq + result code) and retries in the
//  next windows (max 3). Both rockets ignore a repeated seq, so a retry is
//  always safe.
//
//      ZENITH   rx  zl_tlm_t 52 B (5 Hz, carries the window + ACK)
//                   zl_rampa_beacon_t 26 B (pad alone, no rocket on the pogo)
//               tx  zl_uplink_t 16 B
//      ARIADNA  rx  ariadna_tlm_t 50 B (10 Hz)
//                   ariadna_lora_sync_t 28 B (1-2 Hz on the ground: window, ACK, rampa)
//               tx  ariadna_lora_uplink_t 20 B
//      STAND    rx  sl_tlm_t 56 B (2 / 5 Hz), sl_res_t 52 B, sl_curve_t 74 B, sl_burn_t 188 B
//               tx  sl_cmd_pkt_t 14 B — no windows: sent right after a TLM (the
//                   rampa listens whenever it is not transmitting), PING as a
//                   heartbeat (the rampa aborts a countdown without one),
//                   blind repeats when the rampa is not heard (ABORT every 300 ms)
//
//  ---------------------------------------------------------------------
//  USB (115200, text lines "\r\n") — docs/USB_PROTOCOL.md
//    out  $ID  $MODE  $G (1 Hz)  $U (uplink status)  #OK / #ERR
//         ZENITH : $T telemetry, $B rampa beacon   (= the Zenith GS protocol)
//         ARIADNA: $AT telemetry, $AS sync/ACK/rampa
//         STAND  : $ST telemetry, $SR results, $SC curve chunk, $SB burn block (+ $B rampa beacon)
//    in   HELLO | MODE [ZENITH|ARIADNA|STAND] | CMD <NAME> [arg] [arg2|RAMPA]
//         CANCEL | TEST 0|1 | UTC <ms of day> | LEGACY 0|1
//
//  Backward compatible: in ZENITH mode the old gui/ground_station_gui.py of
//  the Zenith repo works unchanged; the old Daidalos ground_visualizer.py
//  works too — its commands (T:1, UTC:..., CMD:PREP, CMD:CD:30:1 ...) switch
//  the station to ARIADNA and turn on the binary frames it expects.
//
//  Core 0: radio, uplink timing, USB — never blocked by the display.
//  Core 1: OLED (23 ms per redraw at 400 kHz) and the buttons.
// =====================================================================
#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>
#include <EEPROM.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "hardware/watchdog.h"
#include "pico/critical_section.h"
#include "zenith_link.h"
#include "ariadna_link.h"
#include "stand_link.h"

#define GS_FW_VERSION  300          // 3.00: + STAND (thrust stand on the rampa)

// The two radio set-ups may only differ in the sync word. If either header
// changes its modulation the station must learn it, so stop the build.
static_assert(ZL_LORA_FREQ_HZ == ARIADNA_LORA_FREQ_HZ, "frequency differs");
static_assert(ZL_LORA_SF == ARIADNA_LORA_SF && ZL_LORA_BW_HZ == ARIADNA_LORA_BW_HZ &&
              ZL_LORA_CR4 == ARIADNA_LORA_CR4 && ZL_LORA_PREAMBLE == ARIADNA_LORA_PREAMBLE,
              "modulation differs");
static_assert(SL_LORA_FREQ_HZ == ZL_LORA_FREQ_HZ && SL_LORA_SF == ZL_LORA_SF && SL_LORA_BW_HZ == ZL_LORA_BW_HZ &&
              SL_LORA_CR4 == ZL_LORA_CR4 && SL_LORA_PREAMBLE == ZL_LORA_PREAMBLE, "stand modulation differs");
static_assert(ZL_WIN_PERIOD_MS == ARIADNA_LORA_WIN_PERIOD_MS && ZL_WIN_DUR_MS == ARIADNA_LORA_WIN_DUR_MS,
              "TDMA window differs");

#define ARIADNA_LORA_SYNCWORD  0x12   // SX127x default, what the Ariadna Master uses
#define ARIADNA_CD_MIN_S       5      // ariadna_rampa_link.h ARIADNA_COUNTDOWN_MIN_MS
#define ARIADNA_CD_MAX_S       3600
#define ARIADNA_RAMPA_KEY      0x01   // ARIADNA_RMP_KEY_ARMED (ariadna_rampa_link.h)

// ---------------------------------------------------------------- hardware
static const int LORA_MISO = 12, LORA_CS = 11, LORA_SCK = 14, LORA_MOSI = 15, LORA_RST = 20, LORA_DIO0 = 21;
static const int OLED_SDA = 4, OLED_SCL = 5;
static const int BTN_L = 3, BTN_C = 6, BTN_R = 10;

static Adafruit_SSD1306 s_oled(128, 64, &Wire, -1);

// ---------------------------------------------------------------- mode
enum : uint8_t { MODE_ZENITH = 0, MODE_ARIADNA = 1, MODE_STAND = 2, MODE_COUNT = 3 };
static const char *MODE_NAME[MODE_COUNT] = { "ZENITH", "ARIADNA", "STAND" };
static uint8_t s_mode = MODE_ZENITH;

#define EE_MAGIC  0x47u
// EEPROM: [0] magic, [1] mode, [2] ~mode

// ---------------------------------------------------------------- names
static const char *ZST_NAME[ZS_COUNT] = { "BOOT", "IDLE", "READY", "CDOWN", "BOOST", "COAST", "DROGUE", "MAIN", "LANDED", "ABORT", "DIAG" };
static const char *AST_NAME[12] = { "BOOT", "IDLE", "ARM", "ASCENT", "APOGEE", "DESC", "GUIDE", "LANDED", "RECOV", "ABORT", "TEST", "ERROR" };
static const char *RS_NAME[8] = { "OFF", "IDLE", "READY", "CDOWN", "COMMIT", "FIRED", "ABORT", "FAULT" };
static const char *SL_NAME[SL_ST_COUNT] = { "SAFE", "ARMING", "ARMED", "CDOWN", "FIRE!", "BURN", "ABORT", "REC" };

// STAND command vocabulary (CMD <NAME> ... in MODE STAND)
static const char *SL_CMD_NAME[SL_C_COUNT] = { "PING", "ARM", "DISARM", "FIRE", "ABORT", "TARE", "RECORD", "STOP",
                                               "RESULT", "CURVE", "BUZZER", "RECTIME", "CAL", "PZERO", "BURN" };
#define UL_DEST_STAND  2u

// One command vocabulary for both rockets (the Zenith numbering, which is
// the superset). ARIADNA maps the ones it knows, see ariadna_cmd().
static const char *CMD_NAME[ZL_UL_COUNT] = { "NOP", "PREPARE", "COUNTDOWN", "ABORT", "SAFE", "PADREF", "MAINALT", "LOCATE", "CAMERA", "REBOOT", "ROCKETPWR" };

/** Unified command -> ARIADNA_UL_*, or -1 when the Ariadna rocket has no such command. */
static int ariadna_cmd(uint8_t c)
{
    switch (c) {
    case ZL_UL_NOP:       return ARIADNA_UL_NOP;
    case ZL_UL_PREPARE:   return ARIADNA_UL_LAUNCH_SOON;
    case ZL_UL_COUNTDOWN: return ARIADNA_UL_COUNTDOWN;
    case ZL_UL_ABORT:     return ARIADNA_UL_ABORT;
    case ZL_UL_SAFE:      return ARIADNA_UL_DISARM;
    case ZL_UL_PADREF:    return ARIADNA_UL_PADREF;
    case ZL_UL_REBOOT:    return ARIADNA_UL_REBOOT;
    default:              return -1;
    }
}
static uint16_t ariadna_token(uint8_t ariadna_ul)
{
    switch (ariadna_ul) {
    case ARIADNA_UL_LAUNCH_SOON: return ARIADNA_UL_TOK_PREP;
    case ARIADNA_UL_COUNTDOWN:   return ARIADNA_UL_TOK_COUNTDOWN;
    case ARIADNA_UL_ABORT:       return ARIADNA_UL_TOK_ABORT;
    case ARIADNA_UL_REBOOT:      return ARIADNA_UL_TOK_REBOOT;
    default:                     return ARIADNA_UL_TOK_NONE;
    }
}

// ---------------------------------------------------------------- state (core 0)
static bool     s_lora_ok = false;
static uint32_t s_rx_ok = 0, s_rx_bad = 0;
static int      s_rssi = 0; static float s_snr = 0;
static bool     s_test = false;
static bool     s_legacy = false;          // binary Ariadna frames for the old ground_visualizer.py

// ZENITH
static zl_tlm_t          s_ztlm = {};
static zl_rampa_beacon_t s_zbcn = {};
static uint32_t          s_ztlm_ms = 0, s_zbcn_ms = 0;
// ARIADNA
static ariadna_tlm_t       s_atlm = {};
static ariadna_lora_sync_t s_async = {};
static uint32_t            s_atlm_ms = 0, s_async_ms = 0;
static uint32_t            s_atlm_dt = 0;  // ms between the last two telemetry packets
// STAND
static sl_tlm_t            s_stlm = {};
static sl_res_t            s_sres = {};
static uint32_t            s_stlm_ms = 0, s_sres_ms = 0, s_srx_ms = 0;   // srx = any packet of the rampa
static bool                s_stx_due = false;     // the rampa just transmitted: our turn
static bool                s_stx_burst = false;   // ... but it was a BURN block: only a real command, no PING
static uint32_t            s_stx_ms = 0;          // our last transmission
static uint32_t            s_sping_ms = 0;
struct StandPending {
    bool active; uint8_t seq, cmd; int32_t arg; uint16_t arg2; uint8_t sent; uint32_t queued_ms;
};
static StandPending s_sul = {};

// PC time (UTC ms of day) — the ARIADNA fallback window grid when the sync
// packets fade, and the clock the Ariadna rocket adopts when it has no GPS.
static uint32_t s_utc_base = 0, s_utc_ref = 0;
static bool     s_utc_valid = false;

// window lock
static uint32_t s_win_open_ms = 0;
static uint32_t s_win_heard_ms = 0;        // when the packet that set it arrived
static bool     s_win_valid = false;
static bool     s_win_final = false;
static uint8_t  s_win_dest = ZL_UL_DEST_ROCKET;   // ZENITH: rocket or rampa

// uplink: one pending command
struct Pending {
    bool     active;
    uint8_t  seq, cmd, dest;       // cmd = unified (ZL_UL_*) numbering
    int32_t  arg;                  // ZENITH: as sent; ARIADNA: REBOOT target / COUNTDOWN pyro channel
    uint32_t t0_ms;                // ARIADNA COUNTDOWN: absolute station millis() of T-0
    uint8_t  sent;                 // copies sent in the current window
    uint8_t  tries;                // windows used
    uint32_t win_ms;               // window the copies went into
    uint32_t queued_ms;            // when it was queued
};
static Pending s_ul = {};
static uint8_t s_seq = 0;

// requests from core 1
static volatile bool s_ready = false;
static volatile bool s_abort_btn = false;
static volatile bool s_mode_btn = false;

// snapshot for the display core
struct Disp {
    uint8_t  mode;
    bool     lora_ok, test, pend, utc;
    uint8_t  pcmd;
    int      rssi; float snr;
    uint32_t rx_ok;
    int32_t  win_in;
    zl_tlm_t zt; zl_rampa_beacon_t zb; uint32_t zt_age, zb_age;
    sl_tlm_t st; sl_res_t sr; uint32_t st_age, sr_age; uint8_t spcmd; bool spend;
    ariadna_tlm_t at; ariadna_lora_sync_t as; uint32_t at_age, as_age, at_dt;
    uint32_t mode_changed_ms;
};
static critical_section_t s_cs;
static Disp s_disp = {};
static uint32_t s_mode_changed_ms = 0;

// ---------------------------------------------------------------- USB out
static void out(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
static void out(const char *fmt, ...)
{
    char b[320];
    va_list ap; va_start(ap, fmt); vsnprintf(b, sizeof b, fmt, ap); va_end(ap);
    if (Serial && Serial.availableForWrite() > (int)strlen(b) + 2) { Serial.print(b); Serial.print("\r\n"); }
}

static void out_frame(uint8_t type, const void *payload, uint8_t len)
{
    uint8_t f[ARIADNA_FRAME_MAX];
    const uint16_t n = ariadna_encode(f, type, payload, len);
    if (n && Serial && Serial.availableForWrite() >= (int)n) Serial.write(f, n);
}

static void emit_ztlm(const zl_tlm_t &t)
{
    out("$T,%d,%.1f,%u,%u,%lu,%.7f,%.7f,%.2f,%ld,%.1f,%.1f,%.2f,%u,%.2f,%u,%u,%.1f,%.2f,%u,%u,%u,%u,%u,%u,%u,%u,%u",
        s_rssi, (double)s_snr, t.state, t.flags, (unsigned long)t.t_ms, t.lat_1e7 * 1e-7, t.lon_1e7 * 1e-7,
        t.alt_cm * 0.01, (long)t.t_rem_ms, t.vz_dms * 0.1, t.vh_dms * 0.1, t.acc_cg * 0.01, t.max_alt_m,
        t.vbat_mv * 0.001, t.next_rx_in_ms, t.main_alt_m, t.tilt_dd * 0.1, t.mach_x100 * 0.01, t.sats, t.pyro,
        t.health, t.rampa_state, t.rampa_flags, t.last_ul_seq, t.last_ul_rc, t.err, t.win_final);
}
static void emit_zbcn(const zl_rampa_beacon_t &b)
{
    out("$B,%d,%.1f,%u,%u,%lu,%.2f,%u,%u,%u,%u,%u,%u,%u,%u,%u",
        s_rssi, (double)s_snr, b.state, b.flags, (unsigned long)b.t_ms, b.vbat_mv * 0.001, b.umb_ma,
        b.cont_mv[0], b.cont_mv[1], b.cont_mv[2], b.next_rx_in_ms, b.faults, b.last_ul_seq, b.last_ul_rc, b.fw);
}
static void emit_atlm(const ariadna_tlm_t &t)
{
    // Raw integers where the GUI wants the exact value (quaternion, t_event
    // with its -32768 sentinel, bit fields), scaled where it is a plain unit.
    out("$AT,%d,%.1f,%lu,%.7f,%.7f,%.1f,%.1f,%.1f,%.2f,%d,%d,%d,%.2f,%.2f,%d,%d,%u,%u,%u,%u,%.2f,%.1f,%.1f,%u,%u,%u,%u,%u,%u,%u,%d,%d",
        s_rssi, (double)s_snr, (unsigned long)t.t_ms, t.lat * 1e-7, t.lon * 1e-7,
        t.alt_dm * 0.1, t.vz_dms * 0.1, t.vh_dms * 0.1, t.acc_cg * 0.01,
        t.qx, t.qy, t.qz, t.tvc_x_cdeg * 0.01, t.tvc_y_cdeg * 0.01, t.thrust_n, t.t_event_ds,
        t.version, t.state, t.nav_phase, t.clamp_pct, t.vbat_cv / 50.0 + 5.0, t.curr_da * 0.1, t.hdop_x10 * 0.1,
        t.sats, t.pyro, t.health, t.flags, t.events, t.uwb, t.t_flight_s, t.px_m, t.py_m);
    if (s_legacy) {
        ariadna_gs_serial_t g; g.tlm = t; g.rssi = (int8_t)s_rssi; g.snr = (int8_t)s_snr;
        out_frame(ARIADNA_PKT_TELEMETRY, &g, sizeof g);
    }
}
static void emit_async(const ariadna_lora_sync_t &s)
{
    out("$AS,%d,%.1f,%lu,%ld,%u,%u,%ld,%u,%u,%u,%u,%u,%u,%u",
        s_rssi, (double)s_snr, (unsigned long)s.rocket_t_ms,
        (long)(s.utc_ms == ARIADNA_UTC_UNKNOWN ? -1 : (int32_t)s.utc_ms), s.next_rx_in_ms, s.rx_dur_ms,
        (long)s.t_launch_rem_ms, s.last_cmd_seq, s.last_cmd_rc, s.rampa_state, s.rampa_flags, s.rampa_fault,
        s.sync_flags, s.rampa_vbat_mv);
    if (s_legacy) {
        ariadna_gs_sync_t g; g.sync = s; g.rssi = (int8_t)s_rssi; g.snr = (int8_t)s_snr;
        out_frame(ARIADNA_PKT_GS_SYNC, &g, sizeof g);
    }
}
static void fmt_t(char *b, size_t n, int16_t dc) { if (dc == SL_T_NONE) snprintf(b, n, "nan"); else snprintf(b, n, "%.1f", dc * 0.1); }
static void emit_stlm(const sl_tlm_t &t)
{
    char f[SL_SUBS * 12 + 4]; size_t k = 0;
    for (int i = 0; i < SL_SUBS; i++) k += (size_t)snprintf(f + k, sizeof f - k, "%.2f,", (double)sl_funpack(t.f[i], t.fexp));
    if (k) f[k - 1] = 0;
    char t1[12], t2[12], p[12], pp[12];
    fmt_t(t1, sizeof t1, t.t1_dc); fmt_t(t2, sizeof t2, t.t2_dc); fmt_t(p, sizeof p, t.p_dpsi); fmt_t(pp, sizeof pp, t.ppeak_dpsi);
    out("$ST,%d,%.1f,%u,%u,%u,%u,%u,%.2f,%.2f,%ld,%s,%.2f,%s,%s,%s,%s,%u,%u,%u,%u,%u,%u,%u",
        s_rssi, (double)s_snr, t.state, t.faults, t.flags, t.ack_seq, t.ack_rc, t.vbat_mv * 0.001, t.v5_mv * 0.001,
        (long)t.t_rel_ms, f, (double)sl_funpack(t.fpeak, t.fexp), p, pp, t1, t2, t.test_no, t.flash_free_pct,
        t.progress, t.sps, t.pyro & 0x03, (t.pyro >> 4) & 0x07, (t.pyro >> 2) & 0x03);   // servo: bit0 configured, bit1 armed
}
static void emit_sres(const sl_res_t &r)
{
    out("$SR,%u,%u,%u,%.3f,%.4f,%.3f,%.3f,%.3f,%.3f,%.2f,%.1f,%.1f,%u,%u,%lu,%u,%d",
        r.test_no, r.flags, r.pyro_ch, (double)r.peak_n, (double)r.impulse_ns, (double)r.burn_s, (double)r.avg_n,
        (double)r.t_peak_s, (double)r.ign_delay_s, (double)r.pmax_psi, r.t1max_dc * 0.1, r.t2max_dc * 0.1,
        r.curve_n, r.curve_hz, (unsigned long)r.n_frames, r.sps, r.t0_idx);
}
static void emit_scrv(const sl_curve_t &c)
{
    char b[300]; size_t k = 0;
    k += (size_t)snprintf(b + k, sizeof b - k, "$SC,%u,%u,%u", c.test_no, c.offset, c.n);
    for (int i = 0; i < c.n && i < SL_CRV_PTS; i++) k += (size_t)snprintf(b + k, sizeof b - k, ",%.2f", (double)sl_funpack(c.f[i], c.fexp));
    for (int i = 0; i < c.n && i < SL_CRV_PTS; i++) k += (size_t)snprintf(b + k, sizeof b - k, ",%.1f", c.p[i] * 0.1);
    out("%s", b);
}
static void emit_sburn(const sl_burn_t &b)
{
    char t1[12], t2[12];
    fmt_t(t1, sizeof t1, b.t1_dc); fmt_t(t2, sizeof t2, b.t2_dc);
    char o[700]; size_t k = 0;
    k += (size_t)snprintf(o + k, sizeof o - k, "$SB,%u,%u,%u,%u,%u,%u,%ld,%ld,%ld,%s,%s", b.test_no, b.idx, b.n_blocks, b.n,
                          b.rate_hz, b.flags, (long)b.t_first_ms, (long)b.burn_a_ms, (long)b.burn_b_ms, t1, t2);
    for (int i = 0; i < b.n && i < SL_BURN_PTS; i++) k += (size_t)snprintf(o + k, sizeof o - k, ",%.2f", (double)sl_funpack(b.f[i], b.fexp));
    for (int i = 0; i < b.n && i < SL_BURN_PTS; i++) k += (size_t)snprintf(o + k, sizeof o - k, ",%.1f", b.p[i] * 0.1);
    if (Serial && Serial.availableForWrite() > (int)k + 2) { Serial.print(o); Serial.print("\r\n"); }
}
static void sul_status(const char *what, uint8_t rc = 0)
{
    if (s_sul.cmd == SL_C_PING) return;
    out("$U,%u,%s,%u,%s,%u,%u", s_sul.seq, s_sul.cmd < SL_C_COUNT ? SL_CMD_NAME[s_sul.cmd] : "?", UL_DEST_STAND, what, rc, s_sul.sent);
}
static void ul_status(const char *what, uint8_t rc = 0)
{
    out("$U,%u,%s,%u,%s,%u,%u", s_ul.seq, s_ul.cmd < ZL_UL_COUNT ? CMD_NAME[s_ul.cmd] : "?", s_ul.dest, what, rc, s_ul.tries);
}
static void emit_mode() { out("$MODE,%s", MODE_NAME[s_mode]); }

// ---------------------------------------------------------------- radio
static void radio_config(void)
{
    if (!s_lora_ok) return;
    LoRa.idle();
    LoRa.setSpreadingFactor(ZL_LORA_SF);
    LoRa.setSignalBandwidth(ZL_LORA_BW_HZ);
    LoRa.setCodingRate4(ZL_LORA_CR4);
    LoRa.setPreambleLength(ZL_LORA_PREAMBLE);
    LoRa.setSyncWord(s_mode == MODE_ZENITH ? ZL_LORA_SYNCWORD : s_mode == MODE_STAND ? SL_LORA_SYNCWORD : ARIADNA_LORA_SYNCWORD);
    LoRa.setTxPower(s_mode == MODE_ZENITH ? ZL_LORA_TX_DBM : s_mode == MODE_STAND ? SL_LORA_TX_DBM : ARIADNA_LORA_TX_DBM);
    LoRa.enableCrc();
    LoRa.receive();
}

static void reset_link_state(void)
{
    if (s_ul.active) ul_status("CANCELLED");
    if (s_sul.active) sul_status("CANCELLED");
    s_ul = {}; s_sul = {};
    s_stlm = {}; s_sres = {}; s_stlm_ms = s_sres_ms = s_srx_ms = 0; s_stx_due = false;
    s_win_valid = false; s_win_final = false;
    s_ztlm = {}; s_zbcn = {}; s_ztlm_ms = s_zbcn_ms = 0;
    s_atlm = {}; s_async = {}; s_atlm_ms = s_async_ms = 0; s_atlm_dt = 0;
    s_rx_ok = s_rx_bad = 0;
}

static void set_mode(uint8_t m, bool persist)
{
    if (m >= MODE_COUNT) return;
    const bool changed = (m != s_mode);
    if (changed) {
        reset_link_state();
        s_mode = m;
        s_test = false;
        radio_config();
        s_mode_changed_ms = millis();
    }
    if (persist && (changed || EEPROM.read(1) != m || EEPROM.read(0) != EE_MAGIC)) {
        EEPROM.write(0, EE_MAGIC); EEPROM.write(1, m); EEPROM.write(2, (uint8_t)~m);
        EEPROM.commit();
    }
    emit_mode();
}

static uint32_t utc_now(uint32_t now)
{
    return s_utc_valid ? (s_utc_base + (uint32_t)(now - s_utc_ref)) % 86400000u : ARIADNA_UTC_UNKNOWN;
}

/** Lock to the sender's next RX window: the packet started `airtime` before it ended. */
static void lock_window(uint32_t rx_end_ms, uint16_t pkt_len, uint16_t next_rx_in_ms, bool final, uint8_t dest)
{
    if (next_rx_in_ms == ZL_NEXT_RX_NONE) return;
    const uint32_t tx_start = rx_end_ms - zl_lora_airtime_us(pkt_len) / 1000u;   // same modulation for both
    s_win_open_ms = tx_start + next_rx_in_ms;
    s_win_heard_ms = rx_end_ms;
    s_win_valid = true;
    s_win_final = final;
    s_win_dest = dest;
}

// ---------------------------------------------------------------- uplink
static bool critical_cmd(uint8_t c) { return c == ZL_UL_ABORT || c == ZL_UL_COUNTDOWN; }

static void queue_cmd(uint8_t cmd, int32_t arg, uint8_t dest, uint32_t now)
{
    s_seq = (uint8_t)(s_seq + 1); if (!s_seq) s_seq = 1;
    s_ul = {};
    s_ul.active = true; s_ul.seq = s_seq; s_ul.cmd = cmd; s_ul.arg = arg; s_ul.dest = dest;
    s_ul.queued_ms = now;
    if (s_mode == MODE_ARIADNA && cmd == ZL_UL_COUNTDOWN) {
        // ARIADNA: T-0 is fixed now; every copy carries the time left to it.
        s_ul.t0_ms = now + (uint32_t)arg * 1000u;
        s_ul.arg = 1;                                    // pyro channel, set by the caller
    }
    ul_status("QUEUED");
}

static bool send_uplink(uint32_t now, bool retry)
{
    if (s_mode == MODE_ZENITH) {
        zl_uplink_t u = {};
        u.seq = s_ul.seq; u.cmd = s_ul.cmd; u.arg = s_ul.arg; u.dest = s_ul.dest;
        u.token = zl_ul_token_for(s_ul.cmd);
        u.flags = (uint8_t)(retry ? ZL_ULF_RETRY : 0);
        zl_lora_seal(&u, sizeof u, ZL_UL_MAGIC);
        if (LoRa.beginPacket()) { LoRa.write((const uint8_t *)&u, sizeof u); LoRa.endPacket(false); }
    } else {
        const int c = ariadna_cmd(s_ul.cmd);
        if (c < 0) return false;
        ariadna_lora_uplink_t u;
        memset(&u, 0, sizeof u);
        u.seq = s_ul.seq; u.cmd = (uint8_t)c;
        u.utc_ms = utc_now(now);
        u.token = ariadna_token((uint8_t)c);
        u.target = (uint8_t)s_ul.arg;
        u.flags = (uint8_t)((s_win_final ? ARIADNA_ULF_FINAL_WIN : 0u) | (retry ? ARIADNA_ULF_RETRY : 0u));
        if (c == ARIADNA_UL_COUNTDOWN) {
            const int32_t rem = (int32_t)(s_ul.t0_ms - now);
            if (rem < (int32_t)(ARIADNA_CD_MIN_S * 1000)) return false;   // too late for this T-0
            u.arg_ms = (uint32_t)rem;
        }
        ariadna_lora_uplink_seal(&u);
        if (LoRa.beginPacket()) { LoRa.write((const uint8_t *)&u, sizeof u); LoRa.endPacket(false); }
    }
    LoRa.receive();
    return true;
}

static void uplink_service(uint32_t now)
{
    if (!s_ul.active || !s_lora_ok) return;
    // Give up 2 s after the third window closed without an ACK (the rocket
    // answers in its first packet after the window), or when no window was
    // ever usable (rocket silent) for 45 s. Never leave a command hanging.
    if (s_ul.tries >= 3 && (int32_t)(now - (s_ul.win_ms + ZL_WIN_DUR_MS + 2000u)) > 0) {
        ul_status("FAILED"); s_ul.active = false; return;
    }
    if (!s_ul.tries && (uint32_t)(now - s_ul.queued_ms) > 45000u) {
        ul_status("FAILED"); s_ul.active = false; return;
    }
    const bool z = (s_mode == MODE_ZENITH);
    const uint16_t off1 = z ? ZL_UL_OFFSET1_MS : ARIADNA_LORA_UL_OFFSET1_MS;
    const uint16_t off2 = z ? ZL_UL_OFFSET2_MS : ARIADNA_LORA_UL_OFFSET2_MS;

    bool have = false;
    uint32_t open = 0;
    if (s_win_valid && (uint32_t)(now - s_win_heard_ms) < 25000u && (!z || s_win_dest == s_ul.dest)) {
        // predict the window forward on the 10 s grid if the lock is old;
        // a passed final (T-1 s) window is not on the grid: wait for a packet
        while ((int32_t)(now - (s_win_open_ms + ZL_WIN_DUR_MS)) > 0) {
            if (s_win_final) { s_win_valid = false; break; }
            s_win_open_ms += ZL_WIN_PERIOD_MS;
        }
        if (s_win_valid) { have = true; open = s_win_open_ms; }
    }
    if (!have && !z && s_utc_valid) {
        // ARIADNA fallback: the rocket's windows sit on UTC 10 s boundaries
        const uint32_t rem = utc_now(now) % ZL_WIN_PERIOD_MS;
        if (rem < ZL_WIN_DUR_MS) { have = true; open = now - rem; }
    }
    if (!have) return;

    const int32_t dt = (int32_t)(now - open);
    if (dt < 0) return;
    if (!s_ul.win_ms || (uint32_t)(open - s_ul.win_ms + 1000u) > 2000u) {   // a new window (not a +-1 s re-lock)
        s_ul.win_ms = open; s_ul.sent = 0; s_ul.tries++;
    }
    if (s_ul.sent == 0 && dt >= (int32_t)off1 && dt < (int32_t)(ZL_WIN_DUR_MS - 40)) {
        if (!send_uplink(now, false)) { ul_status("FAILED"); s_ul.active = false; return; }
        s_ul.sent = 1; ul_status("SENT");
    } else if (s_ul.sent == 1 && critical_cmd(s_ul.cmd) && dt >= (int32_t)off2 && dt < (int32_t)(ZL_WIN_DUR_MS - 35)) {
        if (send_uplink(now, true)) { s_ul.sent = 2; ul_status("SENT2"); }
    }
}

static void check_ack(uint8_t seq, uint8_t rc, uint8_t dest)
{
    if (s_ul.active && s_ul.dest == dest && s_ul.sent && seq == s_ul.seq && rc != ZL_RC_NONE) {
        ul_status("ACK", rc);
        s_ul.active = false;
    }
}

// ---------------------------------------------------------------- STAND uplink
static void stand_queue(uint8_t cmd, int32_t arg, uint16_t arg2)
{
    static uint8_t sseq = 0;                          // own counter: the rampa's dedup is separate
    sseq = (uint8_t)(sseq + 1); if (!sseq) sseq = 1;
    s_sul = {};
    s_sul.active = true; s_sul.seq = sseq; s_sul.cmd = cmd; s_sul.arg = arg; s_sul.arg2 = arg2; s_sul.queued_ms = millis();
    sul_status("QUEUED");
}

static void stand_tx(uint32_t now)
{
    sl_cmd_pkt_t c = {};
    c.seq = s_sul.seq; c.cmd = s_sul.cmd; c.arg = s_sul.arg; c.arg2 = s_sul.arg2; c.token = sl_token_for(s_sul.cmd);
    sl_seal(&c, sizeof c, SL_CMD_MAGIC);
    if (LoRa.beginPacket()) { LoRa.write((const uint8_t *)&c, sizeof c); LoRa.endPacket(false); }
    LoRa.receive();
    s_stx_ms = now;
    if (s_sul.sent < 255) s_sul.sent++;
}

/**
 * STAND: no windows. The rampa listens whenever it is not transmitting, so a
 * command goes out right after one of its packets (s_stx_due), or blind when
 * it has not been heard for a while. Every command (also PING) is the
 * heartbeat that keeps a countdown alive on the rampa.
 */
static void stand_uplink_service(uint32_t now)
{
    if (!s_lora_ok) return;
    const bool heard = s_srx_ms && (uint32_t)(now - s_srx_ms) < 1500u;
    if (s_sul.active) {
        const bool abort = s_sul.cmd == SL_C_ABORT;
        const uint32_t age = now - s_sul.queued_ms;
        if (s_sul.sent >= (abort ? 40 : 12) || age > (abort ? 15000u : 8000u)) {
            sul_status("FAILED"); s_sul.active = false; return;
        }
    }
    bool go = false;
    if (s_stx_due && (uint32_t)(now - s_srx_ms) >= 6u) {           // right after the rampa's packet
        s_stx_due = false;
        go = !s_stx_burst || s_sul.active;                          // inside a burst: no heartbeat
        if (!go) return;
    } else if (!heard) {
        const uint32_t gap = (s_sul.active && s_sul.cmd == SL_C_ABORT) ? 300u : 700u;
        go = (uint32_t)(now - s_stx_ms) >= gap;                     // blind: wakes the rampa / reaches it anyway
    }
    if (!go) return;
    if (!s_sul.active) {
        if ((uint32_t)(now - s_sping_ms) < 380u) return;            // heartbeat ~2.5 Hz
        s_sping_ms = now;
        stand_queue(SL_C_PING, 0, 0);
    }
    stand_tx(now);
    if (s_sul.cmd == SL_C_PING) s_sul.active = false;               // a ping needs no answer
    else sul_status(s_sul.sent > 1 ? "SENT2" : "SENT");
}

static void stand_ack(uint8_t seq, uint8_t rc)
{
    if (s_sul.active && s_sul.sent && seq == s_sul.seq && rc != SL_RC_NONE) {
        sul_status("ACK", rc);
        s_sul.active = false;
    }
}

static int stand_cmd_index(const char *n)
{
    for (int i = 0; i < SL_C_COUNT; i++) if (!strcasecmp(n, SL_CMD_NAME[i])) return i;
    if (!strcasecmp(n, "NOP")) return SL_C_PING;
    if (!strcasecmp(n, "SAFE")) return SL_C_DISARM;
    if (!strcasecmp(n, "COUNTDOWN") || !strcasecmp(n, "CD")) return SL_C_FIRE;
    if (!strcasecmp(n, "REC")) return SL_C_RECORD;
    if (!strcasecmp(n, "LOCATE")) return SL_C_BUZZER;
    return -1;
}

/** CMD <name> [a2] [a3] in MODE STAND. */
static void stand_handle_cmd(const char *name, const char *a2, const char *a3, uint32_t now)
{
    const int c = stand_cmd_index(name);
    if (c < 0) { out("#ERR unknown STAND command %s (PING ARM DISARM FIRE ABORT TARE RECORD STOP RESULT CURVE BURN BUZZER RECTIME CAL PZERO)", name); return; }
    if (s_sul.active && s_sul.cmd == SL_C_ABORT && c != SL_C_ABORT) { out("#ERR abort pending"); return; }
    int32_t arg = a2 ? (int32_t)atol(a2) : 0;
    uint16_t arg2 = 0;
    switch (c) {
    case SL_C_ARM:
        if (arg < 1 || arg > 3) { out("#ERR ARM <igniter 1-3> [pulse ms %d-%d]", SL_FIRE_MS_MIN, SL_FIRE_MS_MAX); return; }
        if (a3) { const long ms = atol(a3); if (ms < SL_FIRE_MS_MIN || ms > SL_FIRE_MS_MAX) { out("#ERR pulse %d-%d ms", SL_FIRE_MS_MIN, SL_FIRE_MS_MAX); return; } arg2 = (uint16_t)ms; }
        break;
    case SL_C_FIRE:
        if (arg < SL_CD_MIN_S || arg > SL_CD_MAX_S) { out("#ERR FIRE <countdown %d-%d s>", SL_CD_MIN_S, SL_CD_MAX_S); return; }
        break;
    case SL_C_RECORD:
        if (arg < 1 || arg > SL_REC_MAX_S) { out("#ERR RECORD <1-%d s>", SL_REC_MAX_S); return; }
        break;
    case SL_C_RECTIME:
        if (arg < SL_REC_MIN_S || arg > SL_REC_MAX_S) { out("#ERR RECTIME <%d-%d s>", SL_REC_MIN_S, SL_REC_MAX_S); return; }
        break;
    case SL_C_CAL: {
        const double n = a2 ? atof(a2) : 0;
        if (n <= 0 || n > 2000000) { out("#ERR CAL <known force N>"); return; }
        arg = (int32_t)(n * 1000.0 + 0.5);
        break; }
    case SL_C_CURVE:
        if (arg < 0) arg = 0;
        break;
    case SL_C_BURN: {                                               // BURN [first block] [blocks, 0 = all]
        if (arg < 0 || arg > 254) { out("#ERR BURN [first block 0-254] [count]"); return; }
        const long n = a3 ? atol(a3) : 0;
        arg2 = (uint16_t)(n < 0 ? 0 : n > 255 ? 255 : n);
        break; }
    default: break;
    }
    stand_queue((uint8_t)c, arg, arg2);
    out("#OK queued %s seq %u", SL_CMD_NAME[c], s_sul.seq);
    (void)now;
}

// ---------------------------------------------------------------- serial commands
static int cmd_index(const char *n)
{
    for (int i = 0; i < ZL_UL_COUNT; i++) if (!strcasecmp(n, CMD_NAME[i])) return i;
    if (!strcasecmp(n, "PREP") || !strcasecmp(n, "LAUNCHSOON")) return ZL_UL_PREPARE;
    if (!strcasecmp(n, "DISARM")) return ZL_UL_SAFE;
    if (!strcasecmp(n, "PING")) return ZL_UL_NOP;
    if (!strcasecmp(n, "CD")) return ZL_UL_COUNTDOWN;
    return -1;
}

static int parse_mode(const char *s)
{
    if (!strcasecmp(s, "ZENITH") || !strcasecmp(s, "SAPPHIRE") || !strcasecmp(s, "Z")) return MODE_ZENITH;
    if (!strcasecmp(s, "ARIADNA") || !strcasecmp(s, "DAIDALOS") || !strcasecmp(s, "A")) return MODE_ARIADNA;
    if (!strcasecmp(s, "STAND") || !strcasecmp(s, "MOTOR") || !strcasecmp(s, "THRUST") || !strcasecmp(s, "TRUST") ||
        !strcasecmp(s, "TAH") || !strcasecmp(s, "S")) return MODE_STAND;
    return -1;
}

/** CMD <name> [a2] [a3] for the current rocket. */
static void handle_cmd(const char *name, const char *a2, const char *a3, uint32_t now)
{
    if (s_mode == MODE_STAND) { stand_handle_cmd(name, a2, a3, now); return; }
    const int c = cmd_index(name);
    if (c < 0) { out("#ERR unknown command %s", name); return; }
    if (s_ul.active && s_ul.cmd == ZL_UL_ABORT && c != ZL_UL_ABORT) { out("#ERR abort pending"); return; }
    int32_t arg = (a2 && strcasecmp(a2, "RAMPA")) ? (int32_t)atol(a2) : 0;

    if (s_mode == MODE_ZENITH) {
        if (c == ZL_UL_COUNTDOWN) {
            if (arg < (int32_t)(ZL_CD_MIN_MS / 1000) || arg > (int32_t)(ZL_CD_MAX_MS / 1000)) {
                out("#ERR countdown %d..%d s", (int)(ZL_CD_MIN_MS / 1000), (int)(ZL_CD_MAX_MS / 1000)); return;
            }
            arg *= 1000;   // ms from the moment the rocket hears it
        }
        uint8_t dest = (c == ZL_UL_ROCKET_PWR) ? ZL_UL_DEST_RAMPA : ZL_UL_DEST_ROCKET;
        if ((a2 && !strcasecmp(a2, "RAMPA")) || (a3 && !strcasecmp(a3, "RAMPA"))) dest = ZL_UL_DEST_RAMPA;
        queue_cmd((uint8_t)c, arg, dest, now);
    } else {
        if (ariadna_cmd((uint8_t)c) < 0) { out("#ERR %s: Ariadna does not have this command", CMD_NAME[c]); return; }
        if (c == ZL_UL_COUNTDOWN) {
            if (arg < ARIADNA_CD_MIN_S || arg > ARIADNA_CD_MAX_S) { out("#ERR countdown %d..%d s", ARIADNA_CD_MIN_S, ARIADNA_CD_MAX_S); return; }
            const int ch = a3 ? atoi(a3) : 1;
            queue_cmd((uint8_t)c, arg, ZL_UL_DEST_ROCKET, now);
            s_ul.arg = (ch >= 1 && ch <= 3) ? ch : 1;      // rampa igniter channel
        } else {
            if (c == ZL_UL_REBOOT && (arg < 0 || arg > 2)) arg = 0;   // 0 all, 1 position, 2 navigation
            queue_cmd((uint8_t)c, c == ZL_UL_REBOOT ? arg : 0, ZL_UL_DEST_ROCKET, now);
        }
    }
    out("#OK queued %s seq %u", CMD_NAME[c], s_ul.seq);
}

/** Commands of the old Daidalos ground_visualizer.py ("T:1", "CMD:CD:30:1" ...). */
static bool legacy_line(char *l, uint32_t now)
{
    const bool legacy = !strncmp(l, "T:", 2) || !strncmp(l, "UTC:", 4) || !strncmp(l, "CMD:", 4);
    if (!legacy) return false;
    if (s_mode != MODE_ARIADNA) set_mode(MODE_ARIADNA, true);
    s_legacy = true;
    if (!strcmp(l, "T:1")) { s_test = true; return true; }
    if (!strcmp(l, "T:0")) { s_test = false; return true; }
    if (!strncmp(l, "UTC:", 4)) {
        const uint32_t u = (uint32_t)strtoul(l + 4, nullptr, 10);
        if (u < 86400000u) { s_utc_base = u; s_utc_ref = now; s_utc_valid = true; }
        return true;
    }
    // CMD:NAME[:a[:b]]
    char *p = l + 4;
    char *f[3] = { p, nullptr, nullptr };
    for (int i = 1; i < 3; i++) { char *c = f[i - 1] ? strchr(f[i - 1], ':') : nullptr; if (c) { *c = 0; f[i] = c + 1; } }
    handle_cmd(f[0], f[1], f[2], now);
    return true;
}

static void serial_line(char *l, uint32_t now)
{
    if (legacy_line(l, now)) return;
    char *a0 = strtok(l, " "), *a1 = strtok(nullptr, " "), *a2 = strtok(nullptr, " "), *a3 = strtok(nullptr, " ");
    if (!a0) return;
    if (!strcasecmp(a0, "HELLO")) {
        out("$ID,Ground Station,%u.%02u,%s", GS_FW_VERSION / 100, GS_FW_VERSION % 100, MODE_NAME[s_mode]);
        emit_mode();
        if (!s_lora_ok) out("#ERR LoRa init failed");
        return;
    }
    if (!strcasecmp(a0, "MODE")) {
        if (!a1) { emit_mode(); return; }
        const int m = parse_mode(a1);
        if (m < 0) { out("#ERR MODE ZENITH|ARIADNA|STAND"); return; }
        set_mode((uint8_t)m, true);
        out("#OK MODE %s", MODE_NAME[s_mode]);
        return;
    }
    if (!strcasecmp(a0, "TEST")) { s_test = a1 && atoi(a1); out("#OK TEST %d", s_test); return; }
    if (!strcasecmp(a0, "LEGACY")) { s_legacy = a1 && atoi(a1); out("#OK LEGACY %d", s_legacy); return; }
    if (!strcasecmp(a0, "UTC") && a1) {
        const uint32_t u = (uint32_t)strtoul(a1, nullptr, 10);
        if (u < 86400000u) { s_utc_base = u; s_utc_ref = now; s_utc_valid = true; }
        return;
    }
    if (!strcasecmp(a0, "CANCEL")) {
        if (s_ul.active) ul_status("CANCELLED");
        if (s_sul.active) sul_status("CANCELLED");
        s_ul.active = false; s_sul.active = false; out("#OK CANCEL"); return;
    }
    if (!strcasecmp(a0, "CMD") && a1) { handle_cmd(a1, a2, a3, now); return; }
    out("#ERR HELLO | MODE [ZENITH|ARIADNA|STAND] | CMD <name> [arg] [arg2|RAMPA] | CANCEL | TEST 0|1 | UTC <ms> | LEGACY 0|1");
}

// ---------------------------------------------------------------- test generators
static void test_zenith(uint32_t now)
{
    static uint32_t next = 0;
    if ((int32_t)(now - next) < 0) return;
    next = now + 200;
    const float t = fmodf(now * 0.001f, 120.0f);
    zl_tlm_t p = {};
    float h = 0, vz = 0; uint8_t st = ZS_READY;
    if (t < 20) { st = ZS_COUNTDOWN; p.t_rem_ms = (int32_t)((20 - t) * 1000); }
    else if (t < 22.5f) { st = ZS_BOOST; vz = (t - 20) * 180; h = 90 * (t - 20) * (t - 20); }
    else if (t < 42) { st = ZS_COAST; const float c = t - 22.5f; vz = 450 - 23 * c; h = 560 + 450 * c - 11.5f * c * c; }
    else if (t < 110) { st = t < 100 ? ZS_DROGUE : ZS_MAIN; const float c = t - 42; vz = st == ZS_DROGUE ? -25 : -6; h = 4960 - 25 * c; if (h < 0) h = 0; }
    else st = ZS_LANDED;
    p.state = st; p.alt_cm = (int32_t)(h * 100); p.vz_dms = (int16_t)(vz * 10); p.max_alt_m = (uint16_t)(h > 0 ? h : 0);
    p.lat_1e7 = 497985411 + (int32_t)(t * 30); p.lon_1e7 = 166912772 + (int32_t)(t * 50);
    p.vbat_mv = 8100; p.sats = 12; p.pyro = 0x47; p.health = 0x3F; p.flags = 0xC3; p.main_alt_m = 300;
    p.mach_x100 = (uint8_t)(fabsf(vz) / 3.4f); p.acc_cg = (int16_t)(st == ZS_BOOST ? 2000 : -30);
    p.rampa_state = st <= ZS_COUNTDOWN ? ZR_ST_COUNTDOWN : ZR_ST_OFFLINE;
    p.next_rx_in_ms = (uint16_t)(10000 - (now % 10000));
    p.t_ms = now;
    s_ztlm = p; s_ztlm_ms = now; s_rssi = -70; s_snr = 9.5f;
    emit_ztlm(p);
}

static void test_ariadna(uint32_t now)
{
    static uint32_t next = 0, next_sync = 0;
    if ((int32_t)(now - next) < 0) return;
    next = now + 1000 / ARIADNA_TLM_HZ;
    const float m = now * 0.001f;
    const float cyc = fmodf(m, 40.0f);
    ariadna_tlm_t p;
    memset(&p, 0, sizeof p);
    p.version = ARIADNA_TLM_VERSION; p.t_ms = now;
    float h, vz, t_event = -1.0f; uint8_t state, phase, events = 0;
    if (cyc < 4.0f)        { state = ARIADNA_STATE_ARMED;    h = 0;   vz = 0;   phase = 0; }
    else if (cyc < 9.0f)   { state = ARIADNA_STATE_ASCENT;   h = (cyc - 4) * 32; vz = 32; phase = 1; }
    else if (cyc < 10.0f)  { state = ARIADNA_STATE_APOGEE;   h = 160; vz = 0;   phase = 1; }
    else if (cyc < 14.0f)  { state = ARIADNA_STATE_DESCENT;  h = 160 - (cyc - 10) * 26; vz = -26; phase = 2;
                             t_event = 14 - cyc; events = 0x01 | (t_event < 0.6f ? 0x02 : 0); }
    else if (cyc < 16.6f)  { state = ARIADNA_STATE_GUIDANCE; h = 56 * (1 - (cyc - 14) / 2.6f); vz = -30 * (1 - (cyc - 14) / 2.7f); phase = 4;
                             t_event = 16.6f - cyc; events = 0x0F | (t_event < 1.7f ? 0x20 : 0) | (h < 7 ? 0x40 : 0); }
    else if (cyc < 22.0f)  { state = ARIADNA_STATE_LANDED;   h = 0; vz = 0; phase = 5; events = 0xEF; }
    else                   { state = ARIADNA_STATE_RECOVERY; h = 0; vz = 0; phase = 5; events = 0xEF; }
    p.state = state; p.nav_phase = phase;
    p.lat = (int32_t)((49.7985411 + sin(m * 0.1) * 0.0004) * 1e7);
    p.lon = (int32_t)((16.6912772 + cos(m * 0.1) * 0.0004) * 1e7);
    p.alt_dm = (int16_t)(h * 10); p.vz_dms = (int16_t)(vz * 10);
    p.vh_dms = (int16_t)((4 + sinf(m) * 3) * 10); p.acc_cg = (int16_t)((phase == 4 ? 4.0f : 1.0f) * 100);
    {
        const float tilt = (5 + 4 * sinf(m * 0.9f)) * 0.0174533f, az = m * 0.6f;
        const float ang = (phase >= 2) ? 3.14159265f - tilt : tilt;
        const float qw = cosf(0.5f * ang), sh = sinf(0.5f * ang);
        const float sg = qw < 0 ? -32767.0f : 32767.0f;
        p.qx = (int16_t)(sh * cosf(az) * sg); p.qy = (int16_t)(sh * sinf(az) * sg); p.qz = 0;
    }
    p.t_event_ds = t_event < 0 ? (int16_t)ARIADNA_TLM_T_UNKNOWN : (int16_t)(t_event * 10);
    p.events = events; p.t_flight_s = (uint8_t)(cyc > 4 ? cyc - 4 : 0);
    p.px_m = (int8_t)(sinf(m * 0.3f) * 6); p.py_m = (int8_t)(cosf(m * 0.2f) * 4);
    if (phase == 4) {
        p.tvc_x_cdeg = (int16_t)(sinf(m * 6) * 350); p.tvc_y_cdeg = (int16_t)(cosf(m * 5) * 250);
        p.clamp_pct = (uint8_t)(55 + (int)(sinf(m * 3) * 40)); p.thrust_n = (int16_t)(95 + (int)(sinf(m * 4) * 20));
    } else p.clamp_pct = 100;
    p.vbat_cv = (uint8_t)((8.10f - fmodf(m, 120) * 0.004f - 5.0f) * 50); p.curr_da = phase == 4 ? 55 : 12;
    p.hdop_x10 = (uint8_t)(9 + (int)(fabsf(sinf(m * 0.5f)) * 8)); p.sats = (uint8_t)(11 + (int)(sinf(m * 0.2f) * 3));
    const uint8_t cont = phase >= 4 ? 0x00 : 0x07, fired = phase >= 4 ? ((events & 0x20) ? 0x05 : 0x01) : 0x00;
    p.pyro = (uint8_t)((cont & 0x0F) | ((fired & 0x0F) << 4));
    p.health = 0xEF | (state >= ARIADNA_STATE_ARMED ? ARIADNA_HLT_LOGGING : 0);
    p.uwb = 0x80 | 0x40 | 0x30 | 4;
    p.flags = ARIADNA_TF_TEST;
    if (state >= ARIADNA_STATE_ARMED) p.flags |= ARIADNA_TF_ARMED | ARIADNA_TF_RBF_OUT;
    if (state >= ARIADNA_STATE_ASCENT) p.flags |= ARIADNA_TF_BREAKWIRE;
    s_rssi = -65 + (int)(sinf(m * 5) * 10); s_snr = 10 + cosf(m * 3) * 3;
    if (s_atlm_ms) s_atlm_dt = now - s_atlm_ms;
    s_atlm = p; s_atlm_ms = now;
    emit_atlm(p);
    if ((int32_t)(now - next_sync) >= 0 && cyc < 4.0f) {
        next_sync = now + 1000;
        ariadna_lora_sync_t s;
        memset(&s, 0, sizeof s);
        s.rocket_t_ms = now; s.utc_ms = utc_now(now); s.next_rx_in_ms = (uint16_t)(10000 - now % 10000);
        s.rx_dur_ms = ARIADNA_LORA_WIN_DUR_MS; s.t_launch_rem_ms = -1; s.last_cmd_seq = s_ul.seq; s.last_cmd_rc = 0;
        s.rampa_state = 2; s.rampa_flags = 0x83; s.rampa_vbat_mv = 12400; s.sync_flags = 0x20;
        ariadna_lora_sync_seal(&s);
        s_async = s; s_async_ms = now;
        emit_async(s);
    }
}

static void test_stand(uint32_t now)
{
    static uint32_t next = 0;
    if ((int32_t)(now - next) < 0) return;
    next = now + SL_TLM_FAST_MS;
    const float c = fmodf(now * 0.001f, 30.0f);
    sl_tlm_t t = {};
    t.state = c < 10 ? SL_ST_ARMED : c < 15 ? SL_ST_COUNTDOWN : c < 16 ? SL_ST_FIRING : c < 22 ? SL_ST_BURN : SL_ST_SAFE;
    t.flags = SL_F_KEY | SL_F_CONT | SL_F_ADS_OK | SL_F_FLASH_OK | SL_F_LC_CAL | SL_F_TH1 | SL_F_TH2 | SL_F_PRESS | SL_F_TARED;
    if (t.state == SL_ST_FIRING) t.flags |= SL_F_ARM_OUT | SL_F_FIRE_OUT;
    if (t.state >= SL_ST_COUNTDOWN && t.state <= SL_ST_BURN) t.flags |= SL_F_REC;
    t.vbat_mv = 8050; t.v5_mv = 4998;
    t.t_rel_ms = (t.state >= SL_ST_COUNTDOWN && t.state <= SL_ST_BURN) ? (int32_t)((c - 15.0f) * 1000) : 0;
    float fmax = 0, fv[SL_SUBS];
    for (int i = 0; i < SL_SUBS; i++) {
        const float tb = c - 15.0f - (SL_SUBS - 1 - i) * 0.02f;
        fv[i] = (tb > 0.2f && tb < 3.2f) ? 180.0f * (tb < 0.5f ? (tb - 0.2f) / 0.3f : 1.0f - (tb - 0.5f) * 0.15f) : 0.0f;
        if (fv[i] > fmax) fmax = fv[i];
    }
    t.fexp = sl_fexp_for(fmax);
    for (int i = 0; i < SL_SUBS; i++) t.f[i] = sl_fpack(fv[i], t.fexp);
    t.fpeak = sl_fpack(fmax, t.fexp);
    t.p_dpsi = (int16_t)(fv[SL_SUBS - 1] * 18.0f); t.ppeak_dpsi = t.p_dpsi;
    t.t1_dc = (int16_t)(215 + (c > 15 ? (c - 15) * 40 : 0)); t.t2_dc = 228;
    t.test_no = 7; t.flash_free_pct = 93; t.progress = 0; t.sps = 2000; t.pyro = 0x01 | 0x10;
    t.ack_seq = s_sul.seq; t.ack_rc = SL_RC_OK;
    s_rssi = -62; s_snr = 9.0f;
    s_stlm = t; s_stlm_ms = now;
    emit_stlm(t);
}

// ---------------------------------------------------------------- receive
static void receive(void)
{
    const int sz = LoRa.parsePacket();
    if (sz <= 0) return;
    uint8_t b[200]; int k = 0;                    // largest: sl_burn_t 188 B
    while (LoRa.available() && k < (int)sizeof b) b[k++] = (uint8_t)LoRa.read();
    while (LoRa.available()) LoRa.read();
    const uint32_t rx = millis();
    s_rssi = LoRa.packetRssi(); s_snr = LoRa.packetSnr();

    if (s_mode == MODE_STAND) {
        sl_curve_t cv;
        if (sl_check(b, (size_t)k, sizeof(sl_tlm_t), SL_TLM_MAGIC)) {
            memcpy(&s_stlm, b, sizeof s_stlm);
            s_stlm_ms = s_srx_ms = rx; s_rx_ok++; s_test = false; s_stx_due = true; s_stx_burst = false;
            stand_ack(s_stlm.ack_seq, s_stlm.ack_rc);
            emit_stlm(s_stlm);
        } else if (sl_check(b, (size_t)k, sizeof(sl_res_t), SL_RES_MAGIC)) {
            memcpy(&s_sres, b, sizeof s_sres);
            s_sres_ms = s_srx_ms = rx; s_rx_ok++; s_stx_due = true; s_stx_burst = false;
            emit_sres(s_sres);
        } else if (sl_check(b, (size_t)k, sizeof(sl_burn_t), SL_BURN_MAGIC)) {
            sl_burn_t bu;
            memcpy(&bu, b, sizeof bu);
            s_srx_ms = rx; s_rx_ok++; s_stx_due = true; s_stx_burst = true;
            if (s_sul.active && s_sul.cmd == SL_C_BURN && s_sul.sent && bu.idx == (uint8_t)s_sul.arg) stand_ack(s_sul.seq, SL_RC_OK);
            emit_sburn(bu);
            return;
        } else if (sl_check(b, (size_t)k, sizeof(sl_curve_t), SL_CRV_MAGIC)) {
            memcpy(&cv, b, sizeof cv);
            s_srx_ms = rx; s_rx_ok++; s_stx_due = true;
            if (s_sul.active && s_sul.cmd == SL_C_CURVE && s_sul.sent && cv.offset == (uint16_t)s_sul.arg) stand_ack(s_sul.seq, SL_RC_OK);
            emit_scrv(cv);
        } else if (zl_lora_check(b, (size_t)k, sizeof(zl_rampa_beacon_t), ZL_BEACON_MAGIC)) {
            // the rampa is still a launch pad: our blind PINGs will switch it over
            memcpy(&s_zbcn, b, sizeof s_zbcn);
            s_zbcn_ms = rx; s_rx_ok++;
            emit_zbcn(s_zbcn);
        } else {
            s_rx_bad++;
        }
        return;
    }

    if (s_mode == MODE_ZENITH) {
        if (zl_lora_check(b, (size_t)k, sizeof(zl_tlm_t), ZL_TLM_MAGIC)) {
            memcpy(&s_ztlm, b, sizeof s_ztlm);
            s_ztlm_ms = rx; s_rx_ok++; s_test = false;
            lock_window(rx, sizeof(zl_tlm_t), s_ztlm.next_rx_in_ms, s_ztlm.win_final != 0, ZL_UL_DEST_ROCKET);
            check_ack(s_ztlm.last_ul_seq, s_ztlm.last_ul_rc, ZL_UL_DEST_ROCKET);
            emit_ztlm(s_ztlm);
        } else if (zl_lora_check(b, (size_t)k, sizeof(zl_rampa_beacon_t), ZL_BEACON_MAGIC)) {
            memcpy(&s_zbcn, b, sizeof s_zbcn);
            s_zbcn_ms = rx; s_rx_ok++;
            // a rampa beacon means no rocket is talking: lock to the rampa
            if (!s_ztlm_ms || (uint32_t)(rx - s_ztlm_ms) > 5000u)
                lock_window(rx, sizeof(zl_rampa_beacon_t), s_zbcn.next_rx_in_ms, false, ZL_UL_DEST_RAMPA);
            check_ack(s_zbcn.last_ul_seq, s_zbcn.last_ul_rc, ZL_UL_DEST_RAMPA);
            emit_zbcn(s_zbcn);
        } else {
            s_rx_bad++;
        }
        return;
    }

    // ARIADNA
    if (k == (int)sizeof(ariadna_tlm_t) && b[34] == ARIADNA_TLM_VERSION) {   // .version at offset 34
        memcpy(&s_atlm, b, sizeof s_atlm);
        if (s_atlm_ms) s_atlm_dt = rx - s_atlm_ms;
        s_atlm_ms = rx; s_rx_ok++; s_test = false;
        emit_atlm(s_atlm);
    } else if (k == (int)sizeof(ariadna_lora_sync_t) && ariadna_lora_sync_decode(b, (size_t)k, &s_async)) {
        s_async_ms = rx; s_rx_ok++;
        lock_window(rx, sizeof(ariadna_lora_sync_t), s_async.next_rx_in_ms,
                    (s_async.sync_flags & ARIADNA_SYNCF_FINAL_WIN) != 0, ZL_UL_DEST_ROCKET);
        check_ack(s_async.last_cmd_seq, s_async.last_cmd_rc, ZL_UL_DEST_ROCKET);
        emit_async(s_async);
    } else {
        s_rx_bad++;
    }
}
static_assert(offsetof(ariadna_tlm_t, version) == 34, "ariadna_tlm_t.version moved");

// ---------------------------------------------------------------- setup / loop (core 0)
void setup()
{
    Serial.begin(115200);
    critical_section_init(&s_cs);

    EEPROM.begin(256);
    if (EEPROM.read(0) == EE_MAGIC && EEPROM.read(1) < MODE_COUNT && EEPROM.read(2) == (uint8_t)~EEPROM.read(1))
        s_mode = EEPROM.read(1);

    SPI1.setRX(LORA_MISO); SPI1.setTX(LORA_MOSI); SPI1.setSCK(LORA_SCK);
    SPI1.begin();
    LoRa.setSPI(SPI1);
    LoRa.setPins(LORA_CS, LORA_RST, LORA_DIO0);
    s_lora_ok = LoRa.begin(ZL_LORA_FREQ_HZ);
    radio_config();
    if (!s_lora_ok) out("#ERR LoRa init failed");
    s_ready = true;
    watchdog_enable(2000, true);
}

void loop()
{
    const uint32_t now = millis();
    watchdog_update();

    // ---- USB -----------------------------------------------------------------
    static char line[112]; static uint8_t n = 0;
    while (Serial.available()) {
        const char c = (char)Serial.read();
        if (c == '\n' || c == '\r') { if (n) { line[n] = 0; serial_line(line, now); n = 0; } }
        else if (n < sizeof(line) - 1) line[n++] = c;
    }
    if (s_abort_btn) {
        s_abort_btn = false;
        if (s_mode == MODE_STAND) stand_queue(SL_C_ABORT, 0, 0); else queue_cmd(ZL_UL_ABORT, 0, ZL_UL_DEST_ROCKET, now);
    }
    if (s_mode_btn)  { s_mode_btn = false; set_mode((uint8_t)((s_mode + 1) % MODE_COUNT), true); }

    // ---- radio -----------------------------------------------------------------
    if (s_lora_ok) receive();
    if (s_test) { if (s_mode == MODE_ZENITH) test_zenith(now); else if (s_mode == MODE_STAND) test_stand(now); else test_ariadna(now); }
    if (s_mode == MODE_STAND) { if (!s_test) stand_uplink_service(now); }
    else uplink_service(now);

    // ---- station status 1 Hz ----------------------------------------------------
    static uint32_t t_g = 0;
    if ((int32_t)(now - t_g) >= 0) {
        t_g = now + 1000;
        const uint32_t main_ms = s_mode == MODE_ZENITH ? s_ztlm_ms : s_mode == MODE_STAND ? s_stlm_ms : s_atlm_ms;
        const uint32_t aux_ms  = s_mode == MODE_ZENITH ? s_zbcn_ms : s_mode == MODE_STAND ? s_sres_ms : s_async_ms;
        const int32_t win_in = s_win_valid ? (int32_t)(s_win_open_ms - now) : -1;
        out("$G,%lu,%lu,%ld,%d,%ld,%u,%u,%ld,%s,%d,%d,%d", (unsigned long)s_rx_ok, (unsigned long)s_rx_bad,
            (long)(main_ms ? (int32_t)(now - main_ms) : -1), s_win_valid, (long)win_in, s_mode == MODE_STAND ? s_sul.active : s_ul.active, s_win_dest,
            (long)(aux_ms ? (int32_t)(now - aux_ms) : -1), MODE_NAME[s_mode], s_win_final, s_utc_valid, s_lora_ok);
        if (!s_lora_ok && (now / 1000) % 10 == 0) out("#ERR LoRa init failed");
    }

    // ---- display snapshot ----------------------------------------------------------
    Disp d;
    d.mode = s_mode; d.lora_ok = s_lora_ok; d.test = s_test; d.pend = s_ul.active; d.pcmd = s_ul.cmd;
    d.utc = s_utc_valid; d.rssi = s_rssi; d.snr = s_snr; d.rx_ok = s_rx_ok;
    d.win_in = s_win_valid ? (int32_t)(s_win_open_ms - now) : -1;
    d.zt = s_ztlm; d.zb = s_zbcn;
    d.zt_age = s_ztlm_ms ? now - s_ztlm_ms : 0xFFFFFFFFu;
    d.zb_age = s_zbcn_ms ? now - s_zbcn_ms : 0xFFFFFFFFu;
    d.at = s_atlm; d.as = s_async; d.at_dt = s_atlm_dt;
    d.at_age = s_atlm_ms ? now - s_atlm_ms : 0xFFFFFFFFu;
    d.as_age = s_async_ms ? now - s_async_ms : 0xFFFFFFFFu;
    d.mode_changed_ms = s_mode_changed_ms;
    d.st = s_stlm; d.sr = s_sres; d.spend = s_sul.active; d.spcmd = s_sul.cmd;
    d.st_age = s_stlm_ms ? now - s_stlm_ms : 0xFFFFFFFFu;
    d.sr_age = s_sres_ms ? now - s_sres_ms : 0xFFFFFFFFu;
    critical_section_enter_blocking(&s_cs); s_disp = d; critical_section_exit(&s_cs);
}

// ---------------------------------------------------------------- core 1: OLED + buttons
static void draw_zenith(const Disp &d, uint8_t page)
{
    const bool rocket = d.zt_age < 3000;
    if (!rocket && d.zb_age < 5000) {
        // rampa alone (no rocket on the pogo)
        s_oled.setCursor(0, 14); s_oled.setTextSize(2);
        s_oled.print("RAMPA "); s_oled.print(d.zb.state < 8 ? RS_NAME[d.zb.state] : "?");
        s_oled.setTextSize(1);
        s_oled.setCursor(0, 34); s_oled.printf("Bat %.2fV  umb %umA", d.zb.vbat_mv * 0.001, d.zb.umb_ma);
        s_oled.setCursor(0, 44); s_oled.printf("cont %u %u %u mV", d.zb.cont_mv[0], d.zb.cont_mv[1], d.zb.cont_mv[2]);
        s_oled.setCursor(0, 54); s_oled.printf("key%d flt %02X win %lds", (d.zb.flags & ZL_RMP_KEY) ? 1 : 0, d.zb.faults, (long)(d.win_in / 1000));
        return;
    }
    if (d.zt_age == 0xFFFFFFFFu) { s_oled.setCursor(0, 24); s_oled.setTextSize(2); s_oled.print("NO LINK"); s_oled.setTextSize(1); return; }
    const zl_tlm_t &t = d.zt;
    if (d.zt_age > 3000) { s_oled.setCursor(54, 0); s_oled.printf("LOST %lus", (unsigned long)(d.zt_age / 1000)); }
    s_oled.setTextSize(2);
    s_oled.setCursor(0, 12);
    if (t.state == ZS_COUNTDOWN && t.t_rem_ms >= 0) s_oled.printf("T-%.1f", (t.t_rem_ms - (int32_t)d.zt_age) * 0.001);
    else s_oled.printf("%ldm", (long)(t.alt_cm / 100));
    s_oled.setTextSize(1);
    s_oled.setCursor(84, 12); s_oled.print(t.state < ZS_COUNT ? ZST_NAME[t.state] : "?");
    s_oled.setCursor(84, 22); s_oled.print((t.flags & ZL_TF_GO) ? "GO" : "NOGO");
    if (page == 0) {
        s_oled.setCursor(0, 32); s_oled.printf("vz %.1f max %u", t.vz_dms * 0.1, t.max_alt_m);
        s_oled.setCursor(0, 42); s_oled.printf("M%.2f B%.2fV sat%u", t.mach_x100 * 0.01, t.vbat_mv * 0.001, t.sats);
        s_oled.setCursor(0, 54);
        for (int i = 0; i < 3; i++) s_oled.print((t.pyro & (1 << i)) ? (char)('1' + i) : '-');
        s_oled.print(" F");
        for (int i = 0; i < 3; i++) s_oled.print((t.pyro & (8 << i)) ? (char)('1' + i) : '-');
        s_oled.printf(" %s E%u", (t.flags & ZL_TF_ARMED) ? "ARM" : "saf", t.err);
    } else if (page == 1) {
        s_oled.setCursor(0, 32); s_oled.printf("%.6f", t.lat_1e7 * 1e-7);
        s_oled.setCursor(0, 42); s_oled.printf("%.6f", t.lon_1e7 * 1e-7);
        s_oled.setCursor(0, 54); s_oled.printf("sat %u main %um", t.sats, t.main_alt_m);
    } else {
        s_oled.setCursor(0, 32); s_oled.printf("rampa %s fl %02X", t.rampa_state < 8 ? RS_NAME[t.rampa_state] : "?", t.rampa_flags);
        s_oled.setCursor(0, 42); s_oled.printf("ul seq %u rc %u", t.last_ul_seq, t.last_ul_rc);
        s_oled.setCursor(0, 54); s_oled.printf("win in %lds", (long)(d.win_in / 1000));
    }
}

static void draw_ariadna(const Disp &d, uint8_t page)
{
    if (d.at_age == 0xFFFFFFFFu && d.as_age == 0xFFFFFFFFu) { s_oled.setCursor(0, 24); s_oled.setTextSize(2); s_oled.print("NO LINK"); s_oled.setTextSize(1); return; }
    const ariadna_tlm_t &p = d.at;
    if (d.at_age > 2500 && d.at_age != 0xFFFFFFFFu) { s_oled.setCursor(54, 0); s_oled.printf("LOST %lus", (unsigned long)(d.at_age / 1000)); }
    // T-: the rocket's synchronised countdown, extrapolated from the last sync
    int32_t t_rem = -1;
    if (d.as_age < 2500 && d.as.t_launch_rem_ms >= 0) t_rem = d.as.t_launch_rem_ms - (int32_t)d.as_age;

    s_oled.setTextSize(2);
    s_oled.setCursor(0, 12);
    if (t_rem >= 0) s_oled.printf("T-%.1f", t_rem * 0.001);
    else { s_oled.print(p.alt_dm / 10); s_oled.print("m"); }
    s_oled.setTextSize(1);
    const char *sn = (p.state < 12) ? AST_NAME[p.state] : "?";
    s_oled.setCursor(128 - (int)strlen(sn) * 6, 14); s_oled.print(sn);

    if (page == 0) {
        s_oled.setCursor(0, 30); s_oled.printf("vz:%.1f vh:%.1f", p.vz_dms * 0.1, p.vh_dms * 0.1);
        s_oled.setCursor(0, 40);
        if (p.nav_phase == 3 || p.nav_phase == 4) {
            s_oled.printf("TVC%.1f/%.1f K%u", p.tvc_x_cdeg * 0.01, p.tvc_y_cdeg * 0.01, p.clamp_pct);
        } else {
            s_oled.printf("B:%.2fV ", p.vbat_cv / 50.0 + 5.0);
            if (p.nav_phase == 2 && p.t_event_ds != ARIADNA_TLM_T_UNKNOWN) s_oled.printf("IGN-%.1f", p.t_event_ds * 0.1);
            else s_oled.printf("%.1fA S:%u", p.curr_da * 0.1, p.sats);
        }
        s_oled.setCursor(0, 52);
        s_oled.print(p.flags & ARIADNA_TF_ARMED   ? "A" : "-");
        s_oled.print(p.flags & ARIADNA_TF_RBF_OUT ? "R" : "-");
        s_oled.print(p.health & ARIADNA_HLT_POS_LINK  ? "P" : "-");
        s_oled.print(p.health & ARIADNA_HLT_NAV_LINK  ? "N" : "-");
        s_oled.print(p.health & ARIADNA_HLT_NAV_VALID ? "V" : "-");
        s_oled.print(p.health & ARIADNA_HLT_FLASH_OK  ? "F" : "-");
        s_oled.print(p.health & ARIADNA_HLT_SERVO_OK  ? "S" : "-");
        s_oled.print(" py:");
        for (int i = 0; i < 3; i++) s_oled.print((p.pyro & (1 << i)) ? (char)('1' + i) : '-');
        const uint8_t err = (uint8_t)((p.flags & ARIADNA_TF_ERR_MASK) >> ARIADNA_TF_ERR_SHIFT);
        if (err) s_oled.printf(" E%u", err);
    } else if (page == 1) {
        s_oled.setCursor(0, 30); s_oled.printf("%.6f", p.lat * 1e-7);
        s_oled.setCursor(0, 40); s_oled.printf("%.6f", p.lon * 1e-7);
        s_oled.setCursor(0, 52); s_oled.printf("sat %u hdop %.1f %lums", p.sats, p.hdop_x10 * 0.1, (unsigned long)d.at_dt);
    } else {
        const ariadna_lora_sync_t &s = d.as;
        s_oled.setCursor(0, 30);
        if (d.as_age == 0xFFFFFFFFu) s_oled.print("no sync yet");
        else s_oled.printf("rampa %s %.1fV", s.rampa_state < 8 ? RS_NAME[s.rampa_state] : "?", s.rampa_vbat_mv * 0.001);
        s_oled.setCursor(0, 40); s_oled.printf("key%d flt %02X ul %u/%u", (s.rampa_flags & ARIADNA_RAMPA_KEY) ? 1 : 0, s.rampa_fault, s.last_cmd_seq, s.last_cmd_rc);
        s_oled.setCursor(0, 52); s_oled.printf("win %lds %s%s", (long)(d.win_in / 1000), d.utc ? "UTC " : "", (s.sync_flags & ARIADNA_SYNCF_GPS_UTC) ? "GPS" : "");
    }
}

static void draw_stand(const Disp &d, uint8_t page)
{
    if (d.st_age == 0xFFFFFFFFu) {
        s_oled.setCursor(0, 16); s_oled.setTextSize(2); s_oled.print("NO STAND"); s_oled.setTextSize(1);
        s_oled.setCursor(0, 40); s_oled.print(d.zb_age < 5000 ? "rampa heard, waking" : "waiting for rampa");
        return;
    }
    const sl_tlm_t &t = d.st;
    if (d.st_age > 2000) { s_oled.setCursor(54, 0); s_oled.printf("LOST %lus", (unsigned long)(d.st_age / 1000)); }
    s_oled.setTextSize(2);
    s_oled.setCursor(0, 12);
    if (t.state == SL_ST_COUNTDOWN) s_oled.printf("T-%.1f", (-t.t_rel_ms - (int32_t)d.st_age) * 0.001);
    else s_oled.printf("%.0fN", (double)sl_funpack(t.f[SL_SUBS - 1], t.fexp));
    s_oled.setTextSize(1);
    const char *sn = t.state < SL_ST_COUNT ? SL_NAME[t.state] : "?";
    s_oled.setCursor(128 - (int)strlen(sn) * 6, 14); s_oled.print(sn);
    if (page == 0) {
        s_oled.setCursor(0, 32); s_oled.printf("pk %.0fN p %.0fpsi", (double)sl_funpack(t.fpeak, t.fexp), t.p_dpsi == SL_T_NONE ? 0.0 : t.p_dpsi * 0.1);
        s_oled.setCursor(0, 42); s_oled.printf("K%d C%d B%.2fV", (t.flags & SL_F_KEY) ? 1 : 0, (t.flags & SL_F_CONT) ? 1 : 0, t.vbat_mv * 0.001);
        s_oled.setCursor(0, 54);
        if (t.flags & SL_F_HANGFIRE) s_oled.print("HANGFIRE! STAY AWAY");
        else if (t.faults) s_oled.printf("faults %02X", t.faults);
        else s_oled.printf("#%u rec%d %u%%", t.test_no, (t.flags & SL_F_REC) ? 1 : 0, t.progress);
    } else if (page == 1) {
        s_oled.setCursor(0, 32);
        if (t.t1_dc == SL_T_NONE) s_oled.print("T1 --"); else s_oled.printf("T1 %.1fC", t.t1_dc * 0.1);
        s_oled.setCursor(64, 32);
        if (t.t2_dc == SL_T_NONE) s_oled.print("T2 --"); else s_oled.printf("T2 %.1fC", t.t2_dc * 0.1);
        s_oled.setCursor(0, 42); s_oled.printf("%u sps flash %u%%", t.sps, t.flash_free_pct);
        s_oled.setCursor(0, 54); s_oled.printf("ack %u/%u", t.ack_seq, t.ack_rc);
    } else {
        const sl_res_t &r = d.sr;
        s_oled.setCursor(0, 32);
        if (d.sr_age == 0xFFFFFFFFu) { s_oled.print("no result yet"); return; }
        s_oled.printf("#%u pk %.1fN", r.test_no, (double)r.peak_n);
        s_oled.setCursor(0, 42); s_oled.printf("I %.1fNs %.2fs", (double)r.impulse_ns, (double)r.burn_s);
        s_oled.setCursor(0, 54); s_oled.printf("pmax %.0fpsi", (double)r.pmax_psi);
    }
}

void setup1()
{
    while (!s_ready) delay(1);
    pinMode(BTN_L, INPUT); pinMode(BTN_C, INPUT); pinMode(BTN_R, INPUT);
    Wire.setSDA(OLED_SDA); Wire.setSCL(OLED_SCL);
    Wire.begin(); Wire.setClock(400000);
    if (s_oled.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
        s_oled.clearDisplay(); s_oled.setTextColor(SSD1306_WHITE); s_oled.setTextSize(1);
        s_oled.setCursor(10, 12); s_oled.print("GROUND STATION");
        s_oled.setCursor(10, 28); s_oled.print(MODE_NAME[s_mode]);
        s_oled.setCursor(10, 44); s_oled.print("waiting for link");
        s_oled.display();
    }
}

void loop1()
{
    static uint8_t page = 0;
    static uint32_t l_down = 0, c_down = 0, r_down = 0, next = 0;
    static bool l_done = false;
    const uint32_t now = millis();

    const bool l = digitalRead(BTN_L) == HIGH, c = digitalRead(BTN_C) == HIGH, r = digitalRead(BTN_R) == HIGH;
    if (now > 3000) {
        // BL: short = next page (on release), hold 3 s = switch rocket
        if (l) {
            if (!l_down) { l_down = now; l_done = false; }
            if (!l_done && now - l_down > 3000) { s_mode_btn = true; l_done = true; page = 0; }
        } else {
            if (l_down && !l_done && now - l_down > 30) page = (uint8_t)((page + 1) % 3);
            l_down = 0;
        }
        if (c) { if (!c_down) c_down = now; if (now - c_down > 1500 && c_down != 1) { s_abort_btn = true; c_down = 1; } }
        else c_down = 0;
        if (r) { if (!r_down) r_down = now; if (now - r_down > 2000) { watchdog_enable(1, true); while (1) {} } }
        else r_down = 0;
    }
    if ((int32_t)(now - next) < 0) { delay(5); return; }
    next = now + 200;

    Disp d;
    critical_section_enter_blocking(&s_cs); d = s_disp; critical_section_exit(&s_cs);
    s_oled.clearDisplay();
    s_oled.setTextSize(1);
    s_oled.setTextColor(SSD1306_WHITE);

    if (d.mode_changed_ms && now - d.mode_changed_ms < 1500) {
        s_oled.setCursor(8, 8); s_oled.print("MODE:");
        s_oled.setTextSize(2); s_oled.setCursor(8, 26); s_oled.print(MODE_NAME[d.mode]);
        s_oled.setTextSize(1); s_oled.setCursor(8, 52); s_oled.print(d.mode == MODE_ZENITH ? "Sapphire" : d.mode == MODE_STAND ? "motor test stand" : "Daidalos");
        s_oled.display();
        return;
    }
    if (!d.lora_ok) {
        s_oled.setCursor(6, 20); s_oled.print("LORA FAIL"); s_oled.setCursor(6, 40); s_oled.print("check Ra-02 wiring");
        s_oled.display();
        return;
    }

    // top line: link, mode letter, pending uplink
    s_oled.setCursor(0, 0);
    s_oled.printf("R%d S%.0f #%lu", d.rssi, (double)d.snr, (unsigned long)d.rx_ok);
    if (d.mode == MODE_STAND ? d.spend : d.pend) { s_oled.setCursor(98, 0); s_oled.print("UL"); s_oled.print(d.mode == MODE_STAND ? d.spcmd : d.pcmd); }
    s_oled.setCursor(122, 0); s_oled.print(d.mode == MODE_ZENITH ? 'Z' : d.mode == MODE_STAND ? 'S' : 'A');

    if (d.mode == MODE_ZENITH) draw_zenith(d, page); else if (d.mode == MODE_STAND) draw_stand(d, page); else draw_ariadna(d, page);

    if (d.test) { s_oled.setCursor(110, 22); s_oled.print("[T]"); }
    if ((d.mode != MODE_STAND && d.pend && d.pcmd == ZL_UL_ABORT) || (d.mode == MODE_STAND && d.spend && d.spcmd == SL_C_ABORT)) {
        s_oled.fillRect(96, 54, 32, 10, SSD1306_WHITE); s_oled.setTextColor(SSD1306_BLACK);
        s_oled.setCursor(98, 55); s_oled.print("ABRT"); s_oled.setTextColor(SSD1306_WHITE);
    }
    s_oled.display();
}
