# USB protokol pozemní stanice (textové řádky, 115200 Bd, `\r\n`)

GUI je jen klient tohoto protokolu – vše jde dělat i ze Serial Monitoru Arduino IDE.

## Režim (raketa)

| Řádek | Popis |
|---|---|
| `MODE` | dotaz → `$MODE,ZENITH` nebo `$MODE,ARIADNA` |
| `MODE ZENITH` / `MODE ARIADNA` | přepne raketu (aliasy `SAPPHIRE`/`Z`, `DAIDALOS`/`A`), uloží do flash, odpoví `$MODE,…` a `#OK MODE …` |

Přepnutí zruší čekající příkaz (`$U,…,CANCELLED`), zahodí zámek RX okna i poslední telemetrii a
přeladí LoRa (jediný rozdíl na vzduchu je sync word: Zenith `0x2B`, Ariadna `0x12`).
Stejně funguje podržení levého tlačítka stanice 3 s – stanice pak sama pošle `$MODE`.

## Výstup

| Řádek | Režim | Pole |
|---|---|---|
| `$ID,Ground Station,<fw>,<MODE>` | oba | na `HELLO` |
| `$MODE,<MODE>` | oba | na `HELLO`, `MODE` a po každém přepnutí |
| `$G,…` (1 Hz) | oba | rx_ok, rx_bad, tlm_age_ms, win_locked, win_in_ms, pending, win_dest, aux_age_ms, MODE, win_final, utc_valid, lora_ok |
| `$U,seq,CMD,dest,STAV,rc,okno` | oba | STAV = `QUEUED` / `SENT` / `SENT2` / `ACK` / `FAILED` / `CANCELLED`; rc = `zl_ul_rc_t` / `ariadna_ul_rc_t` |
| `$T,…` | ZENITH | rssi, snr, state, flags, t_ms, lat, lon, alt_m, t_rem_ms, vz, vh, acc_g, max_alt, vbat_V, next_rx_ms, main_alt, tilt, mach, sats, pyro, health, rampa_state, rampa_flags, last_ul_seq, last_ul_rc, err, win_final (= protokol původní Zenith stanice) |
| `$B,…` | ZENITH | beacon rampy bez rakety: rssi, snr, state, flags, t_ms, vbat_V, umb_ma, cont1..3_mV, next_rx_ms, faults, last_ul_seq, last_ul_rc, fw |
| `$AT,…` | ARIADNA | rssi, snr, t_ms, lat, lon, alt_m, vz, vh, acc_g, qx, qy, qz (int16 „smallest three“), tvc_x, tvc_y [°], thrust_N, t_event_ds (−32768 = neznámo), version, state, nav_phase, clamp_pct, vbat_V, curr_A, hdop, sats, pyro, health, flags, events, uwb, t_flight_s, px_m, py_m |
| `$AS,…` | ARIADNA | sync / ACK / rampa: rssi, snr, rocket_t_ms, utc_ms (−1 = neznámo), next_rx_ms, rx_dur_ms, t_launch_rem_ms, last_cmd_seq, last_cmd_rc, rampa_state, rampa_flags, rampa_fault, sync_flags, rampa_vbat_mV |
| `#OK …` / `#ERR …` | oba | odpověď |

Bitové masky viz `firmware/common/zenith_link.h` (`ZL_TF_*`, `ZL_HL_*`, `ZL_RMP_*`, `ZL_RMPF_*`) a
`firmware/common/ariadna_link.h` (`ARIADNA_HLT_*`, `ARIADNA_TF_*`, `ARIADNA_TUWB_*`, `ARIADNA_SYNCF_*`).

## Příkazy

| Příkaz | Popis |
|---|---|
| `HELLO` | identifikace |
| `MODE [ZENITH\|ARIADNA]` | viz výše |
| `CMD <NAME> [arg] [arg2\|RAMPA]` | zařadí příkaz do nejbližšího RX okna rakety |
| `CANCEL` | zruší čekající příkaz |
| `TEST 0\|1` | stanice sama generuje telemetrii zvolené rakety (zkouška USB a GUI bez rakety) |
| `UTC <ms dne>` | čas z PC – ARIADNA: náhradní okno na UTC 10 s mřížce, když slábnou sync pakety, a čas pro raketu bez GPS |
| `LEGACY 0\|1` | ARIADNA: navíc binární rámce (0xA5 0x1D …) pro starý `ground_visualizer.py` |

| NAME | ZENITH | ARIADNA |
|---|---|---|
| `NOP` (`PING`) | ping | ping / synchronizace času |
| `PREPARE` (`PREP`) | launch soon | `LAUNCH_SOON` (pad reference, rampa PREPARE) |
| `COUNTDOWN <s>` (`CD`) | 10–3600 s od přijetí raketou | 5–3600 s, **T-0 pevně od zařazení**; `arg2` = rozbuška rampy 1–3 |
| `ABORT` | ✓ (2× v okně) | ✓ (2× v okně, i v posledním okně T-1 s) |
| `SAFE` (`DISARM`) | zpět do IDLE | `DISARM` – zrušení přípravy |
| `PADREF` | ✓ | ✓ |
| `REBOOT [cíl]` | Master | 0 vše, 1 Position, 2 Navigation |
| `MAINALT <m>`, `LOCATE 0/1`, `CAMERA 0/1` | ✓ | – (`#ERR … Ariadna does not have this command`) |
| `ROCKETPWR 0/1` | rampě (napájení rakety po pogu) | – |

Stanice drží jeden příkaz, odešle ho v nejbližším RX okně (ABORT a COUNTDOWN dvakrát v témž okně)
a čeká na potvrzení (`seq` + `rc` v další telemetrii / sync paketu). Bez potvrzení to zkusí
v dalších oknech, po třetím oknu hlásí `FAILED`; když raketa 45 s vůbec nevysílá, také `FAILED`.
Obě rakety duplicitní `seq` ignorují, opakování je tedy bezpečné. Čekající ABORT nejde přepsat
jiným příkazem.

## Zpětná kompatibilita

* **ZENITH:** řádky `$T/$B/$U/$G` a příkazy `CMD/CANCEL/TEST/HELLO` jsou stejné jako u původní
  Zenith stanice – `gui/ground_station_gui.py` z repa Zenith funguje beze změny.
* **ARIADNA:** příkazy starého Daidalos `ground_visualizer.py` (`T:1`, `T:0`, `UTC:<ms>`,
  `CMD:PING|PREP|ABORT|PADREF|DISARM|REBOOT[:t]|CD:<s>[:ch]`) stanici přepnou na ARIADNA a zapnou
  binární rámce `ARIADNA_PKT_TELEMETRY` / `ARIADNA_PKT_GS_SYNC`, které starý visualizer čte.
