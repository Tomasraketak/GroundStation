# USB protokol pozemní stanice (textové řádky, 115200 Bd, `\r\n`)

GUI je jen klient tohoto protokolu – vše jde dělat i ze Serial Monitoru Arduino IDE.

## Režim (raketa / test motoru)

| Řádek | Popis |
|---|---|
| `MODE` | dotaz → `$MODE,ZENITH`, `$MODE,ARIADNA` nebo `$MODE,STAND` |
| `MODE ZENITH` / `MODE ARIADNA` / `MODE STAND` | přepne režim (aliasy `SAPPHIRE`/`Z`, `DAIDALOS`/`A`, `MOTOR`/`THRUST`/`TRUST`/`TAH`/`S`), uloží do flash, odpoví `$MODE,…` a `#OK MODE …` |

Přepnutí zruší čekající příkaz (`$U,…,CANCELLED`), zahodí zámek RX okna i poslední telemetrii a
přeladí LoRa (jediný rozdíl na vzduchu je sync word: Zenith a STAND `0x2B`, Ariadna `0x12`; STAND se
od Zenithu liší vlastními pakety `stand_link.h`).
Stejně funguje podržení levého tlačítka stanice 3 s (ZENITH → ARIADNA → STAND → ZENITH) – stanice pak sama pošle `$MODE`.

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
| `$ST,…` | STAND | telemetrie rampy jako zkušebního stavu: rssi, snr, state (`sl_state_t`: 0 SAFE, 1 ARMING, 2 ARMED, 3 COUNTDOWN, 4 FIRING, 5 BURN, 6 ABORT, 7 RECORD), faults (`SL_FLT_*`), flags (`SL_F_*`), ack_seq, ack_rc, vbat_V, v5_V, t_rel_ms (< 0 do T-0), 10× tah (nejstarší první, rovnoměrně od minulého paketu; N, bez kalibrace µV/V), špička tahu, tlak psi, špička tlaku, T1, T2 °C (`nan` = nepřipojeno), test_no, flash volno %, průběh %, vzorků/s, rozbuška, kontinuita (bit0–2), servo bezpečnostního spínače (bit0 nastaveno – klíč sepne servo při ARM, bit1 sepnuto) |
| `$SR,…` | STAND | výsledek testu: test_no, flags (`SL_RF_*`), rozbuška, špička, impulz, hoření s, průměrný tah, čas špičky, zpoždění zážehu, max. tlak psi, max. T1, T2, bodů křivky, Hz křivky, vzorků na flash, vzorků/s, index T-0 v křivce (−1 bez zážehu) |
| `$SC,test,offset,n,f1..fn,p1..pn` | STAND | kus 100 Hz křivky (tah, tlak psi) na `CMD CURVE <offset>` (starší způsob) |
| `$SB,test,blok,bloků,n,Hz,flags,t_first_ms,burn_a_ms,burn_b_ms,T1,T2,f1..fn,p1..pn` | STAND | blok ořízlého zážehu na `CMD BURN`: bod i bloku k je v čase `t_first_ms + (k·40 + i)·1000/Hz` od T-0; `burn_a/b_ms` = detekovaný začátek / konec hoření (−2147483648 = nehořelo); T1/T2 °C na začátku bloku |
| `#OK …` / `#ERR …` | oba | odpověď |

Bitové masky viz `firmware/common/zenith_link.h` (`ZL_TF_*`, `ZL_HL_*`, `ZL_RMP_*`, `ZL_RMPF_*`),
`firmware/common/ariadna_link.h` (`ARIADNA_HLT_*`, `ARIADNA_TF_*`, `ARIADNA_TUWB_*`, `ARIADNA_SYNCF_*`) a
`firmware/common/stand_link.h` (`SL_F_*`, `SL_FLT_*`, `SL_RF_*`). V režimu STAND má `$U` dest `2`.

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

## Režim STAND (rampa jako zkušební stav motoru)

`CMD <NAME> [arg] [arg2]`:

| NAME | Argumenty | Rampa |
|---|---|---|
| `PING` | – | heartbeat (stanice ho posílá sama ~2,5 Hz; ve `$U` se neukazuje) |
| `ARM` | rozbuška 1–3, [puls 50–5000 ms] | ověří blokování, smaže místo pro záznam → ARMED |
| `DISARM` (`SAFE`) | – | zpět do SAFE, zruší zablokovaný ABORT |
| `FIRE` (`COUNTDOWN`, `CD`) | odpočet 3–60 s | jen z ARMED: odpočet na rampě, záznam od T-3 s, zážeh v T-0 |
| `ABORT` | – | kdykoli; nejde přepsat jiným příkazem |
| `TARE` / `PZERO` | – | nula tahu / tlaku |
| `CAL` | známá síla v N (stanice posílá mN) | měřítko tenzometru |
| `RECORD` (`REC`) / `STOP` | s / – | zkušební záznam bez pyro / konec záznamu |
| `RECTIME` | 2–120 s | délka záznamu po T-0 (uloží se v rampě) |
| `RESULT` / `CURVE` | – / první bod | znovu `$SR` / jeden `$SC` (16 bodů) |
| `BURN` | [první blok] [počet, 0 = všechny] | ořízlý zážeh (50 Hz, bloky po 40 bodech) za sebou v jedné dávce; ztracené bloky se vyžádají znovu |
| `BUZZER` (`LOCATE`) | 0 / 1 | bzučák rampy |

Nejsou RX okna: rampa poslouchá vždy, když nevysílá. Stanice pošle čekající příkaz 6 ms po každém paketu
rampy a opakuje ho, dokud ho telemetrie nepotvrdí (`ack_seq` + `ack_rc`: 1 OK, 2 špatný stav, 3 blokování,
4 argument, 5 přerušeno, 6 zaneprázdněno); po 12 odesláních / 8 s `FAILED`. Když rampu 1,5 s neslyší,
vysílá naslepo (PING à 700 ms – tím rampu probudí do režimu test motoru; ABORT à 300 ms, 15 s). `CURVE`
potvrzuje sám paket křivky, `BURN` první blok dávky. Během dávky stanice po blocích neposílá PING
(srazil by se s dalším blokem), jen skutečný příkaz. Rampa duplicitní `seq` neprovede dvakrát.
