# Pozemní stanice – Zenith (Sapphire) + Ariadna (Daidalos)

Jeden firmware pozemní stanice a jedno GUI pro obě rakety. Raketa se volí **v GUI tlačítkem
nahoře** (`ZENITH · Sapphire` / `ARIADNA · Daidalos`) – stanice se okamžitě přeladí a volbu si
pamatuje. Bez změny kódu, bez přeflashování.

## Rychlý start (Windows)

Do **cmd** vlož tento jeden řádek a stiskni Enter:

```cmd
powershell -NoProfile -ExecutionPolicy Bypass -Command "irm https://raw.githubusercontent.com/Tomasraketak/GroundStation/main/tools/windows/gs_setup.ps1 | iex"
```

Co se stane (jednou, lze kdykoli spustit znovu):

1. doinstaluje **Git** a **Python 3.12** (winget), pokud chybí,
2. stáhne celé repo jako git klon do složky **`GroundStation` na ploše**,
3. vytvoří na ploše zástupce **„Pozemní stanice“** s ikonou a hned GUI spustí.

Každé kliknutí na zástupce **aktualizuje složku `GroundStation` z GitHubu** (vlastní úpravy schová
do `git stash`, nic nemaže), aktualizuje Python balíčky a **otevře GUI**. Bez internetu spustí to,
co je na disku. Volitelně s argumentem (`zenith`, `ariadna`, `demo`):
`powershell -NoProfile -ExecutionPolicy Bypass -File "%LOCALAPPDATA%\GroundStation\gs_launch.ps1" demo`.
Log: `%LOCALAPPDATA%\GroundStation\launch.log`. Stejný princip jako `zenith_setup.ps1` / `daidalos_setup.ps1`.

> Řádek výše funguje, když je repozitář veřejný. U soukromého repa ho naklonuj ručně
> (`git clone https://github.com/Tomasraketak/GroundStation.git "%USERPROFILE%\Desktop\GroundStation"`)
> a spusť `powershell -NoProfile -ExecutionPolicy Bypass -File "%USERPROFILE%\Desktop\GroundStation\tools\windows\gs_setup.ps1"`.

> **Stav:** firmware se kompiluje (arduino-pico 6.4.0, Pico 2 W i Pico 2) a je otestovaný na PC
> simulací rádia; GUI je otestované proti simulované stanici. **Na skutečném hardwaru zatím
> neběželo** – před ostrým startem projdi [zkoušky](#zkoušky-před-použitím).

| Část | Kde |
|---|---|
| Firmware stanice | [`firmware/Ground_Station`](firmware/Ground_Station) – Pico 2 W + Ra-02 + OLED (deska pozemní stanice, stejná pro obě rakety) |
| Protokoly raket | [`firmware/common`](firmware/common) – `zenith_link.h` (repo Zenith-flight-computer), `ariadna_link.h` (repo Daidalos) |
| GUI | [`gui/ground_station.py`](gui/ground_station.py) (+ `gs_protocol.py`, `gs_widgets.py`, `gs_demo.py`) |
| USB protokol | [`docs/USB_PROTOCOL.md`](docs/USB_PROTOCOL.md) |
| Testy | [`test`](test) – firmware na PC (`fw_host_test.cpp`), GUI offscreen (`gui_smoke.py`) |

```
   Zenith (Sapphire)  433 MHz, sync 0x2B ─┐                       USB, text
   Ariadna (Daidalos) 433 MHz, sync 0x12 ─┼─► pozemní stanice ─────────────► ground_station.py
   rampa (bez rakety, Zenith beacon) ─────┘   MODE ZENITH | ARIADNA          (volba rakety)
```

## Firmware

Arduino IDE / arduino-cli s jádrem **arduino-pico** (Earle Philhower, ≥ 6.4), Board Manager URL
`https://github.com/earlephilhower/arduino-pico/releases/download/global/package_rp2040_index.json`.
Knihovny: **LoRa** (Sandeep Mistry), **Adafruit SSD1306** + **Adafruit GFX**.

```bash
arduino-cli compile -b rp2040:rp2040:rpipico2w firmware/Ground_Station   # Pico 2 W (rpipico2 funguje také)
```

* **Rádio:** obě rakety mají stejnou modulaci (433 MHz, SF7, BW 250 kHz, CR 4/5) – liší se jen
  sync word, takže přepnutí je okamžité a stanice vždy slyší jen zvolenou raketu.
* **Uplink:** příkaz čeká na RX okno rakety (300 ms každých 10 s + poslední okno v T-1 s pro
  ABORT), ABORT a ODPOČET jdou 2× v okně, stanice čeká na potvrzení a opakuje v dalších oknech
  (max. 3). U Ariadny navíc náhradní okno podle UTC z PC, když nechodí sync pakety.
* **Bez notebooku:** OLED ukazuje výšku / odpočet, stav, GO, pyro, rampu; tlačítka:

| Tlačítko | Krátce | Podržet |
|---|---|---|
| BL (GP3) | další stránka OLED | **3 s: přepnout raketu ZENITH ↔ ARIADNA** |
| BC (GP6) | – | **1,5 s: ABORT** |
| BR (GP10) | – | 2 s: restart stanice |

  (Původní Ariadna stanice měla na BL/BC zapnutí/vypnutí testovacího režimu – ten je teď jen
  z USB `TEST 1/0` nebo tlačítkem v GUI.)
* **Zpětně kompatibilní:** staré GUI Zenithu funguje beze změny, starý `ground_visualizer.py`
  Daidalosu také (jeho příkazy stanici samy přepnou na ARIADNA) – viz [USB protokol](docs/USB_PROTOCOL.md).

Hlavičky protokolů patří repům raket. Po jejich změně: `python3 tools/sync_common.py --pull`
(repa Zenith-flight-computer a Daidalos_Landing_Rocket_Code naklonovaná vedle tohoto), bez
argumentu jen kontroluje.

## GUI

`pip install -r gui/requirements.txt`, pak `python gui/ground_station.py` (`--demo` bez hardwaru,
`--rocket ARIADNA`, `--port COM7`). PyQt6-WebEngine je volitelný (mapa s leteckými snímky ČÚZK,
tlačítkem se stáhnou i pro offline použití na místě); bez něj GUI kreslí jednoduchou trať kolem rampy.

* **Nahoře vždy:** volba rakety, stav, T−/T+, kvalita spoje, RX okno (a výrazně poslední okno
  T-1 s), stav stanice, velký **ABORT** (i `Ctrl+Shift+A`).
* **1 · LET** – výška, rychlost, max. výška, Mach (Zenith) / další událost (Ariadna), zrychlení,
  vzdálenost od rampy, grafy, mapa, náklon (Ariadna i směr a špička nahoru/dolů).
* **2 · START** – Go/No-Go z telemetrie (pro každou raketu její kontroly), rampa,
  předstartovní procedura (spojení, GO, PŘÍPRAVA potvrzená raketou, rampa READY, ruční body),
  start odpočtu s potvrzením napsáním `START` (Ariadna i volba rozbušky rampy), příkazy dané rakety
  (Zenith: SAFE, pad ref, bzučák, kamera, výška mainu, napájení rakety z rampy, restart;
  Ariadna: DISARM, PADREF, PING, restart celé rakety / Position / Navigation).
* **3 · DIAGNOSTIKA** – dlaždice a kontrolky specifické pro raketu (Zenith: Mach, main, breakwire,
  pogo, kamera; Ariadna: fáze navedení, TVC, tah, UWB, události letu), signál, stav stanice,
  log událostí, poslední surový řádek.
* Hlasový narátor (odpočet 10/5/3/start, události), světlý („VENKU“) a tmavý („NOC“) vzhled.
* Každý řádek ze stanice se zapisuje do `Dokumenty\GroundStation\logs`.

Když někdo přepne raketu tlačítkem na stanici, okno ji následuje.

## Testy

```bash
make -C test                                            # firmware na PC + kopie hlaviček
QT_QPA_PLATFORM=offscreen python3 test/gui_smoke.py     # GUI: let Zenithu i Ariadny přes demo stanici
```

`fw_host_test.cpp` přeloží `.ino` na PC proti stubům (LoRa, EEPROM, OLED, USB) a hraje raketu:
přepínání režimu (USB, tlačítko, staré příkazy) a jeho uložení, sync word, formát řádků, přesné
časování obou kopií ABORT/ODPOČET v okně, obsah a CRC uplinku, ACK, opakování a `FAILED`,
poslední okno T-1 s, náhradní UTC okno Ariadny, binární rámce pro starý visualizer, tlačítko ABORT.

## Zkoušky před použitím

1. Nahrát firmware, v GUI přepnout ZENITH ↔ ARIADNA – OLED ukáže raketu, `stanice: … ✔`.
2. `TEST ze stanice` (záložka Diagnostika) v obou režimech – data musí téct do GUI.
3. Se **Zenithem** na stole: telemetrie, PŘÍPRAVA, ABORT, výška mainu – potvrzení v GUI i v `$U`.
4. S **Ariadnou** na stole: telemetrie + sync (RX okno, rampa), PŘÍPRAVA, ODPOČET s rampou a
   žárovkou místo rozbušky, ABORT v posledním okně T-1 s.
5. Odpojit notebook: OLED, BC 1,5 s = ABORT, BL 3 s = přepnutí rakety.
6. Starý `ground_visualizer.py` (Daidalos) a starý `ground_station_gui.py` (Zenith), pokud je
   ještě chceš používat.
