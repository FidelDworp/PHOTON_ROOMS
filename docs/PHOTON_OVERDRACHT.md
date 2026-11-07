# Overdracht — Photon-rooms (PHOTON_ROOMS)

*Stand 7 november 2026 — Filip Delannoy (FiDel), Zarlardinge (BE)*

Dit document start een nieuw gesprek met Claude over de **oude Photon-controllers** (iTroniX PhotoniX shield) die nog in huis draaien. Nieuwe ontwikkeling gebeurt niet hier maar in `ESP32C6_ROOMS` (rooms) en `ESP32C6_ACCESS` (buitenverlichting).

**Werkwijze en afspraken:** zie de [overkoepelende overdracht](https://github.com/FidelDworp/Zarlardinge-Home-Automation/blob/main/docs/ZARLAR_OVERDRACHT.md). Kort: Nederlands · nooit een sketch of script zonder GO · eerst een plan, één stap per versie (versiekop + changelog) en daarna een 48 u-test · alles via OTA · `/json`-keys en bedieningsadressen niet wijzigen zonder akkoord · README en deze overdracht mee bijwerken · na GO compileert Claude en zet de `.bin` in `bin/`.

---

## 1. Wat en waar

| Map | Controller | Stand (7/11) |
|---|---|---|
| `R1-BandB` | B&B | defect, wordt de eerste ESP32-room |
| `R2-BADK` … `R7-ZITPL` | Badkamer, Inkom, Keuken, Wasplaats, Eetplaats, Zitplaats | in productie tot de overstap |
| `R8-ACCESS` | buitenverlichting W1/W2/S | in productie; sinds 6/11 variabele `JSON_status` voor het Dashboard (kopie in `ESP32C6_ACCESS/old/`) |
| `S-HVAC` | verwarming | in productie tot de ESP32 HVAC overneemt |
| `S-ECO_SOLAR`, `S-OUTSIDE` | zonneboiler (vervangen), zonlicht | S-OUTSIDE meet zonlicht voor de Photons |
| `R0-Generic`, `TESTROOM`, `old/` | basis en archief | — |

- **Bouwen:** bij elke commit compileert GitHub Actions met de Particle Cloud (menu "Actions"). **Bestanden niet verplaatsen**: dat breekt de automatische build.
- **Flashen:** door FiDel via de Particle Web IDE, CLI of Particle Dev.
- **Gegevens:** via de Particle Cloud en een Cloudflare Worker (token niet in de browser) naar het Dashboard en het RPi-portaal.

## 2. Afspraken

- Alleen noodzakelijke wijzigingen (zoals `JSON_status` in R8-ACCESS), na GO.
- Na de laatste overstap naar ESP32 wordt deze repo gearchiveerd.

## 3. Volgorde van de overstap

B&B (eerst, Photon defect) → Badkamer → Inkom → Keuken → Wasplaats → Zitplaats; ACCESS1–3 voor de buitenverlichting; HVAC zodra de ESP32 HVAC in productie gaat.
