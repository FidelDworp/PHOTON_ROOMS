/* S-ECO_SOLAR.ino = Energy_Monitor + SOLAR Pump controller for the "ECO-Boiler" Photon in the boiler room.

Versions:
- 27jul26: functie "manual" uitgebreid: PWM direct instellen (0...255) Command = "200" => Draait 200 en relay ON. "0" = relay off. TEST: tot PWM 10 hoor je de pomp starten en stoppen. Regeling kan dus heel laag gaan!
- 26jul26: dEQ vereenvoudigd naar een simpele 1-staps-delta (verschil met de vorige minuut) i.p.v. het glijdend venster van 10 minuten. Reden: dEQ stuurt sinds 25jul26b niets meer aan, is dus zuiver informatief - en het venster introduceerde een nieuwe reboot-bug (een onstabiele allereerste sensormeting na het opstarten bleef 10 minuten in het venster hangen en gaf dan een absurde uitschieter, bv. dEQ=28 kWh). De simpele delta is immuun daarvoor: elke reboot start gewoon met dEQ=0 op de eerste meting, geen geheugen van vóór de herstart (Claude)
- 25jul26b: FUNDAMENTELE VEREENVOUDIGING - de hele dEQ-gebaseerde verlies-beperking (BEPERKT-fase) is verwijderd. dEQ meet de totale boiler-energie, die evengoed daalt door bv. warmwaterverbruik - los van of de zonnelus goed werkt. dT (via de PID) geeft al een direct, ogenblikkelijk antwoord op de enige vraag die telt: is de collector nu warmer dan de boiler? Alle bugs in dit traject (deadlock, stale-read, kwantisatieruis, reboot-blinde-periode) waren symptomen van deze extra laag, niet van de kernregeling. dEQ blijft in het logblad staan (het glijdend venster van 24jul26), maar stuurt de PWM niet langer aan. De echte veiligheidsmodi (nacht, thermosifon, oververhitting, stop-hysterese, opstartdemping) blijven volledig intact (Claude)
- 25jul26: (1) Verlies-streak vervangen door een gegradueerde reactie: een opgebouwde "verliesminuten"-teller die sneller oploopt bij een groter verlies (0,5x-3x tempo t.o.v. een typisch verlies) en het PWM-plafond geleidelijk laat zakken over ~15 minuten i.p.v. een harde knip na een vast aantal cycli - herstelt ook sneller (-3/min) zodra dEQ weer positief is. (2) Anti-windup: de PID-integraalterm wordt bevroren zolang de PWM toch al door de opstart- of verliesbegrenzing tegengehouden wordt, tegen de PWM-piek die ontstond zodra zo'n plafond wegviel. (3) Opstartfase verlengd van 4 naar 6 minuten (Claude)
- 24jul26: dEQ herbouwd als glijdend venster van 10 minuten historiek (i.p.v. een 1x/10' snapshot). Elke minuut een verse waarde, mét de resolutie van een volledig 10-minutenvenster (vermijdt afrondingsruis van EQtot's 2 decimalen bij een korter venster). Volledig zelfstandig aan de Photon - geen enkele koppeling meer met hoe vaak Google Sheets logt, en geen "dEQFresh"-vlag meer nodig (Claude)
- 17jul26b: KRITIEKE BUGFIX - dEQ werd maar 1x per 10' herberekend, maar de verlies-streak-check in solarPump() las die waarde elke minuut opnieuw. Bij een toevallig stale dEQ=0,000 (bv. net na een reflash) liep de streak-teller 8-10x op in evenveel minuten, terwijl EQtot intussen gewoon steeg - de BEPERKT-fase greep dus onterecht in. (Inmiddels vervangen door het glijdend venster hierboven, dat dit probleem structureel oplost i.p.v. te symptoombestrijden) (Claude)
- 17jul26: Fase-gebaseerde overgangsdemping tegen de zware overshoots bij dagbegin/-einde: (1) opstartfase van 4' na elke pompstart met PWM begrensd op 90, dempt de "hete-plug"-piek vóór de PID overneemt; (2) verlies-streak stopt de pomp niet meer hard, maar begrenst enkel PWM naar het minimum (pomp blijft draaien i.p.v. herhaaldelijk te herstarten); (3) stop-hysterese met debounce (2 opeenvolgende cycli onder DT_STOP) tegen het avond-geflipper rond de drempel (Claude)
- 16jul26: KRITIEKE BUGFIX - consecutiveReductions (verlies-streak-teller) werd sinds de PID-ombouw (14jul26b) nooit meer gereset bij een echte herstart van de pomp. Gevolg: eens de teller op 3 stond, bleef de pomp permanent OFF (deadlock), zelfs bij dT tot 38,9°C en Tsol tot bijna 90°C - enkel een reboot herstelde het. Reset toegevoegd in de start-branch van solarPump() (Claude)
- 15jul26: dT-filter (EMA) toegevoegd tegen de "hete-plug"-piek bij pompstart (dT schoot 18-40°C op en stortte dan in) — hysterese én PID gebruiken nu dTFiltered i.p.v. de ruwe waarde. PWM_RAMP_STEP verhoogd van 25 naar 60/min: die was de facto de enige actieve regelaar geworden (elke cyclus identieke trap 25-50-75-100-125 ongeacht dT-piek, PID werd altijd tegen 255 geklemd) (Claude)
- 14jul26b: solarPump() omgebouwd naar PID-regeling (DT_TARGET=2.5°C) i.p.v. drempel+ramp; lost de blijvende 9-minuten aan/uit-cyclus op door de PWM continu naar een evenwicht te laten zoeken i.p.v. te forceren met een minimale looptijd (Claude)
- 14jul26: Hysterese (aan bij dT>3.0, uit pas bij dT<1.0) + minimale looptijd (4') + PWM-ramp (max 40/min) toegevoegd in solarPump() om abrupt aan/uit schakelen en PWM-sprongen te voorkomen (Claude)

Features:
- Reads the six "hardcoded" DS18x20 sensors of the ECO boiler by their HEX sensor code and use them in the loop() by their own names.
Note: The sketch was updated by @ric using "doubles" (Particle.variables can't use floating variables). The library was integrated in this sketch so that it only needs the "OneWire" library. @Ric's function was modified by @BulldogLowell to include CRC checking:
- Calculating hourly heat energy added or consumed + accumulating total energy added or consumed for a full season.
- 1-wire bus scanner: Identify the addresses of the (DS18B20) temperature sensors. This is called manually from the function "Manual"
- Variable(s) for this monitoring are retained in ROM memory so that the accumulated energy demand can be kept for a full year... Make sure Vbat remains powered by a 3V Lithium button cell!
- 1-wire address scanner: List all DS18B20 sensors on the T-BUS

NEW: This sketch checks the difference between the Solar and Boiler temperature and controls the Pumprelay based on the "difference"delta T".
A PWM signal between 0 and 100% is generated to control the Solar pump speed. When Delta T is 10°C, the PWM = 100%, when it's 0°C, PWM = 0%.
To digitize PT1000 temperature, an "Adafruit_MAX31865" module is used: Uses hardware SPI1 module: CS = pin A2 (A2=SS,A3=SCK,A4=MISO,A5=MOSI)

---------------------------------------
PhotoniX shield v.4.0	I/O connections: (* = used)

 D0 - I2C-SDA
 D1 - I2C-SCL
*D2 - TOUCH-COM (= relayPin)
*D3 - RoomSense T-BUS (=Multiple TEMP sensors)
 D4 - PIXEL-line
 D5 - RoomSense PIR
 D6 - RoomSense TEMP/HUM
 D7 - RoomSense GAS-DIG
 A0 - TOUCH-1
 A1 - TOUCH-2
*A2 - RoomSense GAS-ANA (= SPI SS pin)
*A3 - RoomSense LIGHT (= SPI CSK pin)
*A4 - RF out (= SPI MISO pin)
*A5 - LCD-Reset (= SPI MOSI pin)
 A6 - OP1
*A7 - OP2 (= pwmPin)
 TX/RX - Serial comms
---------------------------------------
-
*/

// GENERAL
STARTUP(WiFi.selectAntenna(ANT_EXTERNAL));  // INT of EXT = Stabiel. Géén AUTO gebruiken: Creert disconnects!
STARTUP(System.enableFeature(FEATURE_RETAINED_MEMORY));

// WiFi RECONNECT VARIABLES
static unsigned long lastReconnectAttempt = 0;
const unsigned long RECONNECT_INTERVAL = 30000;  // 30 sec
bool cloudReady = false;

double Hour; // Used to switch solar pump ON/OFF

// WIFI monitoring
int wifiRSSI = WiFi.RSSI();  // ← int

// Memory monitoring
int freemem = System.freeMemory();
int memPERCENT = (freemem * 100) / 82944;  // 80 KB RAM

// Strings for publishing
char str[255]; // Temporary string for all messages published
char data[255]; // Temporary string for all data published
char JSON_temperat[622]; // Publish all temperature variables in one string (Max 622)

// *A2: SPI bus
#include <Adafruit_MAX31865.h>
Adafruit_MAX31865 sensor = Adafruit_MAX31865(A2); // Using hardware SPI1 module: CS = pin A2 (A2=SS,A3=SCK,A4=MISO,A5=MOSI)
//#define RREF 4300.0 // Rref: PT100 = 430.0; PT1000 = 4300.0 => Vervangen door nieuwe regel om vriesprobleem op te lossen!
const float RREF = 4000.0;   // ← 4000.0 (niet 4300!) => PT1000 met standaard Adafruit/Chinese MAX31865 breakout → Rref = 4000 Ω

// *D3 - T-BUS
#include <OneWire.h> //Initialize "OneWire" library
const int oneWirePin = D3; // Compatible to PhotoniX shield...
OneWire ds = OneWire(oneWirePin);

// Store addresses of DS18B20 sensors (Starting with 0x28,) and activate "getTemperatures(0);" in loop() function:
byte addrs0[6][8] = {{0x28,0xFF,0x0D,0x4C,0x05,0x16,0x03,0xC7}, {0x28,0xFF,0x25,0x1A,0x01,0x16,0x04,0xCD}, {0x28,0xFF,0x89,0x19,0x01,0x16,0x04,0x57}, {0x28,0xFF,0x21,0x9F,0x61,0x15,0x03,0xF9}, {0x28,0xFF,0x16,0x6B,0x00,0x16,0x03,0x08}, {0x28,0xFF,0x90,0xA2,0x00,0x16,0x04,0x76}}; // = For 6 "DS18B20" sensors in ECO buffer

// Initialize the names of the sensors as double variables: (=> can be published as "Particle.variables")
double ETopH, ETopL, EMidH, EMidL, EBotH, EBotL; // = 6 sensors (012345) in the ECO boiler
double* temps[] = {&ETopH, &ETopL, &EMidH, &EMidL, &EBotH, &EBotL}; // Group1: 6 sensors (012345) in the ECO boiler

// Initialize the globals for time stamp (Faulty sensor reporting via CRC checking):
char crcErrorJSON[128];
int crcErrorCount[sizeof(temps)/sizeof(temps[0])];
uint32_t tmStamp[sizeof(temps)/sizeof(temps[0])];

// Global variables for energy calculations:
int getTemperaturesInterval = 1 * 60 * 1000; // Sample rate for temperatures
int getTemperaturesLastTime = millis() - getTemperaturesInterval;  // Reset so that it samples immediately at start-up!
double celsius;
double ETmin = 35; // Minimum ECO boiler temperature to calculate "spare" energy (Securing Hot water supply)
double EAv1, EAv2, EAv3, EAv4, EAv5, EAv; // Average temperatures
double EQ1, EQ2, EQ3, EQ4, EQ5, EQtot, dEQ; // Boiler energy
// dEQ als eenvoudige 1-staps-delta t.o.v. de vorige minuut (26jul26) - dEQ is
// sinds 25jul26b zuiver informatief (stuurt de PWM niet meer aan), dus geen
// venster meer nodig. Simpeler én immuun voor het reboot-euvel van het
// glijdend venster (een instabiele eerste meting na opstart die 10 minuten
// later een absurde uitschieter gaf).
double prevEQtot = 0;
bool prevEQtotInit = false;

// Define variables for SOLAR controller: *A7: PWM, *D2: relay
int relayPin = D2;
bool relayState = false; // Initial state of relayPin
double relay = 0; // Initial state of relay = 0 (OFF)
int pwmPin = A7; // On PhotoniX shield, connect these pins (A7, 5V, Gnd) and increase PWM voltage from 3.3v to 5V with OpAmp!

double Tboil = 0; // Boiler temperature where SOLAR liquid enters: EBotH
double Tsun = 0;
double dT = Tsun - Tboil;
double Hysteresis = 1;
double pwmValue = 0; // 0-255 (Use double to calculate)

// PID-regelaar voor pompsnelheid (vervangt drempel+ramp), 14jul26b
double DT_TARGET = 2.5;          // gewenste dT-evenwicht i.p.v. aan/uit-cyclus
double DT_START  = 3.0;          // drempel om de pomp te starten vanuit stilstand
double DT_STOP   = 0.5;          // drempel om de pomp volledig te stoppen (relay uit)
double Kp = 8.0;
double Ki = 0.6;
double Kd = 3.0;
double pidIntegral = 0;
double pidPrevError = 0;
const double PID_I_MAX = 50.0;   // anti-windup clamp op de integraalterm
double PWM_MIN_RUN = 60;         // ondergrens PWM zolang de pomp draait
double PWM_RAMP_STEP = 60;       // extra veiligheidslimiet: max PWM-verandering per minuut, bovenop de PID
unsigned long pumpStartTime = 0;
unsigned long lastPidTime = 0;

// dT-filter (EMA) tegen de "hete-plug"-piek bij pompstart, 15jul26
double dTFiltered = 0;
bool dTFilterInit = false;
const double DT_FILTER_ALPHA = 0.3;  // per minuut; lager = trager/gladder, hoger = reageert sneller

// Fase-gebaseerde overgangsdemping tegen zware overshoots bij dagbegin/-einde, 17jul26/25jul26
const unsigned long SOFT_START_MIN = 6;   // minuten na pompstart dat PWM begrensd blijft
double SOFT_START_PWM_CAP = 90;           // PWM-grens tijdens opstartfase
int belowStopCount = 0;
const int STOP_DEBOUNCE_CYCLES = 2;       // aantal opeenvolgende cycli onder DT_STOP vooraleer echt te stoppen

// Define timer to call solarPump function
const unsigned long pumpInterval = 1 * 60 * 1000;
unsigned long pumpTimer = millis() - pumpInterval;





// Handmatige berekening van PT1000 temperatuur (omzeilt bibliotheekbug: Foute berekening van Tsol als het vriest!)
float readSolarTemp() {
  uint16_t rtd = sensor.readRTD();
  if (rtd == 0 || rtd > 32768) {
    return -127.0;
  }
  float ratio = rtd / 32768.0;
  float resistance = ratio * 4000.0;
  float temperature = (resistance - 1000.0) / 3.850;
  if (temperature < -50 || temperature > 200 || isnan(temperature)) {
    return -127.0;
  }
  return temperature;
}




void setup()
{
  SYSTEM_THREAD(ENABLED);
  WiFi.listen(false);// schakelt "WiFi scanning mode" uit: verstoort connectie bij zwak signaal!
  Particle.connect();
  waitUntil(Particle.connected);  // WACHT TOT WIFI + CLOUD KLAAR IS!

  Time.zone(+2); // Set clock to Belgium time (+1 in winter, +2 in summer)

  // *D3 - T-BUS
  for (int i = 0; i < sizeof(temps)/sizeof(temps[0]); i++) // @BulldogLowell: Initialize the timestamp array: prevent wrong messages if you get a bad CRC error on the first reading after startup...
  {
    tmStamp[i] = Time.now();
  }

  // Initialize pin mode for SOLAR controller
  pinMode(relayPin, OUTPUT);
  pinMode(pwmPin, OUTPUT);

  // Make sure the pump is OFF when starting:
  digitalWrite(relayPin, HIGH);
  relayState = false; relay = 0;

  // Start SPI bus for MAX31865 module:
  SPI.begin();
  sensor.begin(MAX31865_2WIRE); // set to 2WIRE connection

  // GENERAL Particle functions:
  Particle.function("Manual", manual);

  // Initialize Particle variables:
  Particle.variable("JSON_temper", JSON_temperat, STRING); // Publishes all system temperatures

  // Report the CRC errors with sensor ID:
  Particle.variable("CRC_Errors", crcErrorJSON, STRING); // It creates an array of errorcounts of all active sensors. Example: {"errorCount":[17,4,4,14,8,3]} => 17 = sensor 0, 4 = sensor 1, etc...
}




void loop()
{
  Hour = Time.hour();

  // Memory monitoring
    freemem = System.freeMemory();
    memPERCENT = (freemem * 100) / 82944;  // 80 KB RAM

  // === STABIELE RECONNECTIE MET COOLDOWN ===
  if (!Particle.connected())
  {
    if (millis() - lastReconnectAttempt > RECONNECT_INTERVAL)
    {
      Particle.connect();
      lastReconnectAttempt = millis();
    }
  }
  else if (!cloudReady)
  {
    cloudReady = true;
  }

  // === TEMPERATUREN & ENERGIE (elke minuut) ===
  if ((millis() - getTemperaturesLastTime) > getTemperaturesInterval)
  {
    getTemperatures(0);
    getTemperaturesLastTime = millis();

    // --- ENERGIEBEREKENINGEN ---
    EAv1 = (ETopH + ETopL)/2;
    EAv2 = (ETopL + EMidH)/2;
    EAv3 = (EMidH + EMidL)/2;
    EAv4 = (EMidL + EBotH)/2;
    EAv5 = (EBotH + EBotL)/2;
    EAv = (EAv1+EAv2+EAv3+EAv4+EAv5)/5;

    EQ1 = (EAv1-ETmin)*110*1.163/1000;
    EQ2 = (EAv2-ETmin)*90*1.163/1000;
    EQ3 = (EAv3-ETmin)*90*1.163/1000;
    EQ4 = (EAv4-ETmin)*90*1.163/1000;
    EQ5 = (EAv5-ETmin)*110*1.163/1000;
    EQtot = EQ1+EQ2+EQ3+EQ4+EQ5;

    // Eenvoudige 1-staps-delta t.o.v. de vorige minuut (26jul26) - zuiver
    // informatief, stuurt niets meer aan. Op de eerste meting na een reboot
    // (prevEQtotInit nog false) tonen we 0 i.p.v. een misleidende sprong.
    if (!prevEQtotInit) {
      prevEQtot = EQtot;
      prevEQtotInit = true;
      dEQ = 0;
    } else {
      dEQ = EQtot - prevEQtot;
      prevEQtot = EQtot;
    }


    Tsun = readSolarTemp();
    Tboil = EBotH;
    dT = Tsun - Tboil;


    // --- LIVE WIFI & MEM ---
    wifiRSSI = WiFi.RSSI();  // ← int

    // --- JSON ---
    snprintf(JSON_temperat, sizeof(JSON_temperat), "{"
      "\"ETopH\":%.1f,\"ETopL\":%.1f,\"EMidH\":%.1f,\"EMidL\":%.1f,"
     "\"EBotH\":%.1f,\"EBotL\":%.1f,\"EAv\":%.1f,\"EQtot\":%.2f,"
     "\"Solar\":%.1f,\"dT\":%.1f,\"dEQ\":%.3f,\"pwmVal\":%.0f,"
      "\"Relay\":%.0f,\"WiFiSig\":%d,\"Mem\":%d"
    "}",
      ETopH, ETopL, EMidH, EMidL, EBotH, EBotL, EAv, EQtot,
      Tsun, dT, dEQ, pwmValue, relay,
      wifiRSSI, memPERCENT
    );

    // --- EVACUATE (HVAC) ---
    static unsigned long lastEvacuate = 0;
    if (EQtot > 15 && millis() - lastEvacuate >= 300000)
    {
      sprintf(str, "ECO: %.2f kWh", EQtot);
      if (Particle.connected())
      {
        Particle.publish("Status-HEAT:HVAC", str, PRIVATE);
      }
      lastEvacuate = millis();
    }

    // --- PUBLISH STATUS (alleen elke 5 min) ---
    static unsigned long lastSolarPublish = 0;
    if (millis() - lastSolarPublish >= 300000)
    {
      if (Particle.connected())
      {
        Particle.publish("Solar", str, PRIVATE);
      }
      lastSolarPublish = millis();
    }
  }

  // === POMP (elke minuut) ===
  if (millis() - pumpTimer >= pumpInterval)
  {
    solarPump();
    pumpTimer = millis();
  }
}




// Functions

// Define function to map a value from one range to another in solarPump() function
float mapRange(float value, float inputMin, float inputMax, float outputMin, float outputMax)
{
  return (value - inputMin) * (outputMax - outputMin) / (inputMax - inputMin) + outputMin;
}






// Control solar pump speed (PWM value) and ON/OFF state – PID-regelaar, 14jul26b
void solarPump() {
  // Nachtblokkering
  if (Hour < 7 || Hour >= 21) {
    digitalWrite(relayPin, HIGH);
    relayState = false; relay = 0; pwmValue = 0;
    sprintf(str, "Pump OFF - Nachtblokkering");
    analogWrite(pwmPin, 0);
    belowStopCount = 0;
    pidIntegral = 0; pidPrevError = 0;
    return;
  }

  // dT-filter (EMA): dempt de korte, hevige piek van de "hete plug" bij pompstart
  // (dT schoot voorheen naar 18-40°C op en stortte een minuut later in - een
  // meetartefact, geen echt duurzaam warmteoverschot). Hysterese én PID werken
  // hierna met dTFiltered i.p.v. de ruwe dT.
  if (!dTFilterInit) {
    dTFiltered = dT;
    dTFilterInit = true;
  } else {
    dTFiltered += DT_FILTER_ALPHA * (dT - dTFiltered);
  }

  // Hysterese voor de relay zelf, met debounce op de stop-kant: pas echt stoppen
  // na STOP_DEBOUNCE_CYCLES opeenvolgende metingen onder DT_STOP. Dempt het
  // geflipper 's avonds wanneer dT lang rond de drempel blijft hangen.
  bool shouldBeOn;
  if (relayState) {
    if (dTFiltered > DT_STOP) {
      shouldBeOn = true;
      belowStopCount = 0;
    } else {
      belowStopCount++;
      shouldBeOn = (belowStopCount < STOP_DEBOUNCE_CYCLES);
    }
  } else {
    shouldBeOn = (dTFiltered > DT_START);
    belowStopCount = 0;
  }

  // Thermosifon blokkeren: voorkomt terugstroming/afkoeling bij lage Tsun
  if (dTFiltered > DT_START && Tsun < 22.0) {
    shouldBeOn = false;
    sprintf(str, "Pump OFF - Thermosifon (Tsun=%.1fC)", Tsun);
  }

  // Oververhitting: forceert AAN + PWM direct naar max - dé bescherming tegen
  // een oplopende collectortemperatuur, overschrijft alle andere logica
  bool overheat = (Tsun >= 90.0);
  if (overheat) {
    shouldBeOn = true;
  }

  if (!shouldBeOn) {
    if (relayState) sprintf(str, "Pump OFF - dT gefilterd=%.1fC <= stopdrempel", dTFiltered);
    digitalWrite(relayPin, HIGH);
    relayState = false; relay = 0;
    pwmValue = 0;
    pidIntegral = 0;      // reset zodat er geen windup optreedt terwijl de pomp stilstaat
    pidPrevError = 0;
    belowStopCount = 0;
    analogWrite(pwmPin, 0);
    return;
  }

  // --- Pomp is/wordt AAN: PID regelt de snelheid rond DT_TARGET ---
  unsigned long nowMs = millis();
  double dtMin = 1.0;  // tijd sinds vorige PID-stap, in minuten
  bool justStarted = !relayState;

  if (justStarted) {
    pumpStartTime = nowMs;
    pidIntegral = 0;
    pidPrevError = dTFiltered - DT_TARGET;   // voorkomt een D-piek bij de allereerste stap
  } else if (lastPidTime > 0) {
    dtMin = (double)(nowMs - lastPidTime) / 60000.0;
    if (dtMin <= 0) dtMin = 1.0;
  }
  lastPidTime = nowMs;

  digitalWrite(relayPin, LOW);
  relayState = true; relay = 1;

  bool inSoftStart = (nowMs - pumpStartTime) < (SOFT_START_MIN * 60000UL);

  double pwmDoel;

  if (overheat) {
    pwmDoel = 255;
  } else if (Tsun > 75.0) {
    pwmDoel = 180;
  } else {
    double error = dTFiltered - DT_TARGET;

    // Anti-windup: de integraalterm bevriezen tijdens de opstartfase, tegen de
    // PWM-piek die ontstond zodra dat plafond wegviel.
    if (!inSoftStart) {
      pidIntegral = constrain(pidIntegral + error * dtMin, -PID_I_MAX, PID_I_MAX);
    }
    double derivative = (error - pidPrevError) / dtMin;
    pidPrevError = error;

    double pidOutput = (Kp * error) + (Ki * pidIntegral) + (Kd * derivative);
    pwmDoel = constrain(PWM_MIN_RUN + pidOutput, PWM_MIN_RUN, 255);

    // Opstartfase: PWM hard begrensd, ongeacht wat de PID wil - geeft de
    // "hete plug" tijd om weg te stromen vóór de PID volledig overneemt.
    if (inSoftStart) {
      pwmDoel = min(pwmDoel, SOFT_START_PWM_CAP);
    }
  }

  // Extra veiligheidslimiet bovenop de PID: max verandering per minuut
  if (overheat) {
    pwmValue = pwmDoel;
  } else if (pwmValue < pwmDoel) {
    pwmValue = min(pwmValue + PWM_RAMP_STEP * dtMin, pwmDoel);
  } else if (pwmValue > pwmDoel) {
    pwmValue = max(pwmValue - PWM_RAMP_STEP * dtMin, pwmDoel);
  }

  // Status: geeft altijd exact de actieve fase weer
  if (overheat) {
    sprintf(str, "Pump ON (OVERVERHIT) - Tsun=%.1fC >= 90C, PWM=255", Tsun);
  } else if (inSoftStart) {
    sprintf(str, "Pump ON (OPSTART) - dT gefilterd=%.1fC, PWM begrensd op %d", dTFiltered, (int)SOFT_START_PWM_CAP);
  } else {
    sprintf(str, "Pump ON (REGIME) - dT gefilterd=%.1fC PWM=%d (fout=%.1f)", dTFiltered, (int)pwmValue, dTFiltered - DT_TARGET);
  }
  analogWrite(pwmPin, (int)pwmValue);
}







// *D3 - T-BUS
void discoverOneWireDevices(void) // T-BUS Scanner function  => List addresses of all 1-wire devices on the T-BUS (D3) and publish to Particle cloud. For example T-BUS DS18B20 sensors => discoverOneWireDevices();
{
  byte i;
  byte present = 0;
  byte data[12];
  byte addr[8];
  int sensorCount = 0;

  Particle.publish("OneWire", "Looking for 1-wire addresses:", 60, PRIVATE); delay(500);

  while(ds.search(addr) and sensorCount < 20)
  {
    sensorCount++;
    char newAddress[48] = ""; // Make space for the 48 characters of our sensor addresses.
    snprintf(newAddress, sizeof(newAddress), "0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5], addr[6], addr[7]);
    Particle.publish("OneWire", newAddress, 60, PRIVATE); delay(500);

    if ( OneWire::crc8( addr, 7) != addr[7])
    {
      Particle.publish("OneWire", "CRC is not valid!", 60, PRIVATE); delay(500);
      return;
    }
  }

  ds.reset_search();
  Particle.publish("OneWire", "No more addresses!", 60, PRIVATE); delay(500);
  return;
}




// T-BUS collecting temperatures function => Reviewed by Grok!
void getTemperatures(int select)
{
  static unsigned long conversionStart = 0;
  static bool conversionRequested = false;

  // Stap 1: Start conversie (alleen 1x per interval)
  if (!conversionRequested && (millis() - getTemperaturesLastTime > getTemperaturesInterval)) {
    ds.reset();
    ds.skip();
    ds.write(0x44, 0);  // Start temperatuurconversie
    conversionStart = millis();
    conversionRequested = true;
    return;
  }

  // Stap 2: Wacht 1 sec, dan lezen
  if (conversionRequested && (millis() - conversionStart >= 1000)) {
    ds.reset();
    conversionRequested = false;
    getTemperaturesLastTime = millis();

    for (int i = 0; i < sizeof(temps)/sizeof(temps[0]); i++) {
      ds.select(addrs0[i]);
      ds.write(0xBE, 0);
      byte scratchpadData[9];
      for (int j = 0; j < 9; j++) {
        scratchpadData[j] = ds.read();
      }
      byte currentCRC = OneWire::crc8(scratchpadData, 8);
      ds.reset();

      if (currentCRC != scratchpadData[8]) {
        char msg[64];
        if (Time.now() - tmStamp[i] > 3600UL) {
          snprintf(msg, sizeof(msg), "Sensor Timeout on sensor: %d", i);
        } else {
          snprintf(msg, sizeof(msg), "Bad reading on Sensor: %d", i);
        }

        if (Particle.connected()) { // Voorkomt queue-opbouw bij WiFi-drops
          Particle.publish("Alerts", msg, 60, PRIVATE);
        }

        crcErrorCount[i]++;
        continue;
      }

      tmStamp[i] = Time.now();
      int16_t raw = (scratchpadData[1] << 8) | scratchpadData[0];
      celsius = (double)raw * 0.0625;
      *temps[i] = celsius;
    }

    // Bouw CRC JSON
    strcpy(crcErrorJSON, "{\"errorCount\":[");
    for (int i = 0; i < 6; i++) {
      char buf[8];
      itoa(crcErrorCount[i], buf, 10);
      strcat(crcErrorJSON, buf);
      if (i < 5) strcat(crcErrorJSON, ",");
    }
    strcat(crcErrorJSON, "]}");
  }
}


// Particle.function to remote control manually. Can also be called from the loop(): ex = manual("Report");
int manual(String command)
{
  command.trim();
  command.toLowerCase();

  if (command == "report")
  {
    wifiRSSI = WiFi.RSSI();
    sprintf(str, "wifiRSSI: %d", wifiRSSI);
    Particle.publish("Status-ROOM", str, PRIVATE); delay(500);

    memPERCENT = (freemem * 100) / 82944;
    sprintf(str, "FreeMem: %d  Mem:%d%%", freemem, memPERCENT);
    Particle.publish("Status-ROOM", str, PRIVATE); delay(500);

    Particle.publish("Solar", JSON_temperat, PRIVATE); delay(500);

    discoverOneWireDevices();

    return 1000;
  }

  if (command == "on")
  {
    digitalWrite(relayPin, LOW);
    relayState = true;
    relay = 1;
    pwmValue = 255;
    analogWrite(pwmPin, 255);

    Particle.publish("Solar", "Pump MANUAL ON (PWM=255)", PRIVATE);
    return 255;
  }

  if (command == "off")
  {
    digitalWrite(relayPin, HIGH);
    relayState = false;
    relay = 0;
    pwmValue = 0;
    analogWrite(pwmPin, 0);

    Particle.publish("Solar", "Pump MANUAL OFF", PRIVATE);
    return 0;
  }

  if (command == "reset")
  {
    System.reset();
    return -10000;
  }

  // ===== PWM direct instellen (0...255) Command = "200" => Draait 200 en relay ON. "0" = relay off
  int pwm = command.toInt();

  if (command == String(pwm) && pwm >= 0 && pwm <= 255)
  {
    pwmValue = pwm;

    if (pwm == 0)
    {
      digitalWrite(relayPin, HIGH);
      relayState = false;
      relay = 0;
    }
    else
    {
      digitalWrite(relayPin, LOW);
      relayState = true;
      relay = 1;
    }

    analogWrite(pwmPin, pwm);

    sprintf(str, "Pump MANUAL PWM=%d", pwm);
    Particle.publish("Solar", str, PRIVATE);

    return pwm;
  }

  Particle.publish("Solar", command, PRIVATE);
  return -1;
}

