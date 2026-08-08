/* S-ECO_SOLAR.ino = Energy_Monitor + SOLAR Pump controller for the "ECO-Boiler" Photon in the boiler room.

Versions:
- 8aug26 (PID24): Correctie op een onbedoeld neveneffect van 6aug26v2. Een echt koude ochtend (dT start op -47°C) toonde dat VROEGSTART's bootstrap-modus (eerste start van de dag) om de 5 minuten voortijdig gestopt en herstart werd - hortend, met meerdere volledige STOP-cycli vóór REGIME ooit bereikt werd. Oorzaak: v2 liet ELKE VROEGSTART (dus ook bootstrap) het STOP-vangnet delen én liet "dT<=0" de PWM sowieso naar de bodem dwingen. Dat was terecht bedoeld voor de dag-evenwicht-modus (waar het risico is: te sterk vertrekken op een mogelijk verouderde waarde), maar bootstrap heeft dat risico niet - haar PWM is per definitie al voorzichtig, evenredig met de actuele, live gemeten gradiënt. Bij een koude start blijft dT sowieso lang negatief terwijl de gradiënt zelf gezond is - dat hoort zo, het is geen storing. Fix: de dT<=0-regel en het gedeelde STOP-vangnet gelden voortaan enkel nog voor de evenwicht-modus; bootstrap is terug uitgesloten, zoals in PID19-21, en vertrouwt op haar eigen bestaande vangnetten (3 min stabiliteit, 40 min max). Zelf getest: een gereproduceerde koude-ochtendstart (dT begint op -40°C) gaf met de oude logica 2 volledige STOPs in de eerste 90 minuten; met de fix nul, en een vlot doorklimmende PWM (Claude)
- 8aug26 (PID23): Kp verlaagd van 8.0 naar 6.0 - geïsoleerde wijziging, verder niets aangepast. Aanleiding: data van 7aug26 toonde herhaalde, kortstondige REGIME-overshoots bij een voorbijtrekkende zonneflits (fout tot >4°C, PWM tot 122) - telkens veroorzaakt door de P-term (Kp x fout), niet door I of D (eigen test: bij een stevige flits leverde P alleen al ~35 bij, tegenover ~7 voor I en ~3 voor D samen - D draagt nauwelijks bij omdat dT_gefilterd zelf al een EMA is). Eigen simulatie (10 flits-varianten, 8 volledige wolkendagen): Kp=6.0 verlaagt de piek-PWM consistent met 5-15%, zonder de REGIME-steady-state-kwaliteit merkbaar te raken (% tijd binnen 1,5°C en binnen 0,5°C van doel bleef nagenoeg identiek over 8 dagen). Eerlijke kanttekening: dit is een echte fysische afweging, geen bug - bij een stevige flits piekt dT zelf iets hoger (gemiddeld +0,25°C in eigen test) omdat de pomp bewust iets minder fel reageert. Bewust gekozen boven twee alternatieven die wél negatieve neveneffecten bleken te hebben: een feedforward-basis (dag-evenwicht i.p.v. PWM_MIN als vertrekpunt) verlaagde dT-pieken wel, maar verhóógde de PWM-pieken juist (want de basis komt bovenop dezelfde reactieve termen); een strengere REGIME-ramplimiet zou de pomp trager laten volgen op een echte zonsterkte, waardoor dT juist langer en hoger zou oplopen (Claude)
- 6aug26v2: Belangrijke correctie op de dag-evenwichts-PWM van hieronder (6aug26v1), vóór die ooit geüpload werd. Een eigen stress-test (boiler nog heet van een sterke periode, zon bij een nieuwe herstart net zwak) toonde dat v1 - die meteen op de volle evenwichts-PWM startte, met enkel een trage "-15 PWM/min als dT te negatief wordt"-klep - dT tot -30°C kon laten wegzakken vóór de klep bijbeende. v2 lost dit op met twee wijzigingen: (1) VROEGSTART start voortaan altijd voorzichtig op de bodem (EARLY_START_PWM_MIN), ongeacht de modus, en klimt pas ná een settle-venster van 2 minuten (de hete-plug is dan bekend voorbij) geleidelijk naar het evenwicht toe - nooit in één sprong; (2) VROEGSTART deelt voortaan het bestaande, beproefde STOP-vangnet (floorMinutes/STOP_PATIENCE_MIN) i.p.v. daarvan uitgesloten te zijn, dus een herstart die niet aanslaat (dT blijft ≤0) stopt binnen dezelfde 5 minuten die de rest van het systeem al als aanvaardbaar risico hanteert, i.p.v. tot 40 minuten te kunnen doorlopen zonder enig vangnet. Zelf opnieuw getest onder een realistischer fysica-model (dat ook echt energieverlies simuleert als de pomp draait terwijl Tsol kouder is dan BotH, niet enkel het optimistische "geen extractie"-model van eerder): t.o.v. de huidige, al-lopende live-gradiënt-aanpak (PID21) daalden de grote PWM-sprongen tijdens VROEGSTART met 60-75% (10-14 naar 2-5 per dag, over 5 synthetische wolkendagen), terwijl een misgelopen herstart nu voor het eerst een hard begrensd vangnet heeft dat PID21 nooit had (Claude)
- 6aug26v1 (nooit geüpload): eerste versie van de dag-evenwichts-PWM. Kernidee (van Opie): in plaats van bij élke VROEGSTART-herstart blind te reageren op een live-gradiënt die toch net na een stilstand vervuild is door het hete-plug-effect, onthoudt de sketch een traag bijgewerkte "evenwichts-PWM" - de PWM die de dag al bewees te werken tijdens stabiele REGIME-periodes - en start daar bij een nieuwe herstart direct op. Bleek bij eigen stress-test niet veilig genoeg (zie 6aug26v2 hierboven) (Claude)
- 4aug26: VROEGSTART-PWM wordt nu LIVE bijgestuurd - elke minuut herberekend uit de actuele Tsol-gradiënt (dezelfde lineaire formule als voorheen), i.p.v. één keer vastgelegd op het triggermoment. Reden: bij een aanhoudend snel stijgende zon bleef de vaste PWM van 3aug26 de collector niet bijbenen - dT liep dan ongebreideld op (tot >75°C geobserveerd op de installatie) vóór de stabiliteits-uitgang of het 40-minuten-vangnet ooit kon ingrijpen, met een fikse PWM-piek bij de REGIME-instap tot gevolg (tot PWM=224 geobserveerd). Met live bijsturing schaalt de PWM voortaan mee zodra de gradiënt hoog blijft, wat dT vanzelf begrensd houdt. Zelf getest in een aangepast simulatiescript (aanhoudend stijgend zon-scenario): dT-piek daalde van >75°C naar ~8°C, REGIME-instappiek van PWM 195-224 naar PWM 110. Stabiliteits-uitgang, drempels en vangnet blijven ongewijzigd - enkel de PWM-berekening zelf is aangepast (Claude)
- 3aug26: VROEGSTART verfijnd - PWM wordt nu lineair berekend uit de trigger-gradiënt (EARLY_START_PWM_MIN..MAX tussen EARLY_START_GRADIENT_MIN..REF), i.p.v. een vaste PWM. Onder EARLY_START_GRADIENT_MIN (0,5°C/min) geen VROEGSTART meer. De duur is niet langer vast, maar hangt af van dT-stabiliteit: net als bij OPWARMEN wacht VROEGSTART tot dT_gefilterd zelf EARLY_START_STABLE_MINUTES_NEEDED minuten na elkaar minder dan EARLY_START_STABLE_THRESHOLD schommelt, met EARLY_START_MAX_MINUTES als vangnet. Idee: pas als dT_gefilterd vlak is, is de hete-plug echt doorgespoeld en is het circuit op temperatuur - dat voorkomt de PWM-piek die de vaste 20-minutentimer gaf bij het ingaan van REGIME tegen een nog fors achterlopend dT_gefilterd (Claude)
- 2aug26: VROEGSTART toegevoegd als extra, preventieve starttrigger, náást de bestaande trickle-start (dT>0). Terwijl de pomp stilstaat, wordt nu ook de rauwe Tsol-gradiënt (niet dT, niet gefilterd) gevolgd: blijft die 3 minuten na elkaar boven 1,5°C/min, dan start de pomp preventief op een vaste PWM=40 gedurende 20 minuten (geen PID) - dit dient meteen als een natuurlijke dode-tijd-meting, relevant voor de aanhoudende ~9-11-minuten-slingering. Na die 20 minuten neemt de PID (REGIME) de eerstvolgende cyclus meteen over, zonder nog door de gewone OPWARMEN-fase te gaan (die heeft VROEGSTART in feite al gedaan). De klassieke trickle-start blijft ongewijzigd als vangnet bestaan voor een trage of wisselvallige ochtend waarin de gradiënt-drempel nooit gehaald wordt. D-kick-fix geldt ook tijdens VROEGSTART, en de AFBOUW-teller sluit VROEGSTART net als OPWARMEN uit (Claude)
- 1aug26: OPWARMEN vervangt de vaste OPSTART-ramp, n.a.v. de vaststelling dat na een goede dag (1u08-13u46 e.a. stabiel) de resterende slingering zich vooral 's ochtends vroeg voordeed - vermoedelijk niet enkel de sensor, maar het hele leidingcircuit dat nog koud is en pas geleidelijk mee opwarmt (grote dode tijd). Pomp start nu al bij dT>0 (DT_TRICKLE_START, nieuw) i.p.v. te wachten tot DT_START=3.0 (die drempel blijft enkel nog gelden voor de thermosifon-check). PWM klimt daarna traag en zelf-aanpassend: steil bij een snel stijgende dT, voorzichtig bij een vlakke, tot dT_gefilterd 3 minuten na elkaar stabiel is gebleven (het circuit wordt dan als opgewarmd beschouwd) - met een vaste bovengrens van 10 minuten als vangnet. De bestaande STOP-teller (floorMinutes) telt bewust niet mee tijdens deze fase, anders zou een verse trickle-start zichzelf kunnen afbreken. D-kick-fix en anti-windup blijven op dezelfde manier van toepassing als voorheen tijdens de ramp (Claude)
- 31jul26b: PWM_MIN terug opgetrokken van 15 naar 60 (17jul26-waarde) - test van de dode-tijd-hypothese. Vier verschillende PID-instellingen (Kp/Ki/Kd: 3/0,15/0 · 4/0,15/1,2 · 6/0,5/1,2 · 8/0,6/1,2) gaven allemaal hetzelfde ~9-11-minuten-slingerpatroon, wat erop wijst dat de oorzaak niet in de PID-afstelling zit maar in een vertraging (dead time) tussen het gebeuren in de collector en het voelen ervan bij de sensor. Bij een lage doorstroming (PWM 15-30, sinds 28jul26 mogelijk) beweegt het water trager door de leiding, wat die vertraging vergroot - een klassieke oorzaak van dood-tijd-oscillatie die geen enkele PID-instelling kan wegregelen. Op 17jul26 zakte PWM nooit onder 60, vandaar de acht uur lange, stabiele werking toen. Dit kost het voordeel van de zuinige lage-PWM-werking van 28-29jul26, maar test een fundamenteel andere hypothese dan de tot nu toe geprobeerde gain-aanpassingen (Claude)
- 31jul26: Volledig terug naar het bewezen 17jul26-werkpunt, niet enkel de gains: DT_TARGET 1,8→2,5, DT_START 2,0→3,0, Kp 6→8, Ki 0,5→0,6 (Kd blijft 1,2). Een volledige dag data (10u-16u, 23 herhalingen van dezelfde ~10-minuten-cyclus) toonde dat de PID-verzwakking van 30jul26 het probleem niet oploste - het patroon herhaalde zich ook middenin aaneengesloten REGIME-periodes, niet enkel na een herstart, wat wijst op een structureel te krap werkpunt rond een DT_TARGET/DT_HARD_STOP die te dicht bij elkaar liggen, niet enkel op de PID-sterkte. Daarnaast de D-kick-bug hier alsnog gefixt (pidPrevError bevroor tijdens de OPSTART-ramp) - eerder al gevonden en gefixt in de simulator, maar nooit teruggeport naar deze echte sketch (Claude)
- 30jul26: PID-gains dichter bij de oorspronkelijke, bewezen 17jul26-instelling gebracht: Kp 4→6, Ki 0,15→0,5 (Kd blijft 1,2). Data toonde dat de verzwakking van 28jul26 (Kp 8→3/4, Ki 0,6→0,15) niet de eigenlijke overshoot-oorzaak wegnam (die bleek de vaste Tsun>75-override en de "hete-plug"-transiënt na herstart te zijn, ondertussen apart aangepakt), maar wél de strakke vergrendeling rond DT_TARGET kapotmaakte die op 17jul26 acht uur lang standhield (dT 2,3-2,7°C). Vooral Ki was te zwak om kleine afwijkingen actief terug te duwen, vandaar de trage "ademhaling" (dT dreef traag weg tot 15-20°C en terug) i.p.v. een echte lock. Kp niet meteen terug naar de volle 8, om niet te overdrijven samen met de ondertussen toegevoegde Kd=1,2 (die er op 17jul26 nog niet was) (Claude)
- 29jul26c: De vaste "Tsun>75°C → PWM=180"-override verwijderd. Een volledige dag data toonde een griezelig regelmatige zaagtand van precies 11 minuten, telkens crashend exact op het moment dat Tsol de 75°C overschreed: de PID volgde daarvoor keurig geleidelijk (bv. PWM 30→74 terwijl dT van 4→14°C steeg), maar de vaste override sloeg dan in één klap naar 180 - een sprong volledig losgekoppeld van wat de PID net aan het opbouwen was, wat de collector telkens liet crashen. Enkel de échte noodstop (Tsun>=90°C → PWM=255) blijft als vaste override staan; de PID regelt nu ook het hele bereik tussen 75-90°C zelf (Claude)
- 29jul26b: ROOT CAUSE gevonden voor de aanhoudende PWM-pendeling: niet de PID-gains (die van 29jul26a waren correct), maar de PWM_MAX_DELTA_PER_MIN-veiligheidslimiet (60/min) die tijdens REGIME de PID's eigen, correct berekende snellere respons afremde. Data toonde exacte +60-sprongen (40→100, 85→145) terwijl de PID intussen al veel hoger wilde (cmd=168, 145) - dT liep intussen ongestoord door tot 35°C. Verhoogd naar 150/min: de opstartfase gebruikt sowieso haar eigen aparte trage ramp (10/min) en bindt hier nooit, Kd dempt de invoer al via dTFiltered (Claude)
- 29jul26: PID-gains bijgesteld na een felle-ochtendtest: Kp 3→4 (te traag om een snel opkomende zon bij te benen - dT liep tot 40°C op vooraleer de Tsun>75-override ingreep) en Kd 0→1.2 toegevoegd (reageert op de sNELHEID van de verandering, niet enkel de grootte - vangt een snelle stijging vroeger op dan Kp alleen kan, zonder de jitter-gevoeligheid die Kd oorspronkelijk deed uitschakelen, want hij werkt op dTFiltered, niet de ruwe dT). Ook een logging-bug in de shadow gefixt (comment toonde "fout=0.0" tijdens de Tsun>75-override) (Claude)
- 28jul26: FUNDAMENTELE HERBOUW naar 5 fasen, gebaseerd op een reële hardwaretest (PWM=10 bevestigd bruikbaar): (1) OPSTART is nu een echte open-lus ramp (20→30→40 over 3'), niet langer een PID-uitvoer die enkel begrensd werd - vermijdt dat de PID al vanaf de eerste seconde reageert op de ruizige "hete-plug"-transiënt. (2) PWM_MIN drastisch verlaagd naar 15 (cruise-bodem, apart van de opstart-ramp). (3) PID fors rustiger afgesteld (Kp 8→3, Ki 0.6→0.15, Kd 3→0) - de vorige agressieve gains waren de eigenlijke bron van de meeste overshoots die we tot nu toe bestreden met lapmiddelen. (4) DT_TARGET verlaagd naar 1.8°C en DT_START naar 2.0°C - Tsol moet enkel nog kort boven de boilertemperatuur blijven, niet een vaste marge. (5) STOP herdacht als laatste redmiddel: de relay schakelt pas uit nadat de PID al minstens 5' onafgebroken op PWM_MIN staat zonder herstel (nieuwe fase AFBOUW), i.p.v. bij één enkele meting onder een vaste drempel. Nachtblokkering, thermosifon en oververhitting ongewijzigd (Claude, na overleg met ChatGPT-analyse van de gebruiker)
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

// PID-regelaar voor pompsnelheid, 14jul26b - rustiger afgesteld op 28jul26
// na een reële hardwaretest die bevestigde dat de pomp betrouwbaar draait tot PWM=15
double DT_TARGET = 2.5;          // gewenste dT-evenwicht: Tsol moet net boven de boiler blijven
double DT_TRICKLE_START = 0.0;   // 1aug26: nieuwe, vroege starttrigger - pomp start al bij dT>0
                                  // (i.p.v. te wachten tot de kloof groot is), traag via OPWARMEN
double DT_START  = 3.0;          // 1aug26: stuurt de pompstart niet meer aan, enkel nog de
                                  // thermosifon-veiligheid hieronder (ongewijzigd gebleven drempel)
double DT_HARD_STOP = 0.0;       // "geen winst meer"-drempel, enkel relevant i.c.m. PWM_MIN hieronder
double Kp = 6.0;
double Ki = 0.6;
double Kd = 1.2;
double pidIntegral = 0;
double pidPrevError = 0;
const double PID_I_MAX = 50.0;   // anti-windup clamp op de integraalterm
double PWM_MIN = 60;             // cruise-bodem (31jul26: terug naar 60, test van de dode-tijd-hypothese - zie versiehistoriek)
double PWM_MAX_DELTA_PER_MIN = 150;  // extra veiligheidslimiet: max PWM-verandering per minuut, bovenop de PID (29jul26b: 60→150, was de facto de bottleneck tijdens REGIME)
unsigned long pumpStartTime = 0;
unsigned long lastPidTime = 0;
unsigned long pumpStopTime = 0;   // 6aug26: wanneer de pomp laatst stopte - voor de dag-evenwicht-reset

// dT-filter (EMA) tegen de "hete-plug"-piek bij pompstart, 15jul26
double dTFiltered = 0;
bool dTFilterInit = false;
const double DT_FILTER_ALPHA = 0.3;  // per minuut; lager = trager/gladder, hoger = reageert sneller

// OPWARMEN (1aug26, vervangt de vaste OPSTART-ramp van 28jul26): een grote dode
// tijd bleek niet enkel bij de sensor te zitten, maar in het hele leidingcircuit -
// tot alles op temperatuur is, blijft het slingeren ongeacht de PID-instelling.
// PWM klimt daarom traag en zelf-aanpassend: steil bij een felle dT-stijging,
// voorzichtig bij een vlakke. Stopt zodra dT_gefilterd een paar minuten stabiel
// is gebleven (het circuit wordt dan als "opgewarmd" beschouwd) - met een vaste
// bovengrens als vangnet, zodat dit nooit blijft hangen.
const double WARMUP_PWM_START = 20;              // beginwaarde, zelfde als de oude ramp
const double WARMUP_STEP_MIN = 4;                // PWM-toename/min bij een vlakke dT-gradiënt
const double WARMUP_STEP_MAX = 15;               // PWM-toename/min bij een steile dT-gradiënt
const double WARMUP_GRADIENT_REF = 2.0;          // °C/min waarbij de maximale stap al bereikt wordt
const double WARMUP_PWM_CEILING = 90;            // veilig plafond tijdens het opwarmen
const double WARMUP_STABLE_THRESHOLD = 0.3;      // °C/min, onder deze grens telt een minuut als 'stabiel'
const int WARMUP_STABLE_MINUTES_NEEDED = 3;      // zoveel opeenvolgende stabiele minuten = opgewarmd
const double WARMUP_MAX_MINUTES = 10.0;          // vangnet: hoe dan ook uiterlijk na zoveel minuten stoppen
bool warmupActive = false;
double warmupPwmTarget = 0;
double prevDTForGradient = 0;
int warmupStableMinutes = 0;

// VROEGSTART (2aug26 basis, 3aug26: lineaire PWM-formule + stabiliteits-duur)
// Terwijl de pomp stilstaat volgen we de rauwe Tsol-gradiënt (niet dT) - blijft
// die een paar minuten na elkaar boven de ondergrens, dan starten we preventief
// op een PWM die lineair meeschaalt met hoe steil de gradiënt op dat moment is
// (geen PID). De duur is niet vast, maar wacht tot dT_gefilterd zelf stabiel
// is - pas dan is de hete-plug echt doorgespoeld - met een vast vangnet.
const double EARLY_START_GRADIENT_MIN = 0.5;     // °C/min - onder deze drempel geen VROEGSTART
const double EARLY_START_GRADIENT_REF = 3.0;     // °C/min - hierboven verzadigt de PWM al op zijn maximum
const double EARLY_START_PWM_MIN = 30;           // PWM bij exact EARLY_START_GRADIENT_MIN
const double EARLY_START_PWM_MAX = 100;          // PWM bij EARLY_START_GRADIENT_REF en hoger
const double EARLY_START_STABLE_THRESHOLD = 0.3; // °C - zelfde drempel als OPWARMEN
const int EARLY_START_STABLE_MINUTES_NEEDED = 3; // zelfde venster als OPWARMEN
const double EARLY_START_MAX_MINUTES = 40.0;     // vangnet als dT nooit stabiliseert
const int GRADIENT_CONFIRM_MINUTES = 3;          // zoveel minuten na elkaar boven de drempel
bool earlyStartActive = false;
bool earlyStartTriggered = false;      // gezet zodra de gradiënt-drempel gehaald is, vóór de pomp start
double earlyStartTriggerGradient = 0;  // instantane gradiënt op het moment van triggeren - eerste PWM-schatting
unsigned long earlyStartBeginTime = 0;
double earlyStartPwmTarget = 0;        // live berekende PWM, elke minuut bijgewerkt (4aug26)
double earlyStartPrevTsun = 0;         // Tsun van de vorige minuut, voor de live-gradiënt tijdens VROEGSTART
double earlyStartCurrentGradient = 0;  // actuele gradiënt, elke minuut herberekend
double prevDTForEarlyStart = 0;
int earlyStartStableMinutes = 0;
double prevTsunForGradient = 0;
bool tsunGradientTrackInit = false;
int tsunGradientStableMinutes = 0;

// Dag-evenwichts-PWM (6aug26, PID22 v2): een traag bijgewerkte schatting van
// "de PWM die vandaag werkt", opgebouwd tijdens stabiele REGIME-periodes.
// Is dit evenwicht al gekend bij een nieuwe VROEGSTART, dan start de pomp
// er NIET meteen op, maar begint voorzichtig op de bodem en klimt pas na een
// settle-venster (zie EARLY_START_SETTLE_MINUTES hieronder) geleidelijk naar
// dat evenwicht toe - i.p.v. te reageren op een live-gradiënt die toch net
// na een stilstand vervuild is door het hete-plug-effect. Wordt vergeten na
// een lange stilstand (nieuwe dag).
//
// BELANGRIJK, geleerd uit een eigen stress-test vóór dit uitgeleverd werd:
// een eerste versie liet de pomp bij een gekend evenwicht meteen op de volle
// evenwichts-PWM starten, met een simpele "-15 PWM/min als dT te negatief
// wordt"-veiligheidsklep. Bij een test waarbij de boiler nog heet was van een
// eerdere sterke periode maar de zon bij een nieuwe herstart net zwak bleek,
// bleef dT diep negatief hangen (tot -30°C) - de klep reageerde simpelweg te
// traag t.o.v. hoe snel dat kon mislopen. De huidige versie lost dit op met
// twee wijzigingen: (1) VROEGSTART start voortaan altijd voorzichtig op de
// bodem, ongeacht de modus; (2) VROEGSTART deelt voortaan het bestaande,
// beproefde STOP-vangnet (floorMinutes/STOP_PATIENCE_MIN) i.p.v. daarvan
// uitgesloten te zijn - een slechte herstart wordt dus, net als een slechte
// REGIME-periode, binnen 5 minuten gestopt, in plaats van tot 40 minuten
// door te kunnen lopen. Zelf getest: dit begrenst een misgelopen herstart tot
// exact hetzelfde worst-case risico dat de rest van het systeem al accepteert.
const double EQUILIBRIUM_EMA_ALPHA = 0.05;         // hoe traag het evenwicht meebeweegt
const double EQUILIBRIUM_DT_BAND = 1.0;            // enkel bijleren als dT dicht bij doel zit
const double EQUILIBRIUM_CREEP_DT_HIGH = 3.0;      // marge boven DT_TARGET: licht optrekken
const double EQUILIBRIUM_CREEP_PWM_STEP = 5;
const double EQUILIBRIUM_RESET_STAGNANT_MIN = 240.0; // na zo lang stilstand: vergeet het evenwicht
const double EARLY_START_SETTLE_MINUTES = 2.0;     // hete-plug is dan bekend voorbij (2 min, zie spikeRemaining)
const double EARLY_START_CLIMB_STEP = 10;          // hoeveel PWM/min klimmen naar het evenwicht toe
double equilibriumPwm = 0;
bool equilibriumKnown = false;
bool earlyStartUsingEquilibrium = false;   // welke modus deze specifieke VROEGSTART-episode gebruikt

double floorMinutes = 0;                     // hoeveel minuten aan een stuk op PWM_MIN zonder herstel
const double STOP_PATIENCE_MIN = 5.0;        // pas na zoveel minuten op de bodem schakelt de relay echt uit


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
    floorMinutes = 0;
    pidIntegral = 0; pidPrevError = 0;
    warmupActive = false;
    earlyStartActive = false;
    earlyStartStableMinutes = 0;
    tsunGradientTrackInit = false;
    tsunGradientStableMinutes = 0;
    lastPidTime = 0;
    return;
  }

  // dT-filter (EMA): dempt de korte, hevige piek van de "hete plug" bij pompstart
  if (!dTFilterInit) {
    dTFiltered = dT;
    dTFilterInit = true;
  } else {
    dTFiltered += DT_FILTER_ALPHA * (dT - dTFiltered);
  }

  unsigned long nowMs = millis();
  double dtMin = 1.0;  // tijd sinds vorige cyclus, in minuten (enkel zinvol als de pomp al draaide)
  if (relayState && lastPidTime > 0) {
    dtMin = (double)(nowMs - lastPidTime) / 60000.0;
    if (dtMin <= 0) dtMin = 1.0;
  }

  // --- STOP als laatste redmiddel (28jul26) ---
  // De relay schakelt niet meer uit bij één enkele meting onder een vaste drempel.
  // In plaats daarvan: zolang de PID op de PWM-bodem (PWM_MIN) staat ÉN dT niet meer
  // wint, loopt een teller op; pas na STOP_PATIENCE_MIN minuten aan een stuk zonder
  // herstel stopt de pomp echt. Zo krijgt elke tijdelijke dip de kans om vanzelf te
  // herstellen zonder de pomp af te breken (en dus een nieuwe "hete-plug"-herstart
  // te veroorzaken).
  bool shouldBeOn;
  bool atFloor = false;
  if (relayState) {
    // Tijdens OPWARMEN (1aug26) staat de PWM bewust laag/onder PWM_MIN - dat mag
    // niet meetellen als "op de bodem vastzitten", anders zou de stop-teller een
    // verse trickle-start meteen weer kunnen afbreken (OPWARMEN begrenst zichzelf
    // al via WARMUP_MAX_MINUTES).
    //
    // VROEGSTART: sinds 6aug26 v2 deelde ELKE VROEGSTART het STOP-vangnet, om de
    // dag-evenwicht-modus te beschermen tegen een te hoog vertrekpunt. Bleek op
    // 8aug26 een onbedoeld neveneffect te hebben op de BOOTSTRAP-modus (eerste
    // start van de dag): bij een koude ochtend (dT start diep negatief, bv.
    // -47°C vandaag) duurt het normaal een hele tijd vóór dT positief wordt -
    // dat is geen storing, dat hoort zo bij VROEGSTART. Maar doordat bootstrap
    // ook aan de STOP-teller hing, werd de pomp om de 5 minuten voortijdig
    // gestopt en herstart, wat een hortende, herhaaldelijk onderbroken opstart
    // gaf. Daarom (8aug26, PID24): enkel de EVENWICHT-modus van VROEGSTART blijft
    // het STOP-vangnet delen (daar was het risico - te sterk vertrekken op een
    // mogelijk verouderde waarde - ook effectief het probleem); de bootstrap-
    // modus is terug uitgesloten, zoals in PID19-21, en vertrouwt op haar eigen,
    // altijd al bestaande vangnetten (3 min stabiliteit, of 40 min max).
    bool bootstrapVroegstart = earlyStartActive && !earlyStartUsingEquilibrium;
    atFloor = (!warmupActive) && (!bootstrapVroegstart) && (pwmValue <= PWM_MIN + 2.0);

    // Dag-evenwichts-PWM bijleren (6aug26): enkel tijdens een echt stabiele
    // REGIME - niet tijdens OPWARMEN, VROEGSTART of AFBOUW (die zijn per
    // definitie nog niet "op punt"). Traag EMA, zodat een kortstondige dip
    // het evenwicht niet meteen verstoort, maar het wél binnen een uur kan
    // meegroeien als de zon écht sterker/zwakker wordt.
    if (!warmupActive && !earlyStartActive && !atFloor && fabs(dTFiltered - DT_TARGET) < EQUILIBRIUM_DT_BAND) {
      if (!equilibriumKnown) {
        equilibriumPwm = pwmValue;
        equilibriumKnown = true;
      } else {
        equilibriumPwm += EQUILIBRIUM_EMA_ALPHA * (pwmValue - equilibriumPwm);
      }
    }

    bool tooLow = (dTFiltered <= DT_HARD_STOP);
    if (atFloor && tooLow) {
      floorMinutes += dtMin;
    } else {
      floorMinutes = 0;
    }
    shouldBeOn = (floorMinutes < STOP_PATIENCE_MIN);
  } else {
    // 1aug26: vroege trickle-start bij dT>0 i.p.v. te wachten tot DT_START -
    // de OPWARMEN-fase hieronder vangt de rest rustig op
    shouldBeOn = (dTFiltered > DT_TRICKLE_START);
    floorMinutes = 0;

    // VROEGSTART-trigger (2aug26): terwijl de pomp stilstaat volgen we de
    // rauwe Tsol-gradiënt (niet dT, niet gefilterd). Blijft die
    // GRADIENT_CONFIRM_MINUTES minuten na elkaar boven de drempel, dan
    // starten we preventief - nog vóór dT positief is. De teller blijft
    // bewust ook doorlopen over opeenvolgende OFF-cycli heen (geen reset
    // hieronder bij "blijft uit"), enkel een echte pompstart of een terugval
    // onder de drempel zet hem terug op nul.
    if (!tsunGradientTrackInit) {
      prevTsunForGradient = Tsun;
      tsunGradientTrackInit = true;
    } else {
      double tsunGradient = Tsun - prevTsunForGradient;
      prevTsunForGradient = Tsun;
      if (tsunGradient > EARLY_START_GRADIENT_MIN) {
        tsunGradientStableMinutes++;
      } else {
        tsunGradientStableMinutes = 0;
      }
      if (tsunGradientStableMinutes >= GRADIENT_CONFIRM_MINUTES) {
        earlyStartTriggered = true;
        earlyStartTriggerGradient = tsunGradient;   // instantane gradiënt bij triggering, voedt de PWM-formule
        shouldBeOn = true;
      }
    }
  }

  // Thermosifon blokkeren: voorkomt terugstroming/afkoeling bij lage Tsun (directe
  // override). Gebruikt nog steeds DT_START (3.0) - die drempel bleef hiervoor
  // bestaan, enkel de pompstart zelf verhuisde naar DT_TRICKLE_START.
  if (dTFiltered > DT_START && Tsun < 22.0) {
    shouldBeOn = false;
    floorMinutes = 0;
    sprintf(str, "Pump OFF - Thermosifon (Tsun=%.1fC)", Tsun);
  }

  // Oververhitting: forceert AAN + PWM direct naar max - dé bescherming tegen
  // een oplopende collectortemperatuur, overschrijft alle andere logica
  bool overheat = (Tsun >= 90.0);
  if (overheat) {
    shouldBeOn = true;
    floorMinutes = 0;
  }

  if (!shouldBeOn) {
    if (relayState) {
      sprintf(str, "Pump OFF - dT gefilterd=%.1fC, geen herstel na %.0f min op PWM-bodem", dTFiltered, STOP_PATIENCE_MIN);
      pumpStopTime = nowMs;   // onthouden wanneer de pomp stopte (6aug26)
    } else {
      if (equilibriumKnown) {
        sprintf(str, "Pump OFF - dT=%.1fC, wacht op start (Tsol-gradient stabiel %d/%d min, dag-evenwicht=%d)", dTFiltered, tsunGradientStableMinutes, GRADIENT_CONFIRM_MINUTES, (int)round(equilibriumPwm));
      } else {
        sprintf(str, "Pump OFF - dT=%.1fC, wacht op start (Tsol-gradient stabiel %d/%d min, nog geen dag-evenwicht)", dTFiltered, tsunGradientStableMinutes, GRADIENT_CONFIRM_MINUTES);
      }
    }
    // Dag-evenwicht vergeten na een lange stilstand - vangnet voor een nieuwe
    // dag, zodat morgenochtend niet start op gisterens PWM (6aug26)
    if (equilibriumKnown && pumpStopTime > 0 &&
        (double)(nowMs - pumpStopTime) / 60000.0 > EQUILIBRIUM_RESET_STAGNANT_MIN) {
      equilibriumKnown = false;
    }
    digitalWrite(relayPin, HIGH);
    relayState = false; relay = 0;
    pwmValue = 0;
    pidIntegral = 0;      // reset zodat er geen windup optreedt terwijl de pomp stilstaat
    pidPrevError = 0;
    floorMinutes = 0;
    warmupActive = false;    // volgende start begint weer vers
    earlyStartActive = false;
    earlyStartStableMinutes = 0;
    lastPidTime = 0;      // volgende start telt weer vers vanaf dtMin=1
    analogWrite(pwmPin, 0);
    return;
  }

  // --- Pomp is/wordt AAN ---
  bool justStarted = !relayState;

  if (justStarted) {
    pumpStartTime = nowMs;
    pidIntegral = 0;
    pidPrevError = dTFiltered - DT_TARGET;   // voorkomt een D-piek bij de allereerste stap
    floorMinutes = 0;
    tsunGradientTrackInit = false;    // volgende OFF-periode start weer vers
    tsunGradientStableMinutes = 0;

    if (earlyStartTriggered) {
      // Gestart via de Tsol-gradiënt, nog vóór dT positief werd - VROEGSTART
      // i.p.v. de normale OPWARMEN-ramp.
      earlyStartActive = true;
      earlyStartBeginTime = nowMs;
      warmupActive = false;
      prevDTForEarlyStart = dTFiltered;
      earlyStartStableMinutes = 0;
      earlyStartPrevTsun = Tsun;               // baseline voor de live-gradiënt
      earlyStartCurrentGradient = earlyStartTriggerGradient;

      if (equilibriumKnown) {
        // Dag-evenwicht gekend (6aug26 v2): start voorzichtig op de bodem,
        // en klim er pas ná het settle-venster geleidelijk naartoe (zie de
        // VROEGSTART-evaluatie verderop) - i.p.v. er direct op te springen,
        // wat bij een inmiddels te zwakke zon dT te hard kan laten wegzakken.
        earlyStartUsingEquilibrium = true;
        earlyStartPwmTarget = EARLY_START_PWM_MIN;
      } else {
        // Eerste start van de dag - nog geen evenwicht gekend, dus terug op
        // de live-gradiënt-bootstrap van 4aug26. PWM ligt lineair, tussen
        // EARLY_START_PWM_MIN en EARLY_START_PWM_MAX naargelang de gradiënt
        // tussen EARLY_START_GRADIENT_MIN en EARLY_START_GRADIENT_REF ligt,
        // en wordt elke minuut LIVE herberekend (zie verderop).
        earlyStartUsingEquilibrium = false;
        double frac = (earlyStartTriggerGradient - EARLY_START_GRADIENT_MIN) /
                      (EARLY_START_GRADIENT_REF - EARLY_START_GRADIENT_MIN);
        frac = constrain(frac, 0.0, 1.0);
        earlyStartPwmTarget = EARLY_START_PWM_MIN + frac * (EARLY_START_PWM_MAX - EARLY_START_PWM_MIN);
      }
    } else {
      earlyStartActive = false;
      warmupActive = true;
      warmupPwmTarget = WARMUP_PWM_START;
      warmupStableMinutes = 0;
      prevDTForGradient = dTFiltered;
    }
    earlyStartTriggered = false;   // verbruikt - klaar voor de volgende cyclus
  }
  lastPidTime = nowMs;

  digitalWrite(relayPin, LOW);
  relayState = true; relay = 1;

  // OPWARMEN (1aug26): evalueer, ná de allereerste minuut (die heeft nog geen
  // gradiënt om mee te vergelijken), of het circuit intussen stabiel genoeg is
  // om als "opgewarmd" te gelden - of anders hoe hard de PWM deze minuut mag
  // bijklimmen, evenredig met hoe steil dT_gefilterd nog stijgt.
  // VROEGSTART (2aug26 basis, 3aug26: lineaire formule, 4aug26: LIVE
  // bijsturing, 6aug26 v2: settle-venster + dag-evenwicht met geleidelijke
  // klim). De duur wacht in beide gevallen tot dT_gefilterd zelf stabiliseert
  // - net als bij OPWARMEN - met EARLY_START_MAX_MINUTES als vangnet.
  if (earlyStartActive && !justStarted) {
    earlyStartCurrentGradient = Tsun - earlyStartPrevTsun;
    earlyStartPrevTsun = Tsun;
    double minutesSinceStart = (double)(nowMs - earlyStartBeginTime) / 60000.0;

    // Settle-venster (6aug26 v2): de eerste EARLY_START_SETTLE_MINUTES na een
    // herstart is elke meting - in beide modi - potentieel nog vervuild door
    // het hete-plug-effect (zie spikeRemaining/spikeMagnitudeFor). Zolang dat
    // venster loopt, wordt er dus nog helemaal niets herberekend: de PWM
    // blijft op wat bij de start werd ingesteld (de bodem bij een gekend
    // evenwicht, of de trigger-gebaseerde schatting bij de live-bootstrap).
    if (minutesSinceStart > EARLY_START_SETTLE_MINUTES) {
      if (earlyStartUsingEquilibrium) {
        // Dag-evenwicht-modus (8aug26 PID24: dT<=0-veiligheid enkel hier, zie
        // hierboven bij atFloor voor de volledige uitleg waarom dit niet meer
        // voor bootstrap geldt). Werkt het niet (dT<=0), dan terug naar de
        // bodem - dit voedt meteen ook de floorMinutes/STOP-teller hierboven,
        // dus een herstart die zo blijft hangen, stopt vanzelf binnen
        // STOP_PATIENCE_MIN. Werkt het wél: klim geleidelijk naar het gekende
        // dag-evenwicht toe (nooit in één keer) - en licht daarboven uit als
        // dT ver boven het doel blijft hangen (de zon is dan sterker dan het
        // evenwicht nog veronderstelt).
        if (dTFiltered <= DT_HARD_STOP) {
          earlyStartPwmTarget = EARLY_START_PWM_MIN;
        } else if (earlyStartPwmTarget < equilibriumPwm) {
          earlyStartPwmTarget = fmin(equilibriumPwm, earlyStartPwmTarget + EARLY_START_CLIMB_STEP);
        } else if (dTFiltered > DT_TARGET + EQUILIBRIUM_CREEP_DT_HIGH) {
          earlyStartPwmTarget = fmin(EARLY_START_PWM_MAX, earlyStartPwmTarget + EQUILIBRIUM_CREEP_PWM_STEP);
        }
      } else {
        // Nog geen dag-evenwicht gekend (eerste start van de dag) - terug op
        // de live-gradiënt-bootstrap van 4aug26, ONVOORWAARDELIJK (8aug26
        // PID24: geen dT<=0-afkap meer hier - bij een koude ochtendstart blijft
        // dT normaal lang negatief terwijl de gradiënt zelf prima gezond is;
        // dat is geen storing, dat hoort zo bij VROEGSTART).
        double liveFrac = (earlyStartCurrentGradient - EARLY_START_GRADIENT_MIN) /
                           (EARLY_START_GRADIENT_REF - EARLY_START_GRADIENT_MIN);
        liveFrac = constrain(liveFrac, 0.0, 1.0);
        earlyStartPwmTarget = EARLY_START_PWM_MIN + liveFrac * (EARLY_START_PWM_MAX - EARLY_START_PWM_MIN);
      }
    }
    // Binnen het settle-venster zelf: earlyStartPwmTarget blijft ongewijzigd.

    double esGradient = dTFiltered - prevDTForEarlyStart;
    prevDTForEarlyStart = dTFiltered;
    double esAbsGradient = fabs(esGradient);

    if (esAbsGradient < EARLY_START_STABLE_THRESHOLD) {
      earlyStartStableMinutes++;
    } else {
      earlyStartStableMinutes = 0;
    }

    double minutesInEarlyStart = (double)(nowMs - earlyStartBeginTime) / 60000.0;
    if (earlyStartStableMinutes >= EARLY_START_STABLE_MINUTES_NEEDED || minutesInEarlyStart >= EARLY_START_MAX_MINUTES) {
      earlyStartActive = false;   // klaar - PID (REGIME) neemt deze cyclus al over
    }
  }

  if (warmupActive && !justStarted) {
    double gradient = dTFiltered - prevDTForGradient;
    prevDTForGradient = dTFiltered;
    double absGradient = (gradient < 0) ? -gradient : gradient;

    if (absGradient < WARMUP_STABLE_THRESHOLD) {
      warmupStableMinutes++;
    } else {
      warmupStableMinutes = 0;
    }

    double minutesInWarmup = (double)(nowMs - pumpStartTime) / 60000.0;
    if (warmupStableMinutes >= WARMUP_STABLE_MINUTES_NEEDED || minutesInWarmup >= WARMUP_MAX_MINUTES) {
      warmupActive = false;   // circuit als opgewarmd beschouwd - PID neemt deze cyclus al over
    } else {
      double step = WARMUP_STEP_MIN + (WARMUP_STEP_MAX - WARMUP_STEP_MIN) *
                    constrain(gradient / WARMUP_GRADIENT_REF, 0.0, 1.0);
      warmupPwmTarget = constrain(warmupPwmTarget + step, WARMUP_PWM_START, WARMUP_PWM_CEILING);
    }
  }

  double pwmDoel;

  if (overheat) {
    pwmDoel = 255;
  } else if (earlyStartActive) {
    pwmDoel = earlyStartPwmTarget;
    // D-kick-fix: pidPrevError blijft ook tijdens VROEGSTART meelopen, net
    // als tijdens OPWARMEN, zodat REGIME nadien tegen de écht-vorige minuut
    // vergelijkt.
    pidPrevError = dTFiltered - DT_TARGET;
  } else if (warmupActive) {
    pwmDoel = warmupPwmTarget;
    // D-kick-fix (31jul26): pidPrevError blijft ook tijdens OPWARMEN meelopen,
    // zodat REGIME nadien tegen de écht-vorige minuut vergelijkt, niet tegen
    // een fout van vóór de hele opwarmfase.
    pidPrevError = dTFiltered - DT_TARGET;
  } else {
    double error = dTFiltered - DT_TARGET;

    // Anti-windup: integraal bevriezen zolang de PWM toch al op de bodem
    // vastzit - anders stapelt de integraal zich nutteloos verder op.
    if (!atFloor) {
      pidIntegral = constrain(pidIntegral + error * dtMin, -PID_I_MAX, PID_I_MAX);
    }
    double derivative = (error - pidPrevError) / dtMin;
    pidPrevError = error;

    double pidOutput = (Kp * error) + (Ki * pidIntegral) + (Kd * derivative);
    pwmDoel = constrain(PWM_MIN + pidOutput, PWM_MIN, 255);
  }

  // Extra veiligheidslimiet bovenop de PID: max verandering per minuut
  if (overheat) {
    pwmValue = pwmDoel;
  } else if (pwmValue < pwmDoel) {
    pwmValue = min(pwmValue + PWM_MAX_DELTA_PER_MIN * dtMin, pwmDoel);
  } else if (pwmValue > pwmDoel) {
    pwmValue = max(pwmValue - PWM_MAX_DELTA_PER_MIN * dtMin, pwmDoel);
  }

  // Status: geeft altijd exact de actieve fase weer (kort en duidelijk, voor de
  // commentaarkolom in het logblad)
  if (overheat) {
    sprintf(str, "Pump ON (OVERVERHIT) - Tsun=%.1fC >= 90C, PWM=255", Tsun);
  } else if (earlyStartActive) {
    double minutesSinceStartForLog = (double)(nowMs - earlyStartBeginTime) / 60000.0;
    if (earlyStartUsingEquilibrium) {
      if (minutesSinceStartForLog <= EARLY_START_SETTLE_MINUTES) {
        sprintf(str, "Pump ON (VROEGSTART) - settle-venster (%.1f/%.0f min), PWM=%d, evenwicht=%d", minutesSinceStartForLog, EARLY_START_SETTLE_MINUTES, (int)round(earlyStartPwmTarget), (int)round(equilibriumPwm));
      } else {
        sprintf(str, "Pump ON (VROEGSTART) - klimt naar dag-evenwicht %d, nu PWM=%d (dT=%.1fC), stabiel %d/%d min (max %.0f min)", (int)round(equilibriumPwm), (int)round(earlyStartPwmTarget), dTFiltered, earlyStartStableMinutes, EARLY_START_STABLE_MINUTES_NEEDED, EARLY_START_MAX_MINUTES);
      }
    } else {
      if (minutesSinceStartForLog <= EARLY_START_SETTLE_MINUTES) {
        sprintf(str, "Pump ON (VROEGSTART) - settle-venster (%.1f/%.0f min), PWM=%d (bootstrap)", minutesSinceStartForLog, EARLY_START_SETTLE_MINUTES, (int)round(earlyStartPwmTarget));
      } else {
        sprintf(str, "Pump ON (VROEGSTART) - live gradient=%.1fC/min -> PWM=%d, stabiel %d/%d min (max %.0f min)", earlyStartCurrentGradient, (int)round(earlyStartPwmTarget), earlyStartStableMinutes, EARLY_START_STABLE_MINUTES_NEEDED, EARLY_START_MAX_MINUTES);
      }
    }
  } else if (warmupActive) {
    sprintf(str, "Pump ON (OPWARMEN) - dT=%.1fC, PWM=%d, stabiel %d/%d min", dTFiltered, (int)pwmValue, warmupStableMinutes, WARMUP_STABLE_MINUTES_NEEDED);
  } else if (atFloor) {
    sprintf(str, "Pump ON (AFBOUW) - dT gefilterd=%.1fC, PWM-bodem sinds %.1f/%.0f min", dTFiltered, floorMinutes, STOP_PATIENCE_MIN);
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
