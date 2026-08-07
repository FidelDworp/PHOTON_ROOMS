/*
 * ECO-boiler: Data ophalen + alert bij geen data (20 min)
 * 1 trigger: elke 10 minuten
 * Shadow mode: logica identiek aan solarPump() op de Photon (6aug26 v2)
 *   - 6aug26v2: Correctie op v1 (zie eronder), vóór v1 ooit geüpload werd.
 *     Een stress-test toonde dat v1 - meteen op de volle evenwichts-PWM
 *     starten met enkel een trage veiligheidsklep - dT tot -30°C kon laten
 *     wegzakken. v2: VROEGSTART start voortaan altijd voorzichtig op de
 *     bodem en klimt pas na een settle-venster van 2 minuten geleidelijk
 *     naar het evenwicht; en VROEGSTART deelt voortaan het bestaande,
 *     beproefde STOP-vangnet (floorMinutes/STOP_PATIENCE_MIN) i.p.v. daarvan
 *     uitgesloten te zijn. Zie de Photon-sketch voor de volledige uitleg en
 *     de eigen test-cijfers.
 *   - 6aug26v1 (nooit geüpload): eerste versie van de dag-evenwichts-PWM.
 *     Een traag bijgewerkte schatting van de PWM die vandaag al bewees te
 *     werken (tijdens stabiele REGIME), gebruikt bij een nieuwe VROEGSTART
 *     i.p.v. telkens opnieuw te reageren op een live-gradiënt die toch net
 *     na een stilstand vervuild is. Zonder gekend evenwicht (eerste start
 *     van de dag) blijft de live-gradiënt-bootstrap van 4aug26 gelden. Het
 *     evenwicht wordt vergeten na een lange stilstand (vangnet voor een
 *     nieuwe dag).
 *   - 4aug26: VROEGSTART-PWM wordt nu LIVE bijgestuurd - elke minuut
 *     herberekend uit de actuele Tsol-gradiënt, i.p.v. één keer vastgelegd
 *     op het triggermoment. Reden: bij een aanhoudend snel stijgende zon
 *     bleef de vaste PWM de collector niet bijbenen, waardoor dT ongebreideld
 *     kon oplopen (tot >75°C geobserveerd) vóór de stabiliteits-uitgang of het
 *     vangnet kon ingrijpen, met een fikse PWM-piek bij de REGIME-instap tot
 *     gevolg (tot PWM=224 geobserveerd). Zelf getest in een aangepast
 *     simulatiescript: dT-piek daalde van >75°C naar ~8°C, REGIME-instappiek
 *     van PWM 195-224 naar PWM 110. Stabiliteits-uitgang, drempels en vangnet
 *     blijven ongewijzigd.
 *   - 3aug26: VROEGSTART verfijnd - PWM wordt lineair berekend uit de
 *     trigger-gradiënt (EARLY_START_PWM_MIN..MAX tussen EARLY_START_GRADIENT_
 *     MIN..REF), i.p.v. een vaste PWM. Onder EARLY_START_GRADIENT_MIN
 *     (0,5°C/min) geen VROEGSTART meer. De duur hangt nu af van dT-
 *     stabiliteit i.p.v. een vaste timer: net als bij OPWARMEN wacht
 *     VROEGSTART tot dT_gefilterd zelf EARLY_START_STABLE_MINUTES_NEEDED
 *     minuten na elkaar minder dan EARLY_START_STABLE_THRESHOLD schommelt,
 *     met EARLY_START_MAX_MINUTES als vangnet.
 *   - 2aug26: VROEGSTART toegevoegd als extra, preventieve starttrigger,
 *     náást de bestaande trickle-start (dT>0). Terwijl de pomp stilstaat
 *     wordt nu ook de rauwe Tsol-gradiënt (niet dT) gevolgd: blijft die 3
 *     minuten na elkaar boven de drempel, dan start de pomp preventief (geen
 *     PID) - dit dient meteen als een natuurlijke dode-tijd-meting. De
 *     trickle-start blijft ongewijzigd als vangnet bestaan. D-kick-fix en de
 *     AFBOUW-uitsluiting gelden ook tijdens VROEGSTART, net als bij OPWARMEN.
 *     Fase-label "WACHT" onderscheidt "pomp blijft uit, wacht op start" van
 *     een echte "STOP"-overgang in de comment-kolom.
 *   - 1aug26: OPWARMEN vervangt de vaste OPSTART-ramp. Vermoeden: niet enkel
 *     de sensor, maar het hele leidingcircuit is nog koud bij een herstart en
 *     warmt pas geleidelijk mee op (grote dode tijd). Pomp start nu al bij
 *     dT>0 (DT_TRICKLE_START) i.p.v. te wachten tot DT_START=3.0 (die drempel
 *     geldt voortaan enkel nog voor de thermosifon-check). PWM klimt daarna
 *     traag en zelf-aanpassend mee met de dT-gradiënt (steil=sneller,
 *     vlak=trager), tot dT_gefilterd 3 minuten stabiel is gebleven (circuit
 *     "opgewarmd") - met 10 minuten als vaste bovengrens/vangnet. De
 *     stop-teller (floorMinutes) telt bewust niet mee tijdens deze fase.
 *   - 31jul26b: PWM_MIN 15→60 - test van de dode-tijd-hypothese. Vier
 *     verschillende PID-instellingen gaven allemaal hetzelfde ~9-11-minuten-
 *     slingerpatroon, wat wijst op een vertraging (dead time) tussen de
 *     collector en de sensor bij lage doorstroming, i.p.v. een PID-probleem.
 *   - 31jul26: Volledig terug naar het bewezen 17jul26-werkpunt: DT_TARGET
 *     1,8→2,5, DT_START 2,0→3,0, Kp 6→8, Ki 0,5→0,6 (Kd blijft 1,2). Een
 *     volledige dag data toonde dat enkel de gains bijstellen niet volstond -
 *     het patroon (23 herhalingen van dezelfde ~10-minuten-cyclus) herhaalde
 *     zich ook middenin REGIME, niet enkel na een herstart. Daarnaast de
 *     D-kick-bug hier alsnog gefixt (pidPrevError bevroor tijdens de
 *     OPSTART-ramp) - eerder al gevonden en gefixt in de simulator.
 *   - 30jul26: Kp 4→6, Ki 0,15→0,5 - dichter bij de oorspronkelijke, bewezen
 *     17jul26-instelling (die 8u lang dT strak op 2,3-2,7°C hield). De
 *     verzwakking van 28jul26 loste de echte overshoot-oorzaak niet op
 *     (dat bleek de vaste Tsun>75-override en de hete-plug-transiënt, apart
 *     aangepakt) maar maakte wel de vergrendeling rond DT_TARGET kapot.
 *   - 29jul26c: vaste "Tsun>75°C → PWM=180"-override verwijderd. Een volledige
 *     dag data toonde een zaagtand van exact 11 minuten, telkens crashend op
 *     het moment dat Tsol de 75°C overschreed - de PID volgde daarvoor prima
 *     geleidelijk, maar de vaste override sloeg dan los van de PID naar 180.
 *     Enkel de échte noodstop (Tsun>=90°C → 255) blijft als vaste override.
 *   - 29jul26b: PWM_MAX_DELTA_PER_MIN 60→150 - dit was de eigenlijke bottleneck
 *     achter de aanhoudende pendeling, niet de PID-gains (zie versiehistoriek
 *     in de Photon-sketch voor de volledige analyse: exacte +60-sprongen in de
 *     data terwijl de PID intussen een veel hogere waarde wilde).
 *   - 29jul26: Kp 3→4, Kd 0→1.2 (reageert nu ook op de snelheid van de
 *     verandering, niet enkel de grootte - vangt een snel opkomende zon
 *     eerder op). Loggingbug gefixt: "fout" in de REGIME-comment werd soms
 *     niet bijgewerkt tijdens de Tsun>75-override (toonde "fout=0.0"),
 *     nu altijd vers herberekend uit dTFiltered - DT_TARGET.
 *   - 5-FASEN-ONTWERP, gebaseerd op een reële hardwaretest (PWM=10 bevestigd
 *     bruikbaar):
 *       OPSTART : eerste 3' na pompstart, een echte open-lus ramp (20→30→40),
 *                 losgekoppeld van de PID zelf - reageert dus niet op de
 *                 ruizige "hete-plug"-transiënt
 *       REGIME  : PID rond DT_TARGET=1.8°C (Kp=3, Ki=0.15, Kd=0 - fors
 *                 rustiger dan voorheen), begrensd tussen PWM_MIN=15 en 255
 *       AFBOUW  : PWM zit al op de bodem (PWM_MIN) én dT wint niet meer -
 *                 een teller loopt op, de pomp blijft draaien
 *       STOP    : laatste redmiddel - pas als AFBOUW STOP_PATIENCE_MIN=5
 *                 minuten aanhoudt zonder herstel, schakelt de relay uit
 *       OVERVERHIT / THERMOSIFON: ongewijzigde veiligheidsoverrides
 *     dEQ wordt nog opgehaald en getoond (louter informatief), stuurt niets aan.
 *   - Anti-windup: PID-integraal bevroren tijdens OPSTART en tijdens AFBOUW
 *     (PWM al verzadigd op de bodem)
 *   - State (PID-integraal, vorige fout, dT-filter, PWM, floor-teller) blijft
 *     bewaard tussen runs via PropertiesService, met dt gebaseerd op de
 *     werkelijk verstreken tijd sinds de vorige run (belangrijk bij een
 *     10-minuten trigger-interval)
 * Kolomstructuur: A-R = bestaande data, S = commentaar
 */

function collectAndCheck() {
  var sheet = SpreadsheetApp.getActiveSpreadsheet().getActiveSheet();
  var deviceId = "310049000f47343432313031";
  var accessToken = "ba9d9e1ed9f70cc5db24de2db21764a3a3afe28b";
  var url = `https://api.particle.io/v1/devices/${deviceId}/JSON_temper?access_token=${accessToken}`;

  var lastRowBefore = sheet.getLastRow();
  var success = false;

  // --- Parameters (identiek aan solarPump() op de Photon, 1aug26) ---
  var DT_TARGET = 2.5;
  var DT_TRICKLE_START = 0.0;  // 1aug26: nieuwe, vroege starttrigger (i.p.v. DT_START)
  var DT_START = 3.0;          // 1aug26: enkel nog voor de thermosifon-check
  var DT_HARD_STOP = 0.0;
  var Kp = 6.0;
  var Ki = 0.6;
  var Kd = 1.2;
  var PID_I_MAX = 50.0;
  var PWM_MIN = 60;   // 31jul26: terug naar 60, test van de dode-tijd-hypothese
  var PWM_MAX_DELTA_PER_MIN = 150;  // extra veiligheidslimiet, max verandering/minuut bovenop de PID (29jul26b: 60→150)
  var DT_FILTER_ALPHA = 0.3;        // per minuut; lager = trager/gladder
  // OPWARMEN (1aug26, vervangt de vaste ramp) - zie de Photon-sketch voor de volledige uitleg
  var WARMUP_PWM_START = 20;
  var WARMUP_STEP_MIN = 4;
  var WARMUP_STEP_MAX = 15;
  var WARMUP_GRADIENT_REF = 2.0;
  var WARMUP_PWM_CEILING = 90;
  var WARMUP_STABLE_THRESHOLD = 0.3;
  var WARMUP_STABLE_MINUTES_NEEDED = 3;
  var WARMUP_MAX_MINUTES = 10.0;
  // VROEGSTART (2aug26 basis, 3aug26: lineaire PWM-formule + stabiliteits-duur)
  // zie de Photon-sketch voor de volledige uitleg
  var EARLY_START_GRADIENT_MIN = 0.5;
  var EARLY_START_GRADIENT_REF = 3.0;
  var EARLY_START_PWM_MIN = 30;
  var EARLY_START_PWM_MAX = 100;
  var EARLY_START_STABLE_THRESHOLD = 0.3;
  var EARLY_START_STABLE_MINUTES_NEEDED = 3;
  var EARLY_START_MAX_MINUTES = 40.0;
  var GRADIENT_CONFIRM_MINUTES = 3;

  // Dag-evenwichts-PWM (6aug26 v2) - zie de Photon-sketch voor de volledige
  // uitleg, inclusief waarom v1 (meteen op het evenwicht starten + een trage
  // veiligheidsklep) niet veilig genoeg bleek en gecorrigeerd is.
  var EQUILIBRIUM_EMA_ALPHA = 0.05;
  var EQUILIBRIUM_DT_BAND = 1.0;
  var EQUILIBRIUM_CREEP_DT_HIGH = 3.0;
  var EQUILIBRIUM_CREEP_PWM_STEP = 5;
  var EQUILIBRIUM_RESET_STAGNANT_MIN = 240.0;
  var EARLY_START_SETTLE_MINUTES = 2.0;   // hete-plug is dan bekend voorbij
  var EARLY_START_CLIMB_STEP = 10;        // hoeveel PWM/min klimmen naar het evenwicht toe
  var STOP_PATIENCE_MIN = 5.0;      // minuten op de PWM-bodem zonder herstel vooraleer echt te stoppen

  var props = PropertiesService.getScriptProperties();

  try {
    var response = UrlFetchApp.fetch(url, { muteHttpExceptions: true });
    var rawResponse = response.getContentText().trim();

    if (!rawResponse || rawResponse.length < 10) {
      triggerAlert(sheet, "Lege response van Particle API");
      return;
    }

    var apiData = JSON.parse(rawResponse);
    if (!apiData.result) {
      triggerAlert(sheet, "Geen data in API response");
      return;
    }

    var p = JSON.parse(unescape(apiData.result).trim());

    // --- EENVOUDIGE FYSICA: THERMOSIFON + ZON ---
    var dT   = parseFloat(p.dT)    || 0;
    var dEQ  = parseFloat(p.dEQ)   || 0;
    var Tsun = parseFloat(p.Solar) || 0;

    var now = new Date();
    var hour = now.getHours();

    // --- Vorige toestand ophalen (persistente state tussen runs) ---
    var prevPumpOn      = props.getProperty('shadowPumpOn') === 'true';
    var prevPwm         = parseFloat(props.getProperty('shadowPwm')) || 0;
    var pumpStartMs     = parseFloat(props.getProperty('shadowPumpStart')) || 0;
    var floorMinutes    = parseFloat(props.getProperty('shadowFloorMinutes')) || 0;
    var pidIntegral     = parseFloat(props.getProperty('shadowPidIntegral')) || 0;
    var pidPrevError    = parseFloat(props.getProperty('shadowPidPrevError')) || 0;
    var dTFiltered      = props.getProperty('shadowDtFiltered');
    var lastRunMs       = parseFloat(props.getProperty('shadowLastRun')) || now.getTime();
    var elapsedMin      = Math.max((now.getTime() - lastRunMs) / 60000, 0);
    if (elapsedMin <= 0) elapsedMin = 1;
    // OPWARMEN-state (1aug26)
    var warmupActive       = props.getProperty('shadowWarmupActive') === 'true';
    var warmupPwmTarget    = parseFloat(props.getProperty('shadowWarmupPwmTarget')) || 0;
    var prevDTForGradient  = parseFloat(props.getProperty('shadowPrevDTForGradient')) || 0;
    var warmupStableMinutes = parseInt(props.getProperty('shadowWarmupStableMinutes')) || 0;
    // VROEGSTART-state (2aug26)
    var earlyStartActive        = props.getProperty('shadowEarlyStartActive') === 'true';
    var earlyStartTriggered     = props.getProperty('shadowEarlyStartTriggered') === 'true';
    var earlyStartTriggerGradient = parseFloat(props.getProperty('shadowEarlyStartTriggerGradient')) || 0;
    var earlyStartBeginMs       = parseFloat(props.getProperty('shadowEarlyStartBeginMs')) || 0;
    var earlyStartPwmTarget     = parseFloat(props.getProperty('shadowEarlyStartPwmTarget')) || 0;
    var earlyStartPrevTsun      = parseFloat(props.getProperty('shadowEarlyStartPrevTsun')) || 0;
    var earlyStartCurrentGradient = parseFloat(props.getProperty('shadowEarlyStartCurrentGradient')) || 0;
    var prevDTForEarlyStart     = parseFloat(props.getProperty('shadowPrevDTForEarlyStart')) || 0;
    var earlyStartStableMinutes = parseInt(props.getProperty('shadowEarlyStartStableMinutes')) || 0;
    var prevTsunForGradient     = parseFloat(props.getProperty('shadowPrevTsunForGradient')) || 0;
    // Dag-evenwichts-PWM state (6aug26)
    var equilibriumPwm             = parseFloat(props.getProperty('shadowEquilibriumPwm')) || 0;
    var equilibriumKnown           = props.getProperty('shadowEquilibriumKnown') === 'true';
    var earlyStartUsingEquilibrium = props.getProperty('shadowEarlyStartUsingEquilibrium') === 'true';
    var pumpStopMs                 = parseFloat(props.getProperty('shadowPumpStopMs')) || 0;
    var tsunGradientTrackInit   = props.getProperty('shadowTsunGradientTrackInit') === 'true';
    var tsunGradientStableMinutes = parseInt(props.getProperty('shadowTsunGradStableMin')) || 0;

    // dT-filter (EMA): dempt de korte, hevige piek van de "hete plug" bij pompstart
    if (dTFiltered === null) {
      dTFiltered = dT;
    } else {
      dTFiltered = parseFloat(dTFiltered);
      var effAlpha = 1 - Math.pow(1 - DT_FILTER_ALPHA, elapsedMin);
      dTFiltered += effAlpha * (dT - dTFiltered);
    }

    var pumpOn = prevPumpOn;
    var comment = "";
    var phase = "";
    var overheat = (Tsun >= 90);
    var atFloor = false;
    var errorForLog = 0;

    // --- 1. Nachtblokkering ---
    if (hour >= 21 || hour < 7) {
      pumpOn = false;
      phase = "NACHT";
      comment = "Nachtblokkering (21–07u)";
      floorMinutes = 0;
      pidIntegral = 0;
      pidPrevError = 0;
      warmupActive = false;
      earlyStartActive = false;
      earlyStartStableMinutes = 0;
      tsunGradientTrackInit = false;
      tsunGradientStableMinutes = 0;
    }
    else {
      // --- 2. STOP als laatste redmiddel (28jul26) ---
      // De relay schakelt niet meer uit bij één enkele meting onder een vaste
      // drempel. Zolang de PID op de PWM-bodem staat ÉN dT niet meer wint,
      // loopt een teller op; pas na STOP_PATIENCE_MIN minuten zonder herstel
      // stopt de pomp echt. Tijdens OPWARMEN (1aug26) telt "op de bodem staan"
      // bewust niet mee - anders zou een verse trickle-start zichzelf kunnen
      // afbreken, want de PWM staat daar bewust laag/onder PWM_MIN.
      var wouldBeOn;
      if (prevPumpOn) {
        // VROEGSTART wordt hier bewust NIET meer uitgesloten (6aug26 v2) -
        // zie de Photon-sketch voor de volledige uitleg.
        atFloor = (!warmupActive) && (prevPwm <= PWM_MIN + 2.0);

        // Dag-evenwichts-PWM bijleren (6aug26): enkel tijdens een echt
        // stabiele REGIME - zie de Photon-sketch voor de volledige uitleg.
        if (!warmupActive && !earlyStartActive && !atFloor && Math.abs(dTFiltered - DT_TARGET) < EQUILIBRIUM_DT_BAND) {
          if (!equilibriumKnown) {
            equilibriumPwm = prevPwm;
            equilibriumKnown = true;
          } else {
            equilibriumPwm += EQUILIBRIUM_EMA_ALPHA * (prevPwm - equilibriumPwm);
          }
        }

        var tooLow = (dTFiltered <= DT_HARD_STOP);
        if (atFloor && tooLow) {
          floorMinutes += elapsedMin;
        } else {
          floorMinutes = 0;
        }
        wouldBeOn = (floorMinutes < STOP_PATIENCE_MIN);
      } else {
        // 1aug26: vroege trickle-start bij dT>0 i.p.v. te wachten tot DT_START
        wouldBeOn = (dTFiltered > DT_TRICKLE_START);
        floorMinutes = 0;

        // VROEGSTART-trigger (2aug26): zie de Photon-sketch voor de volledige
        // uitleg. De teller blijft bewust ook doorlopen over opeenvolgende
        // runs heen terwijl de pomp uit blijft - enkel een echte pompstart of
        // een terugval onder de drempel zet hem terug op nul.
        if (!tsunGradientTrackInit) {
          prevTsunForGradient = Tsun;
          tsunGradientTrackInit = true;
        } else {
          var tsunGradient = Tsun - prevTsunForGradient;
          prevTsunForGradient = Tsun;
          if (tsunGradient > EARLY_START_GRADIENT_MIN) {
            tsunGradientStableMinutes++;
          } else {
            tsunGradientStableMinutes = 0;
          }
          if (tsunGradientStableMinutes >= GRADIENT_CONFIRM_MINUTES) {
            earlyStartTriggered = true;
            earlyStartTriggerGradient = tsunGradient;
            wouldBeOn = true;
          }
        }
      }

      // --- 3. Thermosifon voorkomen (directe override). Gebruikt nog steeds
      // DT_START (3.0) - die drempel bleef hiervoor bestaan, enkel de pompstart
      // zelf verhuisde naar DT_TRICKLE_START. ---
      if (dTFiltered > DT_START && Tsun < 22) {
        wouldBeOn = false;
        floorMinutes = 0;
        phase = "THERMOSIFON";
        comment = "Thermosifon voorkomen (Tsun=" + Tsun.toFixed(1) + "C < 22C)";
      }

      // --- 4. Oververhitting: forceert AAN, overschrijft alles - dé bescherming
      // tegen een oplopende collectortemperatuur ---
      if (overheat) {
        wouldBeOn = true;
        floorMinutes = 0;
        phase = "OVERVERHIT";
        comment = "Tsun=" + Tsun.toFixed(1) + "C >= 90C, PWM=255";
      }

      pumpOn = wouldBeOn;

      if (!pumpOn) {
        pidIntegral = 0;   // reset zodat er geen windup optreedt terwijl de pomp stilstaat
        pidPrevError = 0;
        floorMinutes = 0;
        warmupActive = false;  // volgende start begint weer vers
        earlyStartActive = false;
        earlyStartStableMinutes = 0;
        if (!phase) {
          if (prevPumpOn) {
            phase = "STOP";
            comment = "dT gefilterd=" + dTFiltered.toFixed(1) + "C, geen herstel na " + STOP_PATIENCE_MIN.toFixed(0) + " min op PWM-bodem";
            pumpStopMs = now.getTime();   // onthouden wanneer de pomp stopte (6aug26)
          } else {
            phase = "WACHT";
            if (equilibriumKnown) {
              comment = "dT=" + dTFiltered.toFixed(1) + "C, wacht op start (Tsol-gradient stabiel " + tsunGradientStableMinutes + "/" + GRADIENT_CONFIRM_MINUTES + " min, dag-evenwicht=" + Math.round(equilibriumPwm) + ")";
            } else {
              comment = "dT=" + dTFiltered.toFixed(1) + "C, wacht op start (Tsol-gradient stabiel " + tsunGradientStableMinutes + "/" + GRADIENT_CONFIRM_MINUTES + " min, nog geen dag-evenwicht)";
            }
          }
        }
        // Dag-evenwicht vergeten na een lange stilstand - vangnet voor een
        // nieuwe dag (6aug26)
        if (equilibriumKnown && pumpStopMs > 0 && (now.getTime() - pumpStopMs) / 60000 > EQUILIBRIUM_RESET_STAGNANT_MIN) {
          equilibriumKnown = false;
        }
      }
    }

    // --- 5. PWM-doel bepalen ---
    var pwmDoel = 0;
    if (pumpOn) {
      if (!prevPumpOn) {
        // Pomp start net: geen D-piek bij de allereerste stap. Kiest tussen
        // VROEGSTART (gestart via de Tsol-gradiënt, nog vóór dT positief
        // werd) en de normale OPWARMEN-ramp (2aug26)
        pidIntegral = 0;
        pidPrevError = dTFiltered - DT_TARGET;
        pumpStartMs = now.getTime();
        floorMinutes = 0;
        tsunGradientTrackInit = false;
        tsunGradientStableMinutes = 0;

        if (earlyStartTriggered) {
          earlyStartActive = true;
          earlyStartBeginMs = now.getTime();
          warmupActive = false;
          prevDTForEarlyStart = dTFiltered;
          earlyStartStableMinutes = 0;
          earlyStartPrevTsun = Tsun;               // baseline voor de live-gradiënt (4aug26)
          earlyStartCurrentGradient = earlyStartTriggerGradient;

          if (equilibriumKnown) {
            // Dag-evenwicht gekend (6aug26 v2): start voorzichtig op de
            // bodem, klim er pas ná het settle-venster geleidelijk naartoe.
            earlyStartUsingEquilibrium = true;
            earlyStartPwmTarget = EARLY_START_PWM_MIN;
          } else {
            // Eerste start van de dag - terug op de live-gradiënt-bootstrap.
            earlyStartUsingEquilibrium = false;
            var frac = (earlyStartTriggerGradient - EARLY_START_GRADIENT_MIN) /
                       (EARLY_START_GRADIENT_REF - EARLY_START_GRADIENT_MIN);
            frac = Math.min(Math.max(frac, 0), 1);
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
      } else if (earlyStartActive) {
        // VROEGSTART (2aug26 basis, 3aug26: lineaire formule, 4aug26: LIVE
        // bijsturing, 6aug26 v2: settle-venster + dag-evenwicht met
        // geleidelijke klim). De duur wacht in beide gevallen tot
        // dT_gefilterd zelf stabiliseert.
        earlyStartCurrentGradient = Tsun - earlyStartPrevTsun;
        earlyStartPrevTsun = Tsun;
        var minutesSinceStart = (now.getTime() - earlyStartBeginMs) / 60000;

        // Settle-venster (6aug26 v2): zie de Photon-sketch voor de volledige
        // uitleg - de eerste EARLY_START_SETTLE_MINUTES wordt nog niets
        // herberekend, earlyStartPwmTarget blijft op de startwaarde staan.
        if (minutesSinceStart > EARLY_START_SETTLE_MINUTES) {
          if (dTFiltered <= DT_HARD_STOP) {
            earlyStartPwmTarget = EARLY_START_PWM_MIN;
          } else if (earlyStartUsingEquilibrium) {
            if (earlyStartPwmTarget < equilibriumPwm) {
              earlyStartPwmTarget = Math.min(equilibriumPwm, earlyStartPwmTarget + EARLY_START_CLIMB_STEP);
            } else if (dTFiltered > DT_TARGET + EQUILIBRIUM_CREEP_DT_HIGH) {
              earlyStartPwmTarget = Math.min(EARLY_START_PWM_MAX, earlyStartPwmTarget + EQUILIBRIUM_CREEP_PWM_STEP);
            }
          } else {
            var liveFrac = (earlyStartCurrentGradient - EARLY_START_GRADIENT_MIN) /
                           (EARLY_START_GRADIENT_REF - EARLY_START_GRADIENT_MIN);
            liveFrac = Math.min(Math.max(liveFrac, 0), 1);
            earlyStartPwmTarget = EARLY_START_PWM_MIN + liveFrac * (EARLY_START_PWM_MAX - EARLY_START_PWM_MIN);
          }
        }

        var esGradient = dTFiltered - prevDTForEarlyStart;
        prevDTForEarlyStart = dTFiltered;
        var esAbsGradient = Math.abs(esGradient);

        if (esAbsGradient < EARLY_START_STABLE_THRESHOLD) {
          earlyStartStableMinutes++;
        } else {
          earlyStartStableMinutes = 0;
        }

        var minutesInEarlyStart = (now.getTime() - earlyStartBeginMs) / 60000;
        if (earlyStartStableMinutes >= EARLY_START_STABLE_MINUTES_NEEDED || minutesInEarlyStart >= EARLY_START_MAX_MINUTES) {
          earlyStartActive = false;   // klaar - PID (REGIME) neemt deze cyclus al over
        }
      } else if (warmupActive) {
        // OPWARMEN (1aug26): evalueer, ná de allereerste minuut, of het circuit
        // intussen stabiel genoeg is om als "opgewarmd" te gelden - of anders
        // hoe hard de PWM deze minuut mag bijklimmen, evenredig met hoe steil
        // dT_gefilterd nog stijgt.
        var gradient = dTFiltered - prevDTForGradient;
        prevDTForGradient = dTFiltered;
        var absGradient = Math.abs(gradient);

        if (absGradient < WARMUP_STABLE_THRESHOLD) {
          warmupStableMinutes++;
        } else {
          warmupStableMinutes = 0;
        }

        var minutesInWarmup = (now.getTime() - pumpStartMs) / 60000;
        if (warmupStableMinutes >= WARMUP_STABLE_MINUTES_NEEDED || minutesInWarmup >= WARMUP_MAX_MINUTES) {
          warmupActive = false;  // circuit als opgewarmd beschouwd - PID neemt deze cyclus al over
        } else {
          var step = WARMUP_STEP_MIN + (WARMUP_STEP_MAX - WARMUP_STEP_MIN) *
                     Math.min(Math.max(gradient / WARMUP_GRADIENT_REF, 0), 1);
          warmupPwmTarget = Math.min(Math.max(warmupPwmTarget + step, WARMUP_PWM_START), WARMUP_PWM_CEILING);
        }
      }

      if (overheat) {
        pwmDoel = 255;
      } else if (earlyStartActive) {
        pwmDoel = earlyStartPwmTarget;
        // D-kick-fix: pidPrevError blijft ook tijdens VROEGSTART meelopen
        // (zie de Photon-sketch voor de volledige uitleg)
        pidPrevError = dTFiltered - DT_TARGET;
      } else if (warmupActive) {
        pwmDoel = warmupPwmTarget;
        // D-kick-fix (31jul26): pidPrevError blijft ook tijdens OPWARMEN
        // meelopen (zie de Photon-sketch voor de volledige uitleg)
        pidPrevError = dTFiltered - DT_TARGET;
      } else {
        var error = dTFiltered - DT_TARGET;
        errorForLog = error;

        // Anti-windup: integraal bevriezen zolang de PWM toch al op de bodem
        // vastzit - anders stapelt de integraal zich nutteloos verder op.
        if (!atFloor) {
          pidIntegral = Math.min(Math.max(pidIntegral + error * elapsedMin, -PID_I_MAX), PID_I_MAX);
        }
        var derivative = (error - pidPrevError) / elapsedMin;
        pidPrevError = error;

        var pidOutput = (Kp * error) + (Ki * pidIntegral) + (Kd * derivative);
        pwmDoel = Math.min(Math.max(PWM_MIN + pidOutput, PWM_MIN), 255);
      }

      if (!phase) {
        if (earlyStartActive) {
          phase = "VROEGSTART";
          var minutesSinceStartForLog = (now.getTime() - earlyStartBeginMs) / 60000;
          if (minutesSinceStartForLog <= EARLY_START_SETTLE_MINUTES) {
            comment = "settle-venster (" + minutesSinceStartForLog.toFixed(1) + "/" + EARLY_START_SETTLE_MINUTES.toFixed(0) + " min), PWM=" + Math.round(earlyStartPwmTarget) + (earlyStartUsingEquilibrium ? (", evenwicht=" + Math.round(equilibriumPwm)) : " (bootstrap)");
          } else if (earlyStartUsingEquilibrium) {
            comment = "klimt naar dag-evenwicht " + Math.round(equilibriumPwm) + ", nu PWM=" + Math.round(earlyStartPwmTarget) + " (dT=" + dTFiltered.toFixed(1) + "C), stabiel " + earlyStartStableMinutes + "/" + EARLY_START_STABLE_MINUTES_NEEDED + " min (max " + EARLY_START_MAX_MINUTES.toFixed(0) + " min)";
          } else {
            comment = "live gradient=" + earlyStartCurrentGradient.toFixed(1) + "C/min -> PWM=" + Math.round(earlyStartPwmTarget) + ", stabiel " + earlyStartStableMinutes + "/" + EARLY_START_STABLE_MINUTES_NEEDED + " min (max " + EARLY_START_MAX_MINUTES.toFixed(0) + " min)";
          }
        } else if (warmupActive) {
          phase = "OPWARMEN";
          comment = "dT=" + dTFiltered.toFixed(1) + "C, PWM=" + Math.round(pwmDoel) + ", stabiel " + warmupStableMinutes + "/" + WARMUP_STABLE_MINUTES_NEEDED + " min";
        } else if (atFloor) {
          phase = "AFBOUW";
          comment = "dT gefilterd=" + dTFiltered.toFixed(1) + "C, PWM-bodem sinds " + floorMinutes.toFixed(1) + "/" + STOP_PATIENCE_MIN.toFixed(0) + " min";
        } else {
          phase = "REGIME";
          comment = "dT gefilterd=" + dTFiltered.toFixed(1) + "C (fout=" + (dTFiltered - DT_TARGET).toFixed(1) + ")";
        }
      }
    }

    // --- 6. PWM geleidelijk laten bewegen (extra veiligheidslimiet bovenop de PID) ---
    var pwmPhysica;
    if (overheat) {
      pwmPhysica = pwmDoel;
    } else {
      var maxStep = PWM_MAX_DELTA_PER_MIN * elapsedMin;
      if (prevPwm < pwmDoel) {
        pwmPhysica = Math.min(prevPwm + maxStep, pwmDoel);
      } else if (prevPwm > pwmDoel) {
        pwmPhysica = Math.max(prevPwm - maxStep, pwmDoel);
      } else {
        pwmPhysica = pwmDoel;
      }
    }
    pwmPhysica = Math.round(pwmPhysica);

    comment = "[" + phase + "] " + comment;

    // --- State opslaan voor volgende run ---
    props.setProperty('shadowPumpOn', pumpOn.toString());
    props.setProperty('shadowPwm', pwmPhysica.toString());
    props.setProperty('shadowPumpStart', pumpStartMs.toString());
    props.setProperty('shadowFloorMinutes', floorMinutes.toString());
    props.setProperty('shadowPidIntegral', pidIntegral.toString());
    props.setProperty('shadowPidPrevError', pidPrevError.toString());
    props.setProperty('shadowDtFiltered', dTFiltered.toString());
    props.setProperty('shadowLastRun', now.getTime().toString());
    props.setProperty('shadowWarmupActive', warmupActive.toString());
    props.setProperty('shadowWarmupPwmTarget', warmupPwmTarget.toString());
    props.setProperty('shadowPrevDTForGradient', prevDTForGradient.toString());
    props.setProperty('shadowWarmupStableMinutes', warmupStableMinutes.toString());
    props.setProperty('shadowEarlyStartActive', earlyStartActive.toString());
    props.setProperty('shadowEarlyStartTriggered', earlyStartTriggered.toString());
    props.setProperty('shadowEarlyStartTriggerGradient', earlyStartTriggerGradient.toString());
    props.setProperty('shadowEarlyStartBeginMs', earlyStartBeginMs.toString());
    props.setProperty('shadowEarlyStartPwmTarget', earlyStartPwmTarget.toString());
    props.setProperty('shadowEarlyStartPrevTsun', earlyStartPrevTsun.toString());
    props.setProperty('shadowEarlyStartCurrentGradient', earlyStartCurrentGradient.toString());
    props.setProperty('shadowPrevDTForEarlyStart', prevDTForEarlyStart.toString());
    props.setProperty('shadowEarlyStartStableMinutes', earlyStartStableMinutes.toString());
    props.setProperty('shadowPrevTsunForGradient', prevTsunForGradient.toString());
    props.setProperty('shadowEquilibriumPwm', equilibriumPwm.toString());
    props.setProperty('shadowEquilibriumKnown', equilibriumKnown.toString());
    props.setProperty('shadowEarlyStartUsingEquilibrium', earlyStartUsingEquilibrium.toString());
    props.setProperty('shadowPumpStopMs', pumpStopMs.toString());
    props.setProperty('shadowTsunGradientTrackInit', tsunGradientTrackInit.toString());
    props.setProperty('shadowTsunGradStableMin', tsunGradientStableMinutes.toString());

    var pumpPhysica = pumpOn ? "ON" : "OFF";

    // 9. Rij toevoegen
    var row = [
      now,
      p.ETopH, p.ETopL, p.EMidH, p.EMidL, p.EBotH, p.EBotL,
      p.EAv, p.EQtot, p.Solar, p.dT, p.dEQ, p.pwmVal, p.Relay, p.WiFiSig, p.Mem,
      pumpPhysica, pwmPhysica,
      comment
    ];

    sheet.appendRow(row);
    success = true;
    Logger.log(`Data: dT=${dT}, dEQ=${dEQ}, Tsun=${Tsun}, Pump=${pumpPhysica}, PWM=${pwmPhysica}, Comment=${comment}`);

  } catch (e) {
    Logger.log("Fout: " + e.toString());
    triggerAlert(sheet, "Fout in script");
    return;
  }

  if (!success || sheet.getLastRow() === lastRowBefore) {
    triggerAlert(sheet, "Geen nieuwe data");
  }
}

/**
 * Alert: geen data >20 min, max 1/u
 */
function triggerAlert(sheet, debugReason) {
  var lastRow = sheet.getLastRow();
  if (lastRow < 2) return;

  var lastTimestamp = sheet.getRange(lastRow, 1).getValue();
  var now = new Date();
  var diffInMinutes = Math.round((now - new Date(lastTimestamp)) / 60000);

  if (diffInMinutes <= 20) return;

  var props = PropertiesService.getScriptProperties();
  var lastAlert = props.getProperty('lastAlertTime');
  var nowMs = now.getTime();
  if (lastAlert && (nowMs - parseInt(lastAlert)) < 3600000) return;
  props.setProperty('lastAlertTime', nowMs.toString());

  var values = sheet.getRange(lastRow, 1, 1, 17).getValues()[0];
  var lastData = {
    timestamp: values[0],
    EQtot: values[8] || 0,
    Solar: values[9] || 0,
    dT: values[10] || 0,
    pwmVal: values[12] || 0,
    Relay: values[14] || 0,
    WiFiSig: values[15] || "?",
    Mem: values[16] || "?"
  };

  var subject = "SOLAR-ECO: GEEN DATA SINDS " + diffInMinutes + " minuten";
  var message = `
<b>WAARSCHUWING: Geen data ontvangen!</b>
<i>Laatste update: ${new Date(lastData.timestamp).toLocaleString('nl-BE')}</i>

<h3>Controleer:</h3>
<ol>
  <li><b>LED:</b> Traag cyaan = OK | Groen = WiFi | Rood = fout</li>
  <li><b>Reset:</b> Grijze blokje 1 min uit → terug in</li>
  <li><b>Status:</b>
    <ul>
      <li>Energie: <b>${parseFloat(lastData.EQtot).toFixed(2)} kWh</b></li>
      <li>Collector: <b>${parseFloat(lastData.Solar).toFixed(1)}°C</b></li>
      <li>dT: <b>${parseFloat(lastData.dT).toFixed(1)}°C</b></li>
      <li>Pomp: <b>${lastData.pwmVal}/255</b></li>
      <li>Relay: <b>${lastData.Relay == 1 ? "AAN" : "UIT"}</b></li>
      <li>WiFi: <b>${lastData.WiFiSig}%</b></li>
      <li>Geheugen: <b>${lastData.Mem}%</b></li>
    </ul>
  </li>
</ol>

<h3>Niet opgelost?</h3>
Check <a href="https://console.particle.io">Particle Console</a> of contacteer <b>Opie</b>.

<i>Max. 1 mail per uur</i>
<br><small>Debug: ${debugReason}</small>
`;

  MailApp.sendEmail({
    to: "filip.delannoy@gmail.com, maartendelannoy@gmail.com",
    subject: subject,
    htmlBody: message
  });

  Logger.log("Alert verstuurd: " + diffInMinutes + " min");
}
