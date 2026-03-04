/**
 * @file    milliohmmeter7.ino
 * @brief   Firmware til Milliohmmeter baseret på ATmega328P (Uno/Nano) og ADS1115.
 * @details Denne firmware håndterer præcisionsmåling af modstande ved hjælp af 4-leder metoden (Kelvin).
 * @details Den indeholder SCPI-lignende seriel kommunikation, datalogning, kalibreringstabeller i PROGMEM
 * @details og understøttelse af I2C LCD display.
 * @details Der måles i området 1 m Ohm til 10 Ohm
 * @details Der er valgt en relativ lille øvre grænse for at fastholde at der kun er en konstantstrøm på 100mA
 * @details Det er det billigste og mest enkle og de fleste gode multimetre kan fint håndtere området over 5 Ohm.
 * @details Det er ikke et professionelt milliohmmeter da det kræver:
 * @details  - Måling med AC (op til 1kHz op til 10mA) eller vending af målestrøm fra 1-5Hz / DC med polaritetsvending : Bruger dette instrument ikke
 * @details  - Reducerer / Eliminerer termospændinger (EMF) (Seebeck‑effekt)
 * @details  - Eliminerer offset i forstærker
 * @details  - Reducerer 1/f‑støj
 * @details  - Automatisk temperaturkompensation for kobber/alu‑ledere : Har dette instrument ikke
 * @details  - Måleområde: 1 µΩ – 2 kΩ : Dette instrument har 1mΩ - 10Ω
 * @details  - Kalibreringsværdier gemmes i NVRAM : Gør dette instrument ikke det gemmer istedet i FLASH ROM / PROGMEM (Kræver omprogrammering ved kalibrering)
 * @details  - 0.25 % eller bedre : Dette instrument har 1%
 * @details  - Opløsning: - 1 µΩ : har dette instrument ikke 
 * @details  - Zero‑adjustment for at eliminere restmodstand : har dette instrument ikke
 * @details  - CAT IV : Overholder dette instrument ikke
 * @details  - Certificeret kalibrering : har dette instrument ikke
 * @details Dette milliohmmeter anvender differential 16 bit indgang så der er max 32767 ADC niveauer. (32767 positive niveauer og 32767 negative niveauer: Vi anvender kun de positive)
 * @details Der anvendes ADS1115 I2C 16bit ADC. Dvs. en ikke temperaturkompenseret ikke helt lineær ADC. Derfor er kalibrering helt nødvendig og man kan kun stole på resultaterne
 * @details når der arbejdes med samme temperatur som ved kalibrering.
 * @details Der anvendes en konstantstrøm på 100mA der genereres af LM317 lineær regulator IC. Denne skal monteres på køleplade for at minimere temperaturdrift.
 * @details Målestrømmen er 100mA dvs. spændingerne der genereres er 1m*0.1 = 100uV til 10000m*0.1=1000mV 
 * @details PGA er dynamisk (4x, 8x eller 16x) baseret på ADC værdien for optimal måleområde
 * @details PGA = 16x for ADC < 3000 (saturerer ved +/- 0.256V)
 * @details PGA = 8x for 3000 <= ADC <= 30000 (saturerer ved +/- 0.512V)
 * @details PGA = 4x for ADC > 30000 (saturerer ved +/- 1.024V)
 * @details Dette instrument er et milliohmmeter til hobbybrug og servicebrug. Det kan ikke anvendes til kalibrering, udviklingsarbejde og videnskabeligt arbejde: ikke til professionel brug 
 * @author  Jan Engelbrecht Pedersen
 * @date    2025-12-15
 * @version 7.0
 */

#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <LiquidCrystal_I2C.h>
#include <RTClib.h>

// ==========================================================================================================================================================================================
//                  HARDWARE KONFIGURATION
// ==========================================================================================================================================================================================

/// @brief Pin nummer til kalibrerings-knappen (input pullup).
#define BUTTON_PIN 2

/// @brief Pin nummer til CALIB_LED / Kalibrerings LED (Lyser når man er i kalibreringsmode)
#define CALIB_LED 3

/// @brief I2C adresse for LCD displayet (typisk 0x27 eller 0x3F).
#define LCD_ADDR 0x27

/// @brief I2C adresse for ADS1115 ADC modulet.
#define ADS_ADDR 0x48

// --- Objekter til hardware styring ---
LiquidCrystal_I2C lcd(LCD_ADDR, 16, 2); ///< LCD objekt (16 tegn, 2 linjer)
Adafruit_ADS1115 ads;                   ///< ADS1115 objekt
RTC_DS1307 rtc;                         ///< Real Time Clock objekt

/// @brief Flag der indikerer om RTC modulet blev fundet og kører.
bool rtcAvailable = false;

// ==========================================================================================================================================================================================
//                  MÅLEPARAMETRE
// ==========================================================================================================================================================================================

/// @brief Bruger 64-bit integer til akkumulering for at undgå overflow under oversampling.
typedef int64_t acc_t;

/// @brief Antal blokke af målinger der skal gennemsnittes (yderste loop).
const uint8_t BLOCKS = 4;

/// @brief Antal målinger pr. blok (oversampling) for at reducere støj.
const uint8_t OVERSAMPLE = 65;

/// @brief Hvor meget modstanden skal ændre sig (i Ohm) før displayet opdateres (hysterese).
const float DISPLAY_CHANGE_THRESHOLD_OHMS = 0.00001f;

/// @brief Debounce tid for knappen i millisekunder.
const unsigned long DEBOUNCE_MS = 50;

/// @brief Antal punkter i kalibreringstabellen.
#define CAL_POINTS 81

/// @brief ADC tærskel for skift fra PGA 16x til 8x
#define ADC_THRESHOLD_LOW 3000

/// @brief ADC tærskel for skift fra PGA 8x til 4x
#define ADC_THRESHOLD_HIGH 10000

/// @brief Enumeration for PGA gain værdier
enum PGAGain {
  PGA_4X = 4,
  PGA_8X = 8,
  PGA_16X = 16
};

/// @brief Aktuel PGA gain værdi
PGAGain currentPGA = PGA_8X;

// ==========================================================================================================================================================================================
//             KALIBRERINGSTABELLER (PROGMEM)
// ==========================================================================================================================================================================================
/* @brief Disse tabeller gemmes i Flash-hukommelsen (PROGMEM) 
 * @param adcTable_4x : Indeholder rå ADC værdier for PGA = 4x
 * @param adcTable_8x : Indeholder rå ADC værdier for PGA = 8x
 * @param adcTable_16x : Indeholder rå ADC værdier for PGA = 16x
 * @param refOhms : Indeholder den tilsvarende modstand i Ohm.
 * @details Ved kalibrering så sørg for at LM317 er afkølet ordentligt med køleplade og at Ambient temperature er 25 grader Celcius
 */

/// @brief Kalibreringstabel: Rå ADC værdier for PGA = 16x (måleområde +/- 0.256V)
const int32_t PROGMEM adcTable_16x[CAL_POINTS] = {
  0, 54, 104, 110, 116, 129, 184, 200, 232, 254, 257, 328, 354, 388,
  420, 436, 468, 532, 596, 683, 720, 768, 816, 803, 940, 1192, 1200, 1716, 1730, 1865, 2135, 2885, 3240,
  3523, 3804, 4220, 6010, 6624, 8704, 9504, 9600, 10496, 11008, 11776, 12416, 13056, 14112,
  15392, 19872, 23072, 24352, 26784, 28192, 32032, 35424, 37152, 39752, 39968, 40608,
  40992, 41248, 42048, 43264, 44832, 48064, 50624, 60224, 62784, 64064, 64192, 65344, 65504, 65506, 65507, 65508, 65509, 655010, 64511, 65560, 65562,65535
};

/// @brief Kalibreringstabel: Rå ADC værdier for PGA = 8x (måleområde +/- 0.512V)
const int32_t PROGMEM adcTable_8x[CAL_POINTS] = {
  0, 16, 32, 40, 48, 56, 64, 80, 96, 112, 128, 144, 160, 176,
  192, 208, 224, 256, 288, 352, 368, 384, 416, 448, 480, 528, 640, 832, 896, 960, 1088, 1280, 1680,
  1752, 1875, 2138, 3178, 3208, 4352, 4752, 4800, 5248, 5504, 5888, 6452, 6664, 7140,
  7603, 9779, 11398, 12176, 13392, 14096, 16016, 17712, 18576, 19876, 19984, 20304,
  20496, 20624, 21024, 21632, 22416, 24032, 25312, 30112, 31392, 32032, 32096, 32672, 32752, 32754, 33000, 33001, 33002, 33003, 33010, 34000, 65504, 65510
};

/// @brief Kalibreringstabel: Rå ADC værdier for PGA = 4x (måleområde +/- 1.024V)
const int32_t PROGMEM adcTable_4x[CAL_POINTS] = {
  0, 8, 16, 20, 24, 28, 32, 40, 48, 56, 64, 72, 80, 88,
  96, 104, 112, 128, 144, 176, 184, 192, 208, 224, 240, 264, 320, 416, 448, 480, 544, 640, 840,
  896, 960, 1096, 1504, 1656, 2176, 2376, 2400, 2624, 2752, 2944, 3104, 3264, 3528,
  3848, 4968, 5768, 6088, 6452, 7028, 8008, 8724, 9196, 9936, 10000, 10160,
  10139, 10103, 10295, 10644, 11199, 11999, 12639, 15039, 15679, 15999, 16031, 16319, 16639, 19199, 22399, 23999, 25599, 27199, 28799, 30399, 31999, 32752
};

/// @brief Kalibreringstabel: Reference modstande i Ohm (fælles for alle PGA værdier)
const float PROGMEM refOhms[CAL_POINTS] = {
  0.00f, 0.003f, 0.0053367f, 0.006f, 0.008f, 0.01055f, 0.011f, 0.013f, 0.015f, 0.017f, 0.02135f, 0.022f, 0.024f, 0.029f, 
  0.03095f, 0.032f, 0.035f, 0.04f, 0.045f, 0.051f, 0.0566f, 0.06f, 0.064f, 0.07f, 0.075f, 0.092f, 0.093f, 0.134f, 0.1396f, 0.15f, 0.165f, 0.2f, 0.252f, 
  0.271f, 0.3f, 0.324f, 0.489f, 0.51715f, 0.68f, 0.741f, 0.75f, 0.82f, 0.86f, 0.92f, 0.968f, 1.01f, 1.1f,
  1.2f, 1.53f, 1.8f, 1.9f, 2.03f, 2.2f, 2.5f, 2.765f, 2.9f, 3.1f, 3.12f, 3.17f,
  3.2f, 3.22f, 3.28f, 3.375f, 3.5f, 3.75f, 3.95f, 4.7f, 4.9f, 5.0f, 5.01f, 5.1f, 5.2f, 6.00f, 7.00f, 7.50f, 8.00f, 8.50f, 9.00f, 9.50f, 10.0f, 11.0f
};

// ==========================================================================================================================================================================================
//                  TILSTANDE OG VARIABLER
// ==========================================================================================================================================================================================

unsigned long lastDebounceTime = 0; ///< Tidspunkt for sidste knap-aktivitet (debounce)
bool lastButtonState = HIGH;        ///< Forrige tilstand af knappen (Input Pullup: HIGH er sluppet)
bool calibrationMode = false;       ///< Flag: Er vi i kalibreringstilstand? Ved opstart: calibrationMode = FALSE : Ikke kalibreringstilstand
float lastDisplayedOhm = -1.0f;     ///< Gemmer sidste viste værdi for at undgå display-flimmer

// ==========================================================================================================================================================================================
//                  DATALOGNING
// ==========================================================================================================================================================================================

/// @brief Antal målinger der gemmes i den cirkulære buffer (Reduceret til 16 for at spare RAM).
#define LOG_SIZE 16

/// @brief Struktur til at holde en enkelt måling.
struct Measurement {
  char timestamp[20]; ///< Tidsstempel som tekst (YYYY-MM-DDTHH:MM:SS)
  float resistance;   ///< Målt modstand i Ohm
};

Measurement logBuffer[LOG_SIZE]; ///< Buffer array til målinger
uint8_t logIndex = 0;            ///< Nuværende indeks i den cirkulære buffer

// ==========================================================================================================================================================================================
//                  PROGMEM HJÆLPEFUNKTIONER
// ==========================================================================================================================================================================================

/**
 * @brief Læser en float værdi fra refOhms tabellen i PROGMEM.
 * @param idx : Indeks i tabellen.
 * @return float : Værdien fra tabellen.
 */
float readRefOhmFromProgmem(int idx) {
  float v;
  memcpy_P(&v, &refOhms[idx], sizeof(float));
  return v;
}

/**
 * @brief Læser en int32_t værdi fra den relevante ADC tabel i PROGMEM baseret på aktuel PGA.
 * @param idx : Indeks i tabellen.
 * @return int32_t : Værdien fra tabellen.
 */
int32_t readAdcFromProgmem(int idx) {
  switch(currentPGA) {
    case PGA_4X:
      return (int32_t)pgm_read_dword_near(&adcTable_4x[idx]);
    case PGA_8X:
      return (int32_t)pgm_read_dword_near(&adcTable_8x[idx]);
    case PGA_16X:
      return (int32_t)pgm_read_dword_near(&adcTable_16x[idx]);
    default:
      return (int32_t)pgm_read_dword_near(&adcTable_8x[idx]);
  }
}

// ==========================================================================================================================================================================================
//                  PGA STYRING
// ==========================================================================================================================================================================================

/**
 * @brief Sætter PGA gain på ADS1115 baseret på den ønskede gain værdi.
 * @param gain : Ønsket PGA gain (PGA_4X, PGA_8X eller PGA_16X).
 */
void setPGAGain(PGAGain gain) {
  currentPGA = gain;
  switch(gain) {
    case PGA_4X:
      ads.setGain(GAIN_FOUR);  // +/- 1.024V
      break;
    case PGA_8X:
      ads.setGain(GAIN_EIGHT);  // +/- 0.512V (oprindelig indstilling)
      break;
    case PGA_16X:
      ads.setGain(GAIN_SIXTEEN);  // +/- 0.256V
      break;
  }
}

/**
 * @brief Vælger optimal PGA baseret på ADC værdi.
 * @param adcVal : Rå ADC værdi.
 * @return PGAGain : Anbefalet PGA gain.
 */
PGAGain selectOptimalPGA(int32_t adcVal) {
  if (adcVal < ADC_THRESHOLD_LOW) {
    return PGA_16X;  // Lav værdi -> højere forstærkning
  } else if (adcVal > ADC_THRESHOLD_HIGH) {
    return PGA_4X;   // Høj værdi -> lavere forstærkning
  } else {
    return PGA_8X;   // Medium værdi -> mellem forstærkning
  }
}

// ==========================================================================================================================================================================================
//                  MÅLEKERNE (ADC & BEREGNING)
// ==========================================================================================================================================================================================

/**
 * @brief Udfører en enkelt differentiel måling på ADS1115 (Kanal 0-1).
 * @return int32_t : Rå ADC værdi.
 */
int32_t readSingleDifferentialRaw() {
  return (int32_t)ads.readADC_Differential_0_1();
}

/**
 * @brief   Udfører en oversamplet måling for at reducere støj.
 * @details Samler (BLOCKS * OVERSAMPLE) målinger og beregner gennemsnittet.
 * @return  int32_t : Gennemsnitlig rå ADC værdi.
 */
int32_t measureAdcRaw() {
  acc_t totalAcc = 0;
  for (uint8_t b = 0; b < BLOCKS; b++) {
    acc_t blockAcc = 0;
    for (uint16_t s = 0; s < OVERSAMPLE; s++) {
      blockAcc += (acc_t)readSingleDifferentialRaw();
    }
    // Bemærk: Integer division her kan medføre lille præcisionstab før total sum
    totalAcc += (blockAcc / OVERSAMPLE);
  }
  return (int32_t)(totalAcc / BLOCKS);
}

/**
 * @brief   Konverterer rå ADC værdi til Ohm ved hjælp af lineær interpolation.
 * @details Slår op i kalibreringstabellerne (adcTable og refOhms). Hvis værdien
 * @details ligger mellem to punkter, beregnes den nøjagtige værdi lineært.
 * @param   adcVal : Den målte rå ADC værdi.
 * @return  float : Beregnet modstand i Ohm. Returnerer -1.0f ved fejl.
 */
float interpolateResistance(int32_t adcVal) {
  // Check nedre grænse
  int32_t low = readAdcFromProgmem(0);
  if (adcVal <= low) return readRefOhmFromProgmem(0);

  // Check øvre grænse
  int32_t high = readAdcFromProgmem(CAL_POINTS - 1);
  if (adcVal >= high) return readRefOhmFromProgmem(CAL_POINTS - 1);

  // Interpolation gennem tabellen
  for (int i = 0; i < CAL_POINTS - 1; i++) {
    int32_t a0 = readAdcFromProgmem(i);
    int32_t a1 = readAdcFromProgmem(i + 1);
    
    // Hvis vi er mellem to punkter i tabellen
    if (adcVal >= a0 && adcVal <= a1) {
      float r0 = readRefOhmFromProgmem(i);
      float r1 = readRefOhmFromProgmem(i + 1);
      
      // Beregn faktoren (hvor langt er vi mellem a0 og a1?)
      float f = ((float)(adcVal - a0)) / ((float)(a1 - a0));
      // Beregn den resulterende modstand
      return r0 + f * (r1 - r0);
    }
  }
  return -1.0f; // Bør ikke nås pga. grænssetjek
}

/**
 * @brief Wrapper funktion til konvertering.
 * @param raw Rå ADC værdi.
 * @return float Modstand i Ohm.
 */
float adcRawToOhm(int32_t raw) {
  return interpolateResistance(raw);
}

// ==========================================================================================================================================================================================
//                  DISPLAY FUNKTIONER
// ==========================================================================================================================================================================================

/**
 * @brief Opstarter display
 */
void displayClear() {
  lcd.init();
  lcd.backlight();
}

/**
 * @brief Viser den rå ADC værdi på LCD displayet.
 * @param raw : Rå værdi der skal vises.
 */
void displayRawAdc(int32_t raw, PGAGain gain) {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("RAW ADC:"));
  lcd.print(F("(G="));
  lcd.print((int)gain);
  lcd.print(F(")"));
  lcd.setCursor(0, 1);
  lcd.print(raw);
}

/**
 * @brief Viser modstanden pænt formateret (uOhm, mOhm eller Ohm) med PGA info.
 * @param ohm : Modstand i Ohm.
 * @param gain : Aktuel PGA gain værdi.
 */
void displayResistance(float ohm, PGAGain gain) {
  lcd.clear();
  lcd.setCursor(0, 0);
  
  // Vis "R: (Gain=X)" baseret på aktuel PGA
  lcd.print(F("R: (Gain="));
  lcd.print((int)gain);
  lcd.print(F(")"));
  
  if (ohm < 0.001f) {
    // Vis mikro-ohm
    lcd.setCursor(0, 1);
    lcd.print(ohm * 1e6f, 2);
    lcd.print(F(" uOhm"));
  } else if (ohm < 1.0f) {
    // Vis milli-ohm
    lcd.setCursor(0, 1);
    lcd.print(ohm * 1000.0f, 3);
    lcd.print(F(" mOhm"));
  } else {
    // Vis ohm
    lcd.setCursor(0, 1);
    lcd.print(ohm, 4);
    lcd.print(F(" Ohm"));
  }
}

/**
 * @brief   Viser en fejlbesked på LCD displayet.
 * @details Denne version håndterer almindelige C-strenge (char*).
 * @param   msg : Fejlbesked.
 */
void displayError(const char* msg) {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("FEJL:"));
  lcd.setCursor(0, 1);
  lcd.print(msg);
}

/**
 * @brief   Viser en fejlbesked på LCD displayet (Overload for Flash strings).
 * @details Denne version håndterer strenge gemt i PROGMEM via F() makroen.
 * @param   msg : Fejlbesked (Flash String Helper).
 */
void displayError(const __FlashStringHelper* msg) {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("FEJL:"));
  lcd.setCursor(0, 1);
  lcd.print(msg);
}

// ==========================================================================================================================================================================================
//                  INPUT & KALIBRERINGS LOGIK
// ==========================================================================================================================================================================================

/**
 * @brief Tjekker knappen og skifter kalibreringstilstand med debounce.
 */
void updateCalibrationMode() {
  bool reading = digitalRead(BUTTON_PIN);
  
  // Hvis inputtet har ændret sig siden sidst
  if (reading != lastButtonState) {
    lastDebounceTime = millis();
    lastButtonState = reading;
  }

  // Hvis tiden siden sidste ændring er større end debounce grænsen
  if (millis() - lastDebounceTime > DEBOUNCE_MS) {
    // LOW betyder trykket ned (pga. INPUT_PULLUP)
    calibrationMode = (reading == LOW);
    lastDebounceTime = 0;
    lastButtonState = true;
  }
}

// ==========================================================================================================================================================================================
//                  TIDSSTEMPEL & LOGNING
// ==========================================================================================================================================================================================

/**
 * @brief   Genererer et tidsstempel til logning.
 * @details Bruger RTC hvis tilgængelig, ellers en "Oppetime" tæller.
 * @param   buf : Buffer hvor tidsstemplet skrives til.
 * @param   len : Længden af buffer.
 */
void getTimestamp(char* buf, size_t len) {
  if (rtcAvailable) {
    DateTime now = rtc.now();
    // Format: YYYY-MM-DDTHH:MM:SS
    snprintf(buf, len, "%04d-%02d-%02dT%02d:%02d:%02d",
             now.year(), now.month(), now.day(),
             now.hour(), now.minute(), now.second());
  } else {
    // Fallback til millis (Dage:Timer:Min:Sek)
    unsigned long ms = millis() / 1000;
    unsigned long s = ms % 60;
    unsigned long m = (ms / 60) % 60;
    unsigned long h = (ms / 3600) % 24;
    unsigned long days = ms / 86400;
    
    snprintf(buf, len, "D%luT%02lu:%02lu:%02lu", days, h, m, s);
  }
}

/**
 * @brief Gemmer en måling i den cirkulære buffer.
 * @param timestamp : Tidsstempel streng.
 * @param resistance : Målt modstand.
 */
void logMeasurement(const char* timestamp, float resistance) {
  strncpy(logBuffer[logIndex].timestamp, timestamp, sizeof(logBuffer[logIndex].timestamp)-1);
  // Sikrer null-terminering
  logBuffer[logIndex].timestamp[sizeof(logBuffer[logIndex].timestamp)-1] = '\0';
  
  logBuffer[logIndex].resistance = resistance;
  
  // Opdater index cirkulært (0 -> 1 -> ... -> 15 -> 0)
  logIndex = (logIndex + 1) % LOG_SIZE;
}

// ==========================================================================================================================================================================================
//                  EKSPORT (CSV / XML)
// ==========================================================================================================================================================================================

/**
 * @brief Udskriver loggen som CSV format til Serial.
 */
void exportCSV() {
  Serial.println(F("timestamp,resistance"));
  for (uint8_t i = 0; i < LOG_SIZE; i++) {
    // Udskriv kun hvis der er data (timestamp er ikke tom)
    if (logBuffer[i].timestamp[0] != '\0') {
      Serial.print(logBuffer[i].timestamp);
      Serial.print(',');
      Serial.println(logBuffer[i].resistance, 6);
    }
  }
}

/**
 * @brief Udskriver loggen som XML format til Serial.
 */
void exportXML() {
  Serial.println(F("<measurements>"));
  for (uint8_t i = 0; i < LOG_SIZE; i++) {
    if (logBuffer[i].timestamp[0] != '\0') {
      Serial.print(F("  <measurement><timestamp>"));
      Serial.print(logBuffer[i].timestamp);
      Serial.print(F("</timestamp><resistance>"));
      Serial.print(logBuffer[i].resistance, 6);
      Serial.println(F("</resistance></measurement>"));
    }
  }
  Serial.println(F("</measurements>"));
}

// ==========================================================================================================================================================================================
//                  SCPI KOMMANDOFORTOLKER
// ==========================================================================================================================================================================================

/**
 * @brief Behandler en modtaget SCPI-kommando.
 * @param cmd : Kommando-streng (skal være uppercase).
 */
void processSCPICommand(const char* cmd) {
  if (strcmp(cmd, "*IDN?") == 0) {
    Serial.println(F("Milliohmmeter v7,Jan Engelbrecht Pedersen,2025-12-15,FW-SCPI-MultiPGA"));
  } else if (strcmp(cmd, "MEAS?") == 0 || strcmp(cmd, "MEASURE:RES?") == 0) {
    // Udfør måling på forespørgsel
    float R = adcRawToOhm(measureAdcRaw());
    Serial.print(R, 6);
    Serial.println(F(" Ohm"));
  } else if (strcmp(cmd, "CAL?") == 0) {
    Serial.println(F("Kalibrering er ikke tilgængelig via SCPI. Brug hardwareknap."));
  } else if (strcmp(cmd, "EXPORT:CSV?") == 0) {
    exportCSV();
  } else if (strcmp(cmd, "EXPORT:XML?") == 0) {
    exportXML();
  } else {
    Serial.println(F("FEJL: Ukendt kommando"));
  }
}

/**
 * @brief   Læser fra seriel porten, opbygger kommandoer og kalder processSCPICommand.
 * @details Håndterer linjeskift (\n eller \r) som afslutning på kommando.
 * @details Konverterer automatisk til store bogstaver.
 */
void handleSerialCommands() {
  static char inputBuf[64];
  static uint8_t idx = 0;
  
  while (Serial.available()) {
    char c = Serial.read();
    
    // Tjek for linjeskift (kommando slut)
    if (c == '\n' || c == '\r') {
      if (idx > 0) {
        inputBuf[idx] = '\0'; // Null-terminer strengen
        
        // Konverter til uppercase 'in-place'
        for (uint8_t i = 0; i < idx; i++) {
          if (inputBuf[i] >= 'a' && inputBuf[i] <= 'z') {
            inputBuf[i] -= ('a' - 'A');
          }
        }
        
        // Behandl kommandoen
        processSCPICommand(inputBuf);
        idx = 0; // Nulstil buffer index
      }
    } else {
      // Gem tegn i buffer hvis der er plads
      if (idx < sizeof(inputBuf) - 1) {
        inputBuf[idx++] = c;
      }
    }
  }
}

// ==========================================================================================================================================================================================
//                  SETUP
// ==========================================================================================================================================================================================

/**
 * @brief   Arduino Setup funktion. Kører én gang ved start.
 * @details Initialiserer Serial, I2C, LCD, ADS1115 og RTC.
 */
void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);
  
  // Sæt knap pin til intern pullup
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  // Sæt CALIB_LED pin til OUTPUT
  pinMode(CALIB_LED, OUTPUT);
  // Sluk CALIB_LED
  digitalWrite(CALIB_LED, LOW);
  // Start LCD
  displayClear();
  lcd.setCursor(0, 0);
  lcd.print(F("Milliohmmeter v7"));
  delay(400);

  // Start ADS1115
  if (!ads.begin()) {
    // Her kaldes nu den overloadede displayError funktion
    displayError(F("ADC FEJL"));
    while (1) delay(1000); // Stop programmet ved fejl
  }
  
  // Indstil gain til 8x +/- 0.512V (standard start værdi)
  setPGAGain(PGA_8X);

  // Indstil højeste datarate (860 SPS)
  ads.setDataRate(RATE_ADS1115_860SPS);

  // Start RTC hvis den er tilsluttet: 
  // Tilsluttet : rtcAvaible = TRUE 
  // Ikke-tilsluttet: rtcAvaible = FALSE
  if (rtc.begin()) {
    rtcAvailable = rtc.isrunning();
  } else {
    rtcAvailable = false;
  }

  // Nulstil log buffer timestamps
  for (uint8_t i = 0; i < LOG_SIZE; i++) logBuffer[i].timestamp[0] = '\0';
}

// ==========================================================================================================================================================================================
//                  LOOP
// ==========================================================================================================================================================================================

/**
 * @brief   Arduino Loop funktion. Kører uendeligt.
 * @details Håndterer:
 * @details 1. Serielle kommandoer.
 * @details 2. Knap status (Kalibrering).
 * @details 3. Måling af modstand.
 * @details 4. Dynamisk PGA justering baseret på ADC værdi.
 * @details 5. Opdatering af display og seriel log.
 */
void loop() {
  handleSerialCommands();
  updateCalibrationMode();
  setPGAGain(PGA_8X);
  // Udfør initial måling (tager lidt tid pga. oversampling)
  int32_t adcRaw = measureAdcRaw();
  
  // Vælg optimal PGA baseret på ADC værdi
  PGAGain optimalPGA = selectOptimalPGA(adcRaw);
  
  // Hvis PGA skal ændres, opdater og mål igen
  if (optimalPGA != currentPGA) {
    setPGAGain(optimalPGA);
    delay(10); // Kort pause for at lade ADC stabilisere
    adcRaw = measureAdcRaw(); // Mål igen med ny PGA
  }

  if (calibrationMode) 
  {
    // Vis rå data for kalibrering
    displayRawAdc(adcRaw,currentPGA);
    Serial.print(F("RAW: "));
    Serial.print(adcRaw);
    Serial.print(F(" PGA: "));
    Serial.println((int)currentPGA);
    // Tænd CALIB_LED
    digitalWrite(CALIB_LED, HIGH);
  } 
  else {
    // Normal måle-mode
    float R = adcRawToOhm(adcRaw);      
    
    if (R < 0) {
      /**
      * @brief Værdi uden for kalibreringsområdet: Vis fejlmelding på display
      */
      // Værdi uden for kalibreringsområdet
      displayError(F("CAL FEJL"));
      // Sluk CALIB_LED
      digitalWrite(CALIB_LED, LOW);
    } else {
      /**
      * @brief Display opdateres kun hvis modstandsændringen er signifikant (for at undgå display flimmer)
      */
      // Opdater kun display hvis ændringen er signifikant (for at undgå flimmer)
      if (lastDisplayedOhm < 0 || fabs(R - lastDisplayedOhm) > DISPLAY_CHANGE_THRESHOLD_OHMS) {
      /**
      * @brief Modstande over 5 Ohm skal give OL melding på display for modstande under vises modstandsværdien
      */
        // Hvis modstand <=5 ohm så vis modstandsværdien ellers vis fejl
        if (R <= 10)
        {
          displayResistance(R, currentPGA);
        }
        else
        {
          displayError(F("OL"));
        }
        lastDisplayedOhm = R;
      }
      /**
      * @brief Send raw ADC data til seriel port
      */      
      // Send data til Serial
      Serial.print(F("ADC: "));
      Serial.print(adcRaw);
      Serial.print(F(" PGA: "));
      Serial.print((int)currentPGA);
      Serial.print(F("x"));
      /**
      * @brief Viser modstanden pænt formateret på serielporten (uOhm, mOhm eller Ohm).
      * @param R Modstand i Ohm.
      */
      if (R < 0.001f) {
        // Vis mikro-ohm
        Serial.print(F(" R: "));
        Serial.print(R*1e6f, 6);
        Serial.println(F(" uOhm"));
      }     
      else if (R < 1.0f) {
        // Vis milli-ohm
        Serial.print(F(" R: "));
        Serial.print(R * 1000.0f, 3);
        Serial.println(F(" mOhm"));
      }     
      else {
        // Vis ohm
        Serial.print(F(" R: "));
        Serial.print(R, 4);
        Serial.println(F(" Ohm"));
      }
      
      // Log til intern buffer
      char ts[20];
      getTimestamp(ts, sizeof(ts));
      logMeasurement(ts, R);
      
      // Sluk CALIB_LED
      digitalWrite(CALIB_LED, LOW);
    }
  }
  delay(50);
}