#include "Arduino.h"
#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <TFT_eSPI.h>
#include "font.h"
#include <EEPROM.h>

#define EXTRA_DUTY 1
#define SUPPLY_MODE 1

Adafruit_ADS1115 ads;
TFT_eSPI tft;
TFT_eSprite spT(&tft);
TFT_eSprite spB(&tft);
TFT_eSprite spH(&tft);
bool uiTopOk = false;
bool uiOk = false;
bool uiBarOk = false;
int uiBarY = 0;
float dutyPct = 0.0f;
uint32_t lastUiUs = 0;
TFT_eSprite spG(&tft);
bool uiGrOk = false;
int uiGrY = 0;
int uiGrH = 0;
const int GR_N = 160;
const uint32_t GR_SAMPLE_MS = 83;
const float GR_MIN_SPAN_C = 2.0f;
const float GR_SET_NEAR_C = 10.0f;
const float GR_SHRINK = 0.15f;
float grLo = 0.0f;
float grHi = 0.0f;
bool grRangeInit = false;
float grT[GR_N];
int grHead = 0;
int grCnt = 0;
uint32_t grMs = 0;
uint32_t uiCost[4] = {9000, 4000, 8000, 5000};
uint8_t uiPart = 0;
uint32_t uiRuns = 0;
const uint32_t UI_PART_US = 12000;

const float UI_NEAR_C = 3.0f;
const float UI_FAR_C = 50.0f;
const int UI_SPR_W = 160;
const int UI_SPR_H = 36;
const int UI_PUSH_Y = 47;
const int UI_T_DEPTH = 8;
const int UI_GAP = 3;
const int UI_BAR_W = 160;
const int UI_BAR_H = 18;
const int UI_TOP_H = 44;
const int UI_STATE_TY = 27;
const int UI_TOP_TY = 7;
const int UI_SUB_BASE = 12;
const int UI_BAR_BW = 130;
const int UI_BAR_BH = 14;
const int UI_BAR_R = 4;

const uint8_t HEATER_PIN = D8;
const float SHUNT_OHMS = 0.1f;
const float SHUNT_SIGN = -1.0f;
const uint32_t I2C_HZ = 400000;

const float tcK = 0.041f / 1.6f;
float coldC = 25.0f;

const float MAX_C = 450.0f;
const float VREF_V = 12.5f;
const float KP_DEF = 0.08f;
const float KI_DEF = 0.03f;
const float KD_DEF = 0.01f;
const float HOLD_DEF = 0.0004f;
const float T_FILTER_S = 0.12f;
const float D_ALPHA = 0.4f;
const float INT_BAND_C = 10.0f;
const float NEG_INT_GAIN = 2.0f;
const float D_BAND_C = 30.0f;
const float MAX_INTEGRAL = 0.6f;
const uint32_t RATE_US = 300000;

const uint32_t WINDOW_US = 50000;
#if EXTRA_DUTY
const uint32_t SETTLE_US = 1500;
#else
const uint32_t SETTLE_US = 3000;
#endif
const uint16_t ADS_RATE = RATE_ADS1115_860SPS;
const uint32_t MIN_PULSE_US = 300;
const uint32_t CUR_READ_US = 3000;
const uint32_t CUR_MIN_PULSE_US = 6000;
const uint32_t CUR_REFRESH_MS = 1000;
const uint32_t PROBE_DEBT_MAX_US = 20000;
uint32_t ampsMs = 0;
uint32_t probeDebtUs = 0;
const uint32_t MIN_ON_US = 20000;
const int BOOST_SKIP = 3;
const float HEAVY_FALL_CPS = 2.0f;
const unsigned long HEAVY_MS = 5000;
const float OPEN_MV = 50.0f;
const float OPEN_SPREAD_MV = 1.0f;
const float OPEN_RATE_CPS = 600.0f;
const int OPEN_LIMIT = 12;
int openScore = 0;
float lastRawC = 0.0f;
#if EXTRA_DUTY
const int TC_SAMPLES = 3;
#else
const int TC_SAMPLES = 5;
#endif
const uint8_t VIN_PIN = A0;
const uint8_t ENC_A = D3;
const uint8_t ENC_B = D4;
const uint8_t ENC_BTN = D0;
const float ENC_STEP_C = 5.0f;
const float ENC_MIN_C = 50.0f;
const int ENC_DIR = 1;
const uint32_t ENC_BTN_DEBOUNCE_MS = 30;
const uint32_t ENC_POLL_US = 2000;
const bool ENC_BTN_ACTIVE_HIGH = false;
const uint32_t ENC_BTN_STUCK_MS = 10000;
volatile uint32_t encEdges = 0;
int32_t encSteps = 0;
uint32_t encPresses = 0;
uint32_t encPolls = 0;
uint32_t encFlips = 0;
bool encDebug = true;
uint32_t encLoops = 0;
uint32_t encSvcCalls = 0;
uint32_t encDbgMs = 0;
uint32_t encDbgL = 0;
uint32_t encDbgS = 0;
uint32_t encDbgP = 0;
uint32_t encDbgE = 0;
uint32_t encDbgF = 0;
uint32_t encDbgDrop = 0;
void encReport();
#if EXTRA_DUTY
const int VIN_SAMPLES = 1;
#else
const int VIN_SAMPLES = 4;
#endif
const float VIN_SCALE_DEF = 0.02437f;

const float MIN_AMPS = 0.1f;
const float MAX_AMPS = 10.0f;
const int SHUNT_LIMIT = 10;
const float SUPPLY_DROP = 0.82f;
const float SUPPLY_WIN_DROP = 0.7f;
const uint32_t SUPPLY_WIN_MS = 3000;
const float SUPPLY_HARD_DUTY = 0.8f;
const float SUPPLY_BASE_DUTY = 0.1f;
const float SUPPLY_MIN_BASE_V = 3.0f;
const float SUPPLY_MIN_V = 9.5f;
const int SUPPLY_LIMIT = 6;


const float PI_F = 3.14159265f;
const float TUNE_D = 0.2f;
const float TUNE_HYST_C = 2.0f;
const float TUNE_BASE_MAX = 0.7f;
const float TUNE_BASE_MIN = 0.01f;
const int TUNE_SKIP = 1;
const int TUNE_CYCLES = 3;
const unsigned long TUNE_MAX_MS = 300000;
const unsigned long LOAD_MAX_MS = 150000;
const float LOAD_DIP_C = 3.0f;
const float LOAD_REC_C = 2.0f;
const float LOAD_OVER_C = 3.0f;
const float LOAD_REC_SLOW_S = 2.0f;
const float ADAPT_STEP = 1.15f;
const float ADAPT_MIN = 0.5f;
const float ADAPT_MAX = 1.6f;

enum TuneState : uint8_t { T_IDLE, T_SETTLE, T_RELAY, L_SETTLE, L_TOUCH, L_MEASURE };

float kp = KP_DEF;
float ki = KI_DEF;
float kd = KD_DEF;
float holdPerC = HOLD_DEF;
const float PRESETS[] = {235.0f, 280.0f, 320.0f, 360.0f, 380.0f, 400.0f};
const int PRESET_N = sizeof(PRESETS) / sizeof(PRESETS[0]);
int presetIdx = 0;
float setC = 100.0f;
bool calMode = false;
bool idleMode = true;

float tempC = 0.0f;
float tcMv = 0.0f;
float amps = 0.0f;
int16_t rawI = 0;
const float vinScale = VIN_SCALE_DEF;
float vinRawOff = 0.0f;
float vinRawOn = 0.0f;
uint32_t vinOnMs = 0;
const float VIN_RED_LOW_V = 10.0f;
const float VIN_RED_HIGH_V = 26.0f;
bool vinRed = false;
double statOnS = 0.0;
double statWh = 0.0;
const uint32_t AUTO_OFF_MS = 300000;
const uint32_t TOP_HOLD_MS = 2000;
bool topStatPage = false;
uint32_t activityMs = 0;
uint32_t vibCount = 0;
uint32_t vibMs = 0;
bool vibLow = false;
uint32_t statRuns = 0;
uint32_t statCycles = 0;
uint32_t statBoots = 0;
uint32_t statPowerBaseS = 0;
const float STAT_CYC_BAND_C = 3.0f;
const float STAT_CYC_DIP_C = 10.0f;
bool cycArmed = false;
float cycSetC = 0.0f;
uint32_t statRunMs = 0;
uint32_t statLastMs = 0;
bool statWasRun = false;
float vinBase = 0.0f;
float vinWinPrev = 0.0f;
float vinWinCur = 0.0f;
uint32_t vinWinMs = 0;
int supplyScore = 0;
int vinTick = 0;
float duty = 0.0f;
float dutyCap = 0.79f;
float iTerm = 0.0f;
float dFilt = 0.0f;
float rateTemp = 0.0f;
int shuntScore = 0;
int boostLeft = 0;
bool fault = false;
const char* faultMsg = "";
bool boost = false;
bool heaterOn = false;
bool pidInit = false;
bool tempInit = false;
uint32_t lastMeasUs = 0;
uint32_t lastCtlUs = 0;
uint32_t rateUs = 0;
uint32_t nextWin = 0;
uint32_t onStartUs = 0;
uint32_t onAccumUs = 0;
uint32_t lastPrintUs = 0;
unsigned long fallSince = 0;
bool heavyLoad = false;
unsigned long lastPrint = 0;
unsigned long settledSince = 0;
unsigned long lastLearn = 0;

uint8_t tuneState = T_IDLE;
unsigned long tuneStartMs = 0;
unsigned long tuneMark = 0;
bool tHigh = false;
bool tCycOn = false;
unsigned long tLastSw = 0;
float tBase = 0.0f;
unsigned long tCycMs = 0;
float tMax = 0.0f;
float tMin = 0.0f;
float tDutyAcc = 0.0f;
float tHiS = 0.0f;
float tLoS = 0.0f;
int tCycIdx = 0;
int tCount = 0;
float tSumAmp = 0.0f;
float tSumPer = 0.0f;
float tSumDuty = 0.0f;
float tSumD = 0.0f;
bool pendValid = false;
float pKp = 0.0f;
float pKi = 0.0f;
float pKd = 0.0f;
float pHold = 0.0f;
unsigned long lStartMs = 0;
float lMin = 0.0f;
float lSatS = 0.0f;
float lOver = 0.0f;
float lRecS = 0.0f;
bool lRec = false;
bool adaptOn = false;
bool adaptRun = false;
unsigned long adaptArm = 0;
float adaptRefKp = 0.0f;
float adaptRefKi = 0.0f;
float adaptS = 1.0f;
float adaptPrevS = 1.0f;
float adaptScore = 0.0f;
int adaptDir = 0;

char cmdBuf[32];
uint8_t cmdLen = 0;

const char* tuneName() {
  switch (tuneState) {
    case T_SETTLE: return "SETTLE";
    case T_RELAY: return "RELAY";
    case L_SETTLE: return "L-SETTLE";
    case L_TOUCH: return "L-TOUCH";
    case L_MEASURE: return "L-MEAS";
    default: return "OFF";
  }
}

void tuneStop(const char* why) {
  tuneState = T_IDLE;
  adaptRun = false;
  adaptArm = 0;
  tuneMark = 0;
  Serial.printf("tune stop: %s\n", why);
}

float holdDuty() {
  return constrain(holdPerC * (setC - coldC), 0.005f, 0.5f);
}

void resetPid() {
  pidInit = false;
  boostLeft = 0;
  settledSince = 0;
  adaptArm = 0;
  tuneMark = 0;
  iTerm = holdDuty();
}

void setFault(const char* msg) {
  if (!fault) faultMsg = msg;
  fault = true;
  if (tuneState != T_IDLE) tuneStop(msg);
}

void clearFault() {
  fault = false;
  faultMsg = "";
  boost = false;
  shuntScore = 0;
  heavyLoad = false;
  fallSince = 0;
  tempInit = false;
  tuneState = T_IDLE;
  adaptRun = false;
  probeDebtUs = 0;
  openScore = 0;
  supplyScore = 0;
  resetPid();
}

float sampleVinRaw() {
  uint32_t sum = 0;
  for (int i = 0; i < VIN_SAMPLES; i++) sum += analogRead(VIN_PIN);
  return (float)sum / VIN_SAMPLES;
}

void vinUpdate(float& slot) {
  slot = sampleVinRaw();
  float v = slot * vinScale;
  if (v < VIN_RED_LOW_V || v > VIN_RED_HIGH_V) vinRed = true;
}

float vinOffV() {
  return vinRawOff * vinScale;
}

float vinOnV() {
  bool fresh = vinRawOn > 0.0f && millis() - vinOnMs < 1000;
  return (fresh ? vinRawOn : vinRawOff) * vinScale;
}

float pwrScale() {
  float v = vinOnV();
  if (v < 5.0f) return 1.0f;
  float r = VREF_V / v;
  return constrain(r * r, 0.2f, 4.0f);
}

void readCurrent() {
  ads.setGain(GAIN_FOUR);
  rawI = ads.readADC_Differential_2_3();
  amps = SHUNT_SIGN * ads.computeVolts(rawI) / SHUNT_OHMS;
  vinUpdate(vinRawOn);
  vinOnMs = millis();
  ampsMs = vinOnMs;}

void checkShunt() {
  bool bad = amps < MIN_AMPS || amps > MAX_AMPS;
  if (bad) shuntScore += 2;
  else if (shuntScore > 0) shuntScore--;
  if (shuntScore >= SHUNT_LIMIT) setFault("SHUNT");
}

void checkSupply() {
#if SUPPLY_MODE == 0
  return;
#elif SUPPLY_MODE == 1
  float v = vinRawOn * vinScale;
  uint32_t nowMs = millis();
  if (nowMs - vinWinMs >= SUPPLY_WIN_MS) {
    vinWinPrev = vinWinCur;
    vinWinCur = v;
    vinWinMs = nowMs;
  } else if (v > vinWinCur) {
    vinWinCur = v;
  }
  float ref = vinWinPrev > vinWinCur ? vinWinPrev : vinWinCur;
  bool bad = ref >= SUPPLY_MIN_BASE_V && v < ref * SUPPLY_WIN_DROP;
  bool hard = boost || duty >= SUPPLY_HARD_DUTY;
  if (hard && vinBase >= SUPPLY_MIN_BASE_V && v < vinBase * SUPPLY_DROP) bad = true;
#else
  bool bad = vinRawOn * vinScale < SUPPLY_MIN_V;
#endif
  if (bad) supplyScore += 2;
  else if (supplyScore > 0) supplyScore--;
  if (supplyScore >= SUPPLY_LIMIT) setFault("SUPPLY");
}

void statAdd(uint32_t dtUs) {
  double s = dtUs * 1e-6;
  statOnS += s;
  statWh += (double)vinOnV() * fabsf(amps) * s / 3600.0;
}

void statService() {
  uint32_t now = millis();
  bool run = !calMode && !idleMode && !fault;
  if (run && !statWasRun) {
    statRuns++;
    activityMs = now;
  }
  if (run) statRunMs += now - statLastMs;
  statWasRun = run;
  statLastMs = now;
  if (setC != cycSetC) {
    cycSetC = setC;
    cycArmed = false;
  }
  if (!run || tuneState != T_IDLE) {
    cycArmed = false;
  } else {
    float err = setC - tempC;
    if (fabsf(err) < STAT_CYC_BAND_C) {
      cycArmed = true;
    } else if (cycArmed && err > STAT_CYC_DIP_C) {
      statCycles++;
      activityMs = now;
      cycArmed = false;
    }
  }
}

uint32_t statPowerS() {
  return statPowerBaseS + millis() / 1000;
}

void statReset() {
  statOnS = 0.0;
  statWh = 0.0;
  statRuns = 0;
  statCycles = 0;
  statRunMs = 0;
}

void statPrint() {
  float runMin = statRunMs / 60000.0f;
  float heatMin = (float)(statOnS / 60.0);
  float dutyPctAvg = statRunMs > 0 ? 100.0f * (float)statOnS / (statRunMs * 0.001f) : 0.0f;
  float avgW = statRunMs > 0 ? (float)(statWh * 3600000.0 / statRunMs) : 0.0f;
  Serial.printf("stats runs=%lu cycles=%lu run=%.1fmin heat=%.1fmin avgduty=%.0f%% energy=%.2fWh (%.4fkWh) avg=%.1fW boots=%lu power=%.1fh vib=%lu\n", (unsigned long)statRuns, (unsigned long)statCycles, runMin, heatMin, dutyPctAvg, (float)statWh, (float)(statWh * 0.001), avgW, (unsigned long)statBoots, statPowerS() / 3600.0f, (unsigned long)vibCount);
}

uint32_t accMarkUs = 0;

void IRAM_ATTR deadmanIsr() {
  GPOC = 1 << HEATER_PIN;
}

void heaterOff() {
  if (heaterOn) {
    uint32_t d = micros() - accMarkUs;
    onAccumUs += d;
    statAdd(d);
  }
  heaterOn = false;
  digitalWrite(HEATER_PIN, LOW);
}

void heaterSet() {
  if (!heaterOn) {
    heaterOn = true;
    onStartUs = micros();
    accMarkUs = onStartUs;
  }
  digitalWrite(HEATER_PIN, HIGH);
  timer1_write(600000);
}

void autoOffService() {
  bool run = !calMode && !idleMode && !fault && tuneState == T_IDLE;
  if (!run || millis() - activityMs < AUTO_OFF_MS) return;
  idleMode = true;
  heaterOff();
  Serial.println("auto-off: idle timeout");
}

const float NTC_R0 = 10000.0f;
const float NTC_BETA = 3950.0f;
const float NTC_RTOP = 9800.0f;
const float NTC_VCC = 2.46f;
const int NTC_EVERY_WIN = 2;
const float NTC_GND_V = 0.05f;
const float NTC_CHATTER_V = 0.03f;
const float NTC_JUMP_C = 8.0f;
const uint8_t NTC_RELOCK_N = 5;
const float NTC_RELOCK_C = 1.0f;
uint8_t ntcTick = 0;
float ntcC = 25.0f;
float coldOff = 0.0f;
bool ntcOk = false;
bool ntcAccepted = false;
bool ntcHave = false;
uint8_t ntcRejN = 0;
float ntcRejC = 0.0f;
uint32_t ntcMs = 0;

int ntcRead(float& out) {
  ads.setGain(GAIN_ONE);
  float v1 = ads.computeVolts(ads.readADC_SingleEnded(0));
  float v = ads.computeVolts(ads.readADC_SingleEnded(0));
  if (v1 < NTC_GND_V || v < NTC_GND_V) return 1;
  if (fabsf(v1 - v) > NTC_CHATTER_V) return 1;
  if (v > NTC_VCC - 0.05f) return 2;
  float r = NTC_RTOP * v / (NTC_VCC - v);
  float c = 1.0f / (1.0f / 298.15f + logf(r / NTC_R0) / NTC_BETA) - 273.15f;
  if (c < -40.0f || c > 125.0f) return 2;
  out = c;
  return 0;
}

void ntcUpdate() {
  float c;
  int s = ntcRead(c);
  if (s == 1) {
    if (!vibLow) vibCount++;
    vibLow = true;
    vibMs = millis();
    activityMs = vibMs;
    ntcAccepted = false;
    return;
  }
  vibLow = false;
  if (s == 2) {
    ntcOk = false;
    ntcAccepted = false;
    ntcRejN = 0;
    return;
  }
  if (ntcHave && fabsf(c - ntcC) > NTC_JUMP_C) {
    if (ntcRejN > 0 && fabsf(c - ntcRejC) < NTC_RELOCK_C) ntcRejN++;
    else ntcRejN = 1;
    ntcRejC = c;
    if (ntcRejN < NTC_RELOCK_N) {
      ntcAccepted = false;
      return;
    }
  }
  ntcRejN = 0;
  ntcC = c;
  ntcOk = true;
  ntcHave = true;
  ntcAccepted = true;
  float cc = ntcC + coldOff;
  if (cc > -20.0f && cc < 80.0f) coldC = cc;
}

const uint32_t SAVE_MAGIC = 0x48454155;
const uint32_t SAVE_DELAY_MS = 3000;
const uint32_t SAVE_STAT_GAP_MS = 10000;
struct SaveData {
  uint32_t magic;
  float setC;
  float onS;
  float wh;
  uint32_t runs;
  uint32_t runMs;
  uint32_t cycles;
  uint32_t boots;
  uint32_t powerS;
  uint32_t chk;
};
const uint32_t SAVE_POWER_STEP_S = 600;
float savedSetC = 0.0f;
float seenSetC = 0.0f;
uint32_t savedRuns = 0;
uint32_t savedRunMs = 0;
uint32_t savedCycles = 0;
uint32_t savedPowerS = 0;
uint32_t setChangeMs = 0;
uint32_t statSaveMs = 0;

uint32_t saveChk(const SaveData& d) {
  const uint8_t* p = (const uint8_t*)&d;
  uint32_t h = 2166136261u;
  for (size_t i = 0; i < sizeof(SaveData) - sizeof(uint32_t); i++) h = (h ^ p[i]) * 16777619u;
  return h;
}

void loadSettings() {
  EEPROM.begin(64);
  SaveData d;
  EEPROM.get(0, d);
  if (d.magic == SAVE_MAGIC && d.chk == saveChk(d)) {
    if (d.setC >= 0.0f && d.setC <= MAX_C - 5.0f) setC = d.setC;
    if (d.onS >= 0.0f && d.wh >= 0.0f) {
      statOnS = d.onS;
      statWh = d.wh;
      statRuns = d.runs;
      statRunMs = d.runMs;
      statCycles = d.cycles;
      statBoots = d.boots;
      statPowerBaseS = d.powerS;
    }
  }
  statBoots++;
  savedSetC = setC;
  seenSetC = setC;
  savedRuns = statRuns;
  savedRunMs = statRunMs;
  savedCycles = statCycles;
  savedPowerS = statPowerS();
}

void saveNow() {
  SaveData d;
  d.magic = SAVE_MAGIC;
  d.setC = setC;
  d.onS = (float)statOnS;
  d.wh = (float)statWh;
  d.runs = statRuns;
  d.runMs = statRunMs;
  d.cycles = statCycles;
  d.boots = statBoots;
  d.powerS = statPowerS();
  d.chk = saveChk(d);
  EEPROM.put(0, d);
  EEPROM.commit();
  savedSetC = setC;
  savedRuns = statRuns;
  savedRunMs = statRunMs;
  savedCycles = statCycles;
  savedPowerS = d.powerS;
  statSaveMs = millis();
}

void saveService() {
  uint32_t now = millis();
  if (setC != seenSetC) {
    seenSetC = setC;
    setChangeMs = now;
    return;
  }
  if (heaterOn || boost || boostLeft > 0) return;
  bool setDirty = setC != savedSetC && now - setChangeMs >= SAVE_DELAY_MS;
  bool statDirty = idleMode && !calMode && (statRuns != savedRuns || statRunMs != savedRunMs || statCycles != savedCycles || statPowerS() - savedPowerS >= SAVE_POWER_STEP_S) && now - statSaveMs >= SAVE_STAT_GAP_MS;
  if (setDirty || statDirty) saveNow();
}

void readTemp() {
  ads.setGain(GAIN_SIXTEEN);
  float vSum = 0.0f;
  float vMin = 1e9f;
  float vMax = -1e9f;
  for (int i = 0; i < TC_SAMPLES; i++) {
    int16_t raw = ads.readADC_Differential_1_3();
    if (raw >= 32767 || raw <= -32768) return setFault("TC_RAIL");
    float v = ads.computeVolts(raw);
    vSum += v;
    if (v < vMin) vMin = v;
    if (v > vMax) vMax = v;
  }
  tcMv = (vSum - vMin - vMax) / (TC_SAMPLES - 2) * 1000.0f;
  if (fabsf(tcMv) > OPEN_MV) return setFault("TC_OPEN");
  float spreadMv = (vMax - vMin) * 1000.0f;
  float raw_c = coldC + tcMv / tcK;
  uint32_t nowUs = micros();
  float dts = constrain((nowUs - lastMeasUs) * 1e-6f, 0.01f, 1.0f);
  bool noisy = spreadMv > OPEN_SPREAD_MV || (tempInit && fabsf(raw_c - lastRawC) / dts > OPEN_RATE_CPS);
  lastRawC = raw_c;
  if (noisy) openScore += 2;
  else if (openScore > 0) openScore--;
  if (openScore >= OPEN_LIMIT) return setFault("TC_OPEN");
  if (!tempInit) {
    tempInit = true;
    tempC = raw_c;
  } else {
    tempC += (dts / (T_FILTER_S + dts)) * (raw_c - tempC);
  }
  lastMeasUs = nowUs;
  if (!calMode) {
    if (tempC > MAX_C) setFault("OVERTEMP");
    else if (tempC < -10.0f) setFault("TC_RANGE");
  }
}

void adaptReset() {
  adaptRefKp = kp;
  adaptRefKi = ki;
  adaptS = 1.0f;
  adaptPrevS = 1.0f;
  adaptScore = 0.0f;
  adaptDir = 0;
  adaptArm = 0;
  tuneMark = 0;
}

void adaptApply(float s) {
  adaptS = constrain(s, ADAPT_MIN, ADAPT_MAX);
  kp = constrain(adaptRefKp * adaptS, 0.0005f, 0.5f);
  ki = constrain(adaptRefKi * adaptS, 0.0f, 0.99f);
}

void applyPending() {
  kp = pKp;
  ki = pKi;
  kd = pKd;
  holdPerC = pHold;
  pendValid = false;
  if (adaptOn) adaptReset();
}

void relayLevels(float& hi, float& lo) {
  float d = constrain(0.8f * tBase, 0.02f, TUNE_D);
  hi = constrain(tBase + d, 0.0f, dutyCap / pwrScale());
  lo = constrain(tBase - d, 0.0f, 1.0f);
}

void relayBegin() {
  tuneState = T_RELAY;
  tBase = constrain(iTerm, TUNE_BASE_MIN, TUNE_BASE_MAX);
  tHigh = false;
  tCycOn = false;
  tLastSw = millis();
  tCycIdx = 0;
  tCount = 0;
  tSumAmp = 0.0f;
  tSumPer = 0.0f;
  tSumDuty = 0.0f;
  tSumD = 0.0f;
  Serial.println("tune relay");
}

void relayCycleDone(unsigned long nowMs, float hi, float lo) {
  float per = (nowMs - tCycMs) * 1e-3f;
  float amp = 0.5f * (tMax - tMin);
  float avgD = per > 0.0f ? tDutyAcc / per : 0.0f;
  tCycIdx++;
  if (tCycIdx > TUNE_SKIP) {
    tSumAmp += amp;
    tSumPer += per;
    tSumDuty += avgD;
    tSumD += 0.5f * (hi - lo);
    tCount++;
    Serial.printf("tune cycle %d per=%.1f amp=%.2f duty=%.3f\n", tCycIdx, per, amp, avgD);
  }
  float tot = tHiS + tLoS;
  if (tot > 0.0f) tBase = constrain(tBase * (1.0f + 0.5f * (tHiS - tLoS) / tot), TUNE_BASE_MIN, TUNE_BASE_MAX);
}

void relayFinish() {
  float n = (float)tCount;
  float a = tSumAmp / n;
  float tu = tSumPer / n;
  float avgD = tSumDuty / n;
  float d = tSumD / n;
  if (a < 0.2f || tu < 1.0f || d <= 0.0f) {
    tuneStop("bad relay data");
    return;
  }
  float ku = 4.0f * d / (PI_F * a);
  pKp = constrain(ku / 2.2f, 0.0005f, 0.5f);
  pKi = constrain(pKp / (2.2f * tu), 0.0f, 0.99f);
  pKd = constrain(pKp * tu / 6.3f, 0.0f, 4.9f);
  pHold = constrain(avgD / (setC - coldC), 0.0001f, 0.01f);
  pendValid = true;
  Serial.printf("tune done Ku=%.4f Tu=%.1f a=%.2f\n", ku, tu, a);
  Serial.printf("tune kp=%.4f ki=%.5f kd=%.4f hold=%.5f\n", pKp, pKi, pKd, pHold);
  Serial.println("tune apply to use");
  tuneState = T_IDLE;
  resetPid();
}

void relayStep(float dt, unsigned long nowMs) {
  float hi;
  float lo;
  relayLevels(hi, lo);
  if (tCycOn) {
    if (tempC > tMax) tMax = tempC;
    if (tempC < tMin) tMin = tempC;
  }
  bool canSwitch = nowMs - tLastSw > 500;
  if (canSwitch && tHigh && tempC > setC + TUNE_HYST_C) {
    tHigh = false;
    tLastSw = nowMs;
  } else if (canSwitch && !tHigh && tempC < setC - TUNE_HYST_C) {
    tHigh = true;
    tLastSw = nowMs;
    if (tCycOn) {
      relayCycleDone(nowMs, hi, lo);
      if (tCount >= TUNE_CYCLES) {
        relayFinish();
        return;
      }
    }
    tCycOn = true;
    tCycMs = nowMs;
    tMax = tempC;
    tMin = tempC;
    tDutyAcc = 0.0f;
    tHiS = 0.0f;
    tLoS = 0.0f;
    relayLevels(hi, lo);
  }
  duty = tHigh ? hi : lo;
  boost = false;
  if (tCycOn) {
    tDutyAcc += duty * dt;
    if (tHigh) tHiS += dt;
    else tLoS += dt;
  }
}

void loadBegin(unsigned long nowMs) {
  tuneState = L_MEASURE;
  tuneStartMs = nowMs;
  lStartMs = nowMs;
  lMin = tempC;
  lSatS = 0.0f;
  lOver = 0.0f;
  lRecS = 0.0f;
  lRec = false;
  Serial.println(adaptRun ? "adapt: measuring" : "tune load: measuring");
}

void adaptFinish(float rec, float over, float satFrac) {
  float score = rec + 2.0f * over;
  if (satFrac > 0.4f) {
    adaptDir = 0;
    Serial.println("adapt: power limited");
    return;
  }
  if (adaptDir != 0) {
    if (score > adaptScore * 1.1f) {
      adaptApply(adaptPrevS);
      adaptDir = 0;
      Serial.printf("adapt: worse, reverted kp=%.4f ki=%.5f\n", kp, ki);
      return;
    }
    bool better = score < adaptScore * 0.9f;
    adaptScore = score;
    if (!better || (rec <= LOAD_REC_SLOW_S && over <= LOAD_OVER_C)) {
      adaptDir = 0;
      Serial.printf("adapt: kept kp=%.4f ki=%.5f\n", kp, ki);
      return;
    }
  } else {
    adaptScore = score;
    if (over > LOAD_OVER_C) {
      adaptDir = -1;
    } else if (rec > LOAD_REC_SLOW_S) {
      adaptDir = 1;
    } else {
      Serial.println("adapt: ok");
      return;
    }
  }
  float next = adaptDir > 0 ? adaptS * ADAPT_STEP : adaptS / ADAPT_STEP;
  if (next < ADAPT_MIN || next > ADAPT_MAX) {
    adaptDir = 0;
    Serial.println("adapt: limit");
    return;
  }
  adaptPrevS = adaptS;
  adaptApply(next);
  Serial.printf("adapt: try kp=%.4f ki=%.5f s=%.2f\n", kp, ki, adaptS);
}

void loadFinish() {
  float dip = setC - lMin;
  float rec = lRec ? lRecS : 60.0f;
  float satFrac = rec > 0.0f ? lSatS / rec : 0.0f;
  Serial.printf("%s dip=%.1f rec=%.1fs over=%.1f sat=%.0f%%\n", adaptRun ? "adapt" : "tune load", dip, rec, lOver, 100.0f * satFrac);
  if (adaptRun) {
    adaptRun = false;
    tuneState = T_IDLE;
    adaptArm = 0;
    tuneMark = 0;
    adaptFinish(rec, lOver, satFrac);
    return;
  }
  pKp = kp;
  pKi = ki;
  pKd = kd;
  pHold = holdPerC;
  pendValid = false;
  if (satFrac > 0.4f) {
    Serial.println("power limited, gains unchanged");
  } else if (lOver > LOAD_OVER_C) {
    pKp = kp * 0.85f;
    pKi = ki * 0.75f;
    pendValid = true;
    Serial.println("overshoot, lowered kp ki");
  } else if (rec > LOAD_REC_SLOW_S) {
    pKp = min(kp * 1.2f, 0.5f);
    pKi = min(ki * 1.3f, 0.99f);
    pendValid = true;
    Serial.println("slow recovery, raised kp ki");
  } else {
    Serial.println("load ok, gains unchanged");
  }
  if (pendValid) {
    Serial.printf("tune kp=%.4f ki=%.5f kd=%.4f hold=%.5f\n", pKp, pKi, pKd, pHold);
    Serial.println("tune apply to use");
  }
  tuneState = T_IDLE;
}

void loadStep(float err, float dt, unsigned long nowMs) {
  float el = (nowMs - lStartMs) * 1e-3f;
  if (tempC < lMin) lMin = tempC;
  if (!lRec) {
    if (duty >= 0.99f) lSatS += dt;
    if (err <= LOAD_REC_C) {
      lRec = true;
      lRecS = el;
    }
  } else if (-err > lOver) {
    lOver = -err;
  }
  if ((lRec && el > lRecS + 10.0f) || el > 60.0f) loadFinish();
}

void tuneObserve(float err, float dt, unsigned long nowMs) {
  if (tuneState == T_SETTLE) {
    if (fabsf(err) < 2.5f) {
      if (tuneMark == 0) tuneMark = nowMs;
      if (nowMs - tuneMark > 4000) relayBegin();
    } else {
      tuneMark = 0;
    }
  } else if (tuneState == L_SETTLE) {
    if (fabsf(err) < 2.5f) {
      if (tuneMark == 0) tuneMark = nowMs;
      if (nowMs - tuneMark > 5000) {
        tuneState = L_TOUCH;
        tuneMark = nowMs;
        Serial.println("tune load: touch tip now");
      }
    } else {
      tuneMark = 0;
    }
  } else if (tuneState == L_TOUCH) {
    if (err > LOAD_DIP_C) loadBegin(nowMs);
    else if (nowMs - tuneMark > 60000) tuneStop("no touch");
  } else if (tuneState == L_MEASURE) {
    loadStep(err, dt, nowMs);
  } else if (tuneState == T_IDLE && adaptOn) {
    if (adaptArm == 0) {
      if (fabsf(err) < 2.5f) {
        if (tuneMark == 0) tuneMark = nowMs;
        if (nowMs - tuneMark > 5000) {
          adaptArm = nowMs;
          tuneMark = 0;
        }
      } else {
        tuneMark = 0;
      }
    } else if (err > LOAD_DIP_C) {
      adaptRun = true;
      tuneStartMs = nowMs;
      loadBegin(nowMs);
    }
  }
}

void control() {
  if (calMode || idleMode) {
    duty = 0.0f;
    boost = false;
    return;
  }

  uint32_t nowUs = micros();
  unsigned long nowMs = millis();
  float dt = pidInit ? constrain((nowUs - lastCtlUs) * 1e-6f, 0.02f, 0.5f) : WINDOW_US * 1e-6f;
  if (!pidInit) {
    rateUs = nowUs;
    rateTemp = tempC;
    dFilt = 0.0f;
    vinBase = vinOffV();
    vinWinPrev = 0.0f;
    vinWinCur = 0.0f;
    vinWinMs = nowMs;
  } else if ((uint32_t)(nowUs - rateUs) >= RATE_US) {
    float r = (tempC - rateTemp) / ((nowUs - rateUs) * 1e-6f);
    dFilt += D_ALPHA * (r - dFilt);
    rateUs = nowUs;
    rateTemp = tempC;
  }
  lastCtlUs = nowUs;
  pidInit = true;

  if (tuneState != T_IDLE) {
    unsigned long lim = tuneState <= T_RELAY ? TUNE_MAX_MS : LOAD_MAX_MS;
    if (nowMs - tuneStartMs > lim) tuneStop("timeout");
  }

  float err = setC - tempC;
  float ps = pwrScale();
  if (tuneState == T_RELAY) relayStep(dt, nowMs);

  if (tuneState != T_RELAY) {
    float dScale = constrain((D_BAND_C - fabsf(err)) / (0.5f * D_BAND_C), 0.0f, 1.0f);
    float uD = -kd * dFilt * dScale;
    float uRaw = kp * err + uD + iTerm;
    boost = uRaw * ps >= 1.0f && err > 0.0f;
    if (boost) {
      iTerm = holdDuty();
      duty = 1.0f;
    } else {
      bool satHi = uRaw * ps >= dutyCap && err > 0.0f;
      if (fabsf(err) < INT_BAND_C && !satHi)
        iTerm += (err < 0.0f ? NEG_INT_GAIN : 1.0f) * ki * err * dt;
      iTerm = constrain(iTerm, 0.0f, MAX_INTEGRAL);
      duty = constrain(kp * err + uD + iTerm, 0.0f, 1.0f / ps);
    }
  }
  if (!boost) duty = constrain(duty * ps, 0.0f, 1.0f);

  tuneObserve(err, dt, nowMs);

  bool heating = !fault && amps >= MIN_AMPS && amps <= MAX_AMPS && duty >= 0.9f;
  if (heating && err > 5.0f && dFilt <= -HEAVY_FALL_CPS) {
    if (fallSince == 0) fallSince = nowMs;
    if (nowMs - fallSince >= HEAVY_MS) heavyLoad = true;
  } else {
    fallSince = 0;
  }
  if (heavyLoad && (fault || err < 3.0f || duty < 0.6f || dFilt > 0.5f)) heavyLoad = false;

  bool settled = fabsf(err) < 2.0f && fabsf(dFilt) < 1.5f && (setC - coldC) > 30.0f;
  if (!settled || tuneState != T_IDLE) {
    settledSince = 0;
  } else {
    if (settledSince == 0) settledSince = nowMs;
    if (nowMs - settledSince > 6000 && nowMs - lastLearn >= 1000) {
      lastLearn = nowMs;
      float target = constrain(iTerm / (setC - coldC), 0.0001f, 0.01f);
      holdPerC += 0.2f * (target - holdPerC);
    }
  }

  if (!boost && duty < SUPPLY_BASE_DUTY && vinRawOff > 0.0f) vinBase = vinOffV();

}

void printStatus() {
  Serial.printf("mode=%s k=%.5f cj=%.1f set=%.1f kp=%.4f ki=%.5f kd=%.4f hold=%.5f tune=%s adapt=%s s=%.2f vin=%.2f vs=%.5f ntc=%.1f%s off=%.1f\n",
                calMode ? "CAL" : (idleMode ? "IDLE" : "RUN"), tcK, coldC, setC, kp, ki, kd, holdPerC, tuneName(), adaptOn ? "on" : "off", adaptS, vinOffV(), vinScale, ntcC, ntcOk ? "" : "(x)", coldOff);
}

bool parseF(const char* s, float& out) {
  char* end;
  float v = strtof(s, &end);
  if (end == s) return false;
  while (*end == ' ') end++;
  if (*end) return false;
  out = v;
  return true;
}

void handleCmd(char* line) {
  activityMs = millis();
  char* arg = strchr(line, ' ');
  float v = 0.0f;
  bool num = false;
  if (arg) {
    *arg++ = 0;
    num = parseF(arg, v);
    if (!num && strcmp(arg, "apply") && strcmp(arg, "load") && strcmp(arg, "on") && strcmp(arg, "off") && strcmp(arg, "reset")) {
      Serial.println("bad arg");
      return;
    }
  }

  if (!strcmp(line, "cal")) {
    if (tuneState != T_IDLE) tuneStop("cal");
    calMode = true;
    idleMode = false;
    heaterOff();
    printStatus();
  } else if (!strcmp(line, "idle")) {
    if (tuneState != T_IDLE) tuneStop("idle");
    calMode = false;
    idleMode = true;
    heaterOff();
    printStatus();
  } else if (!strcmp(line, "enc")) {
    if (arg && !strcmp(arg, "on")) encDebug = true;
    if (arg && !strcmp(arg, "off")) encDebug = false;
    encReport();
  } else if (!strcmp(line, "run")) {
    if (tuneState != T_IDLE) tuneStop("run");
    if (num) setC = constrain(v, 0.0f, MAX_C - 5.0f);
    calMode = false;
    idleMode = false;
    resetPid();
    printStatus();
  } else if (!strcmp(line, "tune")) {
    if (arg && !strcmp(arg, "apply")) {
      if (pendValid) {
        applyPending();
        printStatus();
      } else {
        Serial.println("nothing to apply");
      }
    } else if (fault) {
      Serial.println("fault");
    } else if (arg && !strcmp(arg, "load")) {
      if (calMode || idleMode) {
        Serial.println("run first");
      } else {
        if (pendValid) applyPending();
        adaptRun = false;
        tuneState = L_SETTLE;
        tuneStartMs = millis();
        tuneMark = 0;
        Serial.println("tune load: settling");
      }
    } else {
      if (num) setC = constrain(v, 50.0f, MAX_C - 5.0f);
      if (calMode || idleMode) {
        calMode = false;
        idleMode = false;
        resetPid();
      }
      tuneState = T_SETTLE;
      tuneStartMs = millis();
      tuneMark = 0;
      Serial.println("tune: settling");
    }
  } else if (!strcmp(line, "kp") && num && v >= 0.0f && v < 1.0f) {
    kp = v;
    if (adaptOn) adaptReset();
    printStatus();
  } else if (!strcmp(line, "ki") && num && v >= 0.0f && v < 1.0f) {
    ki = v;
    if (adaptOn) adaptReset();
    printStatus();
  } else if (!strcmp(line, "kd") && num && v >= 0.0f && v < 5.0f) {
    kd = v;
    printStatus();
  } else if (!strcmp(line, "adapt")) {
    adaptOn = !(arg && !strcmp(arg, "off"));
    if (adaptOn) adaptReset();
    else if (adaptRun) tuneStop("adapt off");
    printStatus();
  } else if (!strcmp(line, "cold")) {
    float nc = coldC + v - tempC;
    if (num && !fault && tempInit && nc > -20.0f && nc < 80.0f) {
      float d = v - tempC;
      coldC = nc;
      if (ntcOk) coldOff = nc - ntcC;
      tempC = v;
      rateTemp += d;
      printStatus();
    } else {
      Serial.println("cold <temp> (valid reading needed)");
    }
  } else if (!strcmp(line, "stats")) {
    if (arg && !strcmp(arg, "reset")) statReset();
    statPrint();
  } else if (!strcmp(line, "clear")) {
    clearFault();
    Serial.println("cleared");
  } else if (!strcmp(line, "?")) {
    printStatus();
  } else {
    Serial.println("run [temp] cal tune [temp|load|apply] adapt [off] cold <temp> kp ki kd idle enc stats [reset] clear ?");
  }
}

void readSerial() {
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\n' || ch == '\r') {
      if (cmdLen) {
        cmdBuf[cmdLen] = 0;
        handleCmd(cmdBuf);
        cmdLen = 0;
      }
    } else if (cmdLen < sizeof(cmdBuf) - 1) {
      cmdBuf[cmdLen++] = ch;
    }
  }
}

const uint8_t ES_START = 0;
const uint8_t ES_CW_FINAL = 1;
const uint8_t ES_CW_BEGIN = 2;
const uint8_t ES_CW_NEXT = 3;
const uint8_t ES_CCW_BEGIN = 4;
const uint8_t ES_CCW_FINAL = 5;
const uint8_t ES_CCW_NEXT = 6;
const uint8_t ED_CW = 0x10;
const uint8_t ED_CCW = 0x20;

uint8_t encTable[7][4] = {
  {ES_START, ES_CW_BEGIN, ES_CCW_BEGIN, ES_START},
  {ES_CW_NEXT, ES_START, ES_CW_FINAL, ES_START | ED_CW},
  {ES_CW_NEXT, ES_CW_BEGIN, ES_START, ES_START},
  {ES_CW_NEXT, ES_CW_BEGIN, ES_CW_FINAL, ES_START},
  {ES_CCW_NEXT, ES_START, ES_CCW_BEGIN, ES_START},
  {ES_CCW_NEXT, ES_CCW_FINAL, ES_START, ES_START | ED_CCW},
  {ES_CCW_NEXT, ES_CCW_FINAL, ES_CCW_BEGIN, ES_START},
};
uint8_t encState = ES_START;
volatile int encDelta = 0;
bool encBtnRaw = true;
bool encBtnStable = true;
uint32_t encBtnMs = 0;
uint32_t encBtnDownMs = 0;
bool encBtnStuck = false;
int encBtnAcc = (int)ENC_BTN_DEBOUNCE_MS;
uint32_t encPollUs = 0;

void IRAM_ATTR encIsr() {
  encEdges = encEdges + 1;
  uint32_t g = GPI;
  uint8_t pins = ((g >> ENC_A) & 1) | (((g >> ENC_B) & 1) << 1);
  uint8_t st = encTable[encState][pins];
  encState = st & 0x0f;
  uint8_t dir = st & 0x30;
  if (dir == ED_CW) encDelta = encDelta + 1;
  else if (dir == ED_CCW) encDelta = encDelta - 1;
}

void encInit() {
  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);
  pinMode(ENC_BTN, ENC_BTN_ACTIVE_HIGH ? INPUT_PULLDOWN_16 : INPUT);
  encState = ES_START;
  encDelta = 0;
  attachInterrupt(digitalPinToInterrupt(ENC_A), encIsr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B), encIsr, CHANGE);
}

void encDbg(const char* fmt, ...) __attribute__((format(printf, 1, 2)));

void encDbg(const char* fmt, ...) {
  if (!encDebug) return;
  if (Serial.availableForWrite() < 100) {
    encDbgDrop++;
    return;
  }
  char b[96];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(b, sizeof(b), fmt, ap);
  va_end(ap);
  Serial.print(b);
}

void encDbgTick(uint32_t nowMs) {
  if (!encDebug || nowMs - encDbgMs < 1000) return;
  encDbgMs = nowMs;
  uint32_t g = GPI;
  encDbg("[enc] hb L+%lu S+%lu p+%lu e+%lu f+%lu d=%d AB=%d%d b=%d/%d %s F%d T%d K%d\n", (unsigned long)(encLoops - encDbgL), (unsigned long)(encSvcCalls - encDbgS), (unsigned long)(encPolls - encDbgP), (unsigned long)(encEdges - encDbgE), (unsigned long)(encFlips - encDbgF), (int)encDelta, (int)((g >> ENC_A) & 1), (int)((g >> ENC_B) & 1), encBtnRaw, encBtnStable, calMode ? "CAL" : (idleMode ? "IDLE" : "RUN"), fault, tuneState != T_IDLE, encBtnStuck);
  encDbgL = encLoops;
  encDbgS = encSvcCalls;
  encDbgP = encPolls;
  encDbgE = encEdges;
  encDbgF = encFlips;
}

void encPress() {
  encPresses++;
  activityMs = millis();
  if (fault) {
    clearFault();
    calMode = false;
    idleMode = true;
    heaterOff();
    encDbg("[enc] press: fault cleared -> IDLE\n");
    return;
  }
  if (tuneState != T_IDLE) {
    tuneStop("button");
    calMode = false;
    idleMode = true;
    heaterOff();
    encDbg("[enc] press: tune abort -> IDLE\n");
    return;
  }
  if (calMode || idleMode) {
    calMode = false;
    idleMode = false;
    resetPid();
    encDbg("[enc] press -> RUN set=%.0f\n", setC);
  } else {
    idleMode = true;
    heaterOff();
    encDbg("[enc] press -> IDLE\n");
  }
}

void encReport() {
  Serial.printf("enc A=%d B=%d btn=%d stable=%d polls=%lu edges=%lu flips=%lu steps=%ld presses=%lu ui=%lu/%lu/%lu runs=%lu dbg=%d drop=%lu stuck=%d mode=%s\n", digitalRead(ENC_A), digitalRead(ENC_B), encBtnRaw, encBtnStable, (unsigned long)encPolls, (unsigned long)encEdges, (unsigned long)encFlips, (long)encSteps, (unsigned long)encPresses, (unsigned long)uiCost[0], (unsigned long)uiCost[1], (unsigned long)uiCost[2], (unsigned long)uiRuns, encDebug, (unsigned long)encDbgDrop, encBtnStuck, calMode ? "CAL" : (idleMode ? "IDLE" : "RUN"));
}

const uint32_t ENC_ROT_DELAY_MS = 60;
const uint32_t ENC_ROT_HOLDOFF_MS = 150;
uint32_t encPendMs = 0;
bool encPend = false;
bool encScrolled = true;

void presetScroll(int d) {
  int i = -1;
  for (int k = 0; k < PRESET_N; k++) {
    if (setC == PRESETS[k]) i = k;
  }
  if (i < 0) {
    if (d > 0) {
      i = PRESET_N - 1;
      for (int k = 0; k < PRESET_N; k++) {
        if (PRESETS[k] > setC) {
          i = k;
          break;
        }
      }
      d--;
    } else {
      i = 0;
      for (int k = PRESET_N - 1; k >= 0; k--) {
        if (PRESETS[k] < setC) {
          i = k;
          break;
        }
      }
      d++;
    }
  }
  i = constrain(i + d, 0, PRESET_N - 1);
  presetIdx = i;
  setC = PRESETS[i];
}

void encService() {
  uint32_t nowUs = micros();
  uint32_t nowMs = millis();
  encSvcCalls++;
  encDbgTick(nowMs);
  if (nowUs - encPollUs >= ENC_POLL_US) {
    uint32_t dtMs = (nowUs - encPollUs) / 1000;
    if (dtMs > 20) dtMs = 20;
    encPollUs = nowUs;
    encPolls++;
    bool raw = digitalRead(ENC_BTN) == (ENC_BTN_ACTIVE_HIGH ? HIGH : LOW);
    if (raw != encBtnRaw) {
      encBtnRaw = raw;
      encFlips++;
    }
    if (raw) encBtnAcc += (int)dtMs;
    else encBtnAcc -= (int)dtMs;
    encBtnAcc = constrain(encBtnAcc, 0, (int)ENC_BTN_DEBOUNCE_MS);
    if (!encBtnStable && encBtnAcc >= (int)ENC_BTN_DEBOUNCE_MS) {
      encBtnStable = true;
      encBtnMs = nowMs;
      encBtnDownMs = nowMs;
      encScrolled = false;
      encDbg("[enc] btn DOWN\n");
    } else if (encBtnStable && encBtnAcc <= 0) {
      encBtnStable = false;
      encBtnStuck = false;
      encBtnMs = nowMs;
      encDbg("[enc] btn UP\n");
      if (!encScrolled) encPress();
    } else if (encBtnStable && !encBtnStuck && nowMs - encBtnDownMs >= ENC_BTN_STUCK_MS) {
      encBtnStuck = true;
      encDbg("[enc] btn stuck pressed: check D0 pull-up\n");
    }
    if (encBtnStable && !encScrolled && nowMs - encBtnDownMs >= TOP_HOLD_MS) {
      encScrolled = true;
      topStatPage = !topStatPage;
      activityMs = nowMs;
      if (uiGrOk) {
        uiPart = 3;
        lastUiUs = micros() - UI_PART_US;
      }
      encDbg("[enc] hold -> top page %d\n", topStatPage);
    }
  }
  if (!encPend) {
    if (encDelta == 0) return;
    encPend = true;
    encPendMs = nowMs;
    return;
  }
  if (nowMs - encPendMs < ENC_ROT_DELAY_MS) return;
  encPend = false;
  noInterrupts();
  int d = encDelta;
  encDelta = 0;
  interrupts();
  if (d == 0) return;
  activityMs = nowMs;
  const char* why = nullptr;
  if (nowMs - encBtnMs < ENC_ROT_HOLDOFF_MS) why = "hold";
  else if (fault) why = "fault";
  else if (tuneState != T_IDLE) why = "tune";
  else if (!calMode && !idleMode) why = "run";
  if (why) {
    if (encBtnStable && !strcmp(why, "run")) encScrolled = true;
    encDbg("[enc] rot %+d DROP %s\n", d, why);
    return;
  }
  if (encBtnStable) {
    encScrolled = true;
    presetScroll(ENC_DIR * d);
    encSteps += d;
    resetPid();
    encDbg("[enc] preset %+d set=%.0f\n", d, setC);
    return;
  }
  setC = constrain(setC + ENC_DIR * d * ENC_STEP_C, ENC_MIN_C, MAX_C - 5.0f);
  encSteps += d;
  resetPid();
  encDbg("[enc] rot %+d set=%.0f\n", d, setC);
}

const char* statusText() {
  if (fault) return faultMsg;
  if (calMode) return "CAL";
  if (idleMode) return "IDLE";
  if (tuneState != T_IDLE) return tuneName();
  if (heavyLoad) return "HEAVY LOAD - POWER LIMITED";
  return boost ? "BOOST" : "HOLD";
}

uint16_t tempColor() {
  if (idleMode) return TFT_WHITE;
  float e = fabsf(setC - tempC);
  float t = constrain(1.0f - (e - UI_NEAR_C) / (UI_FAR_C - UI_NEAR_C), 0.0f, 1.0f);
  int r = 255;
  int g = 255;
  int b = 110;
  if (t < 0.5f) g = 110 + (int)(290.0f * t);
  else r = 255 - (int)(290.0f * (t - 0.5f));
  return tft.color565(r, g, b);
}

void barStep() {
  float pct = constrain(dutyPct, 0.0f, 100.0f);
  int bw = UI_BAR_BW;
  int bx = (UI_BAR_W - bw) / 2;
  int by = (UI_BAR_H - UI_BAR_BH) / 2;
  int ir = UI_BAR_R - 2;
  int ih = UI_BAR_BH - 4;
  int fill = (int)lroundf(pct * (bw - 4) / 100.0f);
  char d[8];
  snprintf(d, sizeof(d), "%d%%", (int)lroundf(pct));
  spB.fillSprite(TFT_BLACK);
  spB.drawRoundRect(bx, by, bw, UI_BAR_BH, UI_BAR_R, tft.color565(90, 90, 90));
  if (fill > 0) {
    uint16_t red = tft.color565(255, 50, 50);
    if (fill >= 2 * ir + 1) spB.fillRoundRect(bx + 2, by + 2, fill, ih, ir, red);
    else spB.fillRect(bx + 2, by + 2, fill, ih, red);
  }
  spB.setTextDatum(TC_DATUM);
  spB.setTextColor(TFT_WHITE);
  spB.drawString(d, UI_BAR_W / 2, by + (UI_BAR_BH - spB.fontHeight()) / 2);
  spB.pushSprite(0, uiBarY);
}

const char* stateTag(uint16_t& col) {
  if (fault) {
    col = TFT_RED;
    return faultMsg;
  }
  if (calMode) {
    col = TFT_CYAN;
    return "CAL";
  }
  if (idleMode) {
    col = TFT_LIGHTGREY;
    return "IDLE";
  }
  if (tuneState != T_IDLE) {
    col = TFT_YELLOW;
    return tuneName();
  }
  if (heavyLoad) {
    col = TFT_MAGENTA;
    return "HEAVY LOAD";
  }
  if (boost) {
    col = TFT_ORANGE;
    return "BOOST";
  }
  col = TFT_GREEN;
  return "HOLD";
}

const int CD_COLON_W = 6;
const int CD_DOT_X = 2;
const int CD_DOT_W = 2;
const int CD_DOT_H = 3;
const int CD_DOT_TOP_Y = 4;
const int CD_DOT_BOT_Y = 11;

bool idleCountdownMs(uint32_t& left) {
  if (calMode || idleMode || fault || tuneState != T_IDLE) return false;
  uint32_t el = millis() - activityMs;
  left = el >= AUTO_OFF_MS ? 0 : AUTO_OFF_MS - el;
  return true;
}

void topStep() {
  char vn[10];
  char in[10];
  char pn[10];
  float vin = vinOnV();
  float ieff = fabsf(amps) * dutyPct / 100.0f;
  float pw = fabsf(vin * ieff);
  uint16_t c1 = vinRed ? TFT_RED : tft.color565(255, 215, 0);
  uint16_t c2 = TFT_WHITE;
  uint16_t c3 = tft.color565(255, 140, 50);
  snprintf(vn, sizeof(vn), "%.1fV", vin);
  if (ieff < 9.95f) snprintf(in, sizeof(in), "%.1fA", ieff);
  else snprintf(in, sizeof(in), "%.0fA", ieff);
  snprintf(pn, sizeof(pn), "%.0fW", pw);
  int wv = spH.textWidth(vn);
  int wi = spH.textWidth(in);
  int wp = spH.textWidth(pn);
  int gap = (UI_BAR_W - wv - wi - wp) / 4;
  if (gap < 0) gap = 0;
  int x = gap;
  spH.fillSprite(TFT_BLACK);
  spH.setTextDatum(TL_DATUM);
  spH.setTextColor(c1);
  spH.drawString(vn, x, UI_TOP_TY);
  x += wv + gap;
  spH.setTextColor(c2);
  spH.drawString(in, x, UI_TOP_TY);
  x += wi + gap;
  spH.setTextColor(c3);
  spH.drawString(pn, x, UI_TOP_TY);
  uint16_t sc;
  const char* st = stateTag(sc);
  int ws = spH.textWidth(st);
  uint32_t left = 0;
  bool cd = idleCountdownMs(left);
  char cm[8];
  char cs[8];
  int wm = 0;
  int wcd = 0;
  if (cd) {
    uint32_t sec = (left + 999) / 1000;
    snprintf(cm, sizeof(cm), "%lu", (unsigned long)(sec / 60));
    snprintf(cs, sizeof(cs), "%02lu", (unsigned long)(sec % 60));
    wm = spH.textWidth(cm);
    wcd = wm + CD_COLON_W + spH.textWidth(cs);
  }
  int xs = (UI_BAR_W - ws) / 2;
  if (cd && xs + ws + UI_GAP > UI_BAR_W - wcd) xs = 0;
  if (cd && xs + ws + UI_GAP > UI_BAR_W - wcd) cd = false;
  spH.setTextColor(sc);
  spH.drawString(st, xs, UI_STATE_TY);
  if (cd) {
    uint16_t cc = left <= 30000 ? TFT_RED : TFT_LIGHTGREY;
    int x0 = UI_BAR_W - wcd;
    spH.setTextColor(cc);
    spH.drawString(cm, x0, UI_STATE_TY);
    spH.fillRect(x0 + wm + CD_DOT_X, UI_STATE_TY + CD_DOT_TOP_Y, CD_DOT_W, CD_DOT_H, cc);
    spH.fillRect(x0 + wm + CD_DOT_X, UI_STATE_TY + CD_DOT_BOT_Y, CD_DOT_W, CD_DOT_H, cc);
    spH.drawString(cs, x0 + wm + CD_COLON_W, UI_STATE_TY);
  }
  spH.pushSprite(0, 0);
}

void graphStep() {
  uint32_t now = millis();
  if (grCnt == 0 || now - grMs >= GR_SAMPLE_MS) {
    grMs = now;
    grT[grHead] = tempC;
    grHead = (grHead + 1) % GR_N;
    if (grCnt < GR_N) grCnt++;
  }
  if (topStatPage) {
    char sa[12];
    char sb[12];
    char sc[12];
    float wh = (float)statWh;
    float avgW = statRunMs > 0 ? (float)(statWh * 3600000.0 / statRunMs) : 0.0f;
    if (wh < 100.0f) snprintf(sa, sizeof(sa), "%.1fWH", wh);
    else snprintf(sa, sizeof(sa), "%.0fWH", wh);
    snprintf(sb, sizeof(sb), "%.0fW", avgW);
    if (ntcOk) snprintf(sc, sizeof(sc), "%.0fC", ntcC);
    else snprintf(sc, sizeof(sc), "--");
    int wa = spG.textWidth(sa);
    int wb = spG.textWidth(sb);
    int wc = spG.textWidth(sc);
    int sgap = (UI_BAR_W - wa - wb - wc) / 4;
    if (sgap < 0) sgap = 0;
    int sy = (uiGrH - spG.fontHeight()) / 2;
    if (sy < 0) sy = 0;
    spG.fillSprite(TFT_BLACK);
    spG.setTextDatum(TL_DATUM);
    spG.setTextColor(tft.color565(0, 220, 220));
    int sx = sgap;
    spG.drawString(sa, sx, sy);
    sx += wa + sgap;
    spG.drawString(sb, sx, sy);
    sx += wb + sgap;
    spG.setTextColor(ntcAccepted ? TFT_GREEN : TFT_RED);
    spG.drawString(sc, sx, sy);
    spG.pushSprite(0, uiGrY);
    return;
  }
  float dlo = grT[(grHead - 1 + GR_N) % GR_N];
  float dhi = dlo;
  for (int k = 0; k < grCnt; k++) {
    float v = grT[(grHead - grCnt + k + GR_N) % GR_N];
    if (v < dlo) dlo = v;
    if (v > dhi) dhi = v;
  }
  if (setC >= dlo - GR_SET_NEAR_C && setC <= dhi + GR_SET_NEAR_C) {
    if (setC < dlo) dlo = setC;
    if (setC > dhi) dhi = setC;
  }
  float pad = 0.15f * (dhi - dlo);
  if (pad < 0.5f) pad = 0.5f;
  float tlo = dlo - pad;
  float thi = dhi + pad;
  if (thi - tlo < GR_MIN_SPAN_C) {
    float m = 0.5f * (thi + tlo);
    tlo = m - 0.5f * GR_MIN_SPAN_C;
    thi = m + 0.5f * GR_MIN_SPAN_C;
  }
  if (!grRangeInit) {
    grLo = tlo;
    grHi = thi;
    grRangeInit = true;
  }
  grLo = tlo < grLo ? tlo : grLo + GR_SHRINK * (tlo - grLo);
  grHi = thi > grHi ? thi : grHi + GR_SHRINK * (thi - grHi);
  float lo = grLo;
  float hi = grHi;
  float span = hi - lo;
  int h = uiGrH;
  spG.fillSprite(TFT_BLACK);
  if (setC >= lo && setC <= hi) {
    int ys = (int)lroundf((h - 1) * (hi - setC) / span);
    uint16_t grey = tft.color565(90, 90, 90);
    for (int x = 0; x < UI_BAR_W; x += 6) spG.drawFastHLine(x, ys, 3, grey);
  }
  uint16_t col = tempColor();
  int px = 0;
  int py = 0;
  for (int k = 0; k < grCnt; k++) {
    float v = grT[(grHead - grCnt + k + GR_N) % GR_N];
    int x = UI_BAR_W - 1 - (grCnt - 1 - k) * (UI_BAR_W / GR_N);
    int y = constrain((int)lroundf((h - 1) * (hi - v) / span), 0, h - 1);
    if (k > 0) spG.drawLine(px, py, x, y, col);
    px = x;
    py = y;
  }
  spG.pushSprite(0, uiGrY);
}

void memReport(const char* tag) {
  Serial.printf("[MEM] %s: free %u B, max block %u B, frag %u%%\n", tag, (unsigned)ESP.getFreeHeap(), (unsigned)ESP.getMaxFreeBlockSize(), (unsigned)ESP.getHeapFragmentation());
}

void uiInit() {
  tft.init();
  tft.setRotation(3);
  tft.fillScreen(TFT_BLACK);
  pinMode(HEATER_PIN, OUTPUT);
  digitalWrite(HEATER_PIN, LOW);
  memReport("before sprites");
  spT.setColorDepth(16);
  uiOk = spT.createSprite(UI_SPR_W, UI_SPR_H) != nullptr;
  if (!uiOk) {
    spT.setColorDepth(UI_T_DEPTH);
    uiOk = spT.createSprite(UI_SPR_W, UI_SPR_H) != nullptr;
  }
  memReport(uiOk ? "spT ok" : "spT FAILED");
  if (uiOk) {
    spT.loadFont(nunitoFont);
    spT.setTextWrap(false, false);
  }
  uiBarY = tft.height() - UI_BAR_H;
  spB.setColorDepth(16);
  uiBarOk = spB.createSprite(UI_BAR_W, UI_BAR_H) != nullptr;
  if (!uiBarOk) {
    spB.setColorDepth(8);
    uiBarOk = spB.createSprite(UI_BAR_W, UI_BAR_H) != nullptr;
  }
  memReport(uiBarOk ? "spB ok" : "spB FAILED");
  spH.setColorDepth(16);
  uiTopOk = spH.createSprite(UI_BAR_W, UI_TOP_H) != nullptr;
  if (!uiTopOk) {
    spH.setColorDepth(8);
    uiTopOk = spH.createSprite(UI_BAR_W, UI_TOP_H) != nullptr;
  }
  memReport(uiTopOk ? "spH ok" : "spH FAILED");
  if (uiTopOk) {
    spH.loadFont(nunitoTopFont);
    spH.setTextWrap(false, false);
  }
  if (uiBarOk) {
    spB.loadFont(nunitoBarFont);
    spB.setTextWrap(false, false);
  }
  uiGrY = UI_PUSH_Y + UI_SPR_H;
  uiGrH = uiBarY - uiGrY;
  if (uiGrH > 30) uiGrH = 30;
  if (uiGrH >= 12) {
    spG.setColorDepth(16);
    uiGrOk = spG.createSprite(UI_BAR_W, uiGrH) != nullptr;
    if (!uiGrOk) {
      spG.setColorDepth(8);
      uiGrOk = spG.createSprite(UI_BAR_W, uiGrH) != nullptr;
    }
    if (uiGrOk) {
      spG.loadFont(nunitoTopFont);
      spG.setTextWrap(false, false);
    }
  }
  memReport(uiGrOk ? "spG ok" : "spG skipped");
  memReport("ui ready");
}

const float DISP_TAU_S = 0.5f;
const float DISP_FAST_TAU_S = 0.1f;
const float DISP_FAST_C = 5.0f;
const float DISP_HYST_C = 0.7f;
float dispC = 0.0f;
int dispInt = 0;
uint32_t dispMs = 0;
bool dispInit = false;

void dispUpdate() {
  uint32_t now = millis();
  if (!tempInit || !dispInit) {
    dispC = tempC;
    dispInt = (int)lroundf(tempC);
    dispMs = now;
    dispInit = tempInit;
    return;
  }
  float dt = constrain((now - dispMs) * 1e-3f, 0.0f, 1.0f);
  dispMs = now;
  float tau = fabsf(tempC - dispC) > DISP_FAST_C ? DISP_FAST_TAU_S : DISP_TAU_S;
  dispC += (dt / (tau + dt)) * (tempC - dispC);
  if (fabsf(dispC - dispInt) > DISP_HYST_C) dispInt = (int)lroundf(dispC);
}

void uiMainStep() {
  char big[8];
  char sm[12];
  uint16_t col = tempColor();
  dispUpdate();
  snprintf(big, sizeof(big), "%d", dispInt);
  int n = snprintf(sm, sizeof(sm), "/%d", (int)lroundf(setC));
  if (n > 6) n = 6;
  for (int i = 1; i < n; i++) {
    if (sm[i] >= '0' && sm[i] <= '9') sm[i] = sm[i] - '0' + 'a';
  }
  sm[n++] = '*';
  sm[n++] = 'C';
  sm[n] = 0;
  spT.fillSprite(TFT_BLACK);
  spT.setTextColor(col, TFT_BLACK);
  spT.setTextDatum(TL_DATUM);
  int wBig = spT.textWidth(big);
  int wSm = spT.textWidth(sm);
  int x = (UI_SPR_W - (wBig + UI_GAP + wSm)) / 2;
  if (x < 0) x = 0;
  spT.drawString(big, x, 0);
  spT.drawString(sm, x + wBig + UI_GAP, 0);
  char nb[10];
  if (ntcOk) snprintf(nb, sizeof(nb), "%dUV", constrain((int)lroundf(ntcC), -9, 99));
  else snprintf(nb, sizeof(nb), "--");
  for (char* c = nb; *c; c++) {
    if (*c >= '0' && *c <= '9') *c = *c - '0' + 'K';
    else if (*c == '-') *c = 'w';
  }
  int wn = spT.textWidth(nb);
  int xc = x + wBig + UI_GAP + wSm / 2;
  if (xc < wn / 2) xc = wn / 2;
  if (xc > UI_SPR_W - wn / 2) xc = UI_SPR_W - wn / 2;
  spT.setTextColor(ntcAccepted ? TFT_GREEN : TFT_RED, TFT_BLACK);
  spT.setTextDatum(C_BASELINE);
  spT.drawString(nb, xc, UI_SUB_BASE);
  spT.pushSprite(0, UI_PUSH_Y);
}

void uiStep() {
  if (uiPart == 0) uiMainStep();
  else if (uiPart == 1) barStep();
  else if (uiPart == 2) topStep();
  else graphStep();
  do {
    uiPart = (uiPart + 1) % 4;
  } while ((uiPart == 1 && !uiBarOk) || (uiPart == 2 && !uiTopOk) || (uiPart == 3 && !uiGrOk));
}

void printLine() {
  unsigned long now = millis();
  lastPrint = now;
  uint32_t nowUs = micros();
  uint32_t acc = onAccumUs;
  if (heaterOn) {
    acc += nowUs - accMarkUs;
    statAdd(nowUs - accMarkUs);
    accMarkUs = nowUs;
  }
  float realPct = 100.0f * acc / (float)(nowUs - lastPrintUs);
  if (realPct > 100.0f) realPct = 100.0f;
  dutyPct = realPct;
  onAccumUs = 0;
  lastPrintUs = nowUs;
  Serial.printf("T: %.1f C  |  mV: %.3f", tempC, tcMv);
  Serial.printf("  |  I: %.2f A (%d)  |  D: %.0f%%  |  Ieff: %.2f A  |  V: %.1f V  |  P: %.1f W  |  ", amps, rawI, realPct, amps * realPct / 100.0f, vinOnV(), vinOnV() * amps * realPct / 100.0f);
  if (fault) Serial.print("FAULT ");
  Serial.println(statusText());
}

void waitUntil(uint32_t target) {
  for (;;) {
    int32_t left = (int32_t)(target - micros());
    if (left <= 0) return;
    readSerial();
    encService();
    if (uiOk && left > (int32_t)(uiCost[uiPart] + 500) && (uint32_t)(micros() - lastUiUs) >= UI_PART_US) {
      uint32_t t0 = micros();
      uint8_t part = uiPart;
      uiStep();
      uint32_t cost = micros() - t0;
      if (cost > uiCost[part] || cost + 1500 < uiCost[part]) uiCost[part] = cost + 300;
      uiRuns++;
      lastUiUs = micros();
      continue;
    }
    if (left > 2500 && millis() - lastPrint >= 250) {
      printLine();
      continue;
    }
    yield();
  }
}

void runWindow() {
  nextWin += WINDOW_US;
  uint32_t t0 = micros();
  if ((int32_t)(t0 - nextWin) > (int32_t)WINDOW_US) nextWin = t0 + WINDOW_US;
  uint32_t winStart = nextWin - WINDOW_US;

  if (!fault && !calMode && !idleMode && boostLeft > 0) {
    boostLeft--;
    heaterSet();
    waitUntil(nextWin);
    return;
  }

  if (heaterOn) {
    readCurrent();
    if (micros() - onStartUs >= MIN_ON_US) checkShunt();
    checkSupply();
    heaterOff();
  }
  uint32_t offStart = micros();
  while ((uint32_t)(micros() - offStart) < SETTLE_US) {
  }
  bool ntcThis = (++ntcTick % NTC_EVERY_WIN) == 0;
  if (ntcThis) ntcUpdate();
  readTemp();
  if ((++vinTick & 3) == 0) vinUpdate(vinRawOff);
  if (!fault) control();

  if (fault) {
    duty = 0.0f;
    boost = false;
    boostLeft = 0;
    heaterOff();
    waitUntil(nextWin);
    return;
  }

  uint32_t used = micros() - winStart;
  uint32_t avail = used < WINDOW_US ? WINDOW_US - used : 0;
  if (!boost) {
    dutyCap += 0.05f * ((float)avail / WINDOW_US - dutyCap);
    dutyCap = constrain(dutyCap, 0.5f, 0.95f);
  }
  uint32_t onUs = (uint32_t)(duty * WINDOW_US);
  if (onUs > avail) onUs = avail;
  if (boost) {
    boostLeft = BOOST_SKIP;
    onUs = avail;
    probeDebtUs = 0;
  } else if (onUs > 0 && onUs < CUR_MIN_PULSE_US && avail >= CUR_MIN_PULSE_US && millis() - ampsMs >= CUR_REFRESH_MS) {
    probeDebtUs += CUR_MIN_PULSE_US - onUs;
    if (probeDebtUs > PROBE_DEBT_MAX_US) probeDebtUs = PROBE_DEBT_MAX_US;
    onUs = CUR_MIN_PULSE_US;
  } else if (probeDebtUs > 0) {
    uint32_t cut = onUs / 2;
    if (cut > probeDebtUs) cut = probeDebtUs;
    probeDebtUs -= cut;
    onUs -= cut;
  }
  if (onUs < MIN_PULSE_US) {
    waitUntil(nextWin);
    return;
  }
  heaterSet();
  uint32_t offAt = onStartUs + onUs;
  if (boost) {
    waitUntil(nextWin);
    return;
  }
  if (onUs >= CUR_MIN_PULSE_US) {
    waitUntil(offAt - CUR_READ_US);
    if (!heaterOn) {
      waitUntil(nextWin);
      return;
    }
    readCurrent();
    if (micros() - onStartUs >= MIN_ON_US) checkShunt();
    checkSupply();
  }
  waitUntil(offAt);
  heaterOff();
  waitUntil(nextWin);
}

void setup() {
  Serial.begin(74880);

  pinMode(HEATER_PIN, OUTPUT);
  digitalWrite(HEATER_PIN, LOW);
  timer1_isr_init();
  timer1_attachInterrupt(deadmanIsr);
  timer1_enable(TIM_DIV16, TIM_EDGE, TIM_SINGLE);

  uiInit();
  Wire.begin(D2, D1);
  if (!ads.begin(0x48)) {
    Serial.println("ADS1115 not found");
    while (true) delay(1000);
  }
  Wire.setClock(I2C_HZ);
  ads.setDataRate(ADS_RATE);
  loadSettings();
  saveNow();
  ntcUpdate();
  encInit();
  lastPrintUs = micros();
  nextWin = micros();
}

void loop() {
  encLoops++;
  readSerial();
  encService();
  saveService();
  statService();
  autoOffService();

  if ((int32_t)(micros() - nextWin) >= 0) runWindow();
  else waitUntil(nextWin);
}