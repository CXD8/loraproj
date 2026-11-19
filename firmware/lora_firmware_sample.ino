// LoRa_Check.ino - very basic SX1276 detect / validate sketch for the LORA.PrjPcb board
//
// ESP32-S3-WROOM-1U-N16R8 + SX1276. Single file, no libraries, NEVER TRANSMITS
// (safe to run with no antenna fitted).
//
// At boot (and when you send 'r') it runs these checks and prints PASS / FAIL:
//   1. SPI detect        RegVersion (0x42) reads 0x12
//   2. Register R/W      write + read back patterns in RegSyncWord
//   3. Mode control      sleep -> LoRa sleep -> standby, read back RegOpMode
//   4. Crystal + PLL     PLL locks at 490.3 MHz (CN470) and 916.8 MHz (AU915)
//   5. DIO0 / DIO3       a CAD (channel activity detect, receive only) raises
//                        CadDone on DIO0 and DIO3, and clearing it drops them
//   6. Noise floor       average RSSI on each band (info only)
//
// After that it prints a one-line live status every 5 s (s = print now).
// Serial: 115200 over native USB (Tools > USB CDC On Boot: Enabled).
//
// Pins are the same as ../LoRa_Basic/board_pins.h (taken from the PCB netlist).

#include <Arduino.h>
#include <SPI.h>

// ------------------------------------------------------------------ pins
static const int PIN_NSS  = 10;
static const int PIN_MOSI = 11;
static const int PIN_SCK  = 12;
static const int PIN_MISO = 13;
static const int PIN_DIO0 = 4;
static const int PIN_DIO3 = 7;
static const int PIN_SW_CN470 = 15;  // FEM_CPS -> U3, HIGH = RX port
static const int PIN_SW_AU915 = 16;  // FEM_CTX -> U4, LOW  = RX port

// ------------------------------------------------------------------ registers
static const uint8_t REG_OP_MODE      = 0x01;
static const uint8_t REG_FRF_MSB      = 0x06;
static const uint8_t REG_FRF_MID      = 0x07;
static const uint8_t REG_FRF_LSB      = 0x08;
static const uint8_t REG_PA_CONFIG    = 0x09;
static const uint8_t REG_LNA          = 0x0C;
static const uint8_t REG_IRQ_FLAGS    = 0x12;  // LoRa
static const uint8_t REG_RSSI         = 0x1B;  // LoRa
static const uint8_t REG_MODEM_CFG1   = 0x1D;
static const uint8_t REG_MODEM_CFG2   = 0x1E;
static const uint8_t REG_MODEM_CFG3   = 0x26;
static const uint8_t REG_SYNC_WORD    = 0x39;
static const uint8_t REG_IRQ_FLAGS1   = 0x3E;  // FSK page: bit4 = PllLock
static const uint8_t REG_DIO_MAPPING1 = 0x40;
static const uint8_t REG_VERSION      = 0x42;

static const uint8_t LORA = 0x80, LF = 0x08;
static const uint8_t M_SLEEP = 0, M_STDBY = 1, M_FSRX = 4, M_RXCONT = 5, M_CAD = 7;
static const uint8_t IRQ_CAD_DONE = 0x04;

static const uint32_t FXOSC = 32000000UL;
static const uint32_t F_CN470 = 490300000UL;
static const uint32_t F_AU915 = 916800000UL;

static SPISettings spiCfg(1000000, MSBFIRST, SPI_MODE0);  // slow = tolerant during bring-up
static int passCount = 0, failCount = 0;
static bool detected = false;

// ------------------------------------------------------------------ SPI
static uint8_t rd(uint8_t a) {
  SPI.beginTransaction(spiCfg);
  digitalWrite(PIN_NSS, LOW);
  SPI.transfer(a & 0x7F);
  uint8_t v = SPI.transfer(0);
  digitalWrite(PIN_NSS, HIGH);
  SPI.endTransaction();
  return v;
}

static void wr(uint8_t a, uint8_t v) {
  SPI.beginTransaction(spiCfg);
  digitalWrite(PIN_NSS, LOW);
  SPI.transfer(a | 0x80);
  SPI.transfer(v);
  digitalWrite(PIN_NSS, HIGH);
  SPI.endTransaction();
}

static void setFreq(uint32_t hz) {
  uint64_t frf = ((uint64_t)hz << 19) / FXOSC;
  wr(REG_FRF_MSB, frf >> 16);
  wr(REG_FRF_MID, frf >> 8);
  wr(REG_FRF_LSB, frf);
}

// The LoRa/FSK bit can only change in sleep, so always pass through sleep.
static void gotoSleep(bool lora, bool lf) {
  wr(REG_OP_MODE, (rd(REG_OP_MODE) & 0xF8) | M_SLEEP);  // sleep in current modem
  delay(2);
  wr(REG_OP_MODE, (lora ? LORA : 0) | (lf ? LF : 0) | M_SLEEP);
  delay(2);
}

static void result(const char* name, bool ok, const char* detail) {
  Serial.printf("  [%s] %-16s %s\n", ok ? "PASS" : "FAIL", name, detail);
  ok ? passCount++ : failCount++;
}

static void parkSwitchesRx() {
  digitalWrite(PIN_SW_CN470, HIGH);  // U3 -> RX
  digitalWrite(PIN_SW_AU915, LOW);   // U4 -> RX
}

// ------------------------------------------------------------------ checks
static bool checkDetect() {
  uint8_t v = 0;
  for (int i = 0; i < 5; i++) {
    v = rd(REG_VERSION);
    if (v == 0x12) break;
    delay(10);
  }
  char buf[96];
  if (v == 0x12)
    snprintf(buf, sizeof buf, "RegVersion = 0x12 (SX1276/77/78/79)");
  else if (v == 0x00)
    snprintf(buf, sizeof buf, "RegVersion = 0x00 -> MISO stuck low / radio unpowered / NSS not reaching U2");
  else if (v == 0xFF)
    snprintf(buf, sizeof buf, "RegVersion = 0xFF -> MISO floating (no chip answering)");
  else
    snprintf(buf, sizeof buf, "RegVersion = 0x%02X -> unexpected (wrong chip, SCK/MOSI fault?)", v);
  result("SPI detect", v == 0x12, buf);
  return v == 0x12;
}

static void checkRegisters() {
  gotoSleep(true, false);
  const uint8_t saved = rd(REG_SYNC_WORD);
  const uint8_t pats[] = {0x55, 0xAA, 0x00, 0xFF, 0x12};
  bool ok = true;
  uint8_t badW = 0, badR = 0;
  for (uint8_t p : pats) {
    wr(REG_SYNC_WORD, p);
    uint8_t r = rd(REG_SYNC_WORD);
    if (r != p) { ok = false; badW = p; badR = r; break; }
  }
  wr(REG_SYNC_WORD, saved);
  char buf[64];
  if (ok) snprintf(buf, sizeof buf, "0x55/0xAA/0x00/0xFF read back OK");
  else    snprintf(buf, sizeof buf, "wrote 0x%02X read 0x%02X (MOSI fault?)", badW, badR);
  result("Register R/W", ok, buf);
}

static void checkModes() {
  gotoSleep(true, false);
  const uint8_t s = rd(REG_OP_MODE);
  wr(REG_OP_MODE, LORA | M_STDBY);
  delay(2);
  const uint8_t st = rd(REG_OP_MODE);
  const bool ok = (s & 0x87) == (LORA | M_SLEEP) && (st & 0x87) == (LORA | M_STDBY);
  char buf[64];
  snprintf(buf, sizeof buf, "sleep 0x%02X, standby 0x%02X", s, st);
  result("Mode control", ok, buf);
  gotoSleep(true, false);
}

// PLL lock needs a working 32 MHz crystal (X2) and VCO. Checked in FSK mode,
// where the PllLock flag is readable.
static void checkPll(const char* name, uint32_t hz, bool lf) {
  gotoSleep(false, lf);
  wr(REG_OP_MODE, (lf ? LF : 0) | M_STDBY);
  delay(2);
  setFreq(hz);
  wr(REG_OP_MODE, (lf ? LF : 0) | M_FSRX);
  bool locked = false;
  const uint32_t t0 = millis();
  while (millis() - t0 < 20) {
    if (rd(REG_IRQ_FLAGS1) & 0x10) { locked = true; break; }
  }
  const uint32_t ms = millis() - t0;
  wr(REG_OP_MODE, (lf ? LF : 0) | M_STDBY);
  gotoSleep(true, lf);
  char buf[64];
  if (locked) snprintf(buf, sizeof buf, "%.1f MHz locked in %lu ms", hz / 1e6, (unsigned long)ms);
  else        snprintf(buf, sizeof buf, "%.1f MHz no lock -> check X2 crystal / C load caps", hz / 1e6);
  result(name, locked, buf);
}

// Put the radio in LoRa mode on a band, SF9 / BW125, ready to receive.
static void loraSetup(uint32_t hz, bool lf) {
  gotoSleep(true, lf);
  setFreq(hz);
  wr(REG_PA_CONFIG, 0x00);                  // PA minimum, we never transmit anyway
  wr(REG_LNA, lf ? 0x20 : 0x23);            // max gain, HF boost on the HF port
  wr(REG_MODEM_CFG1, 0x72);                 // BW125, CR4/5, explicit header
  wr(REG_MODEM_CFG2, 0x94);                 // SF9, CRC on
  wr(REG_MODEM_CFG3, 0x04);                 // AGC on
  wr(REG_OP_MODE, LORA | (lf ? LF : 0) | M_STDBY);
  delay(2);
}

// CAD is a receive-only operation that always ends with CadDone. Mapping
// DIO0=10 and DIO3=00 puts CadDone on both pins, so this proves the wiring.
static void checkDio() {
  parkSwitchesRx();
  loraSetup(F_AU915, false);
  wr(REG_DIO_MAPPING1, 0x80);               // DIO0 = CadDone, DIO3 = CadDone
  wr(REG_IRQ_FLAGS, 0xFF);
  delay(1);
  const bool idle0 = digitalRead(PIN_DIO0), idle3 = digitalRead(PIN_DIO3);

  wr(REG_OP_MODE, LORA | M_CAD);
  bool hi0 = false, hi3 = false, flag = false;
  const uint32_t t0 = millis();
  while (millis() - t0 < 100) {             // SF9 CAD takes ~8 ms
    hi0 |= digitalRead(PIN_DIO0);
    hi3 |= digitalRead(PIN_DIO3);
    if (rd(REG_IRQ_FLAGS) & IRQ_CAD_DONE) { flag = true; delay(1); break; }
  }
  hi0 |= digitalRead(PIN_DIO0);
  hi3 |= digitalRead(PIN_DIO3);
  wr(REG_IRQ_FLAGS, 0xFF);
  delay(1);
  const bool low0 = !digitalRead(PIN_DIO0), low3 = !digitalRead(PIN_DIO3);
  wr(REG_DIO_MAPPING1, 0x00);
  gotoSleep(true, false);

  char buf[96];
  if (!flag) {
    result("CAD", false, "CadDone never set (radio not running in LoRa mode)");
    return;
  }
  snprintf(buf, sizeof buf, "IO%d idle %s, rose %s, cleared %s", PIN_DIO0, idle0 ? "HIGH" : "low",
           hi0 ? "yes" : "NO", low0 ? "yes" : "NO");
  result("DIO0", !idle0 && hi0 && low0, buf);
  snprintf(buf, sizeof buf, "IO%d idle %s, rose %s, cleared %s", PIN_DIO3, idle3 ? "HIGH" : "low",
           hi3 ? "yes" : "NO", low3 ? "yes" : "NO");
  result("DIO3", !idle3 && hi3 && low3, buf);
}

static int rssiNow(bool lf) { return (lf ? -164 : -157) + rd(REG_RSSI); }

static int noiseFloor(uint32_t hz, bool lf) {
  parkSwitchesRx();
  loraSetup(hz, lf);
  wr(REG_OP_MODE, LORA | (lf ? LF : 0) | M_RXCONT);
  delay(10);
  long sum = 0;
  for (int i = 0; i < 64; i++) { sum += rssiNow(lf); delay(2); }
  return (int)(sum / 64);
}

static void infoNoise(const char* name, uint32_t hz, bool lf, const char* port) {
  const int n = noiseFloor(hz, lf);
  const char* note = n > -90 ? "HIGH: nearby interference or a transmitter?" :
                     n < -135 ? "very low: suspicious reading" : "looks normal";
  Serial.printf("  [INFO] %-16s %d dBm at %.1f MHz (%s)  %s\n", name, n, hz / 1e6, port, note);
}

static void runChecks() {
  passCount = failCount = 0;
  Serial.println();
  Serial.println("=== SX1276 check ===");
  detected = checkDetect();
  if (detected) {
    checkRegisters();
    checkModes();
    checkPll("PLL CN470", F_CN470, true);
    checkPll("PLL AU915", F_AU915, false);
    checkDio();
    infoNoise("Noise CN470", F_CN470, true, "UFL1");
    infoNoise("Noise AU915", F_AU915, false, "UFL2");
    // Leave the radio listening on AU915 for the live status line.
    loraSetup(F_AU915, false);
    wr(REG_OP_MODE, LORA | M_RXCONT);
  }
  Serial.printf("=== %s: %d passed, %d failed ===\n", failCount ? "FAULT" : "ALL OK", passCount,
                failCount);
  Serial.println("Commands: r = rerun checks, s = status now");
}

static void printStatus() {
  const uint8_t v = rd(REG_VERSION);
  const uint8_t op = rd(REG_OP_MODE);
  if (v != 0x12) {
    Serial.printf("[status] radio NOT DETECTED (RegVersion 0x%02X)\n", v);
    return;
  }
  static const char* modes[] = {"sleep", "standby", "FS-TX", "TX", "FS-RX", "RX", "RX-single", "CAD"};
  const bool lf = op & LF;
  Serial.printf("[status] radio ok  %s %s  %s  RSSI %d dBm\n", op & LORA ? "LoRa" : "FSK",
                modes[op & 7], lf ? "CN470" : "AU915", rssiNow(lf));
}

// ------------------------------------------------------------------ main
void setup() {
  Serial.begin(115200);
  const uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 3000) delay(10);

  pinMode(PIN_NSS, OUTPUT);
  digitalWrite(PIN_NSS, HIGH);
  pinMode(PIN_SW_CN470, OUTPUT);
  pinMode(PIN_SW_AU915, OUTPUT);
  parkSwitchesRx();
  pinMode(PIN_DIO0, INPUT);
  pinMode(PIN_DIO3, INPUT);
  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, -1);
  delay(15);  // SX1276 power-on reset time (no NRESET line on this PCB)

  runChecks();
}

void loop() {
  static uint32_t nextStatus = millis() + 5000;
  while (Serial.available()) {
    switch (tolower(Serial.read())) {
      case 'r': runChecks(); break;
      case 's': printStatus(); break;
    }
  }
  if ((int32_t)(millis() - nextStatus) >= 0) {
    printStatus();
    nextStatus = millis() + 5000;
  }
  delay(5);
}
