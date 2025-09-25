/*
DAQ Arduino Opta - 8-Channel High-Speed ADC Acquisition (Phase 1 MVP)

Notes:
- Targets Arduino Opta (STM32H747XI). Uses HardwareTimer at 10 kHz per-channel sampler.
- Samples 8 channels each tick → 80 kS/s total. Stores to ping-pong buffer; streams over USB serial.
- Serial Plotter decimation: 100 Hz (every 100th sample batch).
- Button toggles acquisition; LEDs show RUN/LOG/ERR.
- This version uses analogRead in a timer callback for portability. For ±50 ns jitter and zero-loss DMA,
  replace the sampler with HAL TIM-TRGO + ADC scan + DMA double-buffer in Phase 2 (hooks marked TODO_HAL).

Configuration:
- SERIAL_BAUD: 921600 recommended
- SAMPLE_RATE_HZ: 10000 (per channel)
- DECIMATION: 100 (for plotter/diagnostics)
- BUFFER_BATCH: number of multi-channel frames per half-buffer (adjust for RAM)
*/

#include <Arduino.h>
#include <mbed.h>

// ----- Pins (adjust if your Opta variant differs)
static const uint8_t ADC_PINS[8] = {A0, A1, A2, A3, A4, A5, A6, A7};

// Built-in UI (Opta has user button and status LEDs; map to available pins/APIs)
#ifndef LED_BUILTIN
#define LED_BUILTIN LEDG
#endif
#ifndef LEDR
#define LEDR  LEDR
#endif
#ifndef LEDG
#define LEDG  LEDG
#endif
#ifndef LEDB
#define LEDB  LEDB
#endif

// ----- Config
static const uint32_t SERIAL_BAUD       = 921600;
static const uint32_t SAMPLE_RATE_HZ    = 5000;    // per channel
static const uint32_t DECIMATION        = 100;      // 100 Hz visualization
static const uint16_t BUFFER_BATCH      = 256;      // frames per half-buffer (8 ch per frame)
static const bool     ENABLE_VOLTS_OUT  = true;     // send computed volts alongside raw
static const float    DIVIDER_FACTOR    = 0.3034f;  // 0-10V → 0-3.3V internal divider
static const float    VREF              = 3.3f;     // MCU ref
static const uint16_t ADC_MAX_COUNTS    = 4095;     // 12-bit default (oversampling emulation optional)

// Derived
static const uint8_t  NUM_CH            = 8;
static const uint32_t FRAME_RATE_HZ     = SAMPLE_RATE_HZ; // frames/sec (each frame has 8 samples)
static const uint32_t TICK_HZ           = FRAME_RATE_HZ;  // one timer tick per frame

// Buffers: ping-pong [2][BUFFER_BATCH][NUM_CH]
static volatile uint16_t adcBuf[2][BUFFER_BATCH][NUM_CH];
static volatile uint16_t writeIdx = 0;
static volatile uint8_t  writePage = 0;
static volatile bool     pageReady[2] = {false, false};

static volatile bool     running = false;
static volatile uint32_t framesTotal = 0;
static volatile uint32_t missedFrames = 0;

mbed::Ticker samplerTicker;

// Button handling
static uint32_t lastButtonToggleMs = 0;

// Diagnostics
static volatile uint32_t decimCount = 0;
static volatile uint16_t lastFrame[NUM_CH] = {0};
static volatile uint32_t pendingTicks = 0; // incremented in ISR, drained in loop()

// Forward decl
void startAcquisition();
void stopAcquisition();
void onTimerTick();
void flushReadyPages();
void printDiagnostics(bool forceLine = false);
float rawToVolts(uint16_t raw);

// ----- Setup
void setup() {
  Serial.begin(SERIAL_BAUD);
  while (!Serial && millis() < 3000) { /* wait briefly */ }

  pinMode(LED_BUILTIN, OUTPUT);
#ifdef LEDR
  pinMode(LEDR, OUTPUT);
#endif
#ifdef LEDG
  pinMode(LEDG, OUTPUT);
#endif
#ifdef LEDB
  pinMode(LEDB, OUTPUT);
#endif

  // User button (Opta built-in)
#ifdef BUTTON
  pinMode(BUTTON, INPUT_PULLUP);
#else
  // Fallback: if your core defines USER_BTN or similar, map it here.
#endif

  // Analog pin setup
  for (uint8_t i = 0; i < NUM_CH; ++i) {
    pinMode(ADC_PINS[i], INPUT);
  }

  // Timer setup handled in start/stop via mbed::Ticker at TICK_HZ (one frame per tick)

  // Banner
  Serial.println("=== ADC Acquisition: Arduino Opta (Phase 1) ===");
  Serial.print("Baud: "); Serial.println(SERIAL_BAUD);
  Serial.print("Per-channel rate: "); Serial.print(SAMPLE_RATE_HZ); Serial.println(" Hz");
  Serial.print("Frame rate: "); Serial.print(FRAME_RATE_HZ); Serial.println(" fps");
  Serial.print("Decimation: /"); Serial.println(DECIMATION);
  Serial.print("Buffer half size (frames): "); Serial.println(BUFFER_BATCH);
  Serial.println("Commands: 's' start, 'x' stop, 'pN' select plot channels 0..3, 'h' help");

  // Default: stop, wait for 's'
  stopAcquisition();
}

// ----- Loop
void loop() {
  // Simple serial command interface
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n'); cmd.trim();
    if (cmd == "s" || cmd == "start") {
      startAcquisition();
    } else if (cmd == "x" || cmd == "stop") {
      stopAcquisition();
    } else if (cmd == "h" || cmd == "help") {
      Serial.println("Commands: s=start, x=stop, h=help");
    }
  }

  // Button toggle (if defined)
#ifdef BUTTON
  if (digitalRead(BUTTON) == LOW) {
    if (millis() - lastButtonToggleMs > 300) {
      if (running) stopAcquisition(); else startAcquisition();
      lastButtonToggleMs = millis();
    }
  }
#endif

  // Drain ticks atomically
  uint32_t n;
  noInterrupts();
  n = pendingTicks;
  pendingTicks = 0;
  interrupts();

  while (n--) {
    if (running) processOneFrame();
  }

  flushReadyPages();     // emits DATA,... lines for Python capture
  // Optional: comment out diagnostics if still too chatty
  // printDiagnostics(false);
}

// ----- Acquisition control
void startAcquisition() {
  noInterrupts();
  writeIdx = 0;
  writePage = 0;
  pageReady[0] = pageReady[1] = false;
  framesTotal = 0;
  missedFrames = 0;
  decimCount = 0;
  running = true;
  interrupts();

  // Attach ticker for periodic sampling; 1.0f / TICK_HZ seconds
  const float period_s = 1.0f / (float)TICK_HZ;
  samplerTicker.detach();
  samplerTicker.attach(mbed::callback(onTimerTick), period_s);

#ifdef LEDG
  digitalWrite(LEDG, HIGH);
#endif
#ifdef LEDR
  digitalWrite(LEDR, LOW);
#endif
  Serial.println("Sampling Status: ACTIVE");
}

void stopAcquisition() {
  samplerTicker.detach();
  noInterrupts();
  running = false;
  interrupts();

#ifdef LEDG
  digitalWrite(LEDG, LOW);
#endif
#ifdef LEDR
  digitalWrite(LEDR, LOW);
#endif
  Serial.println("Sampling Status: STOPPED");
}

// ----- Timer ISR: one frame (8 channels) per tick
void onTimerTick() {
  if (!running) return;
  pendingTicks++;   // nothing else in the interrupt
}

// ----- Convert raw ADC to input volts (0-10V scaled)
float rawToVolts(uint16_t raw) {
  float v_mcu = (float)raw * (VREF / ADC_MAX_COUNTS);
  return (v_mcu / DIVIDER_FACTOR);
}

// ----- Flush ready pages (producer/consumer boundary)
void flushReadyPages() {
  static uint8_t readPage = 0;
  static uint32_t msLastStats = 0;

  if (!pageReady[readPage]) return;

  // Timestamp (ms)
  float t_ms = millis();

  // Output CSV lines: one line per frame in this page
  // Format: Timestamp_ms, A0_raw, A0_volts, ..., A7_raw, A7_volts
  for (uint16_t i = 0; i < BUFFER_BATCH; ++i) {
    Serial.print("DATA,");
    Serial.print(t_ms, 3); // same timestamp for batch (batch-level stamp)
    for (uint8_t ch = 0; ch < NUM_CH; ++ch) {
      uint16_t raw = adcBuf[readPage][i][ch];
      Serial.print(','); Serial.print(raw);
      if (ENABLE_VOLTS_OUT) {
        Serial.print(','); Serial.print(rawToVolts(raw), 4);
      }
    }
    Serial.println();
    t_ms += 1000.0f / FRAME_RATE_HZ; // approximate per-frame ts if needed by PC side
  }

  pageReady[readPage] = false;
  readPage ^= 1;

  // Status LED pulse
  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));

  // Periodic compact status
  uint32_t now = millis();
  if (now - msLastStats > 1000) {
    msLastStats = now;
    // printDiagnostics(true);
  }
}

// ----- Diagnostics line (compact)
void printDiagnostics(bool forceLine) {
  static uint32_t lastMs = 0;
  uint32_t now = millis();
  if (!forceLine && (now - lastMs < 1000)) return;
  lastMs = now;

  Serial.print("# STAT,");  // instead of Serial.print("STAT,");
  Serial.print("Rate_Hz:"); Serial.print(SAMPLE_RATE_HZ);
  Serial.print(",Frames:"); Serial.print(framesTotal);
  Serial.print(",Missed:"); Serial.print(missedFrames);
  Serial.print(",Run:"); Serial.print(running ? "1" : "0");
  Serial.print(",BufHalf:"); Serial.print(BUFFER_BATCH);
  Serial.print(",Ch:8");
  Serial.println();
}

static inline void processOneFrame() {
  volatile uint16_t* frame = adcBuf[writePage][writeIdx];

  for (uint8_t ch = 0; ch < NUM_CH; ++ch) {
    uint16_t v = analogRead(ADC_PINS[ch]);
    frame[ch] = v;
    lastFrame[ch] = v;
  }

  writeIdx++;
  framesTotal++;

  // Decimated live stream (100 Hz) for Serial Plotter: A0..A7 in volts
  decimCount++;
  if (decimCount >= DECIMATION) {
    decimCount = 0;
    for (uint8_t ch = 0; ch < NUM_CH; ++ch) {
      Serial.print('A'); Serial.print(ch); Serial.print(':');
      if (ENABLE_VOLTS_OUT) Serial.print(rawToVolts(lastFrame[ch]), 4);
      else Serial.print(lastFrame[ch]);
      if (ch < (NUM_CH - 1)) Serial.print(' ');
    }
    Serial.println();
  }

  if (writeIdx >= BUFFER_BATCH) {
    if (pageReady[writePage]) {
      missedFrames++;
#ifdef LEDR
      digitalWrite(LEDR, HIGH);
#endif
    } else {
      pageReady[writePage] = true;
    }
    writeIdx = 0;
    writePage ^= 1;
  }
}
