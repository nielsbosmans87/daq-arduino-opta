/*
DAQ Arduino Opta - 8-Channel High-Speed ADC Acquisition (Burst to RAM, then USB Transfer)

Notes:
- Targets Arduino Opta (STM32H747XI). Uses mbed::Ticker to pace frame sampling at 10 kHz.
- Phase model:
  1) Acquisition: sample 8 channels at 10 kHz into RAM (interleaved uint16 frames)
  2) Stop
  3) Transfer: send binary block over USB Serial (magic 'OPTA' + header + raw frames)
- Button can start/stop; PC can command acquisition via "ACQ <ms>".

Configuration:
- SERIAL_BAUD: 921600 recommended
- SAMPLE_RATE_HZ: 10000 (per channel)
- MAX_SAMPLES: RAM cap to keep stable on Opta mbed core
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
static const uint32_t SAMPLE_RATE_HZ    = 5000;   // per channel
static const uint32_t DECIMATION        = 100;     // for optional diagnostics
static const bool     ENABLE_VOLTS_OUT  = false;    // not used during burst; PC computes
static const float    DIVIDER_FACTOR    = 1.0f;  // 0-10V → 0-3.3V internal divider
static const float    VREF              = 10.0f;     // MCU ref
static const uint16_t ADC_MAX_COUNTS    = 4095;     // 12-bit default (oversampling emulation optional)

// Derived
static const uint8_t  NUM_CH            = 8;
static const uint32_t FRAME_RATE_HZ     = SAMPLE_RATE_HZ; // frames/sec (each frame has 8 samples)
static const uint32_t TICK_HZ           = FRAME_RATE_HZ;  // one timer tick per frame

// Burst acquisition sizing (segmented heap allocation)
static const uint32_t BLOCK_SAMPLES     = 2000;    // 2000 frames ≈ 32 KB per block (8 ch × 2B)

// Segmented RAM buffers allocated before acquisition
static uint16_t** sampleBlocks = nullptr;   // array of block pointers, each block holds BLOCK_SAMPLES × NUM_CH
static uint32_t   blockCount = 0;           // number of successfully allocated blocks
static uint32_t   maxSamplesAllocated = 0;  // blockCount × BLOCK_SAMPLES
static volatile uint32_t writeIdx = 0;      // frames written so far
static uint32_t targetSamples = 0;          // frames requested (clamped to available)

enum RunState { IDLE = 0, PRE, ACQ, TRANSFER };
static volatile RunState runState = IDLE;
static volatile uint32_t framesTotal = 0;

// mbed ticker for pacing
static mbed::Ticker frameTicker;

// LED/phasing
enum PrePhase { PRE_NONE = 0, PRE_RED_HOLD, PRE_ORANGE_BLINK };
static volatile bool preAcqActive = false;
static volatile PrePhase prePhase = PRE_NONE;
static uint32_t phaseStartMs = 0;
static uint32_t idleBlinkMs = 0;
static uint32_t orangeBlinkMs = 0;

// Button handling
static uint32_t lastButtonToggleMs = 0;

// Diagnostics
static volatile uint32_t pendingTicks = 0; // incremented in ISR, drained in loop()

// Forward decl
void startAcquisition(uint32_t duration_ms);
void stopAcquisition();
void onFrameTick();
void sendBinaryTransfer();
void printDiagnostics(bool forceLine = false);
static inline void processOneFrame();
static bool allocBlocks(uint32_t samplesNeeded);
static void freeBlocks();

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

  // Timer setup handled in start/stop via mbed::Ticker at TICK_HZ

  // Banner
  Serial.println("=== Arduino Opta ADC - Burst RAM Acquisition (Ticker) ===");
  Serial.print("Baud: "); Serial.println(SERIAL_BAUD);
  Serial.print("Per-channel rate: "); Serial.print(SAMPLE_RATE_HZ); Serial.println(" Hz");
  Serial.print("Block samples per chunk: "); Serial.println(BLOCK_SAMPLES);
  Serial.println("Commands: ACQ <ms>  |  x=stop  |  h=help");

  // Default: stop, wait for 's'
  stopAcquisition();
}

// ----- Loop
void loop() {
  // Command interface: "ACQ <ms>", 'x' stop, 'h' help
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n'); line.trim();
    if (line.length() == 1) {
      char c = line[0];
      if (c == 'x') stopAcquisition();
      else if (c == 'h') {
        Serial.println("Commands: ACQ <ms>  |  x=stop  |  h=help");
      }
    } else if (line.startsWith("ACQ")) {
      uint32_t ms = 0;
      int sp = line.indexOf(' ');
      if (sp > 0) {
        ms = (uint32_t) line.substring(sp + 1).toInt();
      }
      if (ms == 0) ms = 1000; // default 1s
      startAcquisition(ms);
    }
  }

  // LED idle blink: green 0.25s on / 0.25s off when IDLE
#ifdef LEDG
#ifdef LEDR
  if (runState == IDLE) {
    uint32_t nowBlink = millis();
    if (nowBlink - idleBlinkMs >= 250) {
      idleBlinkMs = nowBlink;
      digitalWrite(LEDG, !digitalRead(LEDG));
      digitalWrite(LEDR, LOW);
    }
  }
#endif
#endif

  // Handle pre-acquisition phases
  if (preAcqActive) {
    uint32_t now = millis();
    if (prePhase == PRE_RED_HOLD) {
#ifdef LEDR
      digitalWrite(LEDR, HIGH);
#endif
#ifdef LEDG
      digitalWrite(LEDG, LOW);
#endif
      if (now - phaseStartMs >= 2000) {
        prePhase = PRE_ORANGE_BLINK;
        phaseStartMs = now;
        orangeBlinkMs = now;
      }
    } else if (prePhase == PRE_ORANGE_BLINK) {
      if (now - orangeBlinkMs >= 250) {
        orangeBlinkMs = now;
#ifdef LEDR
        digitalWrite(LEDR, !digitalRead(LEDR));
#endif
#ifdef LEDG
        digitalWrite(LEDG, !digitalRead(LEDG));
#endif
      }
      if (now - phaseStartMs >= 2000) {
        // proceed to acquisition start
#ifdef LEDR
        digitalWrite(LEDR, LOW);
#endif
#ifdef LEDG
        digitalWrite(LEDG, HIGH);
#endif
        preAcqActive = false;
        // Reset counters and start actual acquisition
        noInterrupts();
        writeIdx = 0;
        framesTotal = 0;
        pendingTicks = 0;
        runState = ACQ;
        interrupts();
        Serial.print("[START] ACQ "); Serial.print(targetSamples); Serial.println(" frames");
      }
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
    if (runState == ACQ) processOneFrame();
  }

  if (runState == TRANSFER) {
    sendBinaryTransfer();
    runState = IDLE;
  }
}

// ----- Acquisition control
void startAcquisition(uint32_t duration_ms) {
  if (runState != IDLE) stopAcquisition();
  uint32_t reqSamples = (uint32_t)((uint64_t)duration_ms * FRAME_RATE_HZ / 1000ULL);
  if (reqSamples == 0) reqSamples = 1;
  if (!allocBlocks(reqSamples)) {
    Serial.println("[WARN] Allocation failed; using available RAM blocks if any");
  }
  Serial.print("#ALLOC blocks="); Serial.print(blockCount); Serial.print(" maxSamples="); Serial.println(maxSamplesAllocated);
  targetSamples = (reqSamples > maxSamplesAllocated) ? maxSamplesAllocated : reqSamples;
  noInterrupts();
  writeIdx = 0;
  framesTotal = 0;
  runState = PRE;
  preAcqActive = true;
  prePhase = PRE_RED_HOLD;
  phaseStartMs = millis();
  interrupts();

  const float period_s = 1.0f / (float)TICK_HZ;
  frameTicker.detach();
  frameTicker.attach(mbed::callback(onFrameTick), period_s);

#ifdef LEDG
  digitalWrite(LEDG, LOW);
#endif
#ifdef LEDR
  digitalWrite(LEDR, HIGH); // enter red phase
#endif
  Serial.print("[START] PRE "); Serial.print(targetSamples); Serial.println(" frames");
}

void stopAcquisition() {
  frameTicker.detach();
  noInterrupts();
  runState = IDLE;
  preAcqActive = false;
  prePhase = PRE_NONE;
  interrupts();

#ifdef LEDG
  digitalWrite(LEDG, LOW);
#endif
#ifdef LEDR
  digitalWrite(LEDR, LOW);
#endif
  Serial.println("[STOP] Idle");
}

// ----- Ticker callback
void onFrameTick() {
  if (runState == ACQ) {
    pendingTicks++;
  }
}

// ----- Convert raw ADC to input volts (0-10V scaled)
// Optional helper kept for PC parity (not used during burst)
static inline float rawToVoltsLocal(uint16_t raw) {
  float v_mcu = (float)raw * (VREF / ADC_MAX_COUNTS);
  return (v_mcu / DIVIDER_FACTOR);
}

// ----- Flush ready pages (producer/consumer boundary)
// ----- Binary transfer: magic 'OPTA' + header + interleaved raw uint16 frames
typedef struct __attribute__((packed)) {
  uint8_t  magic[4];      // 'O','P','T','A'
  uint16_t version;       // 1
  uint16_t numChannels;   // 8
  uint32_t sampleRateHz;  // 10000
  uint32_t numSamples;    // frames captured
  float    vref;          // 3.3
  float    divider;       // 0.3034
} OptaBinHeader;

void sendBinaryTransfer() {
  OptaBinHeader hdr;
  hdr.magic[0] = 'O'; hdr.magic[1] = 'P'; hdr.magic[2] = 'T'; hdr.magic[3] = 'A';
  hdr.version = 1;
  hdr.numChannels = NUM_CH;
  hdr.sampleRateHz = SAMPLE_RATE_HZ;
  hdr.numSamples = framesTotal;
  hdr.vref = VREF;
  hdr.divider = DIVIDER_FACTOR;

  Serial.write((uint8_t*)&hdr, sizeof(hdr));

  // stream interleaved frames
  if (sampleBlocks && blockCount > 0) {
    uint32_t remaining = framesTotal;
    for (uint32_t b = 0; b < blockCount && remaining > 0; ++b) {
      uint32_t framesInBlock = (remaining > BLOCK_SAMPLES) ? BLOCK_SAMPLES : remaining;
      Serial.write((uint8_t*)sampleBlocks[b], sizeof(uint16_t) * NUM_CH * framesInBlock);
      remaining -= framesInBlock;
    }
  }

#ifdef LEDB
  digitalWrite(LEDB, LOW);
#endif
  Serial.println(); // newline after binary for safety if opened in text monitor
  Serial.println("[TRANSFER] Complete");
}

// ----- Diagnostics line (compact)
void printDiagnostics(bool forceLine) {
  static uint32_t lastMs = 0;
  uint32_t now = millis();
  if (!forceLine && (now - lastMs < 1000)) return;
  lastMs = now;

  Serial.print("# STAT,");
  Serial.print("Rate_Hz:"); Serial.print(SAMPLE_RATE_HZ);
  Serial.print(",Frames:"); Serial.print(framesTotal);
  Serial.print(",State:"); Serial.print((int)runState);
  Serial.print(",Target:"); Serial.print(targetSamples);
  Serial.print(",Ch:"); Serial.print(NUM_CH);
  Serial.print(",Blocks:"); Serial.print(blockCount);
  Serial.print(",MaxAvail:"); Serial.print(maxSamplesAllocated);
  Serial.println();
}

static inline void processOneFrame() {
  uint32_t idx = writeIdx;
  uint32_t blockIdx = idx / BLOCK_SAMPLES;
  uint32_t offset = idx % BLOCK_SAMPLES;
  if (!sampleBlocks || blockIdx >= blockCount) {
    frameTicker.detach();
    runState = TRANSFER;
    return;
  }
  uint16_t* frame = sampleBlocks[blockIdx] + (offset * NUM_CH);
  for (uint8_t ch = 0; ch < NUM_CH; ++ch) {
    frame[ch] = (uint16_t)analogRead(ADC_PINS[ch]);
  }
  writeIdx++;
  framesTotal++;

  if (framesTotal >= targetSamples) {
    frameTicker.detach();
#ifdef LEDB
    digitalWrite(LEDB, HIGH); // indicate transfer phase
#endif
    runState = TRANSFER;
  }
}

// ----- Allocation helpers
static bool allocBlocks(uint32_t samplesNeeded) {
  // Persistently grow allocation as needed; never free between runs to avoid fragmentation
  uint32_t neededBlocks = (samplesNeeded + BLOCK_SAMPLES - 1) / BLOCK_SAMPLES;
  if (neededBlocks == 0) neededBlocks = 1;

  if (!sampleBlocks) {
    sampleBlocks = (uint16_t**) malloc(sizeof(uint16_t*) * neededBlocks);
    if (!sampleBlocks) {
      blockCount = 0;
      maxSamplesAllocated = 0;
      return false;
    }
    for (uint32_t i = 0; i < neededBlocks; ++i) sampleBlocks[i] = nullptr;
    blockCount = 0;
    maxSamplesAllocated = 0;
  } else if (neededBlocks > blockCount) {
    // grow pointer table
    uint16_t** newTable = (uint16_t**) realloc(sampleBlocks, sizeof(uint16_t*) * neededBlocks);
    if (!newTable) {
      // cannot grow table; proceed with current capacity
      neededBlocks = blockCount;
    } else {
      sampleBlocks = newTable;
      for (uint32_t i = blockCount; i < neededBlocks; ++i) sampleBlocks[i] = nullptr;
    }
  } else {
    // have enough blocks already
  }

  // allocate any missing blocks up to neededBlocks
  for (uint32_t i = blockCount; i < neededBlocks; ++i) {
    uint16_t* blk = (uint16_t*) malloc(sizeof(uint16_t) * NUM_CH * BLOCK_SAMPLES);
    if (!blk) {
      break;
    }
    sampleBlocks[blockCount++] = blk;
    maxSamplesAllocated = blockCount * BLOCK_SAMPLES;
  }

  // If we couldn't reach neededBlocks, still return true if we have at least one block
  maxSamplesAllocated = blockCount * BLOCK_SAMPLES;
  return (blockCount > 0);
}

static void freeBlocks() {
  if (sampleBlocks) {
    for (uint32_t i = 0; i < blockCount; ++i) {
      if (sampleBlocks[i]) free(sampleBlocks[i]);
    }
    free(sampleBlocks);
  }
  sampleBlocks = nullptr;
  blockCount = 0;
  maxSamplesAllocated = 0;
}
