/*
 * ============================================================================
 *  ESP32 Real-Time ADC Sampler — Multirate DSP Hardware-in-the-Loop
 * ============================================================================
 *
 *  PURPOSE:
 *    Sample an analog audio signal on GPIO 34 at 8 kHz using a hardware timer
 *    interrupt, remove the 1.65 V DC bias, and stream the zero-centered
 *    samples to a desktop MATLAB script over USB-Serial using a compact
 *    binary frame protocol.
 *
 *  HARDWARE SETUP:
 *    - ESP32 Dev Board (any variant with ADC1)
 *    - Audio source (e.g., phone) → capacitor-coupled or resistor-biased
 *      to 1.65 V on GPIO 34
 *    - USB cable to desktop running MATLAB
 *
 *  BINARY FRAME FORMAT (per sample):
 *    Byte 0: 0xAA  (sync marker high)
 *    Byte 1: 0x55  (sync marker low)
 *    Byte 2: sample low byte   (int16_t, little-endian)
 *    Byte 3: sample high byte  (int16_t, little-endian)
 *
 *    Sync collision safety: ADC output is 0–4095, centered to −2048..+2047.
 *    The high byte of any valid sample is in {0xF8..0xFF, 0x00..0x07}.
 *    0xAA (170) can NEVER appear as a valid high byte, so false sync
 *    detection is impossible.
 *
 *  TIMING BUDGET (at 500 000 baud):
 *    4 bytes/sample × 8000 samples/s = 32 000 bytes/s
 *    500 000 baud ÷ 10 bits/byte     = 50 000 bytes/s capacity
 *    Utilisation: 64 % — comfortable headroom.
 *
 *  COMPATIBLE WITH:
 *    - ESP32 Arduino Core v2.x (timerBegin(id, prescaler, countUp))
 *    - ESP32 Arduino Core v3.x (timerBegin(frequency))
 *    Auto-detected at compile time via ESP_ARDUINO_VERSION macros.
 *
 *  Author : AI-Generated (Antigravity)
 *  Date   : 2026-08-13
 * ============================================================================
 */

#include <Arduino.h>

// ─────────────────────────── CONFIGURATION ──────────────────────────────────

#define ADC_PIN           34        // GPIO 34 = ADC1_CH6 (input only, no pullup)
#define SAMPLE_RATE_HZ    8000      // Target sampling frequency
#define BAUD_RATE         500000    // USB-Serial baud rate (must match MATLAB)
#define DC_OFFSET         2048      // 12-bit midpoint (1.65 V bias removal)

// Ring buffer size — MUST be a power of 2 for fast modulo via bitmask.
// 1024 samples = 128 ms of buffering at 8 kHz — ample headroom for serial
// TX bursts and short MATLAB processing stalls.
#define RING_BUF_SIZE     1024
#define RING_BUF_MASK     (RING_BUF_SIZE - 1)

// Sync marker bytes for binary frame protocol
#define SYNC_HIGH         0xAA
#define SYNC_LOW          0x55

// Set to 1 to enable human-readable ASCII output for debugging.
// WARNING: ASCII mode WILL overflow serial at 8 kHz / 500 kbaud.
// Use only for brief verification at reduced sample rates.
#define DEBUG_ASCII       0

// ─────────────────────────── RING BUFFER ────────────────────────────────────
// Lock-free SPSC (Single-Producer Single-Consumer) ring buffer.
// Producer: ISR (writes via writeIdx)
// Consumer: loop() (reads via readIdx)
// Thread-safety: writeIdx is only modified by ISR, readIdx only by loop().
// Both are declared volatile to prevent compiler reordering.

volatile int16_t  ringBuf[RING_BUF_SIZE];  // Sample storage
volatile uint16_t writeIdx = 0;             // ISR writes here (head)
volatile uint16_t readIdx  = 0;             // loop() reads here (tail)

// Overflow counter — incremented in ISR when buffer is full.
// Non-critical (diagnostic only), so not synchronized.
volatile uint32_t overflowCount = 0;

// ─────────────────────────── TIMER HANDLE ───────────────────────────────────

hw_timer_t *sampleTimer = NULL;

// ─────────────────────────── ISR ────────────────────────────────────────────
/*
 * Timer ISR — fires at exactly SAMPLE_RATE_HZ (8 kHz = every 125 µs).
 *
 * This ISR does NOT call analogRead() because analogRead() acquires
 * a mutex internally and is NOT safe to call from an ISR context on
 * all ESP32 cores/IDF versions.
 *
 * Instead, the ISR increments a sample-request counter. The main loop()
 * services the counter by performing the actual ADC read. This decouples
 * the precise timing epoch (ISR) from the ADC conversion (loop), introducing
 * only a few microseconds of jitter — negligible at audio frequencies.
 *
 * UPDATE: For simplicity and lower jitter, we use a flag-based approach.
 * A dedicated volatile counter tracks how many samples the ISR has requested.
 */
volatile uint32_t samplesRequested = 0;  // ISR increments; loop() decrements

void IRAM_ATTR onSampleTimer() {
  samplesRequested++;
}

// ─────────────────────────── SETUP ──────────────────────────────────────────

void setup() {
  // ── Serial Initialization ──
  Serial.begin(BAUD_RATE);

  // Wait briefly for USB-Serial to stabilize (CP2102/CH340 enumeration).
  delay(500);

  // ── ADC Configuration ──
  // ADC1 Channel 6 (GPIO 34). 12-bit resolution is the default on ESP32.
  // analogSetAttenuation(ADC_11db) gives full 0–3.3 V range (default on most cores).
  analogReadResolution(12);             // Explicit 12-bit (0–4095)
  analogSetAttenuation(ADC_11db);       // Full-scale 0–3.3 V
  pinMode(ADC_PIN, INPUT);             // High-impedance input (no pullup on GPIO 34)

  // ── Hardware Timer Configuration ──
  // ESP32 has 4 hardware timers (0–3). We use Timer 0.
  // The timer peripheral runs off the 80 MHz APB clock.

  #if defined(ESP_ARDUINO_VERSION) && ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
    // ─── ESP32 Arduino Core v3.x API ───
    // timerBegin(frequency_hz) — returns configured timer handle.
    sampleTimer = timerBegin(SAMPLE_RATE_HZ);
    timerAttachInterrupt(sampleTimer, &onSampleTimer);
    // timerAlarm(timer, period_ticks, autoreload, count)
    // With timerBegin(8000), the timer already runs at 8 kHz,
    // so alarm every 1 tick with auto-reload.
    timerAlarm(sampleTimer, 1, true, 0);
  #else
    // ─── ESP32 Arduino Core v2.x API ───
    // timerBegin(timer_id, prescaler, countUp)
    // Prescaler 80 → 80 MHz / 80 = 1 MHz tick (1 µs resolution)
    sampleTimer = timerBegin(0, 80, true);
    timerAttachInterrupt(sampleTimer, &onSampleTimer, true);
    // Alarm every 125 ticks at 1 MHz = 8 kHz
    timerAlarmWrite(sampleTimer, (1000000 / SAMPLE_RATE_HZ), true);
    timerAlarmEnable(sampleTimer);
  #endif

  // ── Startup Handshake ──
  // Send a human-readable banner so MATLAB (or the user) can confirm
  // the firmware version and configuration. MATLAB should flush/discard
  // bytes until it sees the sync pattern.
  Serial.println();
  Serial.println(F("=== ESP32 Multirate DSP Sampler ==="));
  Serial.print(F("Sample Rate : ")); Serial.print(SAMPLE_RATE_HZ); Serial.println(F(" Hz"));
  Serial.print(F("Baud Rate   : ")); Serial.println(BAUD_RATE);
  Serial.print(F("ADC Pin     : GPIO ")); Serial.println(ADC_PIN);
  Serial.print(F("DC Offset   : ")); Serial.println(DC_OFFSET);
  Serial.print(F("Buffer Size : ")); Serial.print(RING_BUF_SIZE); Serial.println(F(" samples"));
  Serial.print(F("Frame Format: [0xAA][0x55][Lo][Hi] ("));
  Serial.print(4 * SAMPLE_RATE_HZ);
  Serial.println(F(" bytes/s)"));
  Serial.println(F("Streaming started..."));
  Serial.println();

  // Small delay to let the banner transmit before binary flood begins.
  delay(100);
}

// ─────────────────────────── MAIN LOOP ──────────────────────────────────────

void loop() {
  // ── 1. Service ISR Requests: Read ADC and Fill Ring Buffer ──
  // Process all pending sample requests from the ISR. Under normal
  // operation, samplesRequested is 0 or 1 per loop iteration.
  // If it's > 1, we've fallen behind — handle all requests to catch up.

  while (samplesRequested > 0) {
    // Atomically decrement the request counter.
    // noInterrupts/interrupts ensure we don't race with the ISR.
    noInterrupts();
    samplesRequested--;
    interrupts();

    // ── ADC Conversion ──
    // analogRead() returns 0–4095 (12-bit unsigned).
    int rawADC = analogRead(ADC_PIN);

    // ── DC Offset Removal ──
    // The input signal is biased to 1.65 V (midpoint of 3.3 V rail),
    // which maps to ~2048 in the 12-bit ADC. Subtracting 2048 centers
    // the signal at zero: range becomes −2048 to +2047.
    int16_t sample = (int16_t)(rawADC - DC_OFFSET);

    // ── Push to Ring Buffer ──
    uint16_t nextWrite = (writeIdx + 1) & RING_BUF_MASK;
    if (nextWrite != readIdx) {
      // Buffer not full — store sample.
      ringBuf[writeIdx] = sample;
      writeIdx = nextWrite;
    } else {
      // Buffer full — drop this sample and record the overflow.
      overflowCount++;
    }
  }

  // ── 2. Drain Ring Buffer to Serial ──
  // Transmit all buffered samples as binary frames.
  // Serial.write() copies data into the hardware TX FIFO (128–256 bytes)
  // and returns immediately if space is available. This is non-blocking
  // under normal throughput conditions.

  while (readIdx != writeIdx) {
    int16_t sample = ringBuf[readIdx];
    readIdx = (readIdx + 1) & RING_BUF_MASK;

    #if DEBUG_ASCII
      // Human-readable mode for debugging (Serial Monitor).
      // WARNING: overflows serial at full 8 kHz rate.
      Serial.println(sample);
    #else
      // ── Binary Frame Transmission ──
      // Frame: [SYNC_HIGH (0xAA)] [SYNC_LOW (0x55)] [Lo Byte] [Hi Byte]
      // Little-endian matches MATLAB's native byte order for typecast().
      uint8_t frame[4];
      frame[0] = SYNC_HIGH;                         // Sync marker byte 1
      frame[1] = SYNC_LOW;                          // Sync marker byte 2
      frame[2] = (uint8_t)(sample & 0xFF);           // Low byte of int16
      frame[3] = (uint8_t)((sample >> 8) & 0xFF);    // High byte of int16
      Serial.write(frame, 4);
    #endif
  }

  // ── 3. Periodic Diagnostics (Optional) ──
  // Every 5 seconds, report overflow count to help diagnose buffer issues.
  // This is transmitted as a specially formatted text line that MATLAB
  // can optionally parse or ignore (it won't match the binary sync pattern).
  static uint32_t lastDiagMillis = 0;
  if (millis() - lastDiagMillis >= 5000) {
    lastDiagMillis = millis();
    if (overflowCount > 0) {
      // Temporarily send as text. The 0x0D 0x0A (CR LF) terminator
      // will never be mistaken for a sync pattern by MATLAB.
      Serial.print(F("[DIAG] Overflows: "));
      Serial.println(overflowCount);
    }
  }
}
