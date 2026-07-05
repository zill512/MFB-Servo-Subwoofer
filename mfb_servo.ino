// =============================================================================
// mfb_servo.ino — Teensy 4.x MFB Servo Subwoofer Controller
//
// Hardware (per wiring diagram rev 3):
//   PCM1808  I2S in : L = INA240 current sense, R = program audio
//   UDA1334A I2S out: L = drive to Class D sub amp
//   ADXL326  Z-out -> RC (22n||4.7n vs internal 32k) -> A0
//   Teensy = I2S clock master, 48 kHz
//
// Boot sequence:
//   1. Settle ADC/DAC (~1.5 s), measure accel DC offset
//   2. AUTOCAL: stepped-sine sweep, Goertzel mag/phase on ref/accel/current
//   3. Derive Fs, Qms, Qes, Qts, r0, Le, plant gain K; pole-place k_v
//   4. Print full report to USB serial; save cal to EEPROM
//   5. Enter RUN: program passthrough + velocity-feedback servo loop
//
// Build notes:
//   - Requires: Audio, ADC (pedvide), EEPROM libraries
//   - Feedback transport delay = 2 audio blocks + DAC filter. Rebuild with
//     -DAUDIO_BLOCK_SAMPLES=16 (333 us blocks) for usable loop bandwidth.
//     Default 128 (2.67 ms) is fine for autocal bring-up, marginal for servo.
//   - Teensy Audio Library I/O is 16-bit; PCM1808's 24-bit words are
//     truncated. ~93 dB SNR forward path — adequate for sub band. Migration
//     path for full 24-bit: OpenAudio_ArduinoLibrary (F32).
//   - Amp volume knob MUST NOT move between autocal and run: plant gain K
//     and k_v are calibrated at that setting.
//   - Set RE_OHMS below to your DMM-measured series-coil Re (cold).
// =============================================================================

#include <Audio.h>
#include <ADC.h>
#include <EEPROM.h>
#include <utility/imxrt_hw.h>
#include <math.h>

// ------------------------------- Config -------------------------------------
static const float    FS_AUDIO      = 48000.0f;
static const int      ACCEL_PIN     = A0;
static const float    RE_OHMS       = 6.4f;   // <-- DMM-measured, series coils
static const float    CAL_AMP       = 0.10f;  // sweep level, fraction of DAC FS
static const float    TARGET_QC     = 0.50f;  // closed-loop target Q
static const float    ACCEL_HPF_HZ  = 5.0f;   // servo-path DC blocker
static const float    VEL_LEAK_HZ   = 2.0f;   // leaky-integrator corner
static const uint32_t SETTLE_MS     = 250;    // per-tone settle
static const uint32_t MEASURE_MS    = 400;    // per-tone Goertzel window

// Sweep table: dense around expected resonance, sparse HF tail for Le
static const float CAL_FREQS[] = {
  10, 13, 16, 20, 24, 28, 32, 36, 40, 44, 48, 52, 56, 60, 65, 70,
  76, 82, 90, 100, 112, 125, 140, 160, 180, 200, 250, 315, 400,
  800, 1400, 2000
};
static const int N_FREQS = sizeof(CAL_FREQS) / sizeof(CAL_FREQS[0]);

// --------------------------- 48 kHz sample rate ------------------------------
// Standard T4 SAI1 reclock (F. Boesing / PJRC forum)
static void setI2SFreq(int freq) {
  int n1 = 4;
  int n2 = 1 + (24000000 * 27) / (freq * 256 * n1);
  double C = ((double)freq * 256 * n1 * n2) / 24000000;
  int c0 = (int)C;
  int c2 = 10000;
  int c1 = (int)(C * c2) - (c0 * c2);
  set_audioClock(c0, c1, c2, true);
  CCM_CS1CDR = (CCM_CS1CDR & ~(CCM_CS1CDR_SAI1_CLK_PRED_MASK | CCM_CS1CDR_SAI1_CLK_PODF_MASK))
             | CCM_CS1CDR_SAI1_CLK_PRED(n1 - 1)
             | CCM_CS1CDR_SAI1_CLK_PODF(n2 - 1);
}

// --------------------------- Accel acquisition ------------------------------
// ADC free-runs with 8x hardware averaging; IntervalTimer snapshots at 48 kHz
// into a ring buffer the audio object consumes block-aligned.
ADC adc;
IntervalTimer accelTimer;

static const uint32_t RING_SZ  = 1024;              // power of 2
static const uint32_t RING_MSK = RING_SZ - 1;
volatile int32_t  accelRing[RING_SZ];
volatile uint32_t accelWr = 0;
volatile int32_t  accelDC = 2048;                   // updated at boot (12-bit)

static void accelISR() {
  accelRing[accelWr & RING_MSK] = (int32_t)adc.adc0->analogReadContinuous() - accelDC;
  accelWr++;
}

// ------------------------------ Biquad --------------------------------------
struct Biquad {
  float b0 = 1, b1 = 0, b2 = 0, a1 = 0, a2 = 0, z1 = 0, z2 = 0;
  inline float process(float x) {
    float y = b0 * x + z1;
    z1 = b1 * x - a1 * y + z2;
    z2 = b2 * x - a2 * y;
    return y;
  }
  void setHighpass(float f, float Q, float fs) {
    float w = 2.0f * (float)M_PI * f / fs, c = cosf(w), s = sinf(w);
    float al = s / (2.0f * Q), a0 = 1.0f + al;
    b0 = (1 + c) / 2 / a0;  b1 = -(1 + c) / a0;  b2 = b0;
    a1 = (-2 * c) / a0;     a2 = (1 - al) / a0;
    z1 = z2 = 0;
  }
};

// ------------------------------ Goertzel ------------------------------------
struct Goertzel {
  float coeff = 0, cw = 0, sw = 0, s1 = 0, s2 = 0;
  uint32_t n = 0;
  void init(float f, float fs) {
    float w = 2.0f * (float)M_PI * f / fs;
    cw = cosf(w); sw = sinf(w); coeff = 2.0f * cw;
    s1 = s2 = 0; n = 0;
  }
  inline void feed(float x) { float s0 = x + coeff * s1 - s2; s2 = s1; s1 = s0; n++; }
  // complex result (re, im) — phase is relative, consistent across channels
  void result(float &re, float &im) { re = s1 - s2 * cw; im = s2 * sw; }
  float mag() { float re, im; result(re, im); return sqrtf(re * re + im * im) / (n ? n : 1); }
  float phase() { float re, im; result(re, im); return atan2f(im, re); }
};

// --------------------------- Servo audio object -----------------------------
// Input 0: program (i2sIn R) | Input 1: current sense (i2sIn L)
// Output 0: drive to DAC L
// Modes: IDLE (mute), CAL (internal sine gen + 3-channel Goertzel), RUN (servo)
class MfbServo : public AudioStream {
public:
  enum Mode { IDLE, CAL, RUN };
  MfbServo() : AudioStream(2, inQueue) {}

  // --- cal control (called from loop()) ---
  void calStart(float freq) {
    gRef.init(freq, FS_AUDIO); gAcc.init(freq, FS_AUDIO); gCur.init(freq, FS_AUDIO);
    phase = 0.0f;
    phaseInc = 2.0f * (float)M_PI * freq / FS_AUDIO;
    calSettleSamps  = (uint32_t)(SETTLE_MS  * 48);
    calMeasureSamps = (uint32_t)(MEASURE_MS * 48);
    calDone = false;
    mode = CAL;
  }
  bool calFinished() { return calDone; }
  void getCal(float &refM, float &accM, float &accP, float &curM, float &curP) {
    refM = gRef.mag();
    accM = gAcc.mag();  accP = gAcc.phase() - gRef.phase();
    curM = gCur.mag();  curP = gCur.phase() - gRef.phase();
  }

  // --- run control ---
  void setLoop(float kVel, float rampSec) {
    kVelTarget = kVel;
    kVelStep   = kVel / (rampSec * FS_AUDIO);
    kVel_      = 0.0f;
    hpf.setHighpass(ACCEL_HPF_HZ, 0.707f, FS_AUDIO);
    velLeak = expf(-2.0f * (float)M_PI * VEL_LEAK_HZ / FS_AUDIO);
    vel = 0;
    mode = RUN;
  }
  void setIdle() { mode = IDLE; }

  virtual void update() {
    audio_block_t *prog = receiveReadOnly(0);
    audio_block_t *cur  = receiveReadOnly(1);
    audio_block_t *out  = allocate();
    if (!out) { if (prog) release(prog); if (cur) release(cur); return; }

    // consume accel ring, block-aligned (catch up / hold-last on jitter)
    uint32_t avail = accelWr - accelRd;
    if (avail > AUDIO_BLOCK_SAMPLES + 8) accelRd = accelWr - AUDIO_BLOCK_SAMPLES; // resync
    for (int i = 0; i < AUDIO_BLOCK_SAMPLES; i++) {
      if (accelRd != accelWr) { lastAccel = accelRing[accelRd & RING_MSK]; accelRd++; }
      accelBlk[i] = (float)lastAccel * (1.0f / 2048.0f);   // ~±1.0 at ADC FS
    }

    switch (mode) {
      case CAL: {
        for (int i = 0; i < AUDIO_BLOCK_SAMPLES; i++) {
          float s = sinf(phase);
          phase += phaseInc; if (phase > 2.0f * (float)M_PI) phase -= 2.0f * (float)M_PI;
          out->data[i] = (int16_t)(s * CAL_AMP * 32767.0f);
          if (calSettleSamps) { calSettleSamps--; continue; }
          if (calMeasureSamps) {
            gRef.feed(s);
            gAcc.feed(accelBlk[i]);
            gCur.feed(cur ? (float)cur->data[i] * (1.0f / 32768.0f) : 0.0f);
            if (--calMeasureSamps == 0) calDone = true;
          }
        }
        break;
      }
      case RUN: {
        for (int i = 0; i < AUDIO_BLOCK_SAMPLES; i++) {
          if (kVel_ < kVelTarget) kVel_ += kVelStep;           // soft-start
          float a = hpf.process(accelBlk[i]);                  // DC-blocked accel
          vel = velLeak * vel + a * (1.0f / FS_AUDIO);         // leaky ∫a = velocity
          float p = prog ? (float)prog->data[i] * (1.0f / 32768.0f) : 0.0f;
          float y = p - kVel_ * vel * FS_AUDIO;                // fb normalized to samp units
          if (y >  0.95f) y =  0.95f;                          // hard clip guard
          if (y < -0.95f) y = -0.95f;
          out->data[i] = (int16_t)(y * 32767.0f);
        }
        break;
      }
      default:
        memset(out->data, 0, sizeof(out->data));
    }
    transmit(out, 0);
    release(out);
    if (prog) release(prog);
    if (cur)  release(cur);
  }

private:
  audio_block_t *inQueue[2];
  Mode mode = IDLE;
  // cal
  Goertzel gRef, gAcc, gCur;
  float phase = 0, phaseInc = 0;
  uint32_t calSettleSamps = 0, calMeasureSamps = 0;
  volatile bool calDone = false;
  // accel
  uint32_t accelRd = 0;
  int32_t  lastAccel = 0;
  float    accelBlk[AUDIO_BLOCK_SAMPLES];
  // run
  Biquad hpf;
  float vel = 0, velLeak = 0;
  float kVel_ = 0, kVelTarget = 0, kVelStep = 0;
};

// ----------------------------- Audio graph ----------------------------------
AudioInputI2S   i2sIn;      // ch0 = L = current sense, ch1 = R = program
AudioOutputI2S  i2sOut;
MfbServo        servo;
AudioConnection c1(i2sIn, 1, servo, 0);   // program  -> servo in0
AudioConnection c2(i2sIn, 0, servo, 1);   // current  -> servo in1
AudioConnection c3(servo, 0, i2sOut, 0);  // drive    -> DAC L
AudioConnection c4(servo, 0, i2sOut, 1);  // (mirror R, unused)

// ------------------------------ Cal storage ---------------------------------
struct CalData {
  uint32_t magic;            // 0x4D464201
  float fs, qms, qes, qts, r0, le_mH, plantK, kVel;
  float re;                  // Re used at cal time
};
static const uint32_t CAL_MAGIC = 0x4D464201;
static const int      CAL_ADDR  = 0;

// ------------------------------ Results -------------------------------------
static float magAcc[N_FREQS], phAcc[N_FREQS];
static float magZ[N_FREQS];   // |drive|/|current| — relative impedance
static CalData cal;

// log-domain interpolation of a crossing/peak between table points
static float interpX(float x0, float x1, float y0, float y1, float yt) {
  if (y1 == y0) return x0;
  return x0 + (x1 - x0) * (yt - y0) / (y1 - y0);
}

static void deriveParameters() {
  // --- Fs: impedance peak (parabolic interp in log-f) ---
  int ip = 0;
  for (int i = 1; i < N_FREQS - 3; i++)          // exclude HF tail (Le region)
    if (magZ[i] > magZ[ip]) ip = i;
  float fs = CAL_FREQS[ip];
  if (ip > 0 && ip < N_FREQS - 4) {
    float l0 = logf(CAL_FREQS[ip - 1]), l1 = logf(CAL_FREQS[ip]), l2 = logf(CAL_FREQS[ip + 1]);
    float y0 = magZ[ip - 1], y1 = magZ[ip], y2 = magZ[ip + 1];
    float d = (y0 - 2 * y1 + y2);
    if (fabsf(d) > 1e-12f) fs = expf(l1 - 0.5f * (y2 - y0) / d * ((l2 - l0) / 2));
  }
  cal.fs = fs;

  // --- r0 = Zmax/Re, using Z(10 Hz) ≈ Re as the relative-scale anchor ---
  float zRe   = magZ[0];
  float zMax  = magZ[ip];
  cal.r0 = zMax / zRe;
  cal.re = RE_OHMS;

  // --- Qms from half-power points of impedance peak: |Z| = sqrt(r0)*Re ---
  float zHalf = zRe * sqrtf(cal.r0);
  float f1 = 0, f2 = 0;
  for (int i = 1; i <= ip; i++)
    if (magZ[i - 1] < zHalf && magZ[i] >= zHalf)
      { f1 = interpX(CAL_FREQS[i - 1], CAL_FREQS[i], magZ[i - 1], magZ[i], zHalf); break; }
  for (int i = ip; i < N_FREQS - 3; i++)
    if (magZ[i] >= zHalf && magZ[i + 1] < zHalf)
      { f2 = interpX(CAL_FREQS[i], CAL_FREQS[i + 1], magZ[i], magZ[i + 1], zHalf); break; }
  cal.qms = (f1 > 0 && f2 > f1) ? cal.fs * sqrtf(cal.r0) / (f2 - f1) : 0;
  cal.qes = (cal.r0 > 1.001f && cal.qms > 0) ? cal.qms / (cal.r0 - 1.0f) : 0;
  cal.qts = (cal.qms > 0 && cal.qes > 0) ? cal.qms * cal.qes / (cal.qms + cal.qes) : 0;

  // --- Le from HF tail: |Z(f)| = sqrt(Re^2 + (2*pi*f*Le)^2), scaled via zRe ---
  float zHF_ohms = magZ[N_FREQS - 1] / zRe * RE_OHMS;   // Z at 2 kHz in ohms
  float x = zHF_ohms * zHF_ohms - RE_OHMS * RE_OHMS;
  cal.le_mH = (x > 0) ? sqrtf(x) / (2.0f * (float)M_PI * CAL_FREQS[N_FREQS - 1]) * 1000.0f : 0;

  // --- Plant gain K: accel/drive plateau above resonance (2*Fs..400 Hz avg) ---
  float ksum = 0; int kn = 0;
  for (int i = 0; i < N_FREQS - 3; i++)
    if (CAL_FREQS[i] > 2 * cal.fs && CAL_FREQS[i] <= 400) { ksum += magAcc[i]; kn++; }
  cal.plantK = kn ? ksum / kn : magAcc[N_FREQS - 4];

  // --- Pole placement: velocity feedback adds damping ---
  // Plant: a/u = K s^2 / (s^2 + w0/Qts s + w0^2); u = p - kv*(a/s)
  // Closed: s^2 + (w0/Qts + K*kv) s + w0^2  =>  kv = w0 (1/Qc - 1/Qts) / K
  float w0 = 2.0f * (float)M_PI * cal.fs;
  cal.kVel = (cal.qts > 0 && cal.plantK > 0)
           ? w0 * (1.0f / TARGET_QC - 1.0f / cal.qts) / cal.plantK
           : 0;
  if (cal.kVel < 0) cal.kVel = 0;   // driver already deader than target
  cal.magic = CAL_MAGIC;
}

static void printReport() {
  Serial.println(F("\n=========== MFB AUTOCAL REPORT ==========="));
  Serial.println(F("  f(Hz)    |Z|rel    Acc mag   Acc ph(deg)"));
  for (int i = 0; i < N_FREQS; i++) {
    Serial.printf("  %7.1f  %8.4f  %8.5f  %8.1f\n",
      CAL_FREQS[i], magZ[i], magAcc[i], phAcc[i] * 180.0f / (float)M_PI);
  }
  Serial.println(F("------------------------------------------"));
  Serial.printf("  Fs      = %.2f Hz\n", cal.fs);
  Serial.printf("  Re      = %.2f ohm (entered)\n", cal.re);
  Serial.printf("  r0      = %.2f   (Zmax/Re)\n", cal.r0);
  Serial.printf("  Qms     = %.2f\n", cal.qms);
  Serial.printf("  Qes     = %.2f\n", cal.qes);
  Serial.printf("  Qts     = %.3f\n", cal.qts);
  Serial.printf("  Le      = %.2f mH (series, at 2 kHz)\n", cal.le_mH);
  Serial.printf("  PlantK  = %.4f  (accel/drive, normalized)\n", cal.plantK);
  Serial.printf("  Target closed-loop Q = %.2f\n", TARGET_QC);
  Serial.printf("  k_vel   = %.4f  (velocity fb gain)\n", cal.kVel);
  Serial.printf("  Accel DC offset = %ld counts\n", (long)accelDC);
  Serial.println(F("  Cal saved to EEPROM. Entering RUN.\n"));
}

// ------------------------------- Setup / loop -------------------------------
void setup() {
  Serial.begin(115200);
  AudioMemory(24);
  setI2SFreq((int)FS_AUDIO);

  // ADC: 12-bit, 8x hw averaging, free-running; ISR snapshots at 48 kHz
  adc.adc0->setResolution(12);
  adc.adc0->setAveraging(8);
  adc.adc0->setConversionSpeed(ADC_CONVERSION_SPEED::HIGH_SPEED);
  adc.adc0->setSamplingSpeed(ADC_SAMPLING_SPEED::HIGH_SPEED);
  adc.adc0->startContinuous(ACCEL_PIN);
  accelTimer.begin(accelISR, 1000000.0f / FS_AUDIO);

  delay(1500);                        // PCM1808 modulator/HPF settle
  while (!Serial && millis() < 4000) {}

  // Accel DC offset: 0.25 s average
  int64_t acc = 0; const int NDC = 12000;
  for (int i = 0; i < NDC; i++) { acc += (int32_t)adc.adc0->analogReadContinuous(); delayMicroseconds(20); }
  accelDC = (int32_t)(acc / NDC);
  Serial.printf("Accel DC: %ld counts (%.3f V)\n", (long)accelDC, accelDC * 3.3f / 4095.0f);

  // ---------------- Autocal sweep ----------------
  Serial.println(F("AUTOCAL: stepped-sine sweep..."));
  for (int i = 0; i < N_FREQS; i++) {
    servo.calStart(CAL_FREQS[i]);
    while (!servo.calFinished()) { yield(); }
    float rM, aM, aP, cM, cP;
    servo.getCal(rM, aM, aP, cM, cP);
    magAcc[i] = (rM > 0) ? aM / rM : 0;
    phAcc[i]  = aP;
    magZ[i]   = (cM > 1e-9f) ? rM / cM : 0;   // V/I, relative
    Serial.printf("  %6.1f Hz  done\n", CAL_FREQS[i]);
  }
  servo.setIdle();

  deriveParameters();
  EEPROM.put(CAL_ADDR, cal);
  printReport();

  // ---------------- Run ----------------
  servo.setLoop(cal.kVel, 2.0f);      // 2 s soft-start ramp
}

void loop() {
  // periodic status; hook for serial commands (recal, k_vel trim) later
  static uint32_t t = 0;
  if (millis() - t > 5000) {
    t = millis();
    Serial.printf("RUN  cpu=%.1f%%  mem=%d\n", AudioProcessorUsage(), AudioMemoryUsageMax());
  }
}
