# MFB Servo Subwoofer — Digital Motional Feedback Controller

Teensy 4.x digital servo loop for a RECOIL RW8D2 dual-voice-coil 8" driver,
using cone-mounted accelerometer feedback with automatic startup calibration
of the driver's Thiele/Small parameters.

## System

```
Program audio (RCA / Flex Eight line out)
        |
   PCM1808 ADC (I2S, 48 kHz, 24-bit)  <- ch L: INA240 current sense
        |                                ch R: program
   Teensy 4.0/4.1  <- A0: ADXL326 accel (cone-mounted, dust cap)
        |          (I2S clock master; MCLK/BCLK/LRCLK)
   UDA1334A DAC (3-wire I2S, PLL from BCLK)
        |
   Class D sub amp (BTL, Fosi ZA3)
        |
   RW8D2, voice coils in series (4 ohm)
        |
   0.01 ohm shunt (10x 0.1 ohm 10W parallel) -> INA240A1 diff amp -> ADC ch L
```

- Driver current sensed differentially across a floating shunt in the BTL
  loop (INA240, +/-80 V CM, onboard R100 removed)
- Accelerometer: ADXL326 Z-axis, 22 nF || 4.7 nF filter vs internal 32 k
  (~186 Hz anti-alias corner), sampled at 48 kHz via ADC ISR ring buffer
- All modules on a common 5 V star; speaker power loop never touches
  signal ground

## Boot sequence

1. Settle ADC/DAC (1.5 s), measure accelerometer DC offset
2. **Autocal**: 32-point stepped-sine sweep (10 Hz - 2 kHz), per-tone
   Goertzel magnitude/phase on reference, accel, and current channels
3. Derive: Fs (impedance peak, parabolic interp), r0, Qms (half-power),
   Qes, Qts, Le (HF impedance tail), plant gain K (accel plateau)
4. Pole placement: velocity feedback gain
   `k_v = w0 * (1/Qc - 1/Qts) / K`, target closed-loop Qc = 0.5
5. Full report to USB serial; cal struct to EEPROM
6. **Run**: program passthrough minus k_v * velocity (leaky-integrated
   accel), 2 s soft-start ramp, +/-0.95 FS clip guard

## Source

| File | Description |
|---|---|
| `mfb_servo_f32.ino` | **Primary build.** OpenAudio F32 library: 24-bit I2S, float DSP, runtime 48 kHz / 16-sample block config |
| `mfb_servo.ino` | int16 reference build (stock Teensy Audio Library, 16-bit I/O, requires `-DAUDIO_BLOCK_SAMPLES=16` for servo use) |
| `docs/mfb-carrier-pcb-design.md` | Carrier PCB: netlist, placement, routing rules, BOM, bring-up order |
| `docs/mfb-bypass-caps.csv` | Per-node decoupling/coupling capacitor table |
| `docs/mfb-current-sense-wiring.svg` | BTL current-sense wiring detail |

## Dependencies

- Teensyduino (Audio, ADC, EEPROM)
- [OpenAudio_ArduinoLibrary](https://github.com/chipaudette/OpenAudio_ArduinoLibrary)
  (F32 build only)

## Configuration (top of sketch)

| Constant | Meaning | Note |
|---|---|---|
| `RE_OHMS` | DC resistance, coils in series | **Set from DMM before first run** |
| `CAL_AMP` | Sweep level, fraction of DAC FS | Start low; excursion = CAL_AMP x amp gain |
| `TARGET_QC` | Closed-loop target Q | 0.5 default |
| `ACCEL_HPF_HZ` | Servo-path DC blocker | 5 Hz |
| `VEL_LEAK_HZ` | Velocity integrator leak | 2 Hz |

## Bring-up checklist

1. Accel DC ~2048 counts (~1.65 V) on serial; tap test
2. Scope BCLK vs LRC: 32 BCLK per half-cycle = 32-bit slots confirmed
3. First autocal at low amp volume; verify small excursion
4. Sanity-check report (Fs, Qts vs datasheet order-of-magnitude)
5. Mark amp volume knob — plant gain K is calibrated at that setting
6. Loop closes automatically after report (2 s ramp); listen for ringing
   or oscillation, kill power if unstable

## Known limitations / roadmap

- k_v is cold-coil cal; no thermal Re tracking yet (derate ~20% or add
  gain scheduling from the current-sense channel)
- Z(10 Hz) ~ Re anchor biases r0 slightly low for low-Fs drivers
- No excursion limiter beyond FS clip guard
- Feedback transport delay = 2 blocks + DAC interpolation filter; 16-sample
  blocks keep it ~0.7-1 ms

## Power / grounding

- Run: 5 V USB-C 2-prong wall wart into Teensy USB; modules fed from VIN
  pin (VUSB-VIN pad intact)
- Dev: laptop USB; on-charger hum implies grounded laptop PSU — run on
  battery or power-only cable
- DAW source arrives optically (Flex Eight TOSLINK) — galvanically isolated
