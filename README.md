# QMC5883P Magnetometer Calibration

A standalone sketch that performs **hard-iron and soft-iron calibration** of a QMC5883P magnetometer wired to an Arduino Nano RP2040 Connect.

## Why calibrate

Raw magnetometer output is corrupted by two effects:

- **Hard iron** — permanent magnetic offsets from nearby ferrous material (the chassis, motor mounts, screws). Shifts the centre of the X-Y plot away from the origin.
- **Soft iron** — induced magnetic distortion from materials that magnetize in response to the Earth's field. Stretches the X-Y plot into an ellipse instead of a circle.

The calibration measures both. Rotate the robot through several full turns; the sketch captures the X and Y extremes, computes the offset (centre of the circle) and the scale (ellipse aspect ratio), and prints `static constexpr float` lines that paste straight into the EKF's `config.h`.

## Hardware

| Part | Role |
|---|---|
| Arduino Nano RP2040 Connect | Host MCU |
| QMC5883P magnetometer | I²C address `0x2C`, wired to GP12 (SDA) / GP13 (SCL) |

## Why this isn't a five-minute job (gotchas)

This sketch went through several painful iterations. Two issues that bit hardest:

1. **QMC5883L vs QMC5883P register layout.** The two chips share a register map but have *different bit assignments inside CTRL1*. The widely-copied "0x1D" value that works on QMC5883L puts QMC5883P in an invalid state where DRDY never fires. **CTRL1 must be `0x05`** (RNG=01, ×4 range to avoid saturation).
2. **MODE bit lives in CTRL2, not CTRL1.** On the QMC5883P, the continuous-mode bit is bit 0 of CTRL2 (not CTRL1 like the QMC5883L). After the soft reset, you must write `CTRL2 = 0x01` or the sensor sits in standby forever, returning frozen data.

Both fixes are baked into this sketch. If your data shows frozen X/Y values, you almost certainly have one of these two bugs.

## Usage

1. Wire QMC5883P to GP12 (SDA) and GP13 (SCL).
2. Flash the sketch. Open Serial Monitor at **115200** baud.
3. Watch the **diagnostic register dump** (5 reads, 100 ms apart) at startup. If X/Y are 0/0 or 0xFF/0xFF, data isn't flowing — check wiring, CTRL2, and the address.
4. When `GO` appears, **slowly rotate the robot through at least 3 full 360° circles** (~30 s), keeping it as flat as possible.
5. After 60 s, paste the four printed lines into your EKF's `config.h`:

```cpp
static constexpr float MAG_OFFSET_X = ...f;
static constexpr float MAG_OFFSET_Y = ...f;
static constexpr float MAG_SCALE_X  = ...f;
static constexpr float MAG_SCALE_Y  = ...f;
```

## What "good" calibration looks like

| Indicator | Good | Suspicious |
|---|---|---|
| Sample count | > 1000 | < 50 → no data flow |
| X / Y range | a few thousand counts, similar sizes | < 100 → rotate more fully |
| Offsets | small relative to range (< 5 %) | huge → strong hard iron, check for nearby magnets |
| Scales | very close to 1 | strongly asymmetric → significant soft iron |

If the range approaches ±32760 (the int16 saturation), the chip's RNG setting is wrong — `CTRL1` needs more range (e.g. `0x05` → `0x09`).

## What this sketch does NOT do

- **3-D calibration.** This is a 2-D circle method — only X and Y get calibrated. Z is uncalibrated. Fine for heading-only use (yaw); not enough for full MARG orientation tracking on a tilted chassis. For 3-D you'd need a tumbling-sphere routine that captures the X-Y-Z ellipsoid.
- **Bias correction over temperature.** The values are good for the temperature you calibrated at. Hot motors → drift.

## Related projects

- [`imu-madgwick-plot`](https://github.com/FouadRobotics/imu-madgwick-plot) — predecessor (no mag, IMU only)
- [`embedded-ekf-with-mag`](https://github.com/FouadRobotics/embedded-ekf-with-mag) — 5-state EKF that consumes these calibration values
- [`embedded-ekf-cortex-m0`](https://github.com/FouadRobotics/embedded-ekf-cortex-m0) — Madgwick MARG + EKF that also consumes these values

## License

MIT.
