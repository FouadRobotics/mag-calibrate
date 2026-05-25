# QMC5883P Magnetometer Calibration

Standalone sketch for 2-D hard-iron and soft-iron calibration of a **QMC5883P** magnetometer on an Arduino Nano RP2040 Connect. Outputs four `static constexpr float` lines that paste straight into an EKF's `config.h`.

## How it works

1. Boot the chip in continuous mode (CTRL1=0x05, CTRL2=0x01 — see gotchas below).
2. Print a diagnostic dump (5 register reads, 100 ms apart) so you can confirm data is flowing *before* the 60-second calibration window starts.
3. Read X and Y at 20 Hz for 60 seconds. Track running min/max on each axis.
4. Compute offsets (`(max + min) / 2` per axis) and scales (each axis normalised to the average range).
5. Print the four constants.

## Build & run

1. Wire QMC5883P to GP12 (SDA) and GP13 (SCL).
2. Flash `mag_calibrate/mag_calibrate.ino`. Open Serial Monitor at **115200**.
3. Watch the diagnostic dump. **If X/Y are all 0 or all 0xFF, data isn't flowing** — check wiring and the gotchas section.
4. When `GO` appears, slowly rotate the robot through **3+ full 360° circles** (~30 s), keeping it flat.
5. After 60 s, paste the four printed lines into your EKF's `config.h`:

```cpp
static constexpr float MAG_OFFSET_X = ...f;
static constexpr float MAG_OFFSET_Y = ...f;
static constexpr float MAG_SCALE_X  = ...f;
static constexpr float MAG_SCALE_Y  = ...f;
```

## Tune (`mag_calibrate.ino`)

| Constant | Default | Effect |
|---|---|---|
| `QMC_CTRL1_VAL` | `0x05` | RNG=01 (×4 range). Raise to `0x09` (RNG=10) if X or Y still saturate at ±32760. |
| `QMC_CTRL2_VAL` | `0x01` | MODE=continuous, ODR=10 Hz. **Don't drop the `0x01` — it's the mode bit, not just an init flag.** |
| `READ_INTERVAL_MS` | `50` | Poll period. 20 Hz × 60 s ≈ 1200 samples — plenty for a 2-D circle. |
| `CAL_DURATION_MS` | `60000` | Total calibration window. Shorten only if you can rotate faster cleanly. |

## What "good" looks like

| Indicator | Good | Suspicious |
|---|---|---|
| Sample count | > 1000 | < 50 → no data flow |
| X / Y range | a few thousand counts, similar | < 100 → rotate more fully |
| Offsets | < 5 % of range | huge → nearby magnet/motor distorting hard iron |
| Scales | within 5 % of 1.0 | asymmetric → significant soft iron |

If `X_max` ≈ +32760 or `Y_min` ≈ −32760, the range setting (`QMC_CTRL1_VAL`) is too sensitive — bump it up and redo.

## Gotchas (read this before debugging)

The QMC5883P shares its register map with the QMC5883L but **uses different bit assignments**:

- **CTRL1**: the `0x1D` value every blog post recommends is for the **QMC5883L** and puts the QMC5883P in an invalid state. Use `0x05` (RNG bits at positions [3:2], not [5:4]).
- **CTRL2**: the MODE bit lives in **CTRL2 bit 0**, not CTRL1 like the QMC5883L. After the soft reset, you *must* write `CTRL2 = 0x01` or the chip sits in standby forever, returning frozen power-on values.

Both fixes are baked into the sketch. The diagnostic dump catches missing-data cases before you waste 60 seconds rotating.

## What this doesn't do

- **3-D calibration.** Only X and Y are calibrated. Z is fine for heading-only use; not enough for full MARG on a tilted chassis (you'd need a tumbling-sphere routine for that).
- **Temperature compensation.** Values drift as motors heat up.

## Related repos

- [`imu-madgwick-plot`](https://github.com/FouadRobotics/imu-madgwick-plot) — prerequisite (IMU + Madgwick visualisation)
- [`embedded-ekf-with-mag`](https://github.com/FouadRobotics/embedded-ekf-with-mag) — consumes the four output constants
- [`embedded-ekf-cortex-m0`](https://github.com/FouadRobotics/embedded-ekf-cortex-m0) — same calibration, fused via Madgwick MARG

## License

MIT.
