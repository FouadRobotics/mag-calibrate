// mag_calibrate.ino – QMC5883P hard-iron + soft-iron calibration tool.
//
// ── How to use ────────────────────────────────────────────────────────────
//  1. Wire QMC5883P to GP12 (SDA) and GP13 (SCL).  LSM6DSOX can stay connected.
//  2. Flash this sketch.  Open Serial Monitor at 115200 baud.
//  3. Read the diagnostic lines that print first — they show whether data is
//     flowing and what the raw register values look like.
//  4. When "GO" appears, slowly rotate the robot through at least 3 full
//     360-degree circles (~30 s), then hold still until the timer expires.
//  5. Paste the four printed lines into config.h.
//
// ── Board ────────────────────────────────────────────────────────────────
//  Arduino Nano RP2040 Connect, earlephilhower/arduino-pico package.

#include <Wire.h>

// ── QMC5883P constants ────────────────────────────────────────────────────
#define QMC_ADDR      0x2C
#define QMC_XOUT_L    0x00   // start of 6 output bytes (X,Y,Z little-endian int16)
#define QMC_STATUS    0x06   // bit0=DRDY, bit1=OVL, bit2=DOR
#define QMC_CTRL1     0x09
#define QMC_CTRL2     0x0A   // bit7=SOFT_RST
#define QMC_SET_RST   0x0B

// QMC5883P CTRL1 layout: [7:6]=OSR2 [5:4]=OSR1 [3:2]=RNG [1:0]=reserved
// QMC5883P CTRL2 layout: [7]=SOFT_RST [6]=ROL_PNT [2:1]=ODR [0]=MODE
//
// RNG=00 (CTRL1=0x01) saturates on Earth's field; RNG=01 (bit2=1,
// CTRL1=0x05) expands the range 4× so calibration peaks land ~25% of
// full scale.  CTRL2=0x01 sets MODE=continuous on QMC5883P.
#define QMC_CTRL1_VAL 0x05
#define QMC_CTRL2_VAL 0x01

// Read the sensor on this interval regardless of the DRDY bit.
// Avoids getting stuck if DRDY works differently on this chip revision.
#define READ_INTERVAL_MS  50   // 20 Hz max reads

static constexpr uint32_t CAL_DURATION_MS = 60000UL;

// ── I2C helpers ───────────────────────────────────────────────────────────
static void qmc_write(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(QMC_ADDR);
    Wire.write(reg);
    Wire.write(val);
    Wire.endTransmission();
}

// Returns number of bytes actually received.
static uint8_t qmc_read_bytes(uint8_t reg, uint8_t* dst, uint8_t len) {
    Wire.beginTransmission(QMC_ADDR);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return 0;
    Wire.requestFrom((int)QMC_ADDR, (int)len);
    uint8_t got = 0;
    while (Wire.available() && got < len) dst[got++] = Wire.read();
    return got;
}

// Read X and Y unconditionally (no DRDY gate).
static bool qmc_read_xy(int16_t& x, int16_t& y) {
    uint8_t buf[4] = {};
    if (qmc_read_bytes(QMC_XOUT_L, buf, 4) < 4) return false;
    x = (int16_t)((uint16_t)buf[0] | ((uint16_t)buf[1] << 8));
    y = (int16_t)((uint16_t)buf[2] | ((uint16_t)buf[3] << 8));
    return true;
}

// ── Module state ──────────────────────────────────────────────────────────
static int16_t  s_x_min, s_x_max, s_y_min, s_y_max;
static uint32_t s_sample_count = 0;
static uint32_t s_start_ms     = 0;
static bool     s_done         = false;

// ── Print results ─────────────────────────────────────────────────────────
static void print_results() {
    if (s_sample_count < 50) {
        Serial.println(F("\n# WARNING: fewer than 50 samples collected."));
        Serial.println(F("# The sensor produced no data. Check diagnostic output above."));
        Serial.println(F("# If all raw values were 0 or -1, verify the wiring and re-run."));
        return;
    }

    const float range_x   = (s_x_max - s_x_min) / 2.0f;
    const float range_y   = (s_y_max - s_y_min) / 2.0f;

    if (range_x < 100.0f || range_y < 100.0f) {
        Serial.println(F("\n# WARNING: axis range < 100 LSB — rotate more fully next time."));
    }

    const float offset_x  = (s_x_max + s_x_min) / 2.0f;
    const float offset_y  = (s_y_max + s_y_min) / 2.0f;
    const float avg_range = (range_x + range_y) / 2.0f;
    const float scale_x   = (range_x > 1.0f) ? avg_range / range_x : 1.0f;
    const float scale_y   = (range_y > 1.0f) ? avg_range / range_y : 1.0f;

    Serial.println(F("\n# ══════════════════════════════════════════════════"));
    Serial.println(F("# CALIBRATION COMPLETE"));
    Serial.println(F("# ══════════════════════════════════════════════════"));
    Serial.print(F("# Samples : ")); Serial.println(s_sample_count);
    Serial.print(F("# X range : ")); Serial.print(s_x_min);
    Serial.print(F(" to "));         Serial.println(s_x_max);
    Serial.print(F("# Y range : ")); Serial.print(s_y_min);
    Serial.print(F(" to "));         Serial.println(s_y_max);
    Serial.println(F("#"));
    Serial.println(F("# ── Paste into config.h ───────────────────────────"));
    Serial.print(F("static constexpr float MAG_OFFSET_X = "));
    Serial.print(offset_x, 1);  Serial.println(F("f;"));
    Serial.print(F("static constexpr float MAG_OFFSET_Y = "));
    Serial.print(offset_y, 1);  Serial.println(F("f;"));
    Serial.print(F("static constexpr float MAG_SCALE_X  = "));
    Serial.print(scale_x, 5);   Serial.println(F("f;"));
    Serial.print(F("static constexpr float MAG_SCALE_Y  = "));
    Serial.print(scale_y, 5);   Serial.println(F("f;"));
    Serial.println(F("# ───────────────────────────────────────────────────"));
}

// ── Setup ─────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 5000) {}

    Wire.setSDA(12);
    Wire.setSCL(13);
    Wire.begin();

    // ── Probe ──────────────────────────────────────────────────────────────
    Wire.beginTransmission(QMC_ADDR);
    if (Wire.endTransmission() != 0) {
        Serial.println(F("# ERROR: no device at 0x2C. Check SDA/SCL wiring."));
        while (true) delay(1000);
    }
    Serial.println(F("# QMC5883P found at 0x2C."));

    // ── Init ───────────────────────────────────────────────────────────────
    qmc_write(QMC_CTRL2,  0x80);   // soft reset
    delay(20);
    qmc_write(QMC_SET_RST, 0x01);  // mandatory
    delay(5);
    qmc_write(QMC_CTRL1,  QMC_CTRL1_VAL);  // OSR/range defaults (QMC5883L: MODE=continuous here)
    qmc_write(QMC_CTRL2,  QMC_CTRL2_VAL);  // QMC5883P: MODE=continuous is in CTRL2 bit0
    delay(20);

    // ── Diagnostic register dump ───────────────────────────────────────────
    // Print status + all 6 output bytes so we can see if data is flowing
    // before the calibration run starts.
    Serial.println(F("# ── Register diagnostic (5 reads, 100 ms apart) ──"));
    Serial.println(F("# status, X_L, X_H, Y_L, Y_H, Z_L, Z_H  (hex)"));
    for (int i = 0; i < 5; ++i) {
        delay(100);
        uint8_t st = 0;
        qmc_read_bytes(QMC_STATUS, &st, 1);
        uint8_t raw[6] = {};
        qmc_read_bytes(QMC_XOUT_L, raw, 6);

        Serial.print(F("# 0x")); Serial.print(st, HEX);
        for (int b = 0; b < 6; ++b) {
            Serial.print(F("  0x"));
            if (raw[b] < 0x10) Serial.print('0');
            Serial.print(raw[b], HEX);
        }
        // Also decode X and Y as signed integers for readability
        int16_t dx = (int16_t)((uint16_t)raw[0] | ((uint16_t)raw[1] << 8));
        int16_t dy = (int16_t)((uint16_t)raw[2] | ((uint16_t)raw[3] << 8));
        Serial.print(F("   X=")); Serial.print(dx);
        Serial.print(F(" Y=")); Serial.println(dy);
    }
    Serial.println(F("# ── If X/Y are all 0 or -1, data is not flowing. ──"));
    Serial.println(F("#    Check CTRL1/CTRL2 and wiring, then re-flash."));
    Serial.println(F("# ──────────────────────────────────────────────────"));

    // ── Calibration intro ──────────────────────────────────────────────────
    s_x_min = s_y_min =  32767;
    s_x_max = s_y_max = -32768;

    Serial.println(F("# QMC5883P Calibration"));
    Serial.println(F("# Place robot on a FLAT surface."));
    Serial.println(F("# When GO appears: rotate slowly through 3+ full circles."));
    Serial.println(F("# CSV: raw_x, raw_y"));
    delay(3000);
    Serial.println(F("GO — start rotating now!"));
    s_start_ms = millis();
}

// ── Loop ──────────────────────────────────────────────────────────────────
void loop() {
    if (s_done) return;

    // Read on a fixed timer — no DRDY dependency.
    static uint32_t s_last_read = 0;
    const uint32_t  now         = millis();
    if (now - s_last_read >= READ_INTERVAL_MS) {
        s_last_read = now;

        int16_t x, y;
        if (qmc_read_xy(x, y)) {
            if (x < s_x_min) s_x_min = x;
            if (x > s_x_max) s_x_max = x;
            if (y < s_y_min) s_y_min = y;
            if (y > s_y_max) s_y_max = y;
            ++s_sample_count;

            // Print at ~5 Hz for monitoring.
            static uint32_t s_last_print = 0;
            if (now - s_last_print >= 200) {
                s_last_print = now;
                Serial.print(x); Serial.print(','); Serial.println(y);
            }
        }
    }

    // Countdown banner every 10 s.
    static uint32_t s_last_banner = 0;
    const uint32_t  elapsed       = now - s_start_ms;
    if (elapsed - s_last_banner >= 10000UL && elapsed < CAL_DURATION_MS) {
        s_last_banner = elapsed;
        Serial.print(F("# "));
        Serial.print((CAL_DURATION_MS - elapsed) / 1000);
        Serial.println(F(" s remaining..."));
    }

    if (elapsed >= CAL_DURATION_MS) {
        s_done = true;
        print_results();
    }
}
