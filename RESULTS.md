# Results

This file is the source of truth for measured results in the module evaluation. Each entry lists:
- the bag it came from
- the script and code version that produced it
- what was checked

Bags are on the Pi in `~/data/` and copied to `postprocessing/data/`. Times are local (EST) unless marked Z (UTC).

Rig: 5 GPS and 4 IMUs, as in `ros2_ws/src/usv_bringup/config/rig.yaml`. The BNO055 is left out.

---

## 1. Static test (2026-11-22)

**Bag:** `static_20261123T025246Z`
- Recorded 21:55–23:52, 7047 s (1.96 h), 2.1 GB, 8.6 M messages. All topics were recorded (`ros2 bag record -a`). Location: _not noted; fill in_.
- `static_20261123T025246Z_setup/` beside it holds:
  - the config folder
  - the git state: commit 20e8f4c, plus an uncommitted TODO cleanup of `rig.yaml` (poses identical)
  - an empty `params.yaml` (the dump was interrupted)
- The ICM-20948's low-pass filter was **off** (driver default) in this bag.
- MPU9250 and MMC5983MA mag axes were already fixed (driver versions match the repo).

**Health check:** a scratch script, run 2026-11-23.
- All 9 modules ran for the whole bag with no gaps over 0.5 s, apart from the 1 Hz topics.
- No driver warnings, no NaNs, and no frozen accel/gyro samples. About 0.1 % of mag samples are exact repeats.

| Topic group | Rate |
|---|---|
| u-blox fix / fix_velocity / navpvt | 5.0 Hz |
| u-blox navstate (NAV-SAT) | 1 Hz on the M8, 0.25 Hz on the M10/F10 |
| M10-25Q fix (NMEA driver) | 1.0 Hz |
| ICM-20948 imu / mag | 199.9 / 96.9 Hz |
| ISM330+LIS3 imu / mag | 101.8 / 77.1 Hz |
| ISM330+MMC imu / mag | 102.3 / 50.0 Hz |
| MPU9250 imu / mag | 100.0 / 96.9 Hz |

**GPS** (whole bag, stationary, no outlier removal; CEP around the median position)

| Module | 3D + SBAS fix | Satellites, median (range) | pDOP | CEP50 / CEP95 / max | Claimed σ (median) | Repeated positions |
|---|---|---|---|---|---|---|
| SAM-M8Q | 100 % | 17 (12–19) | 1.42 | 1.69 / 3.66 / 5.8 m | 1.26 m (hAcc) | 20.6 % |
| SAM-M10Q | 100 % | 17 (10–22) | 1.69 | 0.83 / 1.92 / 3.5 m | 0.30 m | 18.8 % |
| NEO-F10N | 100 % | 23 (19–24) | 1.21 | 0.78 / 2.71 / 3.9 m | 1.24 m | **56.9 %** |
| M10-25Q (NMEA) | 100 % | – | – | 0.77 / 1.87 / 2.9 m | 0.05 m (HDOP × driver default epe) | 4.8 % |
| Micro M10 | 100 % (first fix at 1 s) | 31 (25–32) | 0.96 | 1.10 / 2.28 / 2.9 m | 0.17 m | 28.2 % |

- The M10s claim far better accuracy than they deliver, and the M10-25Q's NMEA σ is 15× too small. This is the gap "tuned" mode corrects.
- NEO-F10N: repeats 57 % of its 5 Hz positions exactly. Cause not checked (possibly a static-hold-like filter).
- Fix stamps: Pi clock minus stamp is −0 ms on the M8 (it stamps with the Pi clock) and −62 to −99 ms median on the M10/F10 (they stamp with GPS time).
- NMEA from the u-blox modules is **missing**: there is no `/gps/<id>/nmea` topic. The apt `ublox_gps` apparently doesn't publish it. Accepted, not fixed.

**IMU** (whole bag; temperature drift 1–2 °C)

| IMU | \|a\| (m/s²) | Gyro mean (°/s) | Gyro sd (°/s) | Mag mean (µT, raw) | \|B\| raw (µT) | Temp (°C) |
|---|---|---|---|---|---|---|
| MPU9250 | 9.948 | −1.362, 1.550, 0.044 | 0.064, 0.071, 0.058 | 4.3, −20.0, −29.2 | 35.7 | 25.0 → 23.0 |
| ICM-20948 | 10.144 | 0.342, −0.553, 0.345 | **1.129, 1.315, 1.578** (filter off) | 18.7, 8.3, −27.3 | 34.1 | – |
| ISM330+MMC | 9.857 | 0.145, −0.425, −0.282 | 0.046, 0.041, 0.036 | 27.9, −12.7, −46.8 | 56.0 | 16.5 → 16.0 |
| ISM330+LIS3 | 9.665 | −0.049, −0.588, 0.154 | 0.050, 0.044, 0.035 | −65.9, 12.8, −65.4 | 93.7 | 18.6 → 17.6 |

- **ICM-20948 noise:** about 25× the others with its low-pass filter off. The driver was patched 2026-11-23 00:49 (filter 50/51 Hz). Bench check afterwards: gyro sd 0.08 °/s, accel sd 0.016 m/s².
- No static re-run is possible. For the ICM, long-τ drift comes from this bag (a 50 Hz filter doesn't change it), and white noise from the still periods of later bags.
- Not yet computed: Allan deviation, TTFF, k factors. These come from `static_process.sh`, still to be written.

---

## 2. Rig calibration, off the vehicle (2026-11-23)

**Bag:** `calib_20261123T060944Z`
- 5.2 min, outdoors in the open. Steps marked with `ros2 run usv_bringup mark calib`: faces +z −z +x −x +y −y (15 s each, that rig axis up), then a 54 s tumble.
- ICM low-pass filter on.

**Script:** `postprocessing/calibrate.py` (commit 46b4352) → `postprocessing/calib/rig.yaml`.

**Other bags:**
- `calib_20261123T055529Z` was recorded indoors and is not used: all four magnetometers saw the same ±2.2 µT change as the rig moved (residuals 97 % correlated), so the field there wasn't uniform.
- `calib_20261123T060809Z` started early by mistake and should be ignored.

| IMU | Accel bias (m/s²) | Accel scale | Gyro bias (°/s) | Mag offset (µT) | \|B\| before → after (µT) |
|---|---|---|---|---|---|
| ICM-20948 | −0.199, 0.070, 0.170 | 1.001, 0.999, 1.014 | 0.331, −0.605, 0.365 | 7.3, 23.5, 6.7 | 44.9 ± 11.6 → 41.3 ± 0.91 |
| ISM330 + LIS3MDL | 0.107, −0.192, −0.110 | 0.995, 1.002, 0.994 | −0.023, −0.578, 0.175 | −50.3, 26.1, −28.2 | 76.8 ± 22.1 → 42.9 ± 0.75 |
| ISM330 + MMC5983MA | 0.026, −0.184, 0.078 | 0.997, 0.995, 0.994 | 0.161, −0.450, −0.273 | 11.4, 2.0, −5.7 | 45.8 ± 8.3 → 43.6 ± 0.55 |
| MPU9250 | 0.229, −0.021, −0.007 | 0.998, 0.997, 1.011 | −1.261, 1.574, 0.044 | −9.6, −5.9, 2.8 | 42.8 ± 7.6 → 42.3 ± 0.90 |

How the numbers are used (each sensor's own axes):
- **Accel:** `a = (a_raw − bias) / scale`.
- **Mag:** `B = matrix · (B_raw − offset)`. The matrices are in `rig.yaml`.
- **Gyro bias:** for reference only. Each bag uses its own from its start still period.

**Checks**
- **Accel:** all six faces were found for all four IMUs. The ICM and MPU read 1.4 % and 1.1 % high on z, which explains the ICM's \|a\| = 10.14 in the static bag.
- **Repeatability vs the indoor run:**
  - Accel bias within about 0.03 m/s².
  - Gyro bias within 0.05 °/s.
  - Mag offsets within 2 µT, except the ICM's y (21.6 vs 23.5).
- **Gyro bias vs the static bag:** within 0.07 °/s for all four.
- **Mag fit:** full ellipsoid on all four, with at least 421 samples per octant. The \|B\| spread shared by all four is 0.6 µT; each sensor's own part is 0.23–0.47 µT.
- **Field direction agreement:** the calibrated directions, turned into rig axes, agree between sensors to 1.5 / 2.6 / 2.4° median (LIS3 / MPU / MMC). The ICM is 5.0° off (6.6° at the 95th percentile).
- **Gravity-to-field angle** across the six holds varies by 3.4° (ICM), 1.8° (LIS3), 2.3° (MMC) and 3.1° (MPU). The mean is about 141.5°, so the dip is about 51.5°.
  - So each magnetometer is misaligned with its accelerometer by a few degrees, or its board is tilted on the rig. The ellipsoid can't correct that.
  - A mag-to-accel alignment step could be added to `calibrate.py` from this bag if needed.

---

## 3. Motor test and vehicle offset, rig on the Husky (2026-11-23)

**Bag:** `motor_20261123T063104Z`
- 7.4 min. Steps marked with `ros2 run usv_bringup mark motor`: off, low, mid, high, off (60 s each, wheels lifted), then turns (49 s, wheels down).
- Turns: 1080° counter-clockwise, then 1080° clockwise, median 42 °/s, max 74 °/s.

**Script:** `postprocessing/motor.py` (uncommitted as of this run) added `vehicle_offset`, `vehicle_fit` and `motor` to `postprocessing/calib/rig.yaml`.

**Vehicle hard iron** (circle fit to the horizontal field during the turns)

| IMU | Offset (µT) | Horizontal field (µT) | Heading error if uncorrected | Circle residual (µT) |
|---|---|---|---|---|
| ICM-20948 | 2.96 | 23.72 | 7.2° | 1.41 |
| ISM330+LIS3 | 5.16 | 24.59 | 12.1° | 1.35 |
| ISM330+MMC | 4.16 | 25.21 | 9.5° | 1.29 |
| MPU9250 | **8.93** | 24.59 | **21.3°** | 1.48 |

The horizontal field of 24–25 µT is consistent with the calibration bag: 42 µT at a dip of about 51.5° gives a horizontal component of about 26 µT.

**Motor interference, wheels lifted** (against the two `off` steps; worst-case heading error)

| IMU | Heading error low / mid / high | Accel sd off → high (m/s²) | Gyro sd off → high (°/s) |
|---|---|---|---|
| ICM-20948 | 0.08 / 0.12 / 0.14° | 0.032 → 0.060 | 0.144 → 0.190 |
| ISM330+LIS3 | 0.59 / 0.38 / 0.60° | 0.017 → 0.050 | 0.077 → 0.139 |
| ISM330+MMC | 0.12 / 0.09 / 0.12° | 0.014 → 0.049 | 0.072 → 0.139 |
| MPU9250 | 0.11 / 0.15 / 0.19° | 0.029 → 0.058 | 0.113 → 0.165 |

- With the wheels lifted (low current), motor fields are negligible: under 0.2°. The LIS3's 0.4–0.6° doesn't rise with speed, so it is probably that chip's noise and not the motors.
- Vibration raises the accel noise 2–3.5× and the gyro noise 1.3–1.9× at full speed.

**Heading during the turns** (magnetometer heading, with rig and vehicle calibration, against the gyro-integrated heading; scratch analysis 2026-11-23)

| IMU | RMS error | RMS with a turn-direction term | Turn-direction offset |
|---|---|---|---|
| ICM-20948 | 5.68° | 3.76° | ±4.45° |
| ISM330+LIS3 | 5.49° | 3.27° | ±4.60° |
| ISM330+MMC | 5.45° | 3.56° | ±4.31° |
| MPU9250 | 5.78° | 3.86° | ±4.50° |

- Without the vehicle offset the RMS errors are 5.9 / 8.7 / 7.8 / 13.6°, so the offset correction is essential.
- **About 4.5° of the heading error flips sign with turn direction, the same on all four IMUs** (errors 0.56–0.86 correlated between sensors). Something common to the vehicle causes it, not the sensors. Tilt during the turns (about 2°) explains only about 0.2° of it.
- Two candidate causes, which this bag can't separate because it has one block of each direction:
  1. **Motor current under load:** in a skid-steer turn, each side's current reverses with the turn direction. The lifted-wheel test draws too little current to show this.
  2. **A ~0.09 s timing lag** between the magnetometers and the gyros (a rate term fits almost as well: 5.1° per rad/s, i.e. 0.09 s).
- A slow drift term (−0.33 °/s, the same on all four) fits about as well, because the two directions are one block each and so match a ramp in time. Identical drift on four separate gyros is implausible, so it is most likely the same effect.
- To separate the two causes: a few turns each way with the Husky **rotated by hand, motors off**. If the ±4.5° stays, it is timing. If it goes, it is motor current. Turning at two different speeds would also help: a timing lag scales with the rate, current much less.
- The `vehicle_offset` itself is probably fine: equal turning in both directions averages a direction-dependent shift out.
- **Decision (2026-11-23):** not pursued further. The goal is heading while **stationary with the motors off**. There, neither cause applies: no turning, so no lag, and no current. The vehicle offsets above are used as measured. The dynamic test's corner stops measure the stationary heading directly.
