# Results

This file is the source of truth for measured results in the module evaluation. Each entry lists:
- the bag it came from
- the script and code version that produced it
- what was checked

Bags are on the Pi in `~/data/` and copied to `postprocessing/data/`. Times are local (EST) unless marked Z (UTC).

**Summary (2026-11-23, first dynamic run; one short, cut-off run, so treat as preliminary):**
- **Stationary heading (main goal)** needs full calibration (rig + vehicle hard iron). With it, the held heading:
  - is final the moment the vehicle stops (no settling time);
  - repeats to about 1°;
  - is within the 3° target for the **MPU9250 (2.3° RMS)** and **ISM330+MMC5983MA (2.9°)**, borderline for the ICM-20948 (3.0°), and fails for the LIS3MDL (4.6°). These are offset-removed figures; the absolute figures are 2.8–4.7°.
  - A shared ±3° pattern across all IMUs is probably the course truth.
- **Position:** the EKF with IMU heading alone doesn't improve on raw GPS. Tuned noise + GPS velocity does improve the M10s. The **NEO-F10N** is the steadiest (6–9 cm wander at a stop). Stop wander and offsets are within 1 m for all modules, while cross-track p95 is within 1 m only for the NEO-F10N and the Micro M10 (tuned + velocity).
- **Config bug found and fixed:** the sign of `navsat.yaml` `magnetic_declination_radians` (it had rotated GPS against heading by 14.5°).

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
- **Noise figures** from `postprocessing/static_process.py` (2026-11-23), output `postprocessing/static/static_20261123T025246Z/static.yaml`. The compass uses `calib/rig.yaml`. TTFF wasn't measured: the modules already had a fix when recording started.

  | GPS | k (observed σ ÷ claimed σ) | Walk over 4 min | False speed (mean) | k velocity |
  |---|---|---|---|---|
  | SAM-M8Q | 1.12 | 1.74 m | 0.018 m/s | 0.09 |
  | SAM-M10Q | 2.6 | 1.30 m | 0.024 m/s | 0.24 |
  | NEO-F10N | 0.67 | 0.96 m | 0.011 m/s | 0.08 |
  | M10-25Q (NMEA) | 14.4 | 1.23 m | – (velocity is NaN throughout) | – |
  | Micro M10 | 5.35 | 1.38 m | 0.013 m/s | 0.19 |

  | IMU | Gyro Allan dev. at 1 s (°/s) | Gyro bias instability (°/s) | Compass sd at rest | Compass change, first to last 10 min |
  |---|---|---|---|---|
  | ICM-20948 (filter off) | 0.13–0.19 | 0.005–0.008 | 2.25° | −0.25° |
  | ISM330+LIS3 | 0.004–0.006 | 0.0005–0.0014 | 1.15° | −0.08° |
  | ISM330+MMC | 0.005–0.006 | 0.0012–0.0016 | **0.07°** | 0.01° |
  | MPU9250 | 0.006–0.008 | 0.002–0.005 | 1.97° | 0.65° |

  - The u-blox velocity is far *better* than claimed (k_v 0.08–0.24).
  - The M10s' position is 2.6–14× *worse* than claimed. The M8 and NEO-F10N are about honest.

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

---

## 4. Dynamic test, rig on the Husky (2026-11-23)

**Bag:** `dynamic_20261123T072804Z`
- Recorded 02:28–02:52, 1451 s (24 min), 428 MB. `_setup/` holds the config and git state (commit 69caa7d).
- Course: 36 × 54 ft (11.0 × 16.5 m), direction alternating every loop, stops before turning (`mark dynamic`).
- **Cut short by a low Husky battery:**
  - Start `still` (127 s), then loops 1–3 complete (CW, CCW, CW) and loop 4 (CCW) to its last corner.
  - The `stop ccw L4 C4` hold has no `end` mark, and there is no final `still`.
  - 16 corner stops in all. The long holds (60 s nominal) are L1 C1, L2 C1, L3 C2 and L4 C2.
- The Pi then lost power twice (boots ending 02:53:28 and 02:53:37).

**Health**
- All 4 IMUs and 4 of 5 GPS recorded the whole bag with no gaps over 0.5 s. All GPS had a 3D fix 100 % of the time (SAM-M10Q SBAS 77 %).
- No undervoltage during the test. One warning at that boot's start (01:40, switching onto the Husky battery).
- **The SAM-M8Q is missing:** its driver aborted 2 s after launch (exit −6, no log output; most likely the MON-VER poll timed out, the known M8 start problem). It had started fine in the previous launch at 02:07. So the dynamic results have 4 GPS × 4 IMU. A `respawn` on the GPS drivers would have prevented this.

**Pi snapshot:** `postprocessing/data/pi_snapshot_20261123/` contains:
- `config.txt` / `cmdline.txt` and today's `~/.ros/log` (in `pi_files.tgz`)
- package versions, git state, the ICM driver diff and power readings (in `pi_state.txt`)
- the journals of the boots covering the calibration, motor and dynamic runs

### 4a. Stationary heading (dynamic bag, final)

**Processing:**
- `postprocessing/dynamic_process.sh` (2026-11-23), report `postprocessing/dynamic/report/dynamic_20261123T072804Z/` (`summary.yaml`, `report.html`).
- Calibrated raw IMU → `imu_filter_madgwick` (Humble 2.1.6, magnetometer on, ENU) → heading at each of the **17 holds**: the start `still` plus 16 corner stops, at 4 headings (5 / 4 / 4 / 4 holds).
- **Truth** = the direction of the side just driven, from a 36 × 54 ft rectangle fitted to all 4 GPS tracks. The free fit gives 17.0 × 11.6 m against a nominal 16.5 × 11.0 m. The GPS direction of travel before each stop was within ±2.5° of the fitted side.
- **Declination:** 7.27° W (NOAA).
- The EKF heading equals its IMU's Madgwick heading in every combination; ekf_global takes yaw from the IMU only. So these numbers hold for every GPS × IMU pairing.

**Calibration matters most** (settled error RMS over the holds, offset removed, Madgwick gain 0.1):

| IMU | No calibration | Rig calibration only | Rig + vehicle offset |
|---|---|---|---|
| ICM-20948 | 62.9° | 4.4° | **3.0°** |
| ISM330+LIS3 | 90.3° | 5.3° | **4.6°** |
| ISM330+MMC | 33.1° | 6.5° | **2.9°** |
| MPU9250 | 7.6° | 14.3° | **2.3°** |

Without the vehicle offset, the MPU (8.9 µT offset) shows a ±22° one-cycle error with heading.

**With full calibration:**

| IMU | Constant offset | Absolute RMS (holds within 3°) | Offset-removed RMS / max (holds within 3°) | Own error (vs the other 3 IMUs) RMS / max | Repeat sd, same heading | Change during a hold |
|---|---|---|---|---|---|---|
| ICM-20948 | −2.0° | 3.9° (9/17) | 3.0° / 5.4° (12/17) | 1.9° / 3.4° | 1.1° | ≤ 0.4° |
| ISM330+LIS3 | −1.8° | 4.7° (6/17) | 4.6° / 9.3° (4/17) | **3.9° / 8.5°** | 1.5° | ≤ 1.1° |
| ISM330+MMC | −1.5° | 3.4° (9/17) | 2.9° / 5.0° (10/17) | **1.4° / 2.5°** | 1.1° | ≤ 0.1° |
| MPU9250 | −1.4° | 2.8° (11/17) | **2.3° / 4.8° (13/17)** | **1.2° / 2.5°** | 1.1° | ≤ 0.4° |

**Findings**
- **No settling time.** The heading is already at its final value when the vehicle stops, and changes by at most 0.1–1.1° over a 30–60 s hold. The Madgwick gain (0.1 vs 0.3) makes no difference to the held heading.
- **Repeatable to about 1°.** Stops at the same heading agree within about 1.1° (1.5° for the LIS3).
- **The error is a fixed pattern by heading.**
  - Part of it is **shared by all four IMUs**: −4.6 / −3.2 / +1.9 / −0.3° at true ENU headings −124 / −34 / +56 / +146°. That points to the truth (course not exactly rectangular, stops not square) or to a vehicle effect, not to the sensors.
  - With the shared part taken out, each IMU's **own** error is about 1.2–1.9° RMS. The LIS3 is the exception at 3.9° RMS, worst at one heading.
  - Separating truth error from shared vehicle effects needs an independent heading reference. The heading turntable test is still open.
- **Against the 3° target:**
  - Offset-removed RMS: MPU9250 2.3° and MMC 2.9° pass; ICM 3.0° is borderline; LIS3 4.6° fails.
  - On each IMU's own error, all but the LIS3 are within 2°.
  - The constant offset (−1.4 to −2.0°) is the same sign on all four, so it is probably the course truth or the declination, not the IMUs.
- **Absolute errors (no alignment):** 2.8–4.7° RMS.

### 4b. Position (dynamic bag, final)

**Processing:**
- Same script and report as 4a. 4 GPS × 4 IMU robot_localization chains (`ekf_global` from `ekf.yaml` + `navsat_transform`), rig + vehicle calibration, Madgwick gain 0.1.
- Valid run: the third, 07:02–07:51 on 2026-11-23. Two earlier runs are kept under `postprocessing/dynamic/invalid_*`, each with a README:
  1. navsat output (0, 0): the chains had no world → base_link TF.
  2. Wrong declination sign in `navsat.yaml`, now fixed (see CLAUDE.md).
- Check that the run is consistent: on the straights the map-frame direction of travel matches the EKF yaw (−0.4° / +3.4°).
- **Truth:** the rectangle fitted to the GPS tracks themselves. So these numbers measure each track's shape and noise around a common fit, **not** absolute position. A shared offset or walk of all modules isn't seen.

**Definitions:**
- **Cross-track:** distance to the fitted outline on the straights, corners excluded.
- **Hold spread:** RMS wander of the position estimate during a stop.
- **Hold offset:** distance of a stop's mean position from the outline.

Values are the mean over the 4 IMUs. The IMU choice changes these by ≤ 0.05 m, apart from one p95 at 0.14 m.

| GPS | Raw fixes: cross-track RMS / p95, hold spread, hold offset | EKF, noise as shipped | EKF, tuned noise + GPS velocity |
|---|---|---|---|
| NEO-F10N | 0.47 / 0.85, 0.08, 0.41 m | 0.48 / 0.90, 0.09, 0.47 m | 0.49 / 0.88, **0.06**, 0.47 m |
| Micro M10 | 0.55 / 0.95, 0.21, 0.49 m | 0.54 / 1.02, 0.21, 0.52 m | **0.40 / 0.76, 0.13**, 0.48 m |
| SAM-M10Q | 0.82 / 1.60, 0.50, 0.78 m | 0.82 / 1.55, 0.50, 0.81 m | 0.75 / **1.13, 0.30**, 0.65 m |
| M10-25Q (NMEA, 1 Hz, no velocity) | 0.76 / 1.43, 0.31, 0.67 m | 0.78 / 1.48, 0.32, 0.74 m | 0.84 / 1.43, 0.37, 0.74 m |

Shipped noise with GPS velocity, and tuned noise without it, both land within a few cm of the shipped column.

**Findings**
- **Without GPS velocity, the EKF output equals the raw GPS** (within a few cm), with shipped or tuned noise alike. GPS position + IMU heading alone add nothing.
- **Tuned noise + GPS (Doppler) velocity is the only combination that helps:**
  - Micro M10: cross-track 0.55 → 0.40 m RMS, hold spread 0.21 → 0.13 m.
  - SAM-M10Q: p95 1.60 → 1.13 m, hold spread 0.50 → 0.30 m.
  - NEO-F10N: already smooth, unchanged.
  - M10-25Q: unchanged (no velocity).
  - Shipped noise + velocity doesn't help: the M10s claim position σ 2.6–14× too small, so the filter keeps trusting position over velocity.
- **The NEO-F10N (active antenna) is the steadiest GPS:** 6–9 cm wander at a stop, 0.47 m cross-track RMS.
- **Against the ~1 m station-keeping target:**
  - Stop offsets (0.47–0.81 m) and wander (0.06–0.5 m) are all within 1 m on this short run.
  - Cross-track p95 is within 1 m for the NEO-F10N and for the Micro M10 (tuned + velocity), and above it for the SAM-M10Q (1.13 m) and the M10-25Q (1.43 m).
  - Absolute accuracy and slow walk (static test: 1–3 m over 45 min) aren't in these numbers.
- **robot_localization run-to-run variation:** with the 1 Hz M10-25Q, two runs that should be identical (velocity on vs off, no velocity data) separated by up to 2.4 m at moments. Single-run numbers for that module are less certain.
