# Image Rastering Digital Twin

Processing 4 sketch for SEMT0043 Phase 2. The template's line-following
controller has been replaced by a **whole-map sweep**: a perimeter survey
around the map area, followed by a serpentine raster of its interior, with the
surface reconstructed from the five reflectance sensors. Pose estimation fuses
encoder travel, IMU heading/gyro rate and a PWM motion prior. The original
sandbox/live modes, TCP and serial telemetry, telemetry graphs, tracking,
zoom/pan, surface-map loading and robot model are all retained.

All changes are confined to this `DigitalTwin` folder. The Arduino template in
`StampC3_Template` is untouched.

---

## 1. Run it

1. Open `DigitalTwin.pde` in Processing 4 (`Processing.exe`, version 4.5.7 was
   used here) and press Run.
2. The sandbox starts automatically and the status card shows
   `Mode: Sandbox`. No robot or network is needed.
3. The scan begins from the fixed home pose `(0, 0, -90 degrees)`, transits to
   the map's top-left corner, surveys the perimeter, then rasters the interior.
   At the default 8 mm spacing and PWM 120 a whole-map run takes roughly
   2.5 minutes. Press `]` to increase the spacing (fewer, faster passes) or `.`
   to increase speed; both are useful for quick trials. `SPACE` restarts.
4. `surface.png` is the 420 x 297 mm, 1 pixel/mm map supplied with the
   template. Processing also accepts a map in a `data/` folder; a replacement
   must keep the 1 px/mm convention (A3 = 420 x 297 px).

The sketch compiles headlessly with:

```text
Processing.exe cli --sketch="<...>\DigitalTwin" --output="<temp folder>" --force --build
```

---

## 2. Controls

| Key | Action |
| --- | --- |
| `G` | Telemetry graphs (motor PWM, surface sensors, encoders) |
| `T` | Centre the view on the robot (tracking) |
| `M` | Cycle sandbox -> live comparison -> HIL (HIL disabled during a live run) |
| `+` / `-` | Zoom in / out |
| Arrow keys | Pan (turns tracking off) |
| `R` | Show/hide the reconstruction panel |
| `V` | Show/hide the reconstruction as a world-coordinate overlay |
| `SPACE` | Restart the sandbox scan / start a HIL run |
| `[` `]` | Decrease/increase path spacing by 4 mm (restarts the scan) |
| `,` `.` | Decrease/increase cruise PWM (applies immediately) |
| `;` `'` | Decrease/increase sensor footprint radius, 0-10 mm (restarts) |
| `I` | Cycle placement: nearest -> bilinear -> gaussian (restarts) |
| `F` | Toggle display-only gap fill for unmeasured cells |
| `N` | Cycle sensor noise: 0 -> 30 -> 80 DN (restarts) |
| `P` | Cycle pose error: off -> 5% -> 10% -> 15% (restarts) |
| `U` | Toggle IMU/encoder/PWM sensor fusion (restarts) |
| `J` | Cycle fusion method: complementary -> EKF -> UKF (restarts) |
| `K` | Cycle EKF Jacobian: analytic -> numerical -> identity (restarts) |
| `C` | Cycle control law: open -> P -> PID -> LQR -> MPC (restarts) |
| `B` | Cycle reconstruction overlay opacity |
| `Y` | Cycle fading memory: off -> slow -> medium -> fast |
| `Z` | Cycle map content scale (100/75/50/35/25/12.5%) |
| `0` | Toggle `surface.png` / generated tone test card (restarts) |
| `8` | Save the active reconstruction as `reconstruction.png` |
| `9` | Save the generated card as `tone_test_card.png` |
| `H` | Help overlay |

---

## 3. How the whole-map sweep works

`Controller.pde`

* The Simulator sets the scan target from the active surface map, inset by
  **40 mm** on every side (`MAP_EDGE_INSET_MM` in `Simulator.pde`). The target
  is therefore the whole picture, not a small fixed rectangle. The map frame
  is assumed to be +x right, +y up, with lanes running along -y; place the
  physical robot accordingly.
* **Phase 1, TRANSIT**: drive from the current pose to the top-left target
  corner. No reconstruction samples are taken.
* **Phase 2, EDGE**: drive once around the target boundary. This surveys the
  perimeter and anchors the run to the map frame before the interior passes.
  Samples are recorded.
* **Phase 3, RASTER**: a boustrophedon sweep of the interior. Lanes alternate
  direction; at the end of a lane the robot pivots in place, drives the short
  lateral hop, pivots again, and resumes.
* The number of lanes is chosen so the realised spacing is close to the
  requested spacing: `lanes = round(targetWidth / spacing) + 1`. The realised
  spacing is `targetWidth / (lanes - 1)` and is reported in the panel.
* Motion is closed-loop on the **fused pose** (section 5): a proportional
  heading controller steers to each waypoint, slowing within 30 mm. PWM is
  `left = speed - turn`, `right = speed + turn`; positive heading error means
  the target is to the left, so the right wheel runs faster.

The five sensors (DN1..DN5) are mounted forward of the axle at lateral offsets
of about `-31.8, -10.9, 0, +10.9, +31.8 mm`. A single pass therefore paints
five narrow traces, not a solid swath. Path spacing must be small enough, or
the footprint/interpolation wide enough, to join them. That tension is the
main subject of the experiments.

The same decisions can be ported to `Controller.cpp`: the controller uses only
`getSurfaceSensors()`, `getEncoders()`, `getPose()`, the IMU yaw/gyro and
`setMotorPWM()`, runs on the existing 10 ms task timer, and keeps the 50 ms
telemetry task.

---

## 4. Reconstruction

`SurfaceReconstructor.pde`

* Readings use the robot's telemetry units: **400 = white, 2000 = black**. The
  reconstructor maps them back to a 0..255 grey level and accumulates into a
  world-aligned 1 mm grid covering the whole map.
* Each sensor reading is splatted at the sensor's world position, computed
  from the **fused pose** (section 5), i.e. the pose the robot reports. This is
  the same data a PC receives over WiFi.
* Placement modes:
  * `nearest` - one cell; blocky, visible aliasing;
  * `bilinear` - four cells; sub-millimetre positioning (default);
  * `gaussian` - a soft spot (sigma 1.5 mm); fills sparse paths at the cost of
    blur.
* **Gap fill** (`F`, on by default) is a display-only interpolation. Measured
  cells are left exactly as measured, so thin printed marks keep their
  contrast; only cells with no measurement are filled by six passes of an
  8-neighbour average. Interpolated cells are drawn slightly translucent in
  the overlay and are *not* counted as measured in the coverage statistic.
  This separates "what the sensors measured" from "what was inferred".
* Overlapping passes are averaged with a weight sum, so coverage builds up
  rather than being overwritten. Gaps stay transparent in the overlay when
  gap fill is off.
* Metrics are computed against the ideal map, over the target rectangle
  expanded by the sensor array's lateral half-span and forward reach (the
  region the sweep can observe):
  * **Coverage** - cells with accumulated weight >= 0.2, as a fraction of that
    region;
  * **RMSE / MAE** - tone error in 0..255 grey levels, over covered cells.
  Coverage and error will not reach 100% / 0 because the sensor integrates a
  finite spot, the printed line is thinner than the spot, and the outermost
  margin of the map is beyond the sensor array's reach.

The panel shows the **truth** and **reconstruction** crops of the observed
region side by side, plus the current parameters and results. `R` hides it;
`V` draws the recon over the map in world coordinates.

---

## 5. Non-ideal behaviour (for the comparison)

`RobotSimulation.pde` and `SurfaceSensorModel.pde`

* **Footprint** (`;` `'`): the sensor averages nine points over a disc of the
  given radius (default 1.0 mm) instead of sampling one pixel. This models the
  finite spot of a real reflectance sensor and reduces contrast on fine
  features.
* **Noise** (`N`): Gaussian noise added to each reading in sensor units
  (default 0).
* **Pose error** (`P`): the true chassis slips and uses a slightly wrong wheel
  separation while the *reported* odometry stays ideal. Surface sampling and
  drawing use the true pose; the controller and reconstruction use the fused
  pose. The result is ghosting, offset and reduced coverage - the effect a
  purely ideal twin cannot show. The blue trail is the fused path, the orange
  trail is the true path.

### Pose estimation (`PoseFilter.pde`, `U`, `J`, `K`)

`PoseFilter_c` uses the same three inputs for every estimator:

* **Encoders** give wheel travel, converted to millimetres per count;
* **IMU** supplies the yaw rate for prediction and the absolute yaw as the
  measurement;
* **PWM** supplies a low-weight (15%) motion-model prior for wheel travel,
  smoothing encoder quantisation and modelling the commanded actuators.

Three estimators are selectable with `J` (default EKF):

* **Complementary filter** - gyro-rate prediction blended with the absolute
  yaw at gain 0.20 per 10 ms. Cheap and robust.
* **EKF** - state `[x, y, theta]`, midpoint differential-drive prediction, IMU
  yaw measurement. `K` selects the state-transition Jacobian:
  * `analytic` - closed-form derivative;
  * `numerical` - finite-difference linearisation;
  * `identity` - no linearisation, deliberately showing the covariance error.
* **UKF** - seven sigma points from the Cholesky factor of the covariance,
  propagated through the same motion model, with the same yaw measurement.
  Alpha = 1.0, beta = 2.0, kappa = 0 (non-negative weights for this state
  size).

All three use the same process noise, measurement noise and inputs, so a
comparison isolates the estimator. `U` disables fusion entirely and uses raw
wheel odometry. `J`/`K` restart the run, so each result is from one estimator.

With `P` enabled, raw odometry heading drifts because the effective wheel
separation is wrong. The IMU-corrected estimators keep the lane geometry and
reconstruction aligned, while translation slip remains (it is not observable
without an external position reference). The blue trail is the estimator
output; the orange trail is the true chassis pose.

At zero pose-error settings all estimators track the ideal pose, which is a
useful sanity check that the twin, not the filter, is driving the result.

### Control laws (`C`)

`C` cycles five controllers; the default is `P` (closed-loop waypoint
steering). All use the same whole-map waypoint plan and the same fused pose:

* **open** - timed pivot and straight segments computed from the *planned*
  pose. It deliberately ignores measured pose, so it is the open-loop
  baseline.
* **P** (closed loop) - proportional heading-to-target steering with an
  in-place pivot for large errors.
* **PID** - lane keeping on the current segment: proportional/integral on the
  cross-track error plus derivative damping from the heading error.
* **LQR** - the 2x2 discrete Riccati solution for the lane-keeping model
  `[lateral, heading]`, with the gain solved at scan start.
* **MPC** - a receding-horizon search over candidate turn PWM values on the
  same lane model (30-step horizon, 0.3 s).

The PID/LQR/MPC controllers track the infinite segment line, so they also
accept a waypoint once the robot has passed its endpoint by projection. This
avoids the line-follower driving past a corner because it never came within
the point tolerance.

---

## 6. Suggested experiments

Before the first comparison run, follow
[CHARACTERISATION.md](CHARACTERISATION.md): print the test card, measure the
sensor tone curve, footprint, smallest resolvable bar period, motor speed vs
PWM, and odometry error on the real robot. Those numbers choose the twin
parameters and give the report its physical evidence.

Change one factor per run, press `SPACE` to restart, and let the run finish.
Completed runs are appended to `raster_runs.csv`; use `8` and `9` to save
images for the report.

1. **Path spacing** - try 16, 8, 4 mm with bilinear placement and gap fill.
   Hypothesis: denser paths raise *measured* coverage and reduce RMSE but
   increase time roughly linearly. Sparse paths leave the five sensor traces
   visible even after gap filling.
2. **Speed** - try cruise PWM 30, 70, 120.
   Hypothesis: the along-track pitch (`pitch` in the panel, mm per 10 ms
   sample) grows with speed; turning overshoot and lane-end artefacts grow
   too. At high speed the sample pitch can exceed the lane spacing.
3. **Sensor footprint** - try 0, 1, 2, 4 mm.
   Hypothesis: a larger spot blurs the thin printed line and lowers its
   contrast; the ideal image would not predict this.
4. **Placement / gap fill** - restart with nearest, bilinear and gaussian, and
   compare with `F` on and off. Nearest exposes aliasing and gaps; gaussian
   fills by blurring measured cells; bilinear + fill keeps measured cells sharp
   and only invents the empty cells.
5. **Pose error** - try 5%, 10%, 15%.
   Hypothesis: true and fused trails separate; edges become doubled or
   smeared; coverage falls. This is the clearest demonstration of a twin
   assumption that the physical robot will violate.
6. **Sensor fusion** - with pose error at 10%, run once with fusion on and once
   with `U` off. Hypothesis: raw odometry heading drifts and the image
   shears/rotates, while fused heading stays aligned; translation slip still
   shifts the image.
7. **Noise** - try 0, 30, 80 DN, especially on the flat grey steps of the test
   card.
8. **Map fidelity** - press `0` for the generated tone/resolution card: a
   stepped grey wedge, a tone ramp, 1/2/3/5/8 mm bar patterns, checkerboards
   and a slanted edge. It exposes tone response and the resolution limit that
   the circular line map cannot.
9. **Live comparison** - `M` with the robot on its WiFi network. The live
   panel reconstructs from the robot's reported pose and reflectance readings
   while signal = 1. The supplied normal firmware path (Exercise 1) does not
   raster, so either use HIL (section 8) or drive the robot over the map by
   hand to compare paths.
10. **Fading** - `B` cycles the reconstruction overlay opacity; `Y` gives the
    measurements an exponential fading memory. With `Y` on, early parts of the
    map lose confidence before the raster returns, showing what a temporal
    forgetting assumption does to a static scene.
11. **Minimum detectable feature** - press `0` for the test card, then `Z` to
    shrink its printed content (75/50/35/25/12.5%). The sensor footprint and
    1 mm reconstruction grid stay fixed, so the bar blocks and checkerboards
    find the smallest feature the twin can still resolve. The physical
    counterpart is printing the same card at the same reduced scale.

Suggested record for each run: spacing, lane count, cruise PWM, footprint,
noise, pose error, fusion on/off, estimator, control law, map scale, fade,
placement, coverage, RMSE, MAE, elapsed time, plus the panel screenshot and
the recon PNG. The CSV records all of these automatically.

---

## 7. Physical robot comparison

The HIL firmware is in `RoboticsScienceAndSystems/StampC3_Template` (section
8). The standalone normal path still reflects the current Exercise 1 work, so
for a physical raster *without* a PC controller, port the plan from
`Controller.pde` into that firmware:

1. Port the waypoint plan, phase machine and control law, keeping the 10 ms
   update timer, sensor/encoder/pose acquisition, telemetry and
   `server.update()`.
2. Port the pose filter too, or use the robot interface's existing pose:
   encoder counts from `getEncoders()`, yaw/gyro from `RobotIMU` or the
   interface pose, and the commanded PWM from `setMotorPWM()`. The fused pose
   replaces the raw odometry used by the scan controller.
3. Use the same five-sensor geometry as `Robot_c` (45 mm radius, angles
   45/14/0/-14/45 degrees) when reconstructing on the PC. The Processing sketch
   assumes this geometry for both sandbox and live data.
4. Place the robot at the map origin facing -y (the map's "down"), matching the
   simulated start. The live panel reconstructs from the telemetry pose, and
   the scan target is expressed in the map frame.
5. Print the map (or `tone_test_card.png` saved with `9`) at 100% scale,
   420 x 297 mm. The printed card is the physical counterpart of the simulated
   test card; it can be used for a direct twin-vs-robot comparison.

Assumptions to check against the real robot:

* readings are interpreted with the template's 400..2000 scale and 2000 =
  black polarity;
* tone response and footprint are modelled as linear/averaged, whereas real
  sensors vary with distance, ambient light, paper and ink;
* motors are an ideal linear PWM-to-speed map, and wheel slip only exists when
  `P` is enabled;
* the IMU is modelled as the true heading plus noise with no bias, and its
  gyro rate is ideal; a real IMU has bias, scale error and drift;
* the map is an ideal 1 px/mm image; printing and paper introduce their own
  geometry and tone errors;
* live telemetry arrives at roughly 20 Hz, so the physical along-track pitch is
  coarser than the sandbox's 100 Hz; repeated samples at a standstill are
  averaged into the same cells.

---

## 8. Hardware-in-the-loop (HIL)

HIL lets the Processing sketch be the controller: the PC reads the physical
robot's telemetry, computes PWM, and sends it back over the same WiFi TCP
connection. The robot's firmware applies the command and keeps publishing
telemetry, which the twin displays and reconstructs in real time.

### Firmware

The HIL firmware is in the PIO source folder used by
`StampC3_Template_PIO/platformio.ini`:

```text
RoboticsScienceAndSystems/StampC3_Template/Controller.h
RoboticsScienceAndSystems/StampC3_Template/Controller.cpp
RoboticsScienceAndSystems/StampC3_Template/StampC3_Template.ino
```

Commands are newline-terminated ASCII:

| Command | Meaning |
| --- | --- |
| `HIL,1` | Enable computer control. Motors stop until a `PWM` command arrives. |
| `HIL,0` | Leave HIL; return to the normal sketch behaviour. |
| `PWM,<left>,<right>` | Requested PWM, clamped to -200..200. Ignored until `HIL,1`. |
| `STOP` | Zero both motor requests. |

Safety behaviour:

* if no `PWM` command arrives for **500 ms**, both motors are commanded to zero;
* commands are ignored unless HIL was explicitly enabled with `HIL,1`;
* leaving HIL (`HIL,0`) or disconnecting the PC also stops the robot.

While HIL is active the telemetry `signal` field is **2**, so the record is
distinguishable from a normal trial. The 14-field CSV order is unchanged.

The current Exercise 1 firmware disables line following and telemetry in its
normal path; the HIL branch takes priority and publishes the standard
telemetry every 50 ms. Build and upload with:

```text
cd StampC3_Template_PIO
pio run -t upload
```

The firmware compiles cleanly for the `m5stack-stamp-c3` target. Hardware
testing is left until the robot is connected.

### Processing procedure

1. Power the robot and connect the PC to its access point (`myAP`,
   password `myPassword`).
2. Run the Processing sketch. Press `M` twice: Sandbox -> Live comparison ->
   **HIL**. The status should reach `Mode: HIL — awaiting telemetry`.
3. Place the robot at the map origin, facing -y (the map's "down"), with the
   map coordinates aligned to the printed sheet.
4. Press `SPACE`. The PC starts the whole-map plan from the latest telemetry
   pose and sends `PWM,...` at the telemetry rate (about 20 Hz).
5. The reconstruction, telemetry graphs, sensor markers and plan are all drawn
   from the physical data. The PC controller uses the reported pose directly
   (`fusion: external`), because the robot's telemetry pose already is its
   fused estimate; the EKF/UKF experiments are in sandbox.
6. Press `M` to leave HIL. The sketch sends `PWM,0,0` and `HIL,0`; the firmware
   returns to its normal behaviour. Closing the sketch or losing WiFi also
   stops the robot via the 500 ms firmware timeout.

For stationary sensor logs and PWM sweeps without the sketch, use
`hil_bench.py` (see [CHARACTERISATION.md](CHARACTERISATION.md)).

The same sketch is synced to `RoboticsScienceAndSystems-main/DigitalTwin` so
the main processing folder is also HIL-capable.

---

## 9. Files

| File | Status | Purpose |
| --- | --- | --- |
| `Controller.pde` | rewritten | Whole-map plan, five control laws, pose-fusion integration |
| `PoseFilter.pde` | new | Complementary, EKF (three Jacobians) and UKF estimators |
| `Simulator.pde` | modified | Scan target, HIL/live/sandbox modes, panel, fading, scale, logging |
| `RobotSimulation.pde` | modified | True/fused pose, IMU and pose-error models, external sensor/pose hooks |
| `SurfaceSensorModel.pde` | modified | Footprint averaging, noise, shared world geometry |
| `SurfaceReconstructor.pde` | new | Weighted grid, gap fill, fading memory, metrics, display image |
| `TestCard.pde` | new | Generated tone/resolution card |
| `hil_bench.py` | new | TCP bench tool: stationary sensor logs, PWM sweeps |
| `CHARACTERISATION.md` | new | Real-robot measurement procedure and record tables |
| `README.md` | new | This document |
| `raster_runs.csv` | generated | One row per completed sandbox scan |
| `reconstruction.png` | generated | Auto-saved at the end of a run (or `8` on demand) |
| `tone_test_card.png` | generated (`9`) | Printable 420 x 297 mm test card |
| `DigitalTwin.pde` | modified | Window title only |
| `OdometryModel.pde` | modified | Added millimetres-per-encoder-count accessor |
| `../StampC3_Template/Controller.h/.cpp` | modified | HIL command parser, telemetry, 500 ms safety timeout |
| `../StampC3_Template/StampC3_Template.ino` | modified | Calls `pollHil()` every loop |
| `Graphing.pde`, `MotorsModel.pde`, `TCPTelemetry.pde`, `TaskTimer.pde`, `surface.png`, `sketch.properties` | unchanged | Template functionality |

The same Processing sketch is copied to
`RoboticsScienceAndSystems-main/DigitalTwin`. The Arduino HIL sources are
also mirrored to `RoboticsScienceAndSystems/DigitalTwin HIL` for convenience,
but the PIO project builds from `RoboticsScienceAndSystems/StampC3_Template`.

Note: deleting `raster_runs.csv` starts a fresh experiment log.
