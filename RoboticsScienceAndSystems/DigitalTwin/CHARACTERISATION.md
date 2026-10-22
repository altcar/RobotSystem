# Real-Robot Characterisation

The project brief asks for the physical robot to be characterised **before**
the simulation comparison. The aim is a proof of concept: measure the real
sensor/motor behaviour, put those numbers into the twin, then compare one or
two key observables (for example, resolution and tone fidelity) rather than
trying to model everything.

Tools used here:

* `hil_bench.py` in this folder (connects to `192.168.4.1:80`, enables HIL,
  sends bounded PWM, logs the 14-field telemetry);
* the printed `tone_test_card.png` (420 x 297 mm at 1 px/mm);
* the Processing sketch's `HIL` mode for closed-loop raster runs.

Safety: drive mode moves the robot. Put it on the floor with the wheels free,
clear the area, and keep the WiFi link in range. The firmware stops the motors
if no command arrives for 500 ms, and the script always sends `PWM,0,0` and
`HIL,0` on exit.

---

## 1. Tone response (reflectance vs grey)

1. Print `tone_test_card.png` at 100% (420 x 297 mm, actual size). The top-left
   wedge has eight 30 x 50 mm patches from black to white.
2. Place the robot stationary with DN3 centred on a patch. Run:

   ```text
   python hil_bench.py --mode sensor --seconds 5 --out tone_patch_0.csv
   ```

3. Repeat for all eight patches. Record the mean DN1..DN5 per patch.
4. Compare with the twin's model, which maps grey 0 -> 2000 and grey 255 -> 400
   linearly.

| Patch | Nominal grey | DN1 | DN2 | DN3 | DN4 | DN5 | Twin prediction (DN3) |
| --- | --- | --- | --- | --- | --- | --- | --- |
| 1 | 0 | | | | | | 2000 |
| 2 | 36 | | | | | | 1771 |
| 3 | 73 | | | | | | 1542 |
| 4 | 109 | | | | | | 1316 |
| 5 | 146 | | | | | | 1084 |
| 6 | 182 | | | | | | 858 |
| 7 | 219 | | | | | | 626 |
| 8 | 255 | | | | | | 400 |

If the response is clearly non-linear or the range is different, fit the curve
and adjust the mapping in `SurfaceSensorModel.sample()` (or add a gamma /
lookup table). Record the fit in the report; this is a concrete twin
assumption to test.

## 2. Sensor footprint

1. Put a straight black/white edge on white paper in front of the robot, with
   the edge perpendicular to the direction of travel.
2. Lower the robot so the sensors are at their normal height. Command a slow
   straight pass (for example, PWM 20):

   ```text
   python hil_bench.py --mode drive --pwms 20 --hold 5 --out edge_pass.csv
   ```

3. The script logs each sensor at 20 Hz. Plot DN3 against travelled distance
   and measure the 10-90% transition width. Repeat for DN1..DN5.
4. The transition width is a practical estimate of the spot diameter. Compare
   it with the twin's `footprintRadiusMm` (default 1.0 mm) and the
   reconstruction's gaussian splat.

| Sensor | 10-90% width (mm) | Estimated diameter (mm) | Twin footprint |
| --- | --- | --- | --- |
| DN1 | | | 1.0 mm |
| DN2 | | | 1.0 mm |
| DN3 | | | 1.0 mm |
| DN4 | | | 1.0 mm |
| DN5 | | | 1.0 mm |

## 3. Spatial resolution

1. The test card contains vertical bar blocks at 1, 2, 3, 5 and 8 mm periods,
   plus 2 and 4 mm checkerboards.
2. In simulation, press `0` for the test card, `Z` to shrink the content, and
   run a scan. In reality, print the card at the same reduced scale and run the
   same HIL plan.
3. Compute modulation for each block from the reconstruction:
   `(max - min) / (max + min)` over a small window inside the block. Report the
   smallest period whose modulation is above the noise floor.

| Period (mm) | Sim modulation | Real modulation | Visible? |
| --- | --- | --- | --- |
| 1 | | | |
| 2 | | | |
| 3 | | | |
| 5 | | | |
| 8 | | | |

The expected limit is set by the sensor footprint and the path spacing, not by
the 1 mm image grid.

## 4. Motor PWM and encoder scale

```text
python hil_bench.py --mode drive --pwms 20,40,60,80,100 --hold 2 --out motor.csv
```

The script reports encoder counts/s and nominal mm/s for each PWM. Check for:

* a deadband at low PWM;
* left/right asymmetry (the CSV has both encoder counts);
* the linearity of PWM to speed used by `MotorsModel.pde`.

Compare the real slope and deadband with the twin's
`maximumWheelSpeedMmPerMs = 0.25` and zero deadband. If they differ, the sim
speed and timing assumptions are candidates for the main discrepancy.

| PWM | Left counts/s | Right counts/s | Speed (mm/s) |
| --- | --- | --- | --- |
| 20 | | | |
| 40 | | | |
| 60 | | | |
| 80 | | | |
| 100 | | | |

## 5. Odometry and pose

1. Mark a 500 mm straight line. Run a slow straight pass and stop.
2. Compare the reported telemetry `x,y,theta` and encoder distance with the
   tape measurement.
3. Repeat with a deliberate turn (use the HIL raster perimeter, or a short
   custom command) and compare the final heading with a protractor.
4. Record the scale error and heading drift. In the twin, use `P` (pose error)
   to match the observed slip/separation error, and use the EKF/UKF comparison
   to see whether the real IMU/encoder mix behaves like the model.

One suggested primary comparison for the report: **sensor footprint and spatial
resolution**, because it directly determines what image is recoverable and is
easy to measure on both sides. A secondary comparison: **pose error**, because
it explains smearing and ghosting that the ideal twin hides.

## 6. Feed the measurements into the twin

| Measurement | Twin control |
| --- | --- |
| Footprint diameter | `;` / `'` (sensor footprint), or `Robot.setSensorModel` |
| Sensor noise | `N` (0/30/80 DN), or the `noiseSigma` field |
| Tone curve | edit the mapping in `SurfaceSensorModel.sample()` |
| Motor deadband/asymmetry | `MotorsModel.pde` or the pose-error model |
| Odometry scale/drift | `P` (pose error), or `Robot.setPoseErrorModel` |
| Sampling map / start pose | `SCAN_HOME_*` and the printed map alignment |

After matching the obvious parameters, re-run the same raster plan in sandbox
and HIL. Compare the reconstruction PNGs and the coverage/RMSE rows in
`raster_runs.csv`; the remaining differences are the evidence for the report.
