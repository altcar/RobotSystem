# StampC3_Template – PlatformIO project

Builds the supplied `RoboticsScienceAndSystems/StampC3_Template` sketch
without copying it. `src_dir` in `platformio.ini` points at the original
folder, so editing the template there and running a build picks the changes
up directly.

## Commands

```sh
pio run                          # compile
pio run -t upload                # compile + flash to the connected StampC3
pio device monitor               # serial monitor at 115200
pio run -t upload --upload-port COM3   # pin the port if auto-detect picks wrong
```

## Verified

- Build: SUCCESS (`src/StampC3_Template.ino.cpp`, `Robot.cpp`,
  `Controller.cpp`, `RobotIMU.cpp`, `RobotWifiAP.cpp`, `OdometryModel.cpp`,
  `TaskTimer.cpp` all compile).
- Upload: SUCCESS to `COM3` (USB VID:PID `1A86:55D4`), hash verified.
- Run: telemetry CSV streams at 10 Hz, e.g.

  ```
  40024,0.00,0.00,0.00000,0,0,-1,-10,775,625,649,649,875,0
  ```

  Columns are timestamp, x, y, theta, left PWM, right PWM, left/right encoder
  counts, five surface readings, then `signal`. `signal = 0` means the robot
  is waiting; press its button to start line following. No IMU warning was
  printed, so the LSM6 was detected.

## Notes

- `monitor_dtr = 0` / `monitor_rts = 0` are required on this USB-serial
  adapter (CH343). With the defaults, opening the monitor resets the C3 and
  it stays silent.
- Upload speed is 115200 per the checklist; if flashing is unreliable, put the
  C3 into download mode (hold BOOT, tap RESET, release BOOT).
