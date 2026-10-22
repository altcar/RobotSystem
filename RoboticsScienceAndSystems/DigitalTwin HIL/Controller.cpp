#include "Controller.h"
#include <stdio.h>
#include <string.h>

const unsigned long Controller_c::UPDATE_INTERVAL_MS;
const unsigned long Controller_c::TELEMETRY_INTERVAL_MS;
const unsigned long Controller_c::RECOVERY_TIMEOUT_MS;
const uint16_t Controller_c::LINE_THRESHOLD;
constexpr float Controller_c::BASE_PWM;
constexpr float Controller_c::LINE_FOLLOW_GAIN;
constexpr float Controller_c::RECOVERY_LEFT_PWM;
constexpr float Controller_c::RECOVERY_RIGHT_PWM;

Controller_c::Controller_c()
  : update_timer(UPDATE_INTERVAL_MS),
    telemetry_timer(TELEMETRY_INTERVAL_MS) {
  hil_enabled = false;
  hil_left_pwm = 0.0f;
  hil_right_pwm = 0.0f;
  hil_last_command_ms = 0;
  hil_line[0] = '\0';
  hil_line_length = 0;
  reset();
}

void Controller_c::reset() {
  signal = 0;
  mode = WAITING;
  recovery_start_ms = 0;

}

uint8_t Controller_c::getSignal() const {
  return signal;
}

void Controller_c::setSignal(uint8_t requested_signal) {
  if (requested_signal == 0) {
    reset();
    return;
  }

  if (signal == 0) {
    signal = 1;
    mode = FOLLOWING_LINE;
    recovery_start_ms = 0;
  }
}

void Controller_c::update(Robot_c &robot, RobotWifiAP_c &server) {

  // Get the current count of milliseconds since the StampC3 was powered on.
  unsigned long now = robot.getMillis();

  // Check if our TaskTimer_c for the general controller update indicates
  // that it is the correct time to operate.  Note, we give the TaskTimer_c
  // the current count in milliseconds.
  // We do this because we don't want to query the robot faster than at 10ms
  // intervals.
  if (!update_timer.isReady(now)) {

    // TaskTimer_c says it is not ready, so we simply end this call to
    // the controller here by calling return.
    return;
  }

  // The TaskTimer_c was ready, so we now reset the TaskTimer_c so it
  // begins counting again.  We now run the remaining controller code.
  update_timer.resetTimer(now);

  // Hardware-in-the-loop takes priority: the computer supplies the PWM and
  // the firmware only maintains the robot snapshot and telemetry.
  if (hil_enabled) {
    runHil(robot, server, now);
    return;
  }

  // Ask the robot for latest information on surface reflectance sensors.
  robot.getSurfaceSensors();

  // Ask the robot for the latest encoder information, and then use this
  // to update the odometry on board the StampC3 (see OdometryModel.h).
  if (robot.getEncoders()) {
    odometry.update(robot.getLeftEncoderCount(), robot.getRightEncoderCount());
  }

  // Pose estimation from the Pololu 3Pi robot itself.
  robot.getPose();

  // While waiting, a button press sets signal to 1 and starts the physical
  // robot's line follower, even without Processing connected. The transmitted
  // signal also starts an armed Digital Twin live comparison. This is a start
  // control, not a start/stop toggle; further presses are ignored while signal is 1.
  if (signal == 0) {
    if (robot.isButtonPressed()) {
      setSignal(1);
    }
  }

  // The user has started the demonstration, so we run the line following
  // controller code.
  // Exercise 1: line following is disabled so only the sensor test runs.
  // if (signal == 1) {
  //   runLineFollower(robot, now);
  //
  // }

  // Exercise 1: report the five surface sensors once per controller cycle.
  sensorTest(robot, now);

  // Use another TaskTimer_c to limit how often we transmit telemetry data
  // on WiFi and Serial.
  // Exercise 1: telemetry is disabled so it does not corrupt the sensor plot.
  // if (telemetry_timer.isReady(now)) {
  //   telemetry_timer.resetTimer(now);
  //   publishTelemetry(robot, server, now);
  //
  // }
}

void Controller_c::pollHil(RobotWifiAP_c &server) {
  while (server.available() > 0) {
    int c = server.read();
    if (c < 0) {
      break;
    }
    if (c == '\n' || c == '\r') {
      if (hil_line_length > 0) {
        hil_line[hil_line_length] = '\0';
        parseHilLine(hil_line);
        hil_line_length = 0;
      }
    } else if (hil_line_length < sizeof(hil_line) - 1) {
      hil_line[hil_line_length++] = (char)c;
    } else {
      hil_line_length = 0; // discard an overlong line
    }
  }
}

void Controller_c::parseHilLine(char* line) {
  if (strcmp(line, "HIL,1") == 0) {
    hil_enabled = true;
    hil_left_pwm = 0.0f;
    hil_right_pwm = 0.0f;
    hil_last_command_ms = millis();
    signal = 2; // telemetry marker while the computer owns the motors
  } else if (strcmp(line, "HIL,0") == 0) {
    hil_enabled = false;
    hil_left_pwm = 0.0f;
    hil_right_pwm = 0.0f;
    signal = 0;
  } else if (strncmp(line, "PWM,", 4) == 0) {
    float left = 0.0f;
    float right = 0.0f;
    if (hil_enabled && sscanf(line, "PWM,%f,%f", &left, &right) == 2) {
      hil_left_pwm = constrain(left, -200.0f, 200.0f);
      hil_right_pwm = constrain(right, -200.0f, 200.0f);
      hil_last_command_ms = millis();
    }
  } else if (strcmp(line, "STOP") == 0) {
    hil_left_pwm = 0.0f;
    hil_right_pwm = 0.0f;
    hil_last_command_ms = millis();
  }
}

bool Controller_c::isHilEnabled() const {
  return hil_enabled;
}

void Controller_c::runHil(Robot_c &robot, RobotWifiAP_c &server, unsigned long now) {
  robot.getSurfaceSensors();
  if (robot.getEncoders()) {
    odometry.update(robot.getLeftEncoderCount(), robot.getRightEncoderCount());
  }
  robot.getPose();

  // Safety: a lost PC connection must not leave the previous command applied.
  if (now - hil_last_command_ms > 500) {
    hil_left_pwm = 0.0f;
    hil_right_pwm = 0.0f;
  }
  robot.setMotorPWM(hil_left_pwm, hil_right_pwm);

  if (telemetry_timer.isReady(now)) {
    telemetry_timer.resetTimer(now);
    publishTelemetry(robot, server, now);
  }
}

void Controller_c::sensorTest(Robot_c &robot, unsigned long now) {
  (void)now;  // Not needed: the test reports every controller cycle.

  // The robot must stay stationary while the surface is measured.
  robot.setMotorPWM(0, 0);

  // One CSV line per cycle: five raw discharge times in microseconds.
  // Reading[] runs DN1..DN5; the plotter needs numbers, commas and a newline.
  Serial.printf("%u,%u,%u,%u,%u\n",
                robot.surface.reading[0],
                robot.surface.reading[1],
                robot.surface.reading[2],
                robot.surface.reading[3],
                robot.surface.reading[4]);
}

bool Controller_c::lineDetected(const Robot_c &robot) const {
  // Note: only using the central sensor to detect a line.
  if (robot.surface.reading[2] >= LINE_THRESHOLD) {
    return true;
  }

  return false;
}

void Controller_c::runLineFollower(Robot_c &robot, unsigned long now) {
  bool line_detected = lineDetected(robot);

  if (!line_detected && mode == FOLLOWING_LINE) {
    mode = SEARCHING_FOR_LINE;
    recovery_start_ms = now;
  }

  if (mode == SEARCHING_FOR_LINE) {
    if (line_detected) {
      mode = FOLLOWING_LINE;
    } else if (now - recovery_start_ms >= RECOVERY_TIMEOUT_MS) {
      mode = STOPPED;
    }
  }

  if (mode == STOPPED) {
    robot.setMotorPWM(0, 0);
    return;
  }

  if (mode == SEARCHING_FOR_LINE) {
    robot.setMotorPWM(RECOVERY_LEFT_PWM, RECOVERY_RIGHT_PWM);
    return;
  }

  float error = (float)robot.surface.reading[1] - (float)robot.surface.reading[3];
  float turn = error * LINE_FOLLOW_GAIN;
  robot.setMotorPWM(BASE_PWM - turn, BASE_PWM + turn);
}

void Controller_c::publishTelemetry(Robot_c &robot, RobotWifiAP_c &server, unsigned long timestamp_ms) {
  server.printf(
    "%lu,%.2f,%.2f,%.5f,%d,%d,%ld,%ld,%u,%u,%u,%u,%u,%u\n",
    timestamp_ms,
    odometry.pose.x,
    odometry.pose.y,
    odometry.pose.theta,
    robot.getLeftMotorPWM(),
    robot.getRightMotorPWM(),
    (long)robot.getLeftEncoderCount(),
    (long)robot.getRightEncoderCount(),
    robot.surface.reading[0],
    robot.surface.reading[1],
    robot.surface.reading[2],
    robot.surface.reading[3],
    robot.surface.reading[4],
    signal
  );
  if ( Serial ) { // If serial is connected
    Serial.printf(
      "%lu,%.2f,%.2f,%.5f,%d,%d,%ld,%ld,%u,%u,%u,%u,%u,%u\n",
      timestamp_ms,
      odometry.pose.x,
      odometry.pose.y,
      odometry.pose.theta,
      robot.getLeftMotorPWM(),
      robot.getRightMotorPWM(),
      (long)robot.getLeftEncoderCount(),
      (long)robot.getRightEncoderCount(),
      robot.surface.reading[0],
      robot.surface.reading[1],
      robot.surface.reading[2],
      robot.surface.reading[3],
      robot.surface.reading[4],
      signal
    );
  }
}
