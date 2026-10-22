/**
 * @brief Whole-map edge survey followed by a serpentine raster scan.
 * @details The plan has three phases:
 *  1. TRANSIT - drive from the robot's current pose to the first corner of the
 *     requested map area. Samples are not recorded during transit.
 *  2. EDGE - drive once around the area boundary. This anchors the scan to the
 *     map frame, covers the perimeter, and gives the estimator a long, smooth
 *     path before the dense interior passes. Samples are recorded.
 *  3. RASTER - a boustrophedon sweep of the interior, with configurable path
 *     spacing. Samples are recorded.
 *
 * Pose estimation fuses three sources:
 *  - encoder wheel travel (distance);
 *  - the IMU yaw rate for short-term heading, corrected by the IMU's absolute
 *    yaw (a complementary filter);
 *  - the commanded PWM as a low-weight motion-model prior for translation.
 * The fused pose is what the controller navigates with and what the
 * reconstruction uses, so the twin models the same information a physical
 * robot would have. Fusion can be toggled for comparison.
 *
 * The scan area is set by the Simulator from the surface-map bounds, so the
 * sweep follows the map rather than a fixed rectangle. All geometry assumes
 * the map frame: +x right, +y up, and lanes running along -y.
 */
class Controller_c {
  /** @brief Minimum interval between control/acquisition updates, in ms. */
  final int UPDATE_INTERVAL_MS = 10;
  /** @brief Interval reserved for telemetry work, in ms. */
  final int TELEMETRY_INTERVAL_MS = 50;

  // ---------------------------------------------------------------------------
  // Sensor geometry and scan planning (millimetres)
  // ---------------------------------------------------------------------------
  /** @brief Half the lateral span of DN1..DN5 (45 mm * sin 45 degrees). */
  final float SENSOR_HALF_SPAN_MM = 31.8;
  /** @brief Maximum forward reach of a surface sensor, in mm. */
  final float SENSOR_FORWARD_MM = 45.0;
  /** @brief Distance from a waypoint that counts as reached. */
  final float WAYPOINT_TOLERANCE_MM = 4.0;

  // ---------------------------------------------------------------------------
  // Motion control
  // ---------------------------------------------------------------------------
  /** @brief Template baseline PWM. The live cruise value is cruisePwm. */
  final float BASE_PWM = 45.0;
  /** @brief Steering gain, in requested PWM per radian of heading error. */
  final float HEADING_GAIN = 65.0;
  /** @brief Above this heading error the robot pivots rather than arcs. */
  final float PIVOT_ERROR_RAD = 0.35;
  /** @brief Pivot PWM applied when a large heading change is needed. */
  final float PIVOT_PWM = 90.0;
  /** @brief Steering limit used while driving to a waypoint. */
  final float MAX_STEER_PWM = 70.0;
  /** @brief Distance over which the robot slows for a waypoint. */
  final float APPROACH_DISTANCE_MM = 30.0;
  /** @brief Minimum forward PWM during the final approach. */
  final float MIN_APPROACH_PWM = 20.0;

  // ---------------------------------------------------------------------------
  // Pose fusion
  // ---------------------------------------------------------------------------
  /** @brief Nominal full-scale wheel speed of the template motor model. */
  final float NOMINAL_MAX_SPEED_MM_PER_MS = 0.25;
  /** @brief Weight given to the PWM motion prior in the fused distance. */
  final float PWM_PRIOR_WEIGHT = 0.15;
  /** @brief Complementary-filter gain for the absolute IMU heading. */
  final float IMU_HEADING_GAIN = 0.20;

  // ---------------------------------------------------------------------------
  // State
  // ---------------------------------------------------------------------------
  final int WAITING = 0, TRANSIT = 1, EDGE = 2, RASTER = 3, FINISHED = 4;
  int signal = 0;
  int state = WAITING;

  /** @brief Requested path spacing. The realised spacing is derived below. */
  float requestedLaneSpacingMm = 8.0;
  /** @brief Actual spacing after fitting an integer number of lanes. */
  float actualLaneSpacingMm = 8.0;
  int laneCount = 44;
  /** @brief Cruise PWM, adjustable while scanning to study speed effects. */
  float cruisePwm = 120.0;

  // Scan target, set from the surface-map bounds by the Simulator.
  float targetMinX = -35.0;
  float targetMinY = -108.5;
  float targetMaxX = 305.0;
  float targetMaxY = 108.5;
  float targetWidth = 340.0;
  float targetHeight = 217.0;
  boolean targetReady = false;

  // Plan corners and phase boundaries.
  int transitIndex = 0;
  int edgeStartIndex = 0;
  int edgeEndIndex = 0;
  int rasterStartIndex = 0;

  ArrayList<ScanPoint_c> waypoints = new ArrayList<ScanPoint_c>();
  int waypointIndex = 0;
  boolean planReady = false;
  long startMs = 0;
  long lastUpdateMs = -1;

  // Fused pose.
  boolean fusionEnabled = true;
  boolean fusionReady = false;
  boolean externalPose = false;
  RobotPose_c fusedPose = new RobotPose_c();
  PoseFilter_c poseFilter = new PoseFilter_c();
  float lastEncoderLeft = 0.0;
  float lastEncoderRight = 0.0;

  // ---------------------------------------------------------------------------
  // Control-law selection
  // ---------------------------------------------------------------------------
  final int CTRL_OPEN = 0, CTRL_P = 1, CTRL_PID = 2, CTRL_LQR = 3, CTRL_MPC = 4;
  int controlMode = CTRL_P;
  float segmentStartX = 0.0;
  float segmentStartY = 0.0;
  float pidIntegral = 0.0;
  float lqrK1 = 0.0;
  float lqrK2 = 0.0;
  float wheelSeparationMm = 90.0;
  final float PID_KP = 2.0;
  final float PID_KI = 0.0005;
  final float PID_KD = 80.0;

  // Open-loop timed executor.
  final int OPEN_IDLE = 0, OPEN_PIVOT = 1, OPEN_DRIVE = 2;
  int openPhase = OPEN_IDLE;
  long openPhaseStartMs = 0;
  long openPhaseDurationMs = 0;
  float openDriveDistance = 0.0;
  float openPlannedX = 0.0;
  float openPlannedY = 0.0;
  float openPlannedTheta = 0.0;
  float openTargetX = 0.0;
  float openTargetY = 0.0;
  int openTurnSign = 1;

  /** @brief TaskTimer_c to keep aligned with code from Arduino platform. */
  TaskTimer_c update_timer = new TaskTimer_c(UPDATE_INTERVAL_MS);
  TaskTimer_c telemetry_timer = new TaskTimer_c(TELEMETRY_INTERVAL_MS);

  /** @brief Returns the controller to its inactive waiting state. */
  void reset() {
    signal = 0;
    state = WAITING;
    waypoints.clear();
    waypointIndex = 0;
    planReady = false;
    lastUpdateMs = -1;
    fusionReady = false;
    poseFilter.ready = false;
    fusedPose.x = 0.0;
    fusedPose.y = 0.0;
    fusedPose.theta = 0.0;
    segmentStartX = 0.0;
    segmentStartY = 0.0;
    pidIntegral = 0.0;
    openPhase = OPEN_IDLE;
  }

  /** @return The latched start signal: 0 while waiting, 1 once started. */
  int getSignal() {
    return signal;
  }

  /**
   * @brief Starts a scan from the configured target area.
   * @param robot Robot whose pose initialises the fused estimator.
   */
  void beginScan(Robot_c robot) {
    wheelSeparationMm = max(1.0, robot.wheelSeparationMm);
    initialiseFusion(robot);
    segmentStartX = robot.odometry.pose.x;
    segmentStartY = robot.odometry.pose.y;
    pidIntegral = 0.0;
    openPlannedX = robot.odometry.pose.x;
    openPlannedY = robot.odometry.pose.y;
    openPlannedTheta = robot.odometry.pose.theta;
    openPhase = OPEN_IDLE;
    computeLqrGain();
    startMs = robot.getMillis();
    // HIL may supply a clock unrelated to the previous mode's timebase.
    update_timer.resetTimer(robot.getMillis());
    telemetry_timer.resetTimer(robot.getMillis());
    setSignal(1);
  }

  /**
   * @brief Updates the latched start signal.
   * @param requested Zero resets the controller; a non-zero value starts it.
   * @note The scan geometry comes from setScanTarget(), so no robot is needed.
   */
  void setSignal(int requested) {
    if (requested == 0) {
      reset();
      return;
    }
    if (signal == 0) {
      signal = 1;
      state = TRANSIT;
      planReady = false;
      buildScanPlan();
    }
  }

  /**
   * @brief Runs at most one periodic controller update.
   * @param robot Simulated robot supplying cached readings and accepting PWM.
   */
  void update(Robot_c robot) {
    long now = robot.getMillis();

    if (!update_timer.isReady(now)) {
      return;
    }

    update_timer.resetTimer(now);
    lastUpdateMs = now;

    robot.getSurfaceSensors();
    robot.getEncoders();
    robot.getPose();
    updatePoseFilter(robot);

    if (signal == 1) {
      runRasterScan(robot);
    }

    if (telemetry_timer.isReady(now)) {
      telemetry_timer.resetTimer(now);
      publishTelemetry(robot); // Note: in simulation this is a dummy call
    }
  }

  // ---------------------------------------------------------------------------
  // Scan target
  // ---------------------------------------------------------------------------

  /**
   * @brief Sets the map area that the scan should cover.
   * @param minX Left edge to survey, in world mm.
   * @param minY Bottom edge to survey, in world mm.
   * @param maxX Right edge to survey, in world mm.
   * @param maxY Top edge to survey, in world mm.
   */
  void setScanTarget(float minX, float minY, float maxX, float maxY) {
    targetMinX = min(minX, maxX);
    targetMaxX = max(minX, maxX);
    targetMinY = min(minY, maxY);
    targetMaxY = max(minY, maxY);
    targetWidth = targetMaxX - targetMinX;
    targetHeight = targetMaxY - targetMinY;
    targetReady = targetWidth > 20.0 && targetHeight > 20.0;
  }

  // ---------------------------------------------------------------------------
  // Plan construction
  // ---------------------------------------------------------------------------

  /**
   * @brief Builds the transit, perimeter and raster waypoint list.
   * @details The lane count is chosen so the realised spacing is close to the
   * requested spacing while the target area stays fixed.
   */
  void buildScanPlan() {
    if (!targetReady) return;

    laneCount = max(2, round(targetWidth / max(1.0, requestedLaneSpacingMm)) + 1);
    actualLaneSpacingMm = targetWidth / (laneCount - 1);

    waypoints.clear();

    // Transit to the top-left corner. The first edge then runs down the left
    // side, matching the initial lane direction.
    ScanPoint_c p0 = new ScanPoint_c(targetMinX, targetMaxY);
    ScanPoint_c p1 = new ScanPoint_c(targetMinX, targetMinY);
    ScanPoint_c p2 = new ScanPoint_c(targetMaxX, targetMinY);
    ScanPoint_c p3 = new ScanPoint_c(targetMaxX, targetMaxY);
    transitIndex = 0;
    waypoints.add(p0);

    // One complete perimeter, returning to the top-left.
    edgeStartIndex = waypoints.size();
    waypoints.add(p1);
    waypoints.add(p2);
    waypoints.add(p3);
    waypoints.add(p0);
    edgeEndIndex = waypoints.size();
    rasterStartIndex = edgeEndIndex;

    // Serpentine interior. Lane 0 repeats the left edge, which is a small
    // cost that keeps the first raster waypoint directly below the robot.
    float previousEndY = targetMinY;
    for (int lane = 0; lane < laneCount; lane++) {
      float x = targetMinX + lane * actualLaneSpacingMm;
      if (lane > 0) {
        waypoints.add(new ScanPoint_c(x, previousEndY));
      }
      float endY = (lane % 2 == 0) ? targetMinY : targetMaxY;
      waypoints.add(new ScanPoint_c(x, endY));
      previousEndY = endY;
    }

    waypointIndex = 0;
    planReady = true;
  }

  // ---------------------------------------------------------------------------
  // Scan execution
  // ---------------------------------------------------------------------------

  /** @brief Advances the current phase and follows the current waypoint. */
  void runRasterScan(Robot_c robot) {
    if (!planReady || waypoints.size() == 0) {
      robot.setMotorPWM(0, 0);
      return;
    }
    if (state == FINISHED) {
      robot.setMotorPWM(0, 0);
      return;
    }
    if (controlMode == CTRL_OPEN) {
      runOpenLoop(robot);
      return;
    }

    RobotPose_c pose = fusionReady ? fusedPose : robot.odometry.pose;
    ScanPoint_c target = waypoints.get(waypointIndex);
    float dx = target.x - pose.x;
    float dy = target.y - pose.y;
    float distance = sqrt(dx * dx + dy * dy);

    boolean reached = distance <= WAYPOINT_TOLERANCE_MM;
    // A lane-keeping law follows the segment line rather than seeking the
    // point, so it can pass the endpoint slightly off-axis. Treat projection
    // past the endpoint as arrival as well.
    if (!reached && controlMode != CTRL_P) {
      reached = passedWaypoint(pose, target);
    }

    if (reached) {
      float reachedX = target.x;
      float reachedY = target.y;
      waypointIndex++;
      if (waypointIndex >= waypoints.size()) {
        state = FINISHED;
        robot.setMotorPWM(0, 0);
        return;
      }
      updatePhaseAfterWaypoint();
      segmentStartX = reachedX;
      segmentStartY = reachedY;
      pidIntegral = 0.0;
      target = waypoints.get(waypointIndex);
      dx = target.x - pose.x;
      dy = target.y - pose.y;
      distance = sqrt(dx * dx + dy * dy);
    }

    driveTo(robot, pose, dx, dy, distance);
  }

  /** @brief True when the robot has reached or passed a segment endpoint. */
  boolean passedWaypoint(RobotPose_c pose, ScanPoint_c target) {
    float segDx = target.x - segmentStartX;
    float segDy = target.y - segmentStartY;
    float length = sqrt(segDx * segDx + segDy * segDy);
    if (length < 1.0) return true;
    float ux = segDx / length;
    float uy = segDy / length;
    float along = (pose.x - segmentStartX) * ux + (pose.y - segmentStartY) * uy;
    return along >= length - 6.0;
  }

  /** @brief Switches TRANSIT -> EDGE -> RASTER at the plan boundaries. */
  void updatePhaseAfterWaypoint() {
    if (state == TRANSIT && waypointIndex >= edgeStartIndex) {
      state = EDGE;
    }
    if (state == EDGE && waypointIndex >= edgeEndIndex) {
      state = RASTER;
    }
  }

  /**
   * @brief Dispatches the selected control law.
   */
  void driveTo(Robot_c robot, RobotPose_c pose, float dx, float dy, float distance) {
    if (controlMode == CTRL_P) {
      driveProportional(robot, pose, dx, dy, distance);
    } else {
      driveLaneKeeping(robot, pose, dx, dy, distance);
    }
  }

  /**
   * @brief Closed-loop proportional heading-to-target steering.
   * @details Positive heading error means the target is to the left, which
   * requires the right wheel to run faster than the left.
   */
  void driveProportional(Robot_c robot, RobotPose_c pose, float dx, float dy,
    float distance) {
    float desiredHeading = atan2(dy, dx);
    float error = wrapAngle(desiredHeading - pose.theta);

    if (abs(error) > PIVOT_ERROR_RAD) {
      float turn = (error >= 0) ? PIVOT_PWM : -PIVOT_PWM;
      robot.setMotorPWM(-turn, turn);
      return;
    }

    float speed = approachSpeed(distance);
    float turn = constrain(HEADING_GAIN * error, -MAX_STEER_PWM, MAX_STEER_PWM);
    robot.setMotorPWM(speed - turn, speed + turn);
  }

  /**
   * @brief PID, LQR or MPC lane keeping on the current waypoint segment.
   * @details The cross-track error is positive to the left of the segment and
   * the heading error is positive when the robot points left of it. A negative
   * turn reduces both.
   */
  void driveLaneKeeping(Robot_c robot, RobotPose_c pose, float dx, float dy,
    float distance) {
    float targetX = pose.x + dx;
    float targetY = pose.y + dy;
    float segDx = targetX - segmentStartX;
    float segDy = targetY - segmentStartY;
    float segLen = max(1e-3, sqrt(segDx * segDx + segDy * segDy));
    float ux = segDx / segLen;
    float uy = segDy / segLen;
    float relX = pose.x - segmentStartX;
    float relY = pose.y - segmentStartY;
    float lateral = -relX * uy + relY * ux;
    float segmentAngle = atan2(segDy, segDx);
    float psi = wrapAngle(pose.theta - segmentAngle);

    // Large heading errors are better handled by a pivot than by a wide arc.
    float alignError = wrapAngle(segmentAngle - pose.theta);
    if (abs(alignError) > 0.9) {
      float turn = (alignError >= 0) ? PIVOT_PWM : -PIVOT_PWM;
      robot.setMotorPWM(-turn, turn);
      return;
    }

    float speed = approachSpeed(distance);
    float turn;
    if (controlMode == CTRL_PID) {
      pidIntegral = constrain(pidIntegral + lateral * UPDATE_INTERVAL_MS,
        -5000.0, 5000.0);
      turn = -(PID_KP * lateral + PID_KI * pidIntegral + PID_KD * psi);
    } else if (controlMode == CTRL_LQR) {
      turn = -(lqrK1 * lateral + lqrK2 * psi);
    } else {
      turn = mpcTurn(lateral, psi, speed);
    }

    turn = constrain(turn, -MAX_STEER_PWM, MAX_STEER_PWM);
    robot.setMotorPWM(speed - turn, speed + turn);
  }

  float approachSpeed(float distance) {
    if (distance < APPROACH_DISTANCE_MM) {
      return max(MIN_APPROACH_PWM,
        cruisePwm * distance / APPROACH_DISTANCE_MM);
    }
    return cruisePwm;
  }

  /** @brief Receding-horizon search over the lane-keeping model. */
  float mpcTurn(float lateral, float psi, float speed) {
    float dStep = speed / 200.0 * NOMINAL_MAX_SPEED_MM_PER_MS * UPDATE_INTERVAL_MS;
    float bStep = 2.0 * (1.0 / 200.0 * NOMINAL_MAX_SPEED_MM_PER_MS)
      * UPDATE_INTERVAL_MS / wheelSeparationMm;
    float bestTurn = 0.0;
    float bestCost = 1e20;
    for (int candidate = -70; candidate <= 70; candidate += 5) {
      float e = lateral;
      float p = psi;
      float cost = 0.0;
      for (int horizon = 0; horizon < 30; horizon++) {
        e += dStep * p;
        p += bStep * candidate;
        cost += e * e + 10.0 * p * p + 0.01 * candidate * candidate;
      }
      cost += 5.0 * (e * e + 10.0 * p * p);
      if (cost < bestCost) {
        bestCost = cost;
        bestTurn = candidate;
      }
    }
    return bestTurn;
  }

  /**
   * @brief Solves the 2x2 discrete Riccati equation for lane keeping.
   * @details State [lateral mm, heading rad], control [turn PWM]. Q and R are
   * chosen so the closed-loop poles are stable for the nominal cruise speed.
   */
  void computeLqrGain() {
    float dStep = cruisePwm / 200.0 * NOMINAL_MAX_SPEED_MM_PER_MS * UPDATE_INTERVAL_MS;
    float bStep = 2.0 * (1.0 / 200.0 * NOMINAL_MAX_SPEED_MM_PER_MS)
      * UPDATE_INTERVAL_MS / wheelSeparationMm;
    float a00 = 1.0, a01 = dStep, a10 = 0.0, a11 = 1.0;
    float b0 = 0.0, b1 = bStep;
    float p00 = 1.0, p01 = 0.0, p10 = 0.0, p11 = 1e4;
    float q00 = 1.0, q11 = 1e4, r = 1.0;
    for (int iteration = 0; iteration < 300; iteration++) {
      float btpb = b0 * b0 * p00 + 2.0 * b0 * b1 * p01 + b1 * b1 * p11;
      float v0 = b0 * p00 + b1 * p10;
      float v1 = b0 * p01 + b1 * p11;
      float numerator0 = v0 * a00 + v1 * a10;
      float numerator1 = v0 * a01 + v1 * a11;
      float denominator = r + btpb;
      float k0 = numerator0 / denominator;
      float k1 = numerator1 / denominator;
      float m00 = a00 - b0 * k0;
      float m01 = a01 - b0 * k1;
      float m10 = a10 - b1 * k0;
      float m11 = a11 - b1 * k1;
      float atp00 = a00 * p00 + a10 * p10;
      float atp01 = a00 * p01 + a10 * p11;
      float atp10 = a01 * p00 + a11 * p10;
      float atp11 = a01 * p01 + a11 * p11;
      p00 = q00 + atp00 * m00 + atp01 * m10;
      p01 = atp00 * m01 + atp01 * m11;
      p10 = atp10 * m00 + atp11 * m10;
      p11 = q11 + atp10 * m01 + atp11 * m11;
      lqrK1 = k0;
      lqrK2 = k1;
    }
  }

  // ---------------------------------------------------------------------------
  // Open-loop timed executor
  // ---------------------------------------------------------------------------

  /**
   * @brief Executes the plan with timed pivot and straight segments.
   * @details The measured pose is deliberately ignored: the controller commands
   * fixed-duration motions from the planned pose. This is the open-loop
   * baseline against which the feedback laws are compared.
   */
  void runOpenLoop(Robot_c robot) {
    long now = robot.getMillis();
    if (openPhase == OPEN_IDLE) {
      prepareOpenSegment(robot);
      return;
    }

    long elapsed = now - openPhaseStartMs;
    if (openPhase == OPEN_PIVOT) {
      if (elapsed >= openPhaseDurationMs) {
        openPhase = OPEN_DRIVE;
        openPhaseStartMs = now;
        float speedPerMs = cruisePwm / 200.0 * NOMINAL_MAX_SPEED_MM_PER_MS;
        openPhaseDurationMs = round(openDriveDistance / max(1e-6, speedPerMs));
        robot.setMotorPWM(cruisePwm, cruisePwm);
      } else {
        robot.setMotorPWM(-openTurnSign * PIVOT_PWM,
          openTurnSign * PIVOT_PWM);
      }
      return;
    }

    if (elapsed >= openPhaseDurationMs) {
      robot.setMotorPWM(0, 0);
      openPlannedX = openTargetX;
      openPlannedY = openTargetY;
      waypointIndex++;
      if (waypointIndex >= waypoints.size()) {
        state = FINISHED;
        openPhase = OPEN_IDLE;
        return;
      }
      updatePhaseAfterWaypoint();
      prepareOpenSegment(robot);
    } else {
      robot.setMotorPWM(cruisePwm, cruisePwm);
    }
  }

  void prepareOpenSegment(Robot_c robot) {
    if (waypointIndex >= waypoints.size()) {
      state = FINISHED;
      openPhase = OPEN_IDLE;
      return;
    }
    ScanPoint_c target = waypoints.get(waypointIndex);
    float dx = target.x - openPlannedX;
    float dy = target.y - openPlannedY;
    float targetHeading = atan2(dy, dx);
    float delta = wrapAngle(targetHeading - openPlannedTheta);
    float pivotRate = 2.0 * (PIVOT_PWM / 200.0 * NOMINAL_MAX_SPEED_MM_PER_MS)
      / wheelSeparationMm;
    openPhaseDurationMs = round(abs(delta) / max(1e-9, pivotRate));
    openTurnSign = (delta >= 0) ? 1 : -1;
    openPlannedTheta = targetHeading;
    openDriveDistance = sqrt(dx * dx + dy * dy);
    openTargetX = target.x;
    openTargetY = target.y;
    openPhase = OPEN_PIVOT;
    openPhaseStartMs = robot.getMillis();
  }

  // ---------------------------------------------------------------------------
  // Mode selection
  // ---------------------------------------------------------------------------

  void cycleControlMode() {
    controlMode = (controlMode + 1) % 5;
  }

  void setControlMode(int mode) {
    controlMode = constrain(mode, 0, 4);
  }

  String getControlName() {
    if (controlMode == CTRL_OPEN) return "open";
    if (controlMode == CTRL_PID) return "PID";
    if (controlMode == CTRL_LQR) return "LQR";
    if (controlMode == CTRL_MPC) return "MPC";
    return "P";
  }

  float wrapAngle(float angle) {
    while (angle > PI) angle -= TWO_PI;
    while (angle < -PI) angle += TWO_PI;
    return angle;
  }

  // ---------------------------------------------------------------------------
  // Pose fusion
  // ---------------------------------------------------------------------------

  /** @brief Resets the selected estimator to the robot's reported pose. */
  void initialiseFusion(Robot_c robot) {
    poseFilter.reset(robot.odometry.pose);
    fusedPose = poseFilter.pose;
    lastEncoderLeft = robot.odometry.encoders.left;
    lastEncoderRight = robot.odometry.encoders.right;
    fusionReady = true;
  }

  /**
   * @brief Fuses encoder distance, IMU heading and the PWM motion prior.
   * @details Encoders measure wheel rotation. Under wheel slip the chassis
   * travels less than the wheels rotate, which this estimator cannot observe
   * without an external position reference; the PWM prior shares the same
   * assumption. Heading is observable with the IMU, so the selected filter
   * uses the gyro rate as the prediction and the absolute yaw as a measurement.
   */
  void updatePoseFilter(Robot_c robot) {
    if (!fusionEnabled || externalPose) {
      poseFilter.pose.x = robot.odometry.pose.x;
      poseFilter.pose.y = robot.odometry.pose.y;
      poseFilter.pose.theta = robot.odometry.pose.theta;
      fusedPose = poseFilter.pose;
      fusionReady = true;
      return;
    }
    if (!fusionReady) {
      initialiseFusion(robot);
      return;
    }

    float millimetresPerCount = robot.odometry.getMmPerEncoderCount();
    float leftDistance =
      (robot.odometry.encoders.left - lastEncoderLeft) * millimetresPerCount;
    float rightDistance =
      (robot.odometry.encoders.right - lastEncoderRight) * millimetresPerCount;
    float encoderDistance = (leftDistance + rightDistance) / 2.0;

    // Commanded PWM gives an independent, deliberately low-weight estimate of
    // wheel travel. It smooths encoder quantisation and models the actuators.
    float pwmLeftSpeed = robot.motors.left.pwm / 200.0 * NOMINAL_MAX_SPEED_MM_PER_MS;
    float pwmRightSpeed = robot.motors.right.pwm / 200.0 * NOMINAL_MAX_SPEED_MM_PER_MS;
    float pwmDistance = (pwmLeftSpeed + pwmRightSpeed) / 2.0 * UPDATE_INTERVAL_MS;
    float fusedDistance = (1.0 - PWM_PRIOR_WEIGHT) * encoderDistance
      + PWM_PRIOR_WEIGHT * pwmDistance;

    poseFilter.update(fusedDistance,
      robot.getImuYawRate() * UPDATE_INTERVAL_MS,
      robot.getImuYaw());
    fusedPose = poseFilter.pose;

    lastEncoderLeft = robot.odometry.encoders.left;
    lastEncoderRight = robot.odometry.encoders.right;
  }

  /** @brief Enables or disables IMU/PWM fusion for comparison runs. */
  void setFusionEnabled(boolean enabled) {
    fusionEnabled = enabled;
    fusionReady = false;
  }

  void setFusionMethod(int method) {
    poseFilter.method = constrain(method, 0, 2);
  }

  int getFusionMethod() { return poseFilter.method; }

  void setJacobianMode(int mode) {
    poseFilter.jacobianMode = constrain(mode, 0, 2);
  }

  int getJacobianMode() { return poseFilter.jacobianMode; }

  String getFusionName() {
    if (poseFilter.method == poseFilter.EKF) {
      return "ekf/" + poseFilter.jacobianName();
    }
    if (poseFilter.method == poseFilter.UKF) return "ukf";
    return "comp";
  }

  /**
   * @brief Uses the externally supplied pose directly, as required by HIL.
   * @note In HIL the physical robot's telemetry pose already is the estimator
   * output; running a second filter on the PC would double-count the IMU.
   */
  void setExternalPose(boolean enabled) {
    externalPose = enabled;
    fusionReady = false;
  }

  boolean isExternalPose() { return externalPose; }
  boolean isFusionEnabled() { return fusionEnabled; }
  boolean isFusionReady() { return fusionReady; }
  RobotPose_c getFusedPose() { return fusedPose; }

  // ---------------------------------------------------------------------------
  // Queries used by the reconstruction, panel and drawing
  // ---------------------------------------------------------------------------

  boolean isActive() { return signal == 1; }
  boolean isScanning() {
    return signal == 1 && (state == EDGE || state == RASTER);
  }
  boolean isFinished() { return state == FINISHED; }
  boolean hasPlan() { return planReady && waypoints.size() > 0; }
  long getLastUpdateMs() { return lastUpdateMs; }
  long getStartMs() { return startMs; }

  /** @return Fraction of the complete plan that has been reached. */
  float getProgress() {
    if (waypoints.size() == 0) return 0.0;
    return constrain(waypointIndex / (float)waypoints.size(), 0.0, 1.0);
  }

  String getStateName() {
    if (state == WAITING) return "WAITING";
    if (state == TRANSIT) return "TRANSIT";
    if (state == EDGE) return "EDGE";
    if (state == RASTER) return "RASTER";
    return "FINISHED";
  }

  float getActualSpacingMm() { return actualLaneSpacingMm; }
  int getLaneCount() { return laneCount; }
  float getTargetWidthMm() { return targetWidth; }
  float getTargetHeightMm() { return targetHeight; }

  /** @brief Sets the requested path spacing; apply by restarting the scan. */
  void setRequestedLaneSpacingMm(float value) {
    requestedLaneSpacingMm = constrain(value, 4.0, 120.0);
  }

  /** @brief Sets the cruise PWM; applies to the next control update. */
  void setCruisePwm(float value) {
    cruisePwm = constrain(value, 15.0, 200.0);
  }

  /**
   * @brief Tests a world point against the reachable scan region.
   * @details This is the target rectangle expanded by the sensor array's
   * lateral half-span and forward reach. Coverage and tone error are reported
   * over this region, which is the part of the map the sweep can observe.
   */
  boolean insideScanArea(float worldX, float worldY) {
    if (!planReady) return false;
    float along = targetMaxY - worldY;      // 0 at the top edge
    float lateral = worldX - targetMinX;    // 0 at the left edge
    return along >= -SENSOR_FORWARD_MM
      && along <= targetHeight + SENSOR_FORWARD_MM
      && lateral >= -SENSOR_HALF_SPAN_MM
      && lateral <= targetWidth + SENSOR_HALF_SPAN_MM;
  }

  /** @return World axis-aligned bounds {minX, minY, maxX, maxY}. */
  float[] getScanAreaBounds() {
    float[] result = new float[4];
    if (!planReady) return result;
    result[0] = targetMinX - SENSOR_HALF_SPAN_MM;
    result[1] = targetMinY - SENSOR_FORWARD_MM;
    result[2] = targetMaxX + SENSOR_HALF_SPAN_MM;
    result[3] = targetMaxY + SENSOR_FORWARD_MM;
    return result;
  }

  /** @brief Draws the target rectangle, perimeter and planned lanes. */
  void drawPlan() {
    if (!hasPlan()) return;
    pushStyle();

    float[] bounds = getScanAreaBounds();
    noFill();
    stroke(30, 90, 210, 45);
    strokeWeight(1.5);
    rectMode(CORNERS);
    rect(bounds[0], bounds[1], bounds[2], bounds[3]);

    // The perimeter survey, drawn more strongly than the interior lanes.
    stroke(230, 120, 30, 150);
    strokeWeight(2.0);
    rect(targetMinX, targetMinY, targetMaxX, targetMaxY);

    // Lane centre lines.
    stroke(30, 90, 210, 65);
    strokeWeight(1.0);
    for (int lane = 0; lane < laneCount; lane++) {
      float x = targetMinX + lane * actualLaneSpacingMm;
      line(x, targetMinY, x, targetMaxY);
    }

    // Waypoints, with the active target highlighted.
    noStroke();
    for (int i = 0; i < waypoints.size(); i++) {
      ScanPoint_c p = waypoints.get(i);
      if (i < waypointIndex) {
        fill(80, 160, 90, 150);
      } else if (i == waypointIndex) {
        fill(230, 110, 25, 230);
      } else {
        fill(40, 100, 220, 110);
      }
      ellipse(p.x, p.y, 2.5, 2.5);
    }
    popStyle();
  }

  /**
   * @brief Placeholder matching the StampC3 controller's telemetry task.
   * @note Simulator owns TCP reception and graphing. This empty method keeps
   * the 50 ms controller task visible without creating a second transport.
   */
  void publishTelemetry(Robot_c robot) {
  }
}

/** @brief One world-coordinate waypoint or trail point. */
class ScanPoint_c {
  float x;
  float y;

  ScanPoint_c(float xIn, float yIn) {
    x = xIn;
    y = yIn;
  }
}
