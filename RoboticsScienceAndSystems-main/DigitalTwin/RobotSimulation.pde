/**
 * @brief Top-level model of the robot, excluding the environment.
 * @details Coordinates independent motor, surface-sensor, and odometry
 * models. Its public methods deliberately mirror the subset of Robot_c used
 * by Controller_c, while the model components remain visible for revision.
 *
 * Two poses are maintained when a pose-error model is enabled:
 *  - odometry.pose is the pose the robot reports (and the controller uses);
 *  - truePose is the pose actually reached by the simulated chassis and is
 *    the pose used to sample the surface and to draw the robot.
 * With zero wheel slip and zero separation error the two are identical, which
 * preserves the template's ideal behaviour.
 */
class Robot_c {
  /** @brief Radius used only to draw the robot body, in mm. */
  final float bodyRadiusMm = 50.0;
  final float wheelRadiusMm;
  final float wheelSeparationMm;
  final float encoderCountsPerRevolution = 358.3;

  /** @brief Simulated clock, advanced explicitly in milliseconds. */
  long simulationMillis = 0;
  PImage surfaceImage;
  final float surfaceOffsetX;
  final float surfaceOffsetY;
  RobotSurface_c surface = new RobotSurface_c();
  MotorsModel_c motors = new MotorsModel_c();
  OdometryModel_c odometry;
  /** @brief Pose actually reached by the simulated robot. */
  RobotPose_c truePose = new RobotPose_c();
  SurfaceSensorModel_c[] sensors = new SurfaceSensorModel_c[5];

  /** @brief Fraction of wheel travel lost to slip (true motion < reported). */
  float tractionSlip = 0.0;
  /** @brief Fractional error in the effective wheel separation. */
  float separationError = 0.0;
  /** @brief When true, surface readings come from an external source (HIL). */
  boolean externalSensors = false;

  /** @brief Standard deviation of the simulated IMU yaw measurement, rad. */
  float imuNoiseRad = radians(0.5);
  /** @brief True chassis yaw rate from the last physics step, rad/ms. */
  float imuYawRateRadPerMs = 0.0;

  /**
   * @param wheelRadiusMmIn Wheel radius used by the odometry model, in mm.
   * @param wheelSeparationMmIn Distance between wheels, in mm.
   * @param surfaceIn Environment image sampled by the five surface sensors.
   * @param surfaceOffsetXIn Surface-image x offset in world millimetres.
   * @param surfaceOffsetYIn Surface-image y offset in world millimetres.
   */
  Robot_c(float wheelRadiusMmIn, float wheelSeparationMmIn,
    PImage surfaceIn, float surfaceOffsetXIn, float surfaceOffsetYIn) {
    wheelRadiusMm = wheelRadiusMmIn;
    wheelSeparationMm = wheelSeparationMmIn;
    surfaceImage = surfaceIn;
    surfaceOffsetX = surfaceOffsetXIn;
    surfaceOffsetY = surfaceOffsetYIn;
    odometry = new OdometryModel_c(wheelRadiusMm, encoderCountsPerRevolution);
    sensors[0] = new SurfaceSensorModel_c("DN1", 45, 45);
    sensors[1] = new SurfaceSensorModel_c("DN2", 45, 14);
    sensors[2] = new SurfaceSensorModel_c("DN3", 45, 0);
    sensors[3] = new SurfaceSensorModel_c("DN4", 45, -14);
    sensors[4] = new SurfaceSensorModel_c("DN5", 45, -45);
    odometry.reset(0.0, 0.0, -PI/2.0, 0.0, 0.0);
    truePose.x = 0.0;
    truePose.y = 0.0;
    truePose.theta = -PI/2.0;
  }

  /** @return Current simulated time in milliseconds. */
  long getMillis() { return simulationMillis; }
  /** @brief Advances the simulated clock.
   * @param dtMs Positive elapsed model time in milliseconds.
   */
  void advanceTime(int dtMs) { simulationMillis += dtMs; }

  /** @brief Replaces the environment image (for example, a test card). */
  void setSurfaceImage(PImage image) { surfaceImage = image; }

  /** @brief Applies one spot size and noise level to all five sensors. */
  void setSensorModel(float footprintRadiusMm, float noiseSigma) {
    for (int i = 0; i < sensors.length; i++) {
      sensors[i].footprintRadiusMm = max(0.0, footprintRadiusMm);
      sensors[i].noiseSigma = max(0.0, noiseSigma);
    }
  }

  /** @brief Enables externally supplied surface readings (HIL mode). */
  void setExternalSensors(boolean enabled) {
    externalSensors = enabled;
  }

  /**
   * @brief Enables non-ideal motion and reported-pose divergence.
   * @param slipFraction Fraction of commanded wheel travel not achieved.
   * @param separationFraction Relative error in the effective wheel separation.
   * @note These describe the chassis, not the controller. Zero values restore
   * the template's ideal odometry.
   */
  void setPoseErrorModel(float slipFraction, float separationFraction) {
    tractionSlip = constrain(slipFraction, 0.0, 0.5);
    separationError = constrain(separationFraction, 0.0, 0.5);
  }

  /** @brief Sets the simulated IMU noise level. */
  void setImuNoiseRad(float noiseRad) {
    imuNoiseRad = max(0.0, noiseRad);
  }

  /** @return Simulated IMU yaw measurement in radians (true heading + noise). */
  float getImuYaw() {
    if (imuNoiseRad <= 0.0) return truePose.theta;
    return truePose.theta + randomGaussian() * imuNoiseRad;
  }

  /** @return True chassis yaw rate from the last physics step, rad/ms. */
  float getImuYawRate() { return imuYawRateRadPerMs; }

  /** @brief Samples all five surface sensors into surface.reading.
   * @note Sampling uses the true chassis pose, not the reported odometry.
   */
  void getSurfaceSensors() {
    if (externalSensors) return;
    RobotPose_c pose = truePose;
    for (int i = 0; i < sensors.length; i++) {
      surface.reading[i] = sensors[i].sample(surfaceImage, surfaceOffsetX,
        surfaceOffsetY, pose.x, pose.y, pose.theta);
    }
  }

  /** @brief Compatibility method; encoder values update during physics steps. */
  void getEncoders() { }
  /** @brief Compatibility method; pose values update during physics steps. */
  void getPose() { }

  /** @brief Stores requested left and right motor PWM values.
   * @param leftPwm Requested left PWM, nominally -200 to +200.
   * @param rightPwm Requested right PWM, nominally -200 to +200.
   */
  void setMotorPWM(float leftPwm, float rightPwm) {
    motors.setPWM(leftPwm, rightPwm);
  }

  /** @brief Resets pose and encoder state for a controlled run.
   * @param x Initial x position in mm.
   * @param y Initial y position in mm.
   * @param theta Initial heading in radians.
   * @param leftEncoder Initial left encoder count.
   * @param rightEncoder Initial right encoder count.
   */
  void resetForRun(float x, float y, float theta, float leftEncoder,
    float rightEncoder) {
    odometry.reset(x, y, theta, leftEncoder, rightEncoder);
    truePose.x = x;
    truePose.y = y;
    truePose.theta = theta;
    imuYawRateRadPerMs = 0.0;
    setMotorPWM(0, 0);
  }

  /** @brief Advances motor response and differential-drive odometry.
   * @param dtMs Fixed simulation step in milliseconds.
   */
  void updatePhysics(int dtMs) {
    motors.update();
    float leftDistance = motors.left.speedMmPerMs * dtMs;
    float rightDistance = motors.right.speedMmPerMs * dtMs;

    // Encoders see wheel rotation: these are the distances the robot reports.
    odometry.update(leftDistance, rightDistance, wheelSeparationMm);

    if (tractionSlip <= 0.00001 && separationError <= 0.00001) {
      truePose.x = odometry.pose.x;
      truePose.y = odometry.pose.y;
      truePose.theta = odometry.pose.theta;
      imuYawRateRadPerMs = (rightDistance - leftDistance)
        / wheelSeparationMm / dtMs;
      return;
    }

    float effectiveLeft = leftDistance * (1.0 - tractionSlip);
    float effectiveRight = rightDistance * (1.0 - tractionSlip);
    float effectiveSeparation = wheelSeparationMm * (1.0 + separationError);
    float distance = (effectiveLeft + effectiveRight) / 2.0;
    float deltaTheta = (effectiveRight - effectiveLeft) / effectiveSeparation;
    truePose.x += distance * cos(truePose.theta);
    truePose.y += distance * sin(truePose.theta);
    truePose.theta += deltaTheta;
    imuYawRateRadPerMs = deltaTheta / dtMs;
  }

  /** @return The current five-channel surface-sensor readings. */
  float[] sensorReadings() { return surface.reading; }

  void draw() {
    RobotPose_c pose = truePose;
    pushMatrix();
    translate(pose.x, pose.y);
    rotate(pose.theta);
    fill(190, 205, 220);
    stroke(40);
    strokeWeight(2);
    ellipse(0, 0, bodyRadiusMm, bodyRadiusMm);
    stroke(40, 100, 220);
    strokeWeight(3);
    line(0, 0, bodyRadiusMm * .72, 0);
    fill(40, 100, 220);
    noStroke();
    triangle(bodyRadiusMm * .92, 0, bodyRadiusMm * .62, bodyRadiusMm * .14,
      bodyRadiusMm * .62, -bodyRadiusMm * .14);
    for (int i = 0; i < sensors.length; i++) sensors[i].draw(surface.reading[i]);
    popMatrix();
  }
}
