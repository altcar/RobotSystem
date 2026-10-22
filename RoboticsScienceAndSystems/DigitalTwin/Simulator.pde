/**
 * @brief Owns the standalone application: timing, robot model, view, and TCP.
 * @details World positions use millimetres, with +x forward/right and +y
 * upward. Rendering may run at a variable frame rate; model/controller
 * updates use a fixed 10 ms interval.
 *
 * Image-rastering additions:
 *  - the controller performs a serpentine surface sweep;
 *  - a reconstruction grid accumulates the five reflectance channels using
 *    the robot's reported pose;
 *  - the panel compares the reconstruction with the ideal map and reports
 *    coverage, RMSE and MAE;
 *  - optional non-idealities (sensor footprint, sensor noise and chassis pose
 *    error) make the twin behave less like the ideal template.
 *
 * The template's telemetry graphs, tracking, zoom/pan and surface-map
 * generation are retained unchanged.
 */
class Simulator {
  /** @brief Number of milliseconds visible in each telemetry graph. */
  final int graphWindowMs = 5000;
  final int graphSamplePeriodMs = 20;
  // Mirrors the physical robot's 10 ms update interval.
  final int simulationUpdateIntervalMs = 10;
  final int maximumCatchUpSteps = 10;
  /** @brief Default StampC3 access-point address for live comparison. */
  final String tcpHost = "192.168.4.1";
  final int tcpPort = 80;
  final int tcpRetryPeriodMs = 2000;
  final int tcpConnectTimeoutMs = 250;
  final float minimumZoom = 0.25;
  final float maximumZoom = 4.0;
  final int SANDBOX = 0, LIVE_COMPARISON = 1, HIL = 2;

  /** @brief Fixed sandbox start pose; also the scan home corner. */
  final float SCAN_HOME_X = 0.0;
  final float SCAN_HOME_Y = 0.0;
  final float SCAN_HOME_THETA = -HALF_PI;
  /** @brief Inset from the map edge to the perimeter survey, in mm. */
  final float MAP_EDGE_INSET_MM = 40.0;

  /** @brief Surface-image centre in world coordinates, in mm. */
  final float surfaceOffsetX;
  final float surfaceOffsetY;

  PImage surfaceImage;
  /** @brief The original surface.png, kept so the test card can be toggled. */
  PImage originalSurfaceImage;
  /** @brief Optional generated tone/resolution card for sandbox experiments. */
  PImage testCardImage;
  boolean usingTestCard = false;
  // Cache only the currently visible part of the 1 px/mm map. This avoids
  // redrawing the complete source image at 1:1 scale.
  PImage surfaceViewportCache;
  int surfaceCacheX = -1;
  int surfaceCacheY = -1;
  int surfaceCacheWidth = 0;
  int surfaceCacheHeight = 0;
  PApplet applet;
  /** @brief Robot-only Digital Twin model; the surface image is its environment. */
  Robot_c robot;
  Controller_c controller;
  TimeSeriesGraph motorGraph;
  TimeSeriesGraph sensorGraph;
  TimeSeriesGraph encoderGraph;

  // ---------------------------------------------------------------------------
  // Image-rastering state
  // ---------------------------------------------------------------------------
  SurfaceReconstructor_c simReconstructor;
  SurfaceReconstructor_c liveReconstructor;
  SampleSpacing_c simSpacing = new SampleSpacing_c();
  SampleSpacing_c liveSpacing = new SampleSpacing_c();
  ArrayList<ScanPoint_c> trailTrue = new ArrayList<ScanPoint_c>();
  ArrayList<ScanPoint_c> trailReported = new ArrayList<ScanPoint_c>();

  boolean showRasterPanel = true;
  boolean showRasterOverlay = false;
  boolean showHelp = false;
  long lastTrailMs = -1000;
  long scanStartSimMs = 0;
  boolean lastScanFinished = false;
  int noiseLevel = 0;
  int poseErrorLevel = 0;
  int lastStatsMs = 0;
  ReconStats_c cachedStats;
  PImage truthCrop;
  PImage reconCrop;
  int cropX = -1, cropY = -1, cropW = 0, cropH = 0;
  int lastReconCropMs = 0;
  final float[] noiseLevels = {0.0, 30.0, 80.0};
  final float[] poseErrorLevels = {0.0, 0.05, 0.10, 0.15};

  /** @brief Overlay opacity levels for the reconstructed picture. */
  final int[] overlayAlphas = {235, 180, 130, 80, 40};
  int overlayAlphaLevel = 0;
  /** @brief Per-100 ms memory fade factors for old reconstruction samples. */
  final float[] fadeLevels = {1.0, 0.995, 0.98, 0.95};
  int fadeLevel = 0;
  long lastFadeApplyMs = 0;
  /** @brief Content scale of the map, to test the minimum detectable feature. */
  final float[] mapScales = {1.0, 0.75, 0.5, 0.35, 0.25, 0.125};
  int mapScaleLevel = 0;
  float mapScale = 1.0;

  boolean showTelemetry = false;
  boolean trackRobot = false;
  int simulatorMode = SANDBOX;
  boolean liveArmed = false;
  boolean liveRunning = false;
  int previousTcpSignal = 0;
  long liveStartTcpMs = -1;
  long liveStartSimulationMs = -1;
  long lastTcpTimestampMs = -1;

  // Hardware-in-the-loop: PC controller drives the physical robot over TCP.
  boolean hilRunning = false;
  boolean hilEnabled = false;
  long lastHilTelemetryMs = -1;
  long lastHilTimestampMs = -1;
  long hilStartSimMs = 0;
  int hilCommandCount = 0;
  TCPTelemetry_c armedPose = new TCPTelemetry_c();
  TCPTelemetry_c ghost = new TCPTelemetry_c();
  ArrayList<TCPTelemetry_c> tcpQueue = new ArrayList<TCPTelemetry_c>();
  float zoom = 1.0;
  float viewCentreX = 0.0;
  float viewCentreY = 0.0;

  int lastGraphSampleMs = 0;
  int lastSimulationUpdateMs = 0;
  int simulationAccumulatorMs = 0;
  int lastTcpAttemptMs = -tcpRetryPeriodMs;

  Client telemetryClient;
  TCPTelemetry_c tcpTelemetry = new TCPTelemetry_c();
  String tcpStatus = "TCP: not connected";
  String lastTcpLine = "";
  volatile boolean tcpAttemptInProgress = false;
  volatile boolean tcpAttemptFailed = false;
  volatile Socket pendingTcpSocket = null;

  /**
   * @brief Creates the supplied baseline Digital Twin application.
   * @param appletIn Parent Processing application used for networking.
   * @param surfaceOffsetXIn Surface-image x offset in world millimetres.
   * @param surfaceOffsetYIn Surface-image y offset in world millimetres.
   */
  Simulator(PApplet appletIn, float surfaceOffsetXIn, float surfaceOffsetYIn) {
    applet = appletIn;
    surfaceOffsetX = surfaceOffsetXIn;
    surfaceOffsetY = surfaceOffsetYIn;
    ellipseMode(RADIUS);
    strokeJoin(ROUND);
    surfaceImage = loadSurfaceImage();
    originalSurfaceImage = surfaceImage;
    testCardImage = buildToneTestCard(surfaceImage.width, surfaceImage.height);
    robot = new Robot_c(16.0, 90.00, surfaceImage,
      surfaceOffsetX, surfaceOffsetY);
    robot.setSensorModel(1.0, 0.0);
    controller = new Controller_c();
    simReconstructor = new SurfaceReconstructor_c(surfaceImage.width,
      surfaceImage.height, surfaceOffsetX, surfaceOffsetY);
    liveReconstructor = new SurfaceReconstructor_c(surfaceImage.width,
      surfaceImage.height, surfaceOffsetX, surfaceOffsetY);
    lastSimulationUpdateMs = millis();

    motorGraph = new TimeSeriesGraph("Motor PWM", graphWindowMs, true);
    motorGraph.addSeries("Sim L", color(220, 70, 70));
    motorGraph.addSeries("Sim R", color(70, 100, 220));
    motorGraph.addSeries("TCP L", color(145, 60, 190));
    motorGraph.addSeries("TCP R", color(20, 150, 150));
    sensorGraph = new TimeSeriesGraph("Surface sensors", graphWindowMs, false);
    for (int i = 0; i < 5; i++) sensorGraph.addSeries("Sim " + (i + 1), sensorColour(i));
    for (int i = 0; i < 5; i++) sensorGraph.addSeries("TCP " + (i + 1), tcpSensorColour(i));
    encoderGraph = new TimeSeriesGraph("Encoder counts", graphWindowMs, false);
    encoderGraph.addSeries("Sim L", color(220, 70, 70));
    encoderGraph.addSeries("Sim R", color(70, 100, 220));
    encoderGraph.addSeries("TCP L", color(145, 60, 190));
    encoderGraph.addSeries("TCP R", color(20, 150, 150));

    // The sandbox starts scanning immediately, matching the template's
    // automatic start, but now from the fixed raster home pose.
    restartScan();
  }

  /** @brief Updates TCP, advances fixed-step model time, and records graphs. */
  void update() {
    updateTcpConnection();
    updateRobotModel();
    checkScanFinished();
    applyMemoryFade();

    if (trackRobot) {
      viewCentreX = robot.truePose.x;
      viewCentreY = robot.truePose.y;
    }
    recordTelemetry();
  }

  /** @brief Applies the selected fading-memory factor to the active recon. */
  void applyMemoryFade() {
    if (fadeLevel == 0) return;
    long now = millis();
    if (now - lastFadeApplyMs < 100) return;
    lastFadeApplyMs = now;
    SurfaceReconstructor_c recon = activeReconstructor();
    if (recon != null && recon.readingCount > 0) {
      recon.applyFade(fadeLevels[fadeLevel]);
    }
  }

  /**
   * @brief Accumulates elapsed wall-clock time into fixed model updates.
   * @note Long pauses are capped to prevent an unbounded catch-up simulation.
   */
  void updateRobotModel() {
    if (simulatorMode == HIL) {
      // The physical robot is the plant; the controller is stepped from each
      // telemetry record instead of a simulated physics loop.
      lastSimulationUpdateMs = millis();
      simulationAccumulatorMs = 0;
      return;
    }

    int now = millis();
    int elapsedMs = now - lastSimulationUpdateMs;
    lastSimulationUpdateMs = now;

    // Do not try to replay an unbounded amount of time after a breakpoint or
    // long application pause.
    elapsedMs = min(elapsedMs, simulationUpdateIntervalMs * maximumCatchUpSteps);
    simulationAccumulatorMs += elapsedMs;

    int steps = 0;
    while (simulationAccumulatorMs >= simulationUpdateIntervalMs
      && steps < maximumCatchUpSteps) {
      robot.advanceTime(simulationUpdateIntervalMs);
      updateLiveClock();
      controller.update(robot);
      if (simulatorMode == SANDBOX) {
        recordSimReconstruction();
        recordTrail();
      }
      robot.updatePhysics(simulationUpdateIntervalMs);
      simulationAccumulatorMs -= simulationUpdateIntervalMs;
      steps++;
    }
  }

  /** @brief Advances the physical-robot ghost only to matching telemetry time. */
  void updateLiveClock() {
    if (simulatorMode != LIVE_COMPARISON || !liveRunning) return;
    long targetTcpTime = liveStartTcpMs + (robot.getMillis() - liveStartSimulationMs);
    while (tcpQueue.size() > 0 && tcpQueue.get(0).timestampMs <= targetTcpTime) {
      ghost = tcpQueue.remove(0);
    }
  }

  // ---------------------------------------------------------------------------
  // Reconstruction sampling
  // ---------------------------------------------------------------------------

  /**
   * @brief Records one reconstruction sample per sensor from the simulation.
   * @details Uses the reported (odometry) pose, exactly as the physical robot's
   * telemetry does. When the pose-error model is active, that reported pose
   * differs from where the chassis really is, so the reconstruction smears.
   */
  void recordSimReconstruction() {
    if (!controller.isScanning()) return;
    long nowSim = robot.getMillis();
    if (controller.getLastUpdateMs() != nowSim) return;

    RobotPose_c reported = controller.isFusionReady()
      ? controller.getFusedPose() : robot.odometry.pose;
    float[] readings = robot.surface.reading;
    for (int i = 0; i < robot.sensors.length; i++) {
      float wx = robot.sensors[i].worldX(reported.x, reported.y, reported.theta);
      float wy = robot.sensors[i].worldY(reported.x, reported.y, reported.theta);
      simReconstructor.addSample(wx, wy, readings[i]);
    }
    simSpacing.add(reported.x, reported.y);
  }

  /** @brief Records true and fused chassis trails for the world view. */
  void recordTrail() {
    if (!controller.isScanning()) return;
    long nowSim = robot.getMillis();
    if (nowSim - lastTrailMs < 50) return;
    lastTrailMs = nowSim;
    if (trailTrue.size() > 4000) {
      trailTrue.remove(0);
      trailReported.remove(0);
    }
    RobotPose_c reported = controller.isFusionReady()
      ? controller.getFusedPose() : robot.odometry.pose;
    trailTrue.add(new ScanPoint_c(robot.truePose.x, robot.truePose.y));
    trailReported.add(new ScanPoint_c(reported.x, reported.y));
  }

  /** @brief Detects the end of a sandbox scan and appends an experiment row. */
  void checkScanFinished() {
    boolean finished = simulatorMode == SANDBOX && controller.isFinished();
    if (finished && !lastScanFinished) {
      logFinishedRun();
    }
    lastScanFinished = finished;
  }

  /** @brief Renders the surface, overlays, simulated robot and UI. */
  void draw() {

    background(180);
    updateSurfaceViewportCache();

    pushMatrix();
    translate(width / 2.0, height / 2.0);
    scale(zoom, -zoom);
    translate(-viewCentreX, -viewCentreY);
    imageMode(CENTER);
    drawSurfaceViewportCache();
    if (showRasterOverlay) drawReconOverlay();
    controller.drawPlan();
    drawTrails();
    robot.draw();
    if (simulatorMode == LIVE_COMPARISON && ghost.timestampMs >= 0) drawGhost();

    popMatrix();

    drawStatus();
    if (showTelemetry) drawTelemetryPanel();
    if (showRasterPanel) drawRasterPanel();
    if (showHelp) drawHelpOverlay();
  }

  void updateSurfaceViewportCache() {
    // Convert the visible world rectangle to source-image pixels. The image
    // is centred on the configured offset, and its y axis is opposite the
    // world y axis.
    float halfViewWidth = width / (2.0 * zoom);
    float halfViewHeight = height / (2.0 * zoom);
    int visibleX0 = floor(surfaceImage.width / 2.0
      + viewCentreX - surfaceOffsetX - halfViewWidth);
    int visibleX1 = ceil(surfaceImage.width / 2.0
      + viewCentreX - surfaceOffsetX + halfViewWidth);
    int visibleY0 = floor(surfaceImage.height / 2.0
      - viewCentreY + surfaceOffsetY - halfViewHeight);
    int visibleY1 = ceil(surfaceImage.height / 2.0
      - viewCentreY + surfaceOffsetY + halfViewHeight);

    visibleX0 = constrain(visibleX0, 0, surfaceImage.width);
    visibleX1 = constrain(visibleX1, 0, surfaceImage.width);
    visibleY0 = constrain(visibleY0, 0, surfaceImage.height);
    visibleY1 = constrain(visibleY1, 0, surfaceImage.height);
    if (visibleX1 <= visibleX0 || visibleY1 <= visibleY0) {
      surfaceViewportCache = null;
      return;
    }

    boolean cacheContainsView = surfaceViewportCache != null
      && visibleX0 >= surfaceCacheX
      && visibleY0 >= surfaceCacheY
      && visibleX1 <= surfaceCacheX + surfaceCacheWidth
      && visibleY1 <= surfaceCacheY + surfaceCacheHeight;
    if (cacheContainsView) return;

    // The margin lets the robot move or the view pan a little before a new
    // crop is made. It keeps cache creation out of the normal draw path.
    int cacheMargin = ceil(constrain(max(halfViewWidth, halfViewHeight) * 0.15,
      40, 250));
    surfaceCacheX = max(0, visibleX0 - cacheMargin);
    surfaceCacheY = max(0, visibleY0 - cacheMargin);
    int cacheRight = min(surfaceImage.width, visibleX1 + cacheMargin);
    int cacheBottom = min(surfaceImage.height, visibleY1 + cacheMargin);
    surfaceCacheWidth = cacheRight - surfaceCacheX;
    surfaceCacheHeight = cacheBottom - surfaceCacheY;
    surfaceViewportCache = surfaceImage.get(surfaceCacheX, surfaceCacheY,
      surfaceCacheWidth, surfaceCacheHeight);
  }

  void drawSurfaceViewportCache() {
    if (surfaceViewportCache == null) return;

    float worldX = surfaceOffsetX + surfaceCacheX + surfaceCacheWidth / 2.0
      - surfaceImage.width / 2.0;
    float worldY = surfaceOffsetY + surfaceImage.height / 2.0
      - (surfaceCacheY + surfaceCacheHeight / 2.0);
    image(surfaceViewportCache, worldX, worldY);
  }

  /** @brief Draws the reconstructed image in map coordinates, inset on top. */
  void drawReconOverlay() {
    PImage overlay = activeReconstructor().getDisplay(millis());
    if (overlay == null) return;
    pushStyle();
    tint(255, overlayAlphas[overlayAlphaLevel]);
    imageMode(CENTER);
    image(overlay, surfaceOffsetX, surfaceOffsetY);
    noTint();
    popStyle();
  }

  /** @brief Draws true (orange) and reported (blue) chassis trails. */
  void drawTrails() {
    if (trailTrue.size() < 2) return;
    pushStyle();
    noFill();
    strokeWeight(1.5);
    stroke(40, 100, 220, 120);
    beginShape();
    for (ScanPoint_c p : trailReported) vertex(p.x, p.y);
    endShape();
    stroke(235, 125, 30, 175);
    beginShape();
    for (ScanPoint_c p : trailTrue) vertex(p.x, p.y);
    endShape();
    popStyle();
  }

  void drawGhost() {
    float alpha = millis() - ghost.receivedAtMs > 100 ? 128 : 255;
    pushStyle(); pushMatrix();
    translate(ghost.x, ghost.y); rotate(ghost.theta);
    fill(235, 125, 35, alpha); stroke(180, 70, 15, alpha);
    ellipse(0, 0, 50, 50); line(0, 0, 36, 0);
    popMatrix(); popStyle();
  }

  // ---------------------------------------------------------------------------
  // Input
  // ---------------------------------------------------------------------------

  /**
   * @brief Handles the documented keyboard controls.
   * @param pressedKey Character key reported by Processing.
   * @param pressedKeyCode Processing key code for arrow-key panning.
   */
  void handleKey(char pressedKey, int pressedKeyCode) {
    if (pressedKey == 'g' || pressedKey == 'G') showTelemetry = !showTelemetry;
    if (pressedKey == 't' || pressedKey == 'T') trackRobot = !trackRobot;
    if (pressedKey == 'm' || pressedKey == 'M') {
      if (!(simulatorMode == LIVE_COMPARISON && liveRunning)) cycleMode();
    }
    if (pressedKey == 'r' || pressedKey == 'R') showRasterPanel = !showRasterPanel;
    if (pressedKey == 'v' || pressedKey == 'V') showRasterOverlay = !showRasterOverlay;
    if (pressedKey == 'h' || pressedKey == 'H') showHelp = !showHelp;
    if (pressedKey == 'i' || pressedKey == 'I') cyclePlacement();
    if (pressedKey == 'f' || pressedKey == 'F') toggleGapFill();
    if (pressedKey == 'n' || pressedKey == 'N') cycleNoise();
    if (pressedKey == 'p' || pressedKey == 'P') cyclePoseError();
    if (pressedKey == 'u' || pressedKey == 'U') toggleFusion();
    if (pressedKey == 'c' || pressedKey == 'C') {
      controller.cycleControlMode();
      println("Control law " + controller.getControlName());
      restartScan();
    }
    if (pressedKey == 'j' || pressedKey == 'J') {
      controller.setFusionMethod((controller.getFusionMethod() + 1) % 3);
      println("Fusion method " + controller.getFusionName());
      restartScan();
    }
    if (pressedKey == 'k' || pressedKey == 'K') {
      controller.setJacobianMode((controller.getJacobianMode() + 1) % 3);
      println("EKF Jacobian " + controller.getJacobianMode());
      restartScan();
    }
    if (pressedKey == 'b' || pressedKey == 'B') {
      overlayAlphaLevel = (overlayAlphaLevel + 1) % overlayAlphas.length;
    }
    if (pressedKey == 'y' || pressedKey == 'Y') {
      fadeLevel = (fadeLevel + 1) % fadeLevels.length;
    }
    if (pressedKey == 'z' || pressedKey == 'Z') cycleMapScale();
    if (pressedKey == '0') toggleSurfaceMap();
    if (pressedKey == '8') saveReconstruction();
    if (pressedKey == '9') saveTestCard();
    if (pressedKey == ' ') {
      if (simulatorMode == HIL) startHil();
      else restartScan();
    }
    if (pressedKey == '[') changeSpacing(-4.0);
    if (pressedKey == ']') changeSpacing(4.0);
    if (pressedKey == ',') changeCruise(-5.0);
    if (pressedKey == '.') changeCruise(5.0);
    if (pressedKey == ';') changeFootprint(-0.5);
    if (pressedKey == '\'') changeFootprint(0.5);
    if (pressedKey == '+' || pressedKey == '=') zoom = min(maximumZoom, zoom * 1.2);
    if (pressedKey == '-') zoom = max(minimumZoom, zoom / 1.2);

    if (pressedKey == CODED) {
      // Arrow-key panning intentionally disables tracking so the requested
      // view remains visible after the key press.
      float panStepMm = 50.0 / zoom;
      if (pressedKeyCode == LEFT) panBy(-panStepMm, 0);
      if (pressedKeyCode == RIGHT) panBy(panStepMm, 0);
      if (pressedKeyCode == UP) panBy(0, panStepMm);
      if (pressedKeyCode == DOWN) panBy(0, -panStepMm);
    }
  }

  // ---------------------------------------------------------------------------
  // Raster experiment controls
  // ---------------------------------------------------------------------------

  /** @brief Restarts the sandbox scan from the fixed home pose. */
  void restartScan() {
    if (simulatorMode != SANDBOX) return;
    configureScanTarget();
    robot.resetForRun(SCAN_HOME_X, SCAN_HOME_Y, SCAN_HOME_THETA, 0.0, 0.0);
    controller.reset();
    simReconstructor.clear();
    simSpacing.reset();
    trailTrue.clear();
    trailReported.clear();
    lastTrailMs = -1000;
    scanStartSimMs = robot.getMillis();
    lastScanFinished = false;
    cachedStats = null;
    cropX = -1;
    reconCrop = null;
    controller.beginScan(robot);
  }

  /** @brief Sets the scan target to the whole map, inset from its edges. */
  void configureScanTarget() {
    float minX = surfaceOffsetX - surfaceImage.width / 2.0 + MAP_EDGE_INSET_MM;
    float maxX = surfaceOffsetX + surfaceImage.width / 2.0 - MAP_EDGE_INSET_MM;
    float minY = surfaceOffsetY - surfaceImage.height / 2.0 + MAP_EDGE_INSET_MM;
    float maxY = surfaceOffsetY + surfaceImage.height / 2.0 - MAP_EDGE_INSET_MM;
    controller.setScanTarget(minX, minY, maxX, maxY);
  }

  void changeSpacing(float delta) {
    controller.setRequestedLaneSpacingMm(controller.requestedLaneSpacingMm + delta);
    restartScan();
  }

  void changeCruise(float delta) {
    controller.setCruisePwm(controller.cruisePwm + delta);
  }

  void changeFootprint(float delta) {
    float footprint = constrain(robot.sensors[0].footprintRadiusMm + delta, 0.0, 10.0);
    robot.setSensorModel(footprint, robot.sensors[0].noiseSigma);
    restartScan();
  }

  void cycleNoise() {
    noiseLevel = (noiseLevel + 1) % noiseLevels.length;
    robot.setSensorModel(robot.sensors[0].footprintRadiusMm, noiseLevels[noiseLevel]);
    restartScan();
  }

  void cyclePoseError() {
    poseErrorLevel = (poseErrorLevel + 1) % poseErrorLevels.length;
    robot.setPoseErrorModel(poseErrorLevels[poseErrorLevel], poseErrorLevels[poseErrorLevel]);
    restartScan();
  }

  /** @brief Toggles display-only interpolation of unmeasured cells. */
  void toggleGapFill() {
    boolean enabled = !simReconstructor.gapFillEnabled;
    simReconstructor.gapFillEnabled = enabled;
    liveReconstructor.gapFillEnabled = enabled;
    reconCrop = null;
    println("Gap fill " + (enabled ? "on" : "off"));
  }

  /** @brief Toggles IMU/encoder/PWM fusion and restarts for a clean run. */
  void toggleFusion() {
    controller.setFusionEnabled(!controller.isFusionEnabled());
    println("Sensor fusion " + (controller.isFusionEnabled() ? "on" : "off"));
    restartScan();
  }

  void cyclePlacement() {
    int next = (simReconstructor.splatMode + 1) % 3;
    simReconstructor.setPlacement(next);
    liveReconstructor.setPlacement(next);
    cachedStats = null;
    restartScan();
  }

  /** @brief Switches between the supplied map and the generated test card. */
  void toggleSurfaceMap() {
    if (simulatorMode != SANDBOX) return;
    usingTestCard = !usingTestCard;
    applySurfaceMap();
    restartScan();
  }

  /** @brief Applies the active base map and content scale to the robot. */
  void applySurfaceMap() {
    PImage base = usingTestCard ? testCardImage : originalSurfaceImage;
    surfaceImage = (mapScale < 0.999) ? scaledMap(base, mapScale) : base;
    robot.setSurfaceImage(surfaceImage);
    surfaceViewportCache = null;
    surfaceCacheX = -1;
    cachedStats = null;
    cropX = -1;
    truthCrop = null;
    reconCrop = null;
  }

  /** @brief Cycles the map content scale to test the minimum feature size. */
  void cycleMapScale() {
    if (simulatorMode != SANDBOX) return;
    mapScaleLevel = (mapScaleLevel + 1) % mapScales.length;
    mapScale = mapScales[mapScaleLevel];
    applySurfaceMap();
    restartScan();
  }

  /**
   * @brief Resamples a map onto a white 420 x 297 canvas at a smaller scale.
   * @details The robot still scans the same physical area, but the printed
   * features become smaller relative to the fixed sensor footprint. This is
   * the simulation counterpart of printing the map at a reduced scale.
   */
  PImage scaledMap(PImage source, float scale) {
    PGraphics graphics = createGraphics(source.width, source.height);
    graphics.beginDraw();
    graphics.background(255);
    graphics.smooth();
    graphics.imageMode(CENTER);
    int scaledWidth = max(1, round(source.width * scale));
    int scaledHeight = max(1, round(source.height * scale));
    graphics.image(source, source.width / 2.0, source.height / 2.0,
      scaledWidth, scaledHeight);
    graphics.endDraw();
    PImage result = graphics.get();
    graphics.dispose();
    return result;
  }

  /** @brief Saves the active reconstruction as a transparent PNG. */
  void saveReconstruction() {
    SurfaceReconstructor_c recon = activeReconstructor();
    PImage snapshot = recon.snapshot();
    String name = simulatorMode == LIVE_COMPARISON
      ? "reconstruction_live.png" : "reconstruction.png";
    snapshot.save(applet.sketchPath(name));
    println("Saved " + name);
  }

  /** @brief Saves the generated tone test card for printing at 420 x 297 mm. */
  void saveTestCard() {
    testCardImage.save(applet.sketchPath("tone_test_card.png"));
    println("Saved tone_test_card.png (420 x 297 mm at 1 px/mm)");
  }

  // ---------------------------------------------------------------------------
  // Mode switching and live session
  // ---------------------------------------------------------------------------

  /** @brief Cycles sandbox -> live comparison -> HIL -> sandbox. */
  void cycleMode() {
    if (simulatorMode == SANDBOX) {
      simulatorMode = LIVE_COMPARISON;
      resetLiveSession();
      cachedStats = null;
      reconCrop = null;
    } else if (simulatorMode == LIVE_COMPARISON) {
      stopHil();
      simulatorMode = HIL;
      resetLiveSession();
      prepareHil();
    } else {
      stopHil();
      simulatorMode = SANDBOX;
      resetLiveSession();
      cachedStats = null;
      reconCrop = null;
      restartScan();
    }
  }

  /** @brief Enters HIL: the PC controller will command the physical robot. */
  void prepareHil() {
    configureScanTarget();
    robot.setExternalSensors(true);
    controller.setExternalPose(true);
    controller.reset();
    hilRunning = false;
    hilEnabled = true;
    lastHilTelemetryMs = -1;
    lastHilTimestampMs = -1;
    sendHilCommand("HIL,1");
    println("HIL mode: waiting for telemetry; press SPACE to start");
  }

  /** @brief Starts a HIL run from the latest telemetry pose. */
  void startHil() {
    if (simulatorMode != HIL) return;
    if (lastHilTelemetryMs < 0 || millis() - lastHilTelemetryMs > 500) {
      println("HIL: no recent telemetry");
      return;
    }
    configureScanTarget();
    robot.setExternalSensors(true);
    controller.reset();
    controller.setExternalPose(true);
    controller.beginScan(robot);
    liveReconstructor.clear();
    liveSpacing.reset();
    trailTrue.clear();
    trailReported.clear();
    hilStartSimMs = robot.getMillis();
    hilCommandCount = 0;
    hilRunning = true;
    println("HIL run started");
  }

  /** @brief Stops sending commands and returns the firmware to normal mode. */
  void stopHil() {
    if (hilRunning) sendHilCommand("PWM,0,0");
    sendHilCommand("HIL,0");
    hilRunning = false;
    hilEnabled = false;
    robot.setExternalSensors(false);
    controller.setExternalPose(false);
    robot.setMotorPWM(0, 0);
  }

  void sendHilCommand(String command) {
    if (telemetryClient == null) return;
    telemetryClient.write(command + "\n");
  }

  /**
   * @brief Applies one physical telemetry record to the HIL proxy and controller.
   * @details The physical pose and readings become the controller's inputs; the
   * requested PWM is sent back over the same TCP connection. The controller is
   * stepped at the telemetry rate, so the loop closes through the real robot.
   */
  void handleHilSample(TCPTelemetry_c sample) {
    lastHilTelemetryMs = millis();
    if (lastHilTimestampMs >= 0 && sample.timestampMs < lastHilTimestampMs) {
      hilRunning = false;
    }
    lastHilTimestampMs = sample.timestampMs;

    robot.odometry.pose.x = sample.x;
    robot.odometry.pose.y = sample.y;
    robot.odometry.pose.theta = sample.theta;
    robot.truePose.x = sample.x;
    robot.truePose.y = sample.y;
    robot.truePose.theta = sample.theta;
    robot.odometry.encoders.left = sample.encoderLeft;
    robot.odometry.encoders.right = sample.encoderRight;
    robot.simulationMillis = sample.timestampMs;
    robot.motors.left.pwm = sample.pwmLeft;
    robot.motors.right.pwm = sample.pwmRight;
    for (int i = 0; i < 5; i++) {
      robot.surface.reading[i] = sample.sensorReadings[i];
    }

    if (!hilRunning) return;

    for (int i = 0; i < robot.sensors.length; i++) {
      float wx = robot.sensors[i].worldX(sample.x, sample.y, sample.theta);
      float wy = robot.sensors[i].worldY(sample.x, sample.y, sample.theta);
      liveReconstructor.addSample(wx, wy, sample.sensorReadings[i]);
    }
    liveSpacing.add(sample.x, sample.y);
    if (trailReported.size() > 4000) {
      trailReported.remove(0);
      trailTrue.remove(0);
    }
    trailReported.add(new ScanPoint_c(sample.x, sample.y));
    trailTrue.add(new ScanPoint_c(sample.x, sample.y));

    controller.update(robot);
    sendHilCommand("PWM," + round(robot.motors.left.pwm)
      + "," + round(robot.motors.right.pwm));
    hilCommandCount++;
    cachedStats = null;
  }

  void panBy(float deltaX, float deltaY) {
    trackRobot = false;
    viewCentreX += deltaX;
    viewCentreY += deltaY;
  }

  // ---------------------------------------------------------------------------
  // Status and reconstruction panel
  // ---------------------------------------------------------------------------

  void drawStatus() {
    int margin = uiMargin();
    int contentWidth = statusWidth();
    float lineHeight = max(18, 16 * uiScale());

    pushStyle();
    rectMode(CORNER);
    noStroke();
    fill(255, 255, 255, 205);
    rect(margin - 6, margin - 6, contentWidth + 12, lineHeight * 5 + 12, 5);
    fill(30);
    textSize(max(12, 13 * uiScale()));
    text("Robot pose: " + nf(robot.truePose.x, 0, 1) + ", "
      + nf(robot.truePose.y, 0, 1) + ", " + nf(robot.truePose.theta, 0, 2),
      margin, margin + lineHeight, contentWidth, lineHeight);
    text("+/-: zoom   Arrows: pan   T: tracking "
      + (trackRobot ? "on" : "off") + "   G: telemetry   M: mode",
      margin, margin + lineHeight * 2, contentWidth, lineHeight);
    text(tcpStatus, margin, margin + lineHeight * 3, contentWidth, lineHeight);
    text(liveStatus(), margin, margin + lineHeight * 4, contentWidth, lineHeight);
    text("Raster: " + controller.getStateName() + "  "
      + nf(controller.getProgress() * 100.0, 0, 0) + "%   "
      + controller.getControlName() + " / "
      + fusionLabel()
      + "   spacing " + nf(controller.getActualSpacingMm(), 0, 1)
      + " mm   PWM " + nf(controller.cruisePwm, 0, 0), margin,
      margin + lineHeight * 5, contentWidth, lineHeight);
    popStyle();
  }

  String liveStatus() {
    if (simulatorMode == SANDBOX) return "Mode: Sandbox";
    if (simulatorMode == HIL) {
      if (telemetryClient == null) return "Mode: HIL — Offline";
      if (lastHilTelemetryMs < 0 || millis() - lastHilTelemetryMs > 500) {
        return "Mode: HIL — awaiting telemetry";
      }
      return hilRunning
        ? "Mode: HIL — PC controller driving the robot"
        : "Mode: HIL — ready; SPACE starts; M leaves";
    }
    if (telemetryClient == null) return "Mode: Live comparison — Offline";
    if (!liveArmed) return "Mode: Live comparison — Awaiting fresh signal 0";
    if (!liveRunning) return "Mode: Live comparison — Armed";
    return "Mode: Live comparison — Running; ghost = robot-reported odometry";
  }

  /** @brief Draws the truth/reconstruction comparison card. */
  void drawRasterPanel() {
    SurfaceReconstructor_c recon = activeReconstructor();
    int now = millis();
    PImage reconDisplay = recon.getDisplay(now);
    updatePanelCrops(reconDisplay, now);
    ReconStats_c stats = activeStats();

    float marginX = uiMargin();
    float panelWidth = min(constrain(width * 0.44, 320, 500),
      width - 2 * marginX);
    float pad = max(8, 9 * uiScale());
    float thumbWidth = (panelWidth - pad * 3) / 2.0;
    float cropAspect = cropW / (float)max(1, cropH);
    float lineHeight = max(13, 12 * uiScale());
    float labelHeight = max(12, 11 * uiScale());
    float thumbHeight = min(thumbWidth / cropAspect, max(50, height - 300));
    float titleHeight = max(18, 17 * uiScale());
    float panelHeight = pad + titleHeight + 12 + labelHeight + thumbHeight
      + pad + lineHeight * 9 + pad;
    float panelX = marginX;
    float panelY = height - panelHeight - marginX;

    pushStyle();
    rectMode(CORNER);
    noStroke();
    fill(252, 252, 252, 238);
    rect(panelX, panelY, panelWidth, panelHeight, 6);
    fill(30);
    textSize(max(12, 12.5 * uiScale()));
    text("Image rastering — reconstructed surface",
      panelX + pad, panelY + pad + titleHeight - 4);

    float leftX = panelX + pad;
    float rightX = leftX + thumbWidth + pad;
    float labelY = panelY + pad + titleHeight + 12 + labelHeight - 4;
    float imageY = panelY + pad + titleHeight + 12 + labelHeight;

    textSize(max(9, 10 * uiScale()));
    fill(60);
    text("Truth (" + mapName() + ")", leftX, labelY);
    text("Reconstructed", rightX, labelY);

    imageMode(CORNER);
    if (truthCrop != null) {
      image(truthCrop, leftX, imageY, thumbWidth, thumbHeight);
    }
    fill(90);
    noStroke();
    rect(rightX, imageY, thumbWidth, thumbHeight);
    if (reconCrop != null) {
      image(reconCrop, rightX, imageY, thumbWidth, thumbHeight);
    }
    noFill();
    stroke(150);
    rect(leftX, imageY, thumbWidth, thumbHeight);
    rect(rightX, imageY, thumbWidth, thumbHeight);
    noStroke();

    float textY = imageY + thumbHeight + pad + lineHeight;
    fill(30);
    textSize(max(10, 10.5 * uiScale()));

    String coverage = stats.hasData() ? nf(stats.coverage * 100.0, 0, 1) + "%" : "--";
    String rmse = stats.hasData() ? nf(stats.rmse, 0, 1) : "--";
    String mae = stats.hasData() ? nf(stats.mae, 0, 1) : "--";
    text("Measured " + coverage
      + (recon.gapFillEnabled ? " + fill" : "")
      + "   RMSE " + rmse + "   MAE " + mae, leftX, textY);
    textY += lineHeight;

    text("Spacing " + nf(controller.getActualSpacingMm(), 0, 1) + " mm ("
      + controller.getLaneCount() + " lanes)   pitch "
      + nf(activeSpacing().mean(), 0, 2) + " mm", leftX, textY);
    textY += lineHeight;

    text("Footprint " + nf(robot.sensors[2].footprintRadiusMm, 0, 1)
      + " mm   noise " + nf(robot.sensors[2].noiseSigma, 0, 0)
      + " DN   pose error " + poseErrorName(),
      leftX, textY);
    textY += lineHeight;

    String fadeText = "fade " + overlayAlphas[overlayAlphaLevel];
    if (fadeLevel > 0) fadeText += " mem" + nf(fadeLevels[fadeLevel], 0, 3);
    text("Control " + controller.getControlName() + "   fusion "
      + fusionLabel() + "   map " + nf(mapScale * 100.0, 0, 0) + "%   "
      + fadeText, leftX, textY);
    textY += lineHeight;

    text("Placement " + recon.placementName()
      + (recon.gapFillEnabled ? " + fill" : "") + "   readings "
      + recon.readingCount + "   speed " + nf(controller.cruisePwm, 0, 0)
      + " PWM", leftX, textY);
    textY += lineHeight;

    text("State " + controller.getStateName() + "   progress "
      + nf(controller.getProgress() * 100.0, 0, 0) + "%   elapsed "
      + nf(scanElapsedSeconds(), 0, 1) + " s", leftX, textY);
    textY += lineHeight;

    textSize(max(9, 9.5 * uiScale()));
    fill(70);
    text("[ ] spacing   , . speed   ; ' footprint   I placement   N noise",
      leftX, textY);
    textY += lineHeight;
    text("P pose   C control   J fusion   K jacobian   U fusion on/off",
      leftX, textY);
    textY += lineHeight;
    text("B fade  Y memory  Z scale  0 map  8/9 save  V overlay  R panel  SPACE  H help",
      leftX, textY);

    popStyle();
  }

  /** @brief Pixel rectangle of the nominal scan area, clamped to the map. */
  int[] scanPixelRect() {
    int[] rect = {0, 0, surfaceImage.width, surfaceImage.height};
    if (!controller.hasPlan()) return rect;
    float[] bounds = controller.getScanAreaBounds();
    int x0 = floor(surfaceImage.width / 2.0 + bounds[0] - surfaceOffsetX);
    int x1 = ceil(surfaceImage.width / 2.0 + bounds[2] - surfaceOffsetX);
    int y0 = floor(surfaceImage.height / 2.0 - bounds[3] + surfaceOffsetY);
    int y1 = ceil(surfaceImage.height / 2.0 - bounds[1] + surfaceOffsetY);
    x0 = constrain(x0, 0, surfaceImage.width - 1);
    x1 = constrain(x1, x0 + 1, surfaceImage.width);
    y0 = constrain(y0, 0, surfaceImage.height - 1);
    y1 = constrain(y1, y0 + 1, surfaceImage.height);
    rect[0] = x0;
    rect[1] = y0;
    rect[2] = x1 - x0;
    rect[3] = y1 - y0;
    return rect;
  }

  /** @brief Caches cropped truth and reconstruction images for the panel. */
  void updatePanelCrops(PImage reconDisplay, int now) {
    int[] rect = scanPixelRect();
    boolean changed = rect[0] != cropX || rect[1] != cropY
      || rect[2] != cropW || rect[3] != cropH;
    if (changed) {
      cropX = rect[0];
      cropY = rect[1];
      cropW = rect[2];
      cropH = rect[3];
      truthCrop = surfaceImage.get(cropX, cropY, cropW, cropH);
      reconCrop = null;
    }
    if (reconDisplay != null && (reconCrop == null
      || now - lastReconCropMs > 200)) {
      reconCrop = reconDisplay.get(cropX, cropY, cropW, cropH);
      lastReconCropMs = now;
    }
  }

  void drawHelpOverlay() {
    String[] lines = {
      "G telemetry    T tracking    M sandbox/live/HIL    +/- zoom    arrows pan",
      "R reconstruction panel    V world overlay    SPACE restart / start HIL",
      "[ ] path spacing (restarts)    , . cruise PWM (applies immediately)",
      "; ' sensor footprint (mm)    I placement    N noise    P pose error",
      "F gap-fill display    U sensor fusion on/off    (fusion restarts)",
      "C control law: open / P / PID / LQR / MPC    J fusion method",
      "K EKF Jacobian: analytic / numerical / identity",
      "B overlay opacity    Y fading memory    Z map content scale",
      "0 toggle surface.png / generated tone test card (restarts)",
      "8 save the current reconstruction PNG   9 save the tone test card PNG",
      "",
      "Experiments: change one factor per run. The panel reports coverage and",
      "tone RMSE against the ideal map. Completed runs are appended to",
      "raster_runs.csv in the sketch folder.",
      "",
      "Footprint and noise describe the sensor. Pose error makes the true",
      "chassis slip while reported odometry stays ideal, so the reconstruction",
      "shows smearing and offset that the digital twin would otherwise hide.",
      "",
      "Spacing, footprint, noise, placement and pose-error changes restart the",
      "sandbox scan so each run uses one consistent setting.",
      "HIL: see README section 8; the firmware stops after 500 ms without PWM.",
      "",
      "H closes this help."
    };

    pushStyle();
    rectMode(CORNER);
    float cardWidth = min(width - 40, 640);
    float cardHeight = min(height - 40, 44 + lines.length * 19);
    float cardX = (width - cardWidth) / 2.0;
    float cardY = (height - cardHeight) / 2.0;
    noStroke();
    fill(25, 25, 25, 226);
    rect(cardX, cardY, cardWidth, cardHeight, 8);
    fill(245);
    textSize(13);
    float textX = cardX + 18;
    float textY = cardY + 28;
    for (int i = 0; i < lines.length; i++) {
      text(lines[i], textX, textY + i * 19);
    }
    popStyle();
  }

  SurfaceReconstructor_c activeReconstructor() {
    return (simulatorMode == LIVE_COMPARISON || simulatorMode == HIL)
      ? liveReconstructor : simReconstructor;
  }

  SampleSpacing_c activeSpacing() {
    return (simulatorMode == LIVE_COMPARISON || simulatorMode == HIL)
      ? liveSpacing : simSpacing;
  }

  ReconStats_c activeStats() {
    int now = millis();
    if (cachedStats == null || now - lastStatsMs > 200) {
      cachedStats = activeReconstructor().compare(surfaceImage, controller);
      lastStatsMs = now;
    }
    return cachedStats;
  }

  float scanElapsedSeconds() {
    if (simulatorMode == HIL) {
      if (!hilRunning) return 0.0;
      return (robot.getMillis() - hilStartSimMs) / 1000.0;
    }
    if (simulatorMode == LIVE_COMPARISON) {
      if (!liveRunning) return 0.0;
      return (robot.getMillis() - liveStartSimulationMs) / 1000.0;
    }
    return (robot.getMillis() - scanStartSimMs) / 1000.0;
  }

  String mapName() {
    return usingTestCard ? "test card" : "surface.png";
  }

  String poseErrorName() {
    if (robot.tractionSlip <= 0.00001 && robot.separationError <= 0.00001) {
      return "off";
    }
    return nf(robot.tractionSlip * 100.0, 0, 0) + "%";
  }

  String fusionLabel() {
    if (controller.isExternalPose()) return "external";
    return controller.isFusionEnabled() ? controller.getFusionName() : "off";
  }

  /** @brief Appends one completed run to raster_runs.csv for later analysis. */
  void logFinishedRun() {
    if (simulatorMode != SANDBOX) return;
    ReconStats_c stats = simReconstructor.compare(surfaceImage, controller);
    float elapsed = (robot.getMillis() - scanStartSimMs) / 1000.0;
    String path = applet.sketchPath("raster_runs.csv");
    boolean newFile = !new java.io.File(path).exists();
    try {
      java.io.FileWriter writer = new java.io.FileWriter(path, true);
      if (newFile) {
        writer.write("map,spacing_mm,lane_count,cruise_pwm,footprint_mm,"
          + "noise_dn,pose_error_pct,fusion,estimator,control,map_scale_pct,"
          + "fade,placement,coverage_pct,rmse,mae,elapsed_s,readings\n");
      }
      writer.write((usingTestCard ? "test_card" : "surface")
        + "," + nf(controller.getActualSpacingMm(), 0, 2)
        + "," + controller.getLaneCount()
        + "," + nf(controller.cruisePwm, 0, 1)
        + "," + nf(robot.sensors[0].footprintRadiusMm, 0, 2)
        + "," + nf(robot.sensors[0].noiseSigma, 0, 1)
        + "," + nf(robot.tractionSlip * 100.0, 0, 1)
        + "," + (controller.isFusionEnabled() ? "on" : "off")
        + "," + fusionLabel()
        + "," + controller.getControlName()
        + "," + nf(mapScale * 100.0, 0, 0)
        + "," + overlayAlphas[overlayAlphaLevel] + "/"
        + nf(fadeLevels[fadeLevel], 0, 3)
        + "," + simReconstructor.placementName()
        + "," + nf(stats.coverage * 100.0, 0, 2)
        + "," + nf(stats.rmse, 0, 2)
        + "," + nf(stats.mae, 0, 2)
        + "," + nf(elapsed, 0, 2)
        + "," + simReconstructor.readingCount + "\n");
      writer.close();
    }
    catch (Exception error) {
      println("Could not write raster_runs.csv: " + error.getMessage());
    }
    // Keep the final image alongside the metrics so a long run is not lost if
    // the window is closed before 8 is pressed.
    simReconstructor.snapshot().save(applet.sketchPath("reconstruction.png"));
  }

  /** @brief Adds current simulation and, when recent, TCP values to graphs. */
  void recordTelemetry() {
    int now = millis();
    if (now - lastGraphSampleMs < graphSamplePeriodMs) return;
    lastGraphSampleMs = now;
    float[] simulatedSensors = robot.sensorReadings();
    if (tcpTelemetry.isRecent(now)) {
      motorGraph.addSample(now, new float[] {robot.motors.left.pwm, robot.motors.right.pwm,
        tcpTelemetry.pwmLeft, tcpTelemetry.pwmRight});
      sensorGraph.addSample(now, combine(simulatedSensors, tcpTelemetry.sensorReadings));
      encoderGraph.addSample(now, new float[] {robot.odometry.encoders.left,
        robot.odometry.encoders.right, tcpTelemetry.encoderLeft, tcpTelemetry.encoderRight});
    } else {
      motorGraph.addSample(now, new float[] {
        robot.motors.left.pwm, robot.motors.right.pwm
      });
      sensorGraph.addSample(now, simulatedSensors);
      encoderGraph.addSample(now, new float[] {
        robot.odometry.encoders.left, robot.odometry.encoders.right
      });
    }
  }

  float[] combine(float[] first, float[] second) {
    float[] combined = new float[first.length + second.length];
    for (int i = 0; i < first.length; i++) combined[i] = first[i];
    for (int i = 0; i < second.length; i++) combined[first.length + i] = second[i];
    return combined;
  }

  void drawTelemetryPanel() {
    int margin = uiMargin();
    int panelWidth = telemetryPanelWidth();
    int panelX = width - panelWidth;
    int graphWidth = panelWidth - 2 * margin;
    float titleHeight = max(28, 24 * uiScale());
    float graphGap = margin;
    float graphHeight = (height - titleHeight - margin * 4 - graphGap * 2) / 3.0;

    // The panel deliberately fills the available height. This keeps graph
    // backgrounds attached to their plots after a resize, including when the
    // window is made smaller than its normal working size.
    graphHeight = max(1, graphHeight);
    float graphY = titleHeight + margin;
    pushStyle();
    rectMode(CORNER);
    noStroke();
    fill(248, 248, 248, 235);
    rect(panelX, 0, panelWidth, height);
    fill(30);
    textSize(max(12, 13 * uiScale()));
    text("Telemetry (press G to hide)", panelX + margin, titleHeight - margin / 2.0);
    motorGraph.draw(panelX + margin, graphY, graphWidth, graphHeight);
    graphY += graphHeight + graphGap;
    sensorGraph.draw(panelX + margin, graphY, graphWidth, graphHeight);
    graphY += graphHeight + graphGap;
    encoderGraph.draw(panelX + margin, graphY, graphWidth, graphHeight);
    popStyle();
  }

  float uiScale() {
    return constrain(min(width / 1000.0, height / 800.0), 0.85, 1.5);
  }

  int uiMargin() {
    return max(10, round(12 * uiScale()));
  }

  int telemetryPanelWidth() {
    int availableWidth = max(180, width - 2 * uiMargin());
    int preferredWidth = round(width * 0.32);
    return min(max(260, preferredWidth), availableWidth);
  }

  int statusWidth() {
    int availableRight = showTelemetry ? width - telemetryPanelWidth() : width;
    return max(1, availableRight - 2 * uiMargin());
  }

  color sensorColour(int index) {
    color[] colours = {color(160, 60, 60), color(210, 120, 50),
      color(80, 150, 80), color(60, 120, 190), color(140, 70, 170)};
    return colours[index];
  }

  color tcpSensorColour(int index) {
    color[] colours = {color(225, 110, 110), color(245, 180, 70),
      color(105, 190, 105), color(80, 160, 230), color(185, 100, 210)};
    return colours[index];
  }

  /**
   * @brief Loads surface.png or creates the supplied A3 circular-line fallback.
   * @return A 1 pixel/mm surface image positioned using the configured offset.
   */
  PImage loadSurfaceImage() {
    PImage image = loadImage("surface.png");
    if (image != null && image.width > 0 && image.height > 0) return image;

    // A3 landscape is 420 x 297 mm. At 1 px/mm this generated test surface
    // has the same physical dimensions as the printed student sheet.
    image = createImage(420, 297, RGB);
    image.loadPixels();
    float circleRadius = 125.0;
    float lineWidth = 19.0;
    for (int py = 0; py < image.height; py++) {
      for (int px = 0; px < image.width; px++) {
        float dx = px - image.width / 2.0;
        float dy = py - image.height / 2.0;
        float distanceFromCentre = sqrt(dx * dx + dy * dy);
        boolean onLine = abs(distanceFromCentre - circleRadius) <= lineWidth / 2.0;
        image.pixels[py * image.width + px] = onLine ? color(0) : color(255);
      }
    }
    image.updatePixels();
    return image;
  }

  /** @brief Maintains the TCP connection and consumes bounded CSV input. */
  void updateTcpConnection() {
    int now = millis();
    if (telemetryClient != null && !telemetryClient.active()) {
      telemetryClient.stop();
      telemetryClient = null;
      tcpStatus = "TCP: disconnected; retrying";
      resetLiveSession();
    }

    if (pendingTcpSocket != null) {
      Socket connectedSocket = pendingTcpSocket;
      pendingTcpSocket = null;
      tcpAttemptInProgress = false;
      try {
        telemetryClient = new Client(applet, connectedSocket);
        tcpStatus = "TCP: connected to " + tcpHost + ":" + tcpPort;
        if (simulatorMode == HIL) sendHilCommand("HIL,1");
      }
      catch (Exception error) {
        telemetryClient = null;
        tcpStatus = "TCP: unavailable; retrying";
        closeSocket(connectedSocket);
      }
    }

    if (tcpAttemptFailed) {
      tcpAttemptFailed = false;
      tcpAttemptInProgress = false;
      tcpStatus = "TCP: unavailable; retrying";
    }

    if (telemetryClient == null && !tcpAttemptInProgress
      && now - lastTcpAttemptMs >= tcpRetryPeriodMs) {
      beginTcpConnectionAttempt();
    }

    int linesRead = 0;
    while (telemetryClient != null && telemetryClient.available() > 0 && linesRead < 20) {
      String line = telemetryClient.readStringUntil('\n');
      if (line == null) break;
      lastTcpLine = trim(line);
      if (tcpTelemetry.parse(lastTcpLine, now)) handleTcpSample(tcpTelemetry);
      linesRead++;
    }
  }

  /**
   * @brief Uses a parsed physical-robot record for graphing and live alignment.
   * @param sample Newly received StampC3 telemetry record.
   */
  void handleTcpSample(TCPTelemetry_c sample) {
    if (lastTcpTimestampMs >= 0 && sample.timestampMs < lastTcpTimestampMs) {
      resetLiveSession();
    }
    lastTcpTimestampMs = sample.timestampMs;
    TCPTelemetry_c copy = sample.copy();
    if (simulatorMode == HIL) {
      handleHilSample(sample);
      return;
    }
    if (simulatorMode != LIVE_COMPARISON) return;

    if (liveRunning) tcpQueue.add(copy);

    if (sample.signal == 0) {
      armedPose = sample.copy();
      liveArmed = true;
    }
    if (liveArmed && !liveRunning && previousTcpSignal == 0 && sample.signal == 1) {
      controller.reset();
      robot.resetForRun(armedPose.x, armedPose.y, armedPose.theta,
        armedPose.encoderLeft, armedPose.encoderRight);
      controller.beginScan(robot);
      liveStartTcpMs = sample.timestampMs;
      liveStartSimulationMs = robot.getMillis();
      ghost = armedPose.copy();
      liveRunning = true;
      liveReconstructor.clear();
      liveSpacing.reset();
      tcpQueue.clear();
    }
    previousTcpSignal = sample.signal;

    // Reconstruct only while the physical run is active, using the pose the
    // robot itself reports and the same nominal sensor geometry.
    if (liveRunning && sample.signal == 1) {
      for (int i = 0; i < robot.sensors.length; i++) {
        float wx = robot.sensors[i].worldX(sample.x, sample.y, sample.theta);
        float wy = robot.sensors[i].worldY(sample.x, sample.y, sample.theta);
        liveReconstructor.addSample(wx, wy, sample.sensorReadings[i]);
      }
      liveSpacing.add(sample.x, sample.y);
      cachedStats = null;
    }
  }

  /** @brief Clears buffered TCP state and disarms the live-comparison run. */
  void resetLiveSession() {
    tcpQueue.clear();
    liveArmed = false;
    liveRunning = false;
    previousTcpSignal = 0;
    lastTcpTimestampMs = -1;
    ghost = new TCPTelemetry_c();
    liveReconstructor.clear();
    liveSpacing.reset();
    cachedStats = null;
    hilRunning = false;
    lastHilTelemetryMs = -1;
    lastHilTimestampMs = -1;
    if (simulatorMode == LIVE_COMPARISON || simulatorMode == HIL) {
      controller.reset();
    }
  }

  /** @brief Starts a short asynchronous connection attempt to the StampC3. */
  void beginTcpConnectionAttempt() {
    lastTcpAttemptMs = millis();
    tcpAttemptInProgress = true;
    tcpStatus = "TCP: connecting to " + tcpHost + ":" + tcpPort;
    new Thread(new Runnable() {
      public void run() {
        connectTcpInBackground();
      }
    }).start();
  }

  void connectTcpInBackground() {
    Socket socket = new Socket();
    try {
      socket.connect(new InetSocketAddress(tcpHost, tcpPort), tcpConnectTimeoutMs);
      pendingTcpSocket = socket;
    }
    catch (Exception error) {
      closeSocket(socket);
      tcpAttemptFailed = true;
    }
  }

  void closeSocket(Socket socket) {
    try {
      socket.close();
    }
    catch (Exception ignored) {
    }
  }
}
