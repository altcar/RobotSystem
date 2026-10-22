/**
 * @brief Selectable pose estimator: complementary filter, EKF or UKF.
 * @details The motion input is the measured wheel distance and the IMU yaw
 * rate; the measurement is the IMU absolute yaw. This is the same information
 * available on the physical robot (encoders, gyro, interface yaw), so the
 * methods can be compared under identical inputs.
 *
 * EKF Jacobian options:
 *  - ANALYTIC: closed-form derivative of the midpoint motion model;
 *  - NUMERICAL: finite-difference linearisation of the same model;
 *  - IDENTITY: no linearisation, intentionally showing the covariance error.
 */
class PoseFilter_c {
  final int COMPLEMENTARY = 0, EKF = 1, UKF = 2;
  final int JAC_ANALYTIC = 0, JAC_NUMERICAL = 1, JAC_IDENTITY = 2;

  int method = EKF;
  int jacobianMode = JAC_ANALYTIC;
  float complementaryGain = 0.20;
  /** @brief Standard deviation of the yaw measurement, radians. */
  float imuYawSigma = 0.0087;
  /** @brief Per-step process noise in x/y, mm. */
  float processXY = 0.15;
  /** @brief Per-step process noise in heading, radians. */
  float processTheta = radians(0.30);

  boolean ready = false;
  RobotPose_c pose = new RobotPose_c();
  float[][] P = new float[3][3];

  // EKF prediction output, reused to avoid per-step allocation.
  float[][] covariance = new float[3][3];

  // UKF parameters. Alpha 1 gives non-negative weights for n = 3.
  final float UKF_ALPHA = 1.0;
  final float UKF_BETA = 2.0;
  final float UKF_KAPPA = 0.0;

  /** @brief Resets the estimate and covariance to a known pose. */
  void reset(float x, float y, float theta) {
    pose.x = x;
    pose.y = y;
    pose.theta = wrapAngle(theta);
    P = identity3();
    P[0][0] = 1.0;
    P[1][1] = 1.0;
    P[2][2] = 0.0001;
    ready = true;
  }

  void reset(RobotPose_c source) {
    reset(source.x, source.y, source.theta);
  }

  /**
   * @brief Advances the filter by one control interval.
   * @param distance Wheel travel since the previous call, mm.
   * @param deltaTheta IMU yaw rate over the interval, radians.
   * @param imuYaw Absolute IMU yaw at the end of the interval, radians.
   */
  void update(float distance, float deltaTheta, float imuYaw) {
    if (!ready) return;
    if (method == COMPLEMENTARY) {
      complementary(distance, deltaTheta, imuYaw);
    } else if (method == UKF) {
      ukf(distance, deltaTheta, imuYaw);
    } else {
      ekf(distance, deltaTheta, imuYaw);
    }
  }

  String methodName() {
    if (method == COMPLEMENTARY) return "comp";
    if (method == UKF) return "ukf";
    return "ekf";
  }

  String jacobianName() {
    if (jacobianMode == JAC_NUMERICAL) return "num";
    if (jacobianMode == JAC_IDENTITY) return "ident";
    return "analytic";
  }

  // ---------------------------------------------------------------------------
  // Complementary filter
  // ---------------------------------------------------------------------------
  void complementary(float distance, float deltaTheta, float imuYaw) {
    float mid = pose.theta + 0.5 * deltaTheta;
    pose.x += distance * cos(mid);
    pose.y += distance * sin(mid);
    float predicted = wrapAngle(pose.theta + deltaTheta);
    pose.theta = wrapAngle(predicted
      + complementaryGain * wrapAngle(imuYaw - predicted));
  }

  // ---------------------------------------------------------------------------
  // Extended Kalman filter
  // ---------------------------------------------------------------------------
  void ekf(float distance, float deltaTheta, float imuYaw) {
    float mid = pose.theta + 0.5 * deltaTheta;
    float predictedX = pose.x + distance * cos(mid);
    float predictedY = pose.y + distance * sin(mid);
    float predictedTheta = wrapAngle(pose.theta + deltaTheta);

    float[][] A = jacobian(distance, deltaTheta);
    float[][] AP = multiply3(A, P);
    P = add3(multiply3(AP, transpose3(A)), processNoise(distance));

    pose.x = predictedX;
    pose.y = predictedY;
    pose.theta = predictedTheta;

    float innovation = wrapAngle(imuYaw - pose.theta);
    float S = P[2][2] + imuYawSigma * imuYawSigma;
    if (S < 1e-9) return;
    float k0 = P[0][2] / S;
    float k1 = P[1][2] / S;
    float k2 = P[2][2] / S;

    pose.x += k0 * innovation;
    pose.y += k1 * innovation;
    pose.theta = wrapAngle(pose.theta + k2 * innovation);

    // P = (I - K H) P with H selecting the heading.
    float[][] updated = new float[3][3];
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
        updated[i][j] = P[i][j] - ((i == 0) ? k0 : (i == 1) ? k1 : k2) * P[2][j];
      }
    }
    P = symmetrize3(updated);
  }

  /** @brief Derivative of the midpoint motion model with respect to state. */
  float[][] jacobian(float distance, float deltaTheta) {
    if (jacobianMode == JAC_IDENTITY) return identity3();

    if (jacobianMode == JAC_NUMERICAL) {
      float[][] result = new float[3][3];
      float[] base = new float[3];
      motion(pose.x, pose.y, pose.theta, distance, deltaTheta, base);
      float[] perturbed = new float[3];
      float[] step = {1e-2, 1e-2, 1e-4};
      for (int j = 0; j < 3; j++) {
        float x = pose.x;
        float y = pose.y;
        float t = pose.theta;
        if (j == 0) x += step[j];
        if (j == 1) y += step[j];
        if (j == 2) t += step[j];
        motion(x, y, t, distance, deltaTheta, perturbed);
        result[0][j] = (perturbed[0] - base[0]) / step[j];
        result[1][j] = (perturbed[1] - base[1]) / step[j];
        result[2][j] = wrapAngle(perturbed[2] - base[2]) / step[j];
      }
      return result;
    }

    float mid = pose.theta + 0.5 * deltaTheta;
    float[][] result = identity3();
    result[0][2] = -distance * sin(mid);
    result[1][2] = distance * cos(mid);
    return result;
  }

  void motion(float x, float y, float theta, float distance,
    float deltaTheta, float[] out) {
    float mid = theta + 0.5 * deltaTheta;
    out[0] = x + distance * cos(mid);
    out[1] = y + distance * sin(mid);
    out[2] = wrapAngle(theta + deltaTheta);
  }

  float[][] processNoise(float distance) {
    float xy = processXY + 0.05 * abs(distance);
    float[][] Q = new float[3][3];
    Q[0][0] = xy * xy;
    Q[1][1] = xy * xy;
    Q[2][2] = processTheta * processTheta;
    return Q;
  }

  // ---------------------------------------------------------------------------
  // Unscented Kalman filter
  // ---------------------------------------------------------------------------
  void ukf(float distance, float deltaTheta, float imuYaw) {
    int n = 3;
    float lambda = UKF_ALPHA * UKF_ALPHA * (n + UKF_KAPPA) - n;
    float denominator = n + lambda;
    if (denominator < 1e-4) denominator = 1e-4;

    float weightMean0 = lambda / denominator;
    float weightCov0 = weightMean0 + (1.0 - UKF_ALPHA * UKF_ALPHA + UKF_BETA);
    float weight = 0.5 / denominator;

    // Sigma points from the Cholesky factor of (n + lambda) P.
    float[][] scaled = scale3(P, denominator);
    float[][] L = cholesky3(scaled);
    float[][] sigma = new float[2 * n + 1][3];
    sigma[0][0] = pose.x; sigma[0][1] = pose.y; sigma[0][2] = pose.theta;
    for (int i = 0; i < n; i++) {
      // The i-th column of a lower-triangular L has zeros above the diagonal.
      float[] column = new float[3];
      for (int k = 0; k < 3; k++) {
        column[k] = (k >= i) ? L[k][i] : 0.0;
      }
      for (int k = 0; k < 3; k++) {
        sigma[1 + i][k] = stateAt(k) + column[k];
        sigma[1 + n + i][k] = stateAt(k) - column[k];
      }
    }

    // Predict every sigma point through the same motion model.
    float[][] predicted = new float[2 * n + 1][3];
    float[] out = new float[3];
    for (int i = 0; i < predicted.length; i++) {
      motion(sigma[i][0], sigma[i][1], sigma[i][2], distance, deltaTheta, out);
      predicted[i][0] = out[0];
      predicted[i][1] = out[1];
      predicted[i][2] = out[2];
    }

    float[] mean = new float[3];
    for (int i = 0; i < predicted.length; i++) {
      float w = (i == 0) ? weightMean0 : weight;
      mean[0] += w * predicted[i][0];
      mean[1] += w * predicted[i][1];
      mean[2] += w * predicted[i][2];
    }
    mean[2] = wrapAngle(mean[2]);

    float[][] cov = new float[3][3];
    for (int i = 0; i < predicted.length; i++) {
      float w = (i == 0) ? weightCov0 : weight;
      float dx = predicted[i][0] - mean[0];
      float dy = predicted[i][1] - mean[1];
      float dt = wrapAngle(predicted[i][2] - mean[2]);
      cov[0][0] += w * dx * dx;
      cov[1][1] += w * dy * dy;
      cov[2][2] += w * dt * dt;
      cov[0][1] += w * dx * dy;
      cov[1][0] += w * dx * dy;
      cov[0][2] += w * dx * dt;
      cov[2][0] += w * dx * dt;
      cov[1][2] += w * dy * dt;
      cov[2][1] += w * dy * dt;
    }
    float[][] Q = processNoise(distance);
    for (int i = 0; i < 3; i++)
      for (int j = 0; j < 3; j++) cov[i][j] += Q[i][j];
    cov = symmetrize3(cov);

    // Heading measurement update.
    float measurementMean = 0.0;
    for (int i = 0; i < predicted.length; i++) {
      float w = (i == 0) ? weightMean0 : weight;
      measurementMean += w * predicted[i][2];
    }
    measurementMean = wrapAngle(measurementMean);

    float S = imuYawSigma * imuYawSigma;
    float[] cross = new float[3];
    for (int i = 0; i < predicted.length; i++) {
      float w = (i == 0) ? weightCov0 : weight;
      float dz = wrapAngle(predicted[i][2] - measurementMean);
      S += w * dz * dz;
      cross[0] += w * (predicted[i][0] - mean[0]) * dz;
      cross[1] += w * (predicted[i][1] - mean[1]) * dz;
      cross[2] += w * (predicted[i][2] - mean[2]) * dz;
    }
    if (S < 1e-8) S = 1e-8;
    float innovation = wrapAngle(imuYaw - measurementMean);
    float K0 = cross[0] / S;
    float K1 = cross[1] / S;
    float K2 = cross[2] / S;

    pose.x = mean[0] + K0 * innovation;
    pose.y = mean[1] + K1 * innovation;
    pose.theta = wrapAngle(mean[2] + K2 * innovation);

    float[][] updated = new float[3][3];
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
        float k = (i == 0) ? K0 : (i == 1) ? K1 : K2;
        updated[i][j] = cov[i][j] - k * S * ((j == 0) ? K0 : (j == 1) ? K1 : K2);
      }
    }
    P = symmetrize3(updated);
  }

  float stateAt(int index) {
    if (index == 0) return pose.x;
    if (index == 1) return pose.y;
    return pose.theta;
  }

  // ---------------------------------------------------------------------------
  // Small fixed-size matrix helpers
  // ---------------------------------------------------------------------------
  float[][] identity3() {
    float[][] result = new float[3][3];
    result[0][0] = 1.0;
    result[1][1] = 1.0;
    result[2][2] = 1.0;
    return result;
  }

  float[][] multiply3(float[][] a, float[][] b) {
    float[][] result = new float[3][3];
    for (int i = 0; i < 3; i++)
      for (int j = 0; j < 3; j++)
        for (int k = 0; k < 3; k++) result[i][j] += a[i][k] * b[k][j];
    return result;
  }

  float[][] transpose3(float[][] a) {
    float[][] result = new float[3][3];
    for (int i = 0; i < 3; i++)
      for (int j = 0; j < 3; j++) result[i][j] = a[j][i];
    return result;
  }

  float[][] add3(float[][] a, float[][] b) {
    float[][] result = new float[3][3];
    for (int i = 0; i < 3; i++)
      for (int j = 0; j < 3; j++) result[i][j] = a[i][j] + b[i][j];
    return result;
  }

  float[][] scale3(float[][] a, float scale) {
    float[][] result = new float[3][3];
    for (int i = 0; i < 3; i++)
      for (int j = 0; j < 3; j++) result[i][j] = a[i][j] * scale;
    return result;
  }

  float[][] symmetrize3(float[][] a) {
    float[][] result = new float[3][3];
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
        result[i][j] = 0.5 * (a[i][j] + a[j][i]);
      }
    }
    return result;
  }

  /** @brief Lower-triangular Cholesky factor; diagonal floors keep it stable. */
  float[][] cholesky3(float[][] a) {
    float[][] L = new float[3][3];
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j <= i; j++) {
        float sum = a[i][j];
        for (int k = 0; k < j; k++) sum -= L[i][k] * L[j][k];
        if (i == j) {
          L[i][i] = sqrt(max(1e-9, sum));
        } else {
          L[i][j] = (abs(L[j][j]) < 1e-9) ? 0.0 : sum / L[j][j];
        }
      }
    }
    return L;
  }

  float wrapAngle(float angle) {
    while (angle > PI) angle -= TWO_PI;
    while (angle < -PI) angle += TWO_PI;
    return angle;
  }
}
