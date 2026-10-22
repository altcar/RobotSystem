/**
 * @brief Grid reconstruction of the scanned surface from reflectance readings.
 * @details Each reading is a tone in sensor units (400 = white, 2000 = black).
 * The reconstructor converts it back to a 0..255 grey level and accumulates it
 * into a world-aligned grid at 1 cell per millimetre, matching the surface
 * image and the sensor model.
 *
 * Placement mode controls how one reading is written into the grid:
 *  - NEAREST writes to the single nearest cell (visible aliasing);
 *  - BILINEAR splits the reading over the four surrounding cells;
 *  - GAUSSIAN spreads it over a small spot, approximating a soft sensor
 *    footprint and deliberately blurring fine detail.
 *
 * The grid stores a weighted mean, so overlapping passes average rather than
 * overwrite. Cells with no weight are transparent in the display image, which
 * makes uncovered gaps directly visible.
 */
class SurfaceReconstructor_c {
  final int SPLAT_NEAREST = 0;
  final int SPLAT_BILINEAR = 1;
  final int SPLAT_GAUSSIAN = 2;

  final int width;
  final int height;
  final float offsetX;
  final float offsetY;

  final float[] toneSum;
  final float[] weightSum;
  int splatMode = SPLAT_BILINEAR;
  /** @brief Gaussian spot standard deviation used by SPLAT_GAUSSIAN, in mm. */
  float gaussianSigmaMm = 1.5;
  /** @brief Minimum accumulated weight for a cell to count as covered. */
  final float coverageWeightThreshold = 0.20;
  /**
   * @brief Fill unmeasured cells from their measured neighbours for display.
   * @details Only empty cells are changed; measured cells keep their sharp
   * value. This avoids the contrast loss caused by averaging a broad kernel
   * over every cell while still presenting a complete image.
   */
  boolean gapFillEnabled = true;
  float[] fillValues;
  float[] fillScratch;
  long readingCount = 0;

  PImage displayImage;
  boolean dirty = true;
  int lastBuildMs = -1000000;

  SurfaceReconstructor_c(int widthIn, int heightIn, float offsetXIn,
    float offsetYIn) {
    width = widthIn;
    height = heightIn;
    offsetX = offsetXIn;
    offsetY = offsetYIn;
    toneSum = new float[width * height];
    weightSum = new float[width * height];
  }

  /** @brief Discards all accumulated readings. */
  void clear() {
    for (int i = 0; i < toneSum.length; i++) {
      toneSum[i] = 0.0;
      weightSum[i] = 0.0;
    }
    readingCount = 0;
    dirty = true;
    displayImage = null;
  }

  void setPlacement(int mode) {
    splatMode = constrain(mode, SPLAT_NEAREST, SPLAT_GAUSSIAN);
    dirty = true;
  }

  /**
   * @brief Applies exponential forgetting to the accumulated measurements.
   * @param factor Multiplier applied to both tone and weight, per call.
   * @note Scaling both keeps the weighted mean unchanged while reducing the
   * confidence of old cells; the display alpha follows the weight so a cell
   * that has not been revisited visibly fades.
   */
  void applyFade(float factor) {
    if (factor >= 0.9999) return;
    for (int i = 0; i < weightSum.length; i++) {
      toneSum[i] *= factor;
      weightSum[i] *= factor;
    }
    dirty = true;
  }

  String placementName() {
    if (splatMode == SPLAT_NEAREST) return "nearest";
    if (splatMode == SPLAT_GAUSSIAN) return "gaussian";
    return "bilinear";
  }

  /**
   * @brief Adds one sensor reading at a world position.
   * @param worldX Sensor x in mm.
   * @param worldY Sensor y in mm.
   * @param reading Sensor reading in the telemetry's 400..2000 units.
   */
  void addSample(float worldX, float worldY, float reading) {
    float tone = constrain(map(reading, 400, 2000, 255, 0), 0, 255);
    float cx = width / 2.0 + worldX - offsetX;
    float cy = height / 2.0 - worldY + offsetY;

    if (splatMode == SPLAT_NEAREST) {
      addWeight(floor(cx), floor(cy), 1.0, tone);
    } else if (splatMode == SPLAT_BILINEAR) {
      float gx = cx - 0.5;
      float gy = cy - 0.5;
      int x0 = floor(gx);
      int y0 = floor(gy);
      float fx = gx - x0;
      float fy = gy - y0;
      addWeight(x0, y0, (1 - fx) * (1 - fy), tone);
      addWeight(x0 + 1, y0, fx * (1 - fy), tone);
      addWeight(x0, y0 + 1, (1 - fx) * fy, tone);
      addWeight(x0 + 1, y0 + 1, fx * fy, tone);
    } else {
      int radius = ceil(gaussianSigmaMm * 3.0);
      float denominator = 2.0 * gaussianSigmaMm * gaussianSigmaMm;
      for (int dy = -radius; dy <= radius; dy++) {
        for (int dx = -radius; dx <= radius; dx++) {
          float cellX = cx + dx;
          float cellY = cy + dy;
          float offsetFromCentreX = cellX - cx;
          float offsetFromCentreY = cellY - cy;
          float distanceSquared = offsetFromCentreX * offsetFromCentreX
            + offsetFromCentreY * offsetFromCentreY;
          float weight = exp(-distanceSquared / denominator);
          addWeight(floor(cellX), floor(cellY), weight, tone);
        }
      }
    }
    readingCount++;
    dirty = true;
  }

  void addWeight(int pixelX, int pixelY, float weight, float tone) {
    if (weight <= 0.0 || pixelX < 0 || pixelX >= width
      || pixelY < 0 || pixelY >= height) {
      return;
    }
    int index = pixelY * width + pixelX;
    toneSum[index] += tone * weight;
    weightSum[index] += weight;
  }

  /**
   * @brief Returns a display image, rebuilding at most every 100 ms.
   * @details Covered cells are opaque grey; uncovered cells are transparent,
   * so the world overlay shows exactly what the robot has measured.
   */
  PImage getDisplay(int nowMs) {
    if (displayImage == null) {
      displayImage = createImage(width, height, ARGB);
      rebuildDisplay();
      lastBuildMs = nowMs;
      return displayImage;
    }
    if (dirty && nowMs - lastBuildMs >= 200) {
      rebuildDisplay();
      lastBuildMs = nowMs;
    }
    return displayImage;
  }

  /** @brief Forces a rebuild and returns the current display image. */
  PImage snapshot() {
    if (displayImage == null) displayImage = createImage(width, height, ARGB);
    rebuildDisplay();
    return displayImage;
  }

  /**
   * @brief Diffuses measured tones into empty cells without touching them.
   * @details Six passes of an 8-neighbour average fill gaps up to roughly
   * 6 mm. Measured cells are copied unchanged, so the edges of printed
   * features are preserved; only the unmeasured area is inferred.
   */
  void fillEmptyCells() {
    int count = width * height;
    if (fillValues == null || fillValues.length != count) {
      fillValues = new float[count];
      fillScratch = new float[count];
    }

    int empty = 0;
    for (int i = 0; i < count; i++) {
      if (weightSum[i] > 0.001) {
        fillValues[i] = toneSum[i] / weightSum[i];
      } else {
        fillValues[i] = -1.0;
        empty++;
      }
    }
    if (empty == 0) return;

    for (int pass = 0; pass < 6; pass++) {
      boolean changed = false;
      for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
          int index = y * width + x;
          if (fillValues[index] >= 0.0) {
            fillScratch[index] = fillValues[index];
            continue;
          }
          float sum = 0.0;
          int neighbours = 0;
          for (int dy = -1; dy <= 1; dy++) {
            int ny = y + dy;
            if (ny < 0 || ny >= height) continue;
            for (int dx = -1; dx <= 1; dx++) {
              if (dx == 0 && dy == 0) continue;
              int nx = x + dx;
              if (nx < 0 || nx >= width) continue;
              float value = fillValues[ny * width + nx];
              if (value >= 0.0) {
                sum += value;
                neighbours++;
              }
            }
          }
          if (neighbours > 0) {
            fillScratch[index] = sum / neighbours;
            changed = true;
          } else {
            fillScratch[index] = -1.0;
          }
        }
      }
      float[] swap = fillValues;
      fillValues = fillScratch;
      fillScratch = swap;
      if (!changed) break;
    }
  }

  void rebuildDisplay() {
    displayImage.loadPixels();
    if (gapFillEnabled) fillEmptyCells();
    for (int i = 0; i < weightSum.length; i++) {
      boolean measured = weightSum[i] > 0.001;
      float value;
      int alpha;
      if (measured) {
        value = toneSum[i] / weightSum[i];
        // Confidence-weighted opacity: a lightly sampled or faded cell is
        // faint, a repeatedly sampled one is opaque.
        alpha = (int)constrain(round(weightSum[i] * 80.0), 40, 235);
      } else if (gapFillEnabled && fillValues != null && fillValues[i] >= 0.0) {
        value = fillValues[i];
        alpha = 185; // interpolated cells are slightly translucent
      } else {
        displayImage.pixels[i] = color(0, 0, 0, 0);
        continue;
      }
      int level = round(constrain(value, 0, 255));
      displayImage.pixels[i] = color(level, level, level, alpha);
    }
    displayImage.updatePixels();
    dirty = false;
  }

  /**
   * @brief Compares the reconstruction with the ideal map.
   * @param truth Ground-truth surface image (only available in simulation).
   * @param plan Controller supplying the scan rectangle.
   * @return Coverage and tone error over cells inside the scan rectangle.
   * @note Coverage counts every image cell inside the nominal rectangle, so
   * path spacing, turn geometry and pose error all reduce it. RMSE/MAE are
   * computed only over covered cells, in 0..255 grey levels.
   */
  ReconStats_c compare(PImage truth, Controller_c plan) {
    ReconStats_c stats = new ReconStats_c();
    stats.readingCount = readingCount;
    if (!plan.hasPlan()) return stats;

    float[] bounds = plan.getScanAreaBounds();
    int x0 = floor(width / 2.0 + bounds[0] - offsetX) - 1;
    int x1 = ceil(width / 2.0 + bounds[2] - offsetX) + 1;
    int y0 = floor(height / 2.0 - bounds[3] + offsetY) - 1;
    int y1 = ceil(height / 2.0 - bounds[1] + offsetY) + 1;
    x0 = constrain(x0, 0, width - 1);
    x1 = constrain(x1, 0, width - 1);
    y0 = constrain(y0, 0, height - 1);
    y1 = constrain(y1, 0, height - 1);

    if (truth.pixels == null || truth.pixels.length != truth.width * truth.height) {
      truth.loadPixels();
    }

    double squaredError = 0.0;
    double absoluteError = 0.0;
    for (int py = y0; py <= y1; py++) {
      for (int px = x0; px <= x1; px++) {
        float worldX = offsetX - width / 2.0 + (px + 0.5);
        float worldY = offsetY + height / 2.0 - (py + 0.5);
        if (!plan.insideScanArea(worldX, worldY)) continue;

        int index = py * width + px;
        stats.areaCells++;
        if (weightSum[index] < coverageWeightThreshold) continue;

        stats.coveredCells++;
        float truthTone = brightness(truth.pixels[index]);
        float reconTone = toneSum[index] / weightSum[index];
        float error = reconTone - truthTone;
        squaredError += error * error;
        absoluteError += abs(error);
      }
    }

    if (stats.areaCells > 0) {
      stats.coverage = stats.coveredCells / (float)stats.areaCells;
    }
    if (stats.coveredCells > 0) {
      stats.rmse = sqrt((float)(squaredError / stats.coveredCells));
      stats.mae = (float)(absoluteError / stats.coveredCells);
    }
    return stats;
  }
}

/** @brief Coverage and tone-error results for one reconstruction. */
class ReconStats_c {
  int areaCells = 0;
  int coveredCells = 0;
  long readingCount = 0;
  float coverage = 0.0;
  float rmse = 0.0;
  float mae = 0.0;

  boolean hasData() { return coveredCells > 0; }
}

/**
 * @brief Mean path distance between consecutive reported poses.
 * @details Used to report the along-track sample pitch, which grows with
 * speed for a fixed sampling interval.
 */
class SampleSpacing_c {
  float lastX = 0.0, lastY = 0.0;
  float sum = 0.0;
  int count = 0;
  boolean hasLast = false;

  void reset() {
    hasLast = false;
    sum = 0.0;
    count = 0;
  }

  /** @brief Adds one reported pose; implausible jumps restart the estimate. */
  void add(float x, float y) {
    if (hasLast) {
      float distance = dist(lastX, lastY, x, y);
      if (distance < 200.0) {
        sum += distance;
        count++;
      }
    }
    lastX = x;
    lastY = y;
    hasLast = true;
  }

  float mean() {
    return count > 0 ? sum / count : 0.0;
  }
}
