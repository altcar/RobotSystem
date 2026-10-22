/** @brief Cached five-channel surface-sensor data exposed to the controller. */
class RobotSurface_c {
  float[] reading = {400, 400, 400, 400, 400};
}

/**
 * @brief Image-sampling model for one downward-facing surface sensor.
 * @details Local coordinates are in mm in the robot frame. The source image
 * is treated as 1 pixel per mm, centred on its configured world offset, with
 * image y inverted relative to world y.
 *
 * The model has three refinements over the template's point sample:
 *  - footprintRadiusMm averages a small disc, approximating the finite spot
 *    seen by a real reflectance sensor;
 *  - noiseSigma adds Gaussian reading noise, in the same 0..2000 units used by
 *    the telemetry;
 *  - the world-position helpers let the reconstruction use exactly the same
 *    sensor geometry without duplicating it.
 */
class SurfaceSensorModel_c {
  String name;
  float localX;
  float localY;
  /** @brief Radius of the sampled disc, in mm. Zero means a point sample. */
  float footprintRadiusMm = 2.5;
  /** @brief Standard deviation of reading noise in sensor units. */
  float noiseSigma = 0.0;

  /**
   * @param nameIn Human-readable channel name, such as DN1.
   * @param radiusMm Sensor distance from the robot centre, in mm.
   * @param angleDegrees Sensor angle in the robot frame, in degrees.
   */
  SurfaceSensorModel_c(String nameIn, float radiusMm, float angleDegrees) {
    name = nameIn;
    localX = radiusMm * cos(radians(angleDegrees));
    localY = radiusMm * sin(radians(angleDegrees));
  }

  /** @return World x of this sensor, in mm. */
  float worldX(float robotX, float robotY, float robotTheta) {
    return robotX + localX * cos(robotTheta) - localY * sin(robotTheta);
  }

  /** @return World y of this sensor, in mm. */
  float worldY(float robotX, float robotY, float robotTheta) {
    return robotY + localX * sin(robotTheta) + localY * cos(robotTheta);
  }

  /**
   * @brief Converts an image brightness sample into a sensor reading.
   * @param image Surface image, centred on the supplied world offset.
   * @param surfaceOffsetX Surface-image x offset in world millimetres.
   * @param surfaceOffsetY Surface-image y offset in world millimetres.
   * @param robotX Robot x position in mm.
   * @param robotY Robot y position in mm.
   * @param robotTheta Robot heading in radians.
   * @return 2000 for black, 400 for white or a location outside the image.
   */
  float sample(PImage image, float surfaceOffsetX, float surfaceOffsetY,
    float robotX, float robotY, float robotTheta) {
    float wx = worldX(robotX, robotY, robotTheta);
    float wy = worldY(robotX, robotY, robotTheta);

    float brightnessValue;
    if (footprintRadiusMm <= 0.01) {
      brightnessValue = brightnessAtWorld(image, surfaceOffsetX, surfaceOffsetY, wx, wy);
    } else {
      brightnessValue = footprintBrightness(image, surfaceOffsetX, surfaceOffsetY, wx, wy);
    }

    float reading = map(brightnessValue, 0, 255, 2000, 400);
    if (noiseSigma > 0.0) {
      reading += randomGaussian() * noiseSigma;
    }
    return constrain(reading, 0, 2000);
  }

  /** @brief Average brightness over nine points inside the sensor spot. */
  float footprintBrightness(PImage image, float surfaceOffsetX,
    float surfaceOffsetY, float worldX, float worldY) {
    float sum = brightnessAtWorld(image, surfaceOffsetX, surfaceOffsetY, worldX, worldY);
    float radius = footprintRadiusMm * 0.75;
    for (int i = 0; i < 8; i++) {
      float angle = i * TWO_PI / 8.0;
      sum += brightnessAtWorld(image, surfaceOffsetX, surfaceOffsetY,
        worldX + cos(angle) * radius, worldY + sin(angle) * radius);
    }
    return sum / 9.0;
  }

  /**
   * @brief Reads brightness at a world position.
   * @return 0..255, or 255 (white) outside the image, matching the template's
   * outer boundary behaviour.
   */
  float brightnessAtWorld(PImage image, float surfaceOffsetX,
    float surfaceOffsetY, float worldX, float worldY) {
    int pixelX = floor(image.width / 2.0 + worldX - surfaceOffsetX);
    int pixelY = floor(image.height / 2.0 - worldY + surfaceOffsetY);
    if (pixelX < 0 || pixelX >= image.width
      || pixelY < 0 || pixelY >= image.height) {
      return 255;
    }
    if (image.pixels == null || image.pixels.length != image.width * image.height) {
      image.loadPixels();
    }
    return brightness(image.pixels[pixelY * image.width + pixelX]);
  }

  /** @brief Draws a small sensor marker in the robot frame.
   * @param reading Current ideal sensor reading, used to set marker shade.
   */
  void draw(float reading) {
    pushStyle();
    fill(map(reading, 400, 2000, 220, 20));
    stroke(20);
    rectMode(CENTER);
    rect(localX, localY, 3, 3);
    popStyle();
  }
}
