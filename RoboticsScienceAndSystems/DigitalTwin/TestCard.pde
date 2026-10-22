/**
 * @brief Builds an optional tone and resolution test card for sandbox work.
 * @details The card exercises the parts of the reconstruction that the
 * supplied circular-line map cannot: a stepped grey wedge for tone fidelity,
 * a continuous ramp, vertical bar patterns at several pitches for resolution,
 * checkerboards and a slanted edge for sampling artefacts.
 *
 * Dimensions are in millimetres, matching the 1 pixel per millimetre surface
 * convention. The physical robot must be run over the printed equivalent for a
 * like-for-like comparison; this card exists to expose the model's behaviour.
 */
PImage buildToneTestCard(int widthPx, int heightPx) {
  PImage card = createImage(widthPx, heightPx, RGB);
  card.loadPixels();

  for (int i = 0; i < card.pixels.length; i++) {
    card.pixels[i] = color(255);
  }

  // Eight-step grey wedge (black to white), 30 x 50 mm each.
  int wedgeY = 20;
  int wedgeHeight = 50;
  for (int step = 0; step < 8; step++) {
    int value = round(step * 255.0 / 7.0);
    int x0 = 20 + step * 30;
    fillRect(card, x0, wedgeY, x0 + 30, wedgeY + wedgeHeight, value);
  }

  // Continuous horizontal tone ramp.
  for (int y = 90; y < 130; y++) {
    for (int x = 20; x < 300; x++) {
      int value = round(map(x, 20, 299, 0, 255));
      setPixel(card, x, y, value);
    }
  }

  // Vertical bar patterns at 1, 2, 3, 5 and 8 mm periods.
  int[] periods = {2, 4, 6, 10, 16};
  int barsY0 = 160;
  int barsY1 = 230;
  for (int block = 0; block < periods.length; block++) {
    int blockX = 20 + block * 50;
    int period = periods[block];
    for (int y = barsY0; y < barsY1; y++) {
      for (int x = blockX; x < blockX + 40; x++) {
        boolean black = ((x - blockX) / (period / 2)) % 2 == 0;
        setPixel(card, x, y, black ? 0 : 255);
      }
    }
  }

  // Checkerboards at 2 mm and 4 mm squares.
  drawChecker(card, 280, 160, 60, 60, 2);
  drawChecker(card, 280, 230, 60, 60, 4);

  // Slanted high-contrast edge; useful for spotting smearing from pose error.
  for (int y = 160; y < 250; y++) {
    for (int x = 350; x < 410; x++) {
      boolean black = (x - 350) + (y - 160) > 50;
      setPixel(card, x, y, black ? 0 : 255);
    }
  }

  // A coarse dot grid gives a second, sparse feature for visual alignment.
  for (int gy = 0; gy < 4; gy++) {
    for (int gx = 0; gx < 4; gx++) {
      int centreX = 360 + gx * 12;
      int centreY = 100 + gy * 12;
      for (int y = -2; y <= 2; y++) {
        for (int x = -2; x <= 2; x++) {
          setPixel(card, centreX + x, centreY + y, 0);
        }
      }
    }
  }

  card.updatePixels();
  return card;
}

void drawChecker(PImage card, int x0, int y0, int w, int h, int cell) {
  for (int y = y0; y < y0 + h; y++) {
    for (int x = x0; x < x0 + w; x++) {
      int cellX = (x - x0) / cell;
      int cellY = (y - y0) / cell;
      boolean black = ((cellX + cellY) % 2) == 0;
      setPixel(card, x, y, black ? 0 : 255);
    }
  }
}

void fillRect(PImage card, int x0, int y0, int x1, int y1, int value) {
  for (int y = y0; y < y1; y++) {
    for (int x = x0; x < x1; x++) {
      setPixel(card, x, y, value);
    }
  }
}

void setPixel(PImage card, int x, int y, int value) {
  if (x < 0 || x >= card.width || y < 0 || y >= card.height) return;
  card.pixels[y * card.width + x] = color(value);
}
