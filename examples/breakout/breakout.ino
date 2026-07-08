// Breakout Demo — EPD_Painter2 edition. Autonomous multi-ball brick basher,
// no paddle required. Balls bounce forever; bricks reset when all cleared.
//
// Port notes vs the original EPD_Painter demo:
//   - No paint() and no full-screen redraws: the sketch draws only DELTAS
//     (erase-old + draw-new per ball, one white rect per killed brick) and
//     the 50Hz simulation tick moves the ink. The original hit ~20fps with
//     blocking paints — the fastest e-paper driver out there. This edition
//     changes the axis of speed: 50 game steps/s of command latency, with
//     every pixel's transition running concurrently instead of per-frame.
//   - Every ball drags a fading comet tail — pixels mid-flight back to
//     white. That is the ink physics rendering motion blur, not a shader.
//   - Composite draws are wrapped in beginUpdate()/endUpdate() so the
//     concurrent tick scan never sees a half-made frame.

// Choose your board (or leave both commented for auto-detect).
//#define EPD_PAINTER2_PRESET_M5PAPER_S3
//#define EPD_PAINTER2_PRESET_LILYGO_T5_S3_GPS

#include <Arduino.h>
#include "EPD_Painter2.h"

EPD_Painter2 epd(EPD_PAINTER2_PRESET);

// --- Game constants ---
#define BALL_SIZE    32
#define BALL_SPEED   12.0f   // px per 20ms tick (~600 px/s, 50 steps/s)
#define NUM_BALLS     3

#define BRICK_COLS    8
#define BRICK_ROWS    6
#define BRICK_PAD     3
#define BRICK_TOP    10
#define BRICK_H      28

// Brick shades by row band (16-grey scale: 0=white .. 15=black)
static const uint8_t kBrickShade[3] = { 4, 9, 15 };

// --- Game state ---
struct Ball {
  float x, y;
  float vx, vy;
  int   px, py;      // last drawn position (-1 = not drawn)
};

struct Brick {
  bool alive;
  uint8_t shade;
};

Ball  balls[NUM_BALLS];
Brick bricks[BRICK_ROWS][BRICK_COLS];

int screenW, screenH;
int brickW;

// Circle chords: for each row of the ball, x-offset and width of the span.
uint8_t chordX[BALL_SIZE], chordW[BALL_SIZE];

void initChords() {
  const float r = BALL_SIZE / 2.0f;
  for (int i = 0; i < BALL_SIZE; i++) {
    float dy = (i + 0.5f) - r;
    float half = sqrtf(r * r - dy * dy);
    int x0 = (int)roundf(r - half);
    int x1 = (int)roundf(r + half);
    chordX[i] = x0;
    chordW[i] = x1 - x0;
  }
}

void brickRect(int r, int c, int &bx, int &by, int &bw, int &bh) {
  bw = brickW;
  bh = BRICK_H;
  bx = BRICK_PAD + c * (brickW + BRICK_PAD);
  by = BRICK_TOP + r * (BRICK_H + BRICK_PAD);
}

void drawBrickField() {
  epd.beginUpdate();
  for (int r = 0; r < BRICK_ROWS; r++) {
    for (int c = 0; c < BRICK_COLS; c++) {
      if (!bricks[r][c].alive) continue;
      int bx, by, bw, bh;
      brickRect(r, c, bx, by, bw, bh);
      epd.fillRect(bx, by, bw, bh, bricks[r][c].shade);
    }
  }
  epd.endUpdate();
}

void initBricks() {
  for (int r = 0; r < BRICK_ROWS; r++)
    for (int c = 0; c < BRICK_COLS; c++)
      bricks[r][c] = { true, kBrickShade[r % 3] };
  drawBrickField();
}

void initBalls() {
  balls[0] = { screenW * 0.25f, screenH - 40.0f,  BALL_SPEED * 0.7f, -BALL_SPEED,        -1, -1 };
  balls[1] = { screenW * 0.50f, screenH - 60.0f, -BALL_SPEED * 0.5f, -BALL_SPEED * 0.9f, -1, -1 };
  balls[2] = { screenW * 0.75f, screenH - 50.0f,  BALL_SPEED * 0.6f, -BALL_SPEED * 0.8f, -1, -1 };
}

// Physics + brick collision. Erases any killed brick (caller holds update).
void updateBall(Ball &ball) {
  ball.x += ball.vx;
  ball.y += ball.vy;

  if (ball.x <= 0)                       { ball.x = 0;                    ball.vx = -ball.vx; }
  else if (ball.x + BALL_SIZE >= screenW){ ball.x = screenW - BALL_SIZE;  ball.vx = -ball.vx; }
  if (ball.y <= 0)                       { ball.y = 0;                    ball.vy = -ball.vy; }
  else if (ball.y + BALL_SIZE >= screenH){ ball.y = screenH - BALL_SIZE;  ball.vy = -ball.vy; }

  for (int r = 0; r < BRICK_ROWS; r++) {
    for (int c = 0; c < BRICK_COLS; c++) {
      if (!bricks[r][c].alive) continue;

      int bx, by, bw, bh;
      brickRect(r, c, bx, by, bw, bh);

      if (ball.x + BALL_SIZE > bx && ball.x < bx + bw &&
          ball.y + BALL_SIZE > by && ball.y < by + bh) {

        bricks[r][c].alive = false;
        epd.fillRect(bx, by, bw, bh, 0);   // brick fades out via the tick

        float overlapLeft   = (ball.x + BALL_SIZE) - bx;
        float overlapRight  = (bx + bw) - ball.x;
        float overlapTop    = (ball.y + BALL_SIZE) - by;
        float overlapBottom = (by + bh) - ball.y;

        if (min(overlapLeft, overlapRight) < min(overlapTop, overlapBottom))
          ball.vx = -ball.vx;
        else
          ball.vy = -ball.vy;

        return;   // one brick per ball per frame
      }
    }
  }
}

void eraseBall(const Ball &ball) {
  if (ball.px < 0) return;
  epd.fillRect(ball.px, ball.py, BALL_SIZE, BALL_SIZE, 0);
}

void drawBall(Ball &ball) {
  int x = (int)ball.x, y = (int)ball.y;
  for (int i = 0; i < BALL_SIZE; i++) {
    epd.fillRect(x + chordX[i], y + i, chordW[i], 1, 15);
  }
  ball.px = x;
  ball.py = y;
}

void setup() {
  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 4000) delay(10);

  if (!epd.begin()) {
    Serial.println("EPD_Painter2 init failed!");
    while (1) delay(1000);
  }

  // 16-grey calibration + dynamic pulse width at 50Hz: ball travel rides
  // full 20ms coarse pulses, brick shades land on 7ms fine pulses with the
  // measured LUT.
  static const uint8_t kGreys[16] =
    { 0, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 16, 26 };
  epd.setPulseWindow(7000);
  epd.setGreyPositions(kGreys);
  epd.setTravelBoost(7);   // one full 20ms pulse ≈ 7 fine positions

  screenW = epd.width();
  screenH = epd.height();
  brickW = (screenW - BRICK_PAD) / BRICK_COLS - BRICK_PAD;

  initChords();
  initBricks();
  initBalls();
  Serial.println("Breakout at 50Hz — no paddle, no mercy.");
}

void loop() {
  // waitVBL(): block until the tick finishes its frame. Everything drawn
  // between here and the next frame is picked up whole — and the game steps
  // exactly once per simulation tick, phase-locked, no timers needed.
  epd.waitFrame();

  epd.beginUpdate();
  for (int i = 0; i < NUM_BALLS; i++) eraseBall(balls[i]);
  for (int i = 0; i < NUM_BALLS; i++) updateBall(balls[i]);
  for (int i = 0; i < NUM_BALLS; i++) drawBall(balls[i]);
  epd.endUpdate();

  // Reset when the wall is gone
  bool anyAlive = false;
  for (int r = 0; r < BRICK_ROWS && !anyAlive; r++)
    for (int c = 0; c < BRICK_COLS && !anyAlive; c++)
      anyAlive = bricks[r][c].alive;
  if (!anyAlive) initBricks();

  static uint32_t lastStats = 0;
  if (millis() - lastStats > 2000) {
    lastStats = millis();
    auto s = epd.getStats();
    Serial.printf("tick=%luus (max %luus) rows=%u\n",
                  (unsigned long)s.lastTickUs, (unsigned long)s.maxTickUs, s.activeRows);
  }

}
