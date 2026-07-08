// ARKANOID on EPD_Painter2 — with a real paddle: a USB mouse on the USB-C
// port in OTG host mode.
//
// Hardware: USB-C OTG adapter + any USB mouse. Once the mouse is plugged
// in, the port belongs to it: no serial, and reflashing means unplugging.
// If the mouse doesn't light up, the board isn't sourcing VBUS — use a
// powered OTG hub (or a Y-cable) so the mouse gets its 5V.
//
// The game:
//   - Mouse moves the paddle, left click serves. Paddle english: where the
//     ball meets the paddle sets its angle.
//   - Bricks: top rows are 2-hit silver (they lighten when cracked),
//     lower rows 1-hit shades. Clear the wall to level up (speed rises).
//   - Falling power-up capsules: E = expand paddle, T = triple ball,
//     S = slow. Catch them with the paddle.
//   - 3 lives, score, game over — click to restart.
//
// Driver stack: 50Hz, calibrated 16 greys, travel boost, waitFrame() game
// loop, live deghosting scrubbing the trails while you play.

#define EPD_PAINTER2_PRESET_M5PAPER_S3   // OTG host wiring is board-specific

#include <Arduino.h>
#include "EPD_Painter2_Adafruit.h"
#include "usb_mouse.h"
#include "touch_pad.h"

// The M5PaperS3 doesn't source VBUS, so a bare OTG adapter won't power a
// mouse (a POWERED OTG hub will). Happily the built-in GT911 touch panel
// is the better paddle anyway: slide a finger anywhere, tap to serve.
// Set to 1 if touch X runs opposite to the paddle.
#define TOUCH_FLIP_X 0

EPD_Painter2Adafruit gfx(EPD_PAINTER2_PRESET);

// --- Layout ---------------------------------------------------------------
static const int BRICK_COLS = 12, BRICK_ROWS = 6;
static const int BRICK_TOP = 70, BRICK_H = 26, BRICK_PAD = 4;
static const int BALL = 16;
static const int PAD_H = 12;
static const int HUD_H = 34;

// --- State ----------------------------------------------------------------
struct Ball { float x, y, vx, vy; bool live; int px, py; };
struct Cap  { float x, y; char kind; bool live; int px, py; };

static Ball balls[3];
static Cap  caps[4];
static uint8_t brickHits[BRICK_ROWS][BRICK_COLS];   // 0 = gone
static int W, H, brickW;
static float padX; static int padW = 140, padY, padPX = -1, padPW = 140;
static int lives, level; static long score, lastScore = -1, lastLives = -1;
static float speed;
static bool serving, gameOver;
static uint8_t prevBtn = 0;

static uint8_t brickShade(int r, uint8_t hits) {
  if (r < 2) return hits == 2 ? 11 : 6;   // silver: darkens twice-hit look
  return (uint8_t)(4 + (r % 3) * 4);      // shaded rows: 4/8/12
}

static void drawBrick(int r, int c) {
  const int bx = BRICK_PAD + c * (brickW + BRICK_PAD);
  const int by = BRICK_TOP + r * (BRICK_H + BRICK_PAD);
  if (brickHits[r][c]) gfx.fillRect(bx, by, brickW, BRICK_H, 255 - brickShade(r, brickHits[r][c]) * 17);
  else                 gfx.fillRect(bx, by, brickW, BRICK_H, 0xFF);
}

static void drawWall() {
  gfx.beginUpdate();
  for (int r = 0; r < BRICK_ROWS; r++)
    for (int c = 0; c < BRICK_COLS; c++) drawBrick(r, c);
  gfx.endUpdate();
}

static void buildWall() {
  for (int r = 0; r < BRICK_ROWS; r++)
    for (int c = 0; c < BRICK_COLS; c++) {
      bool present = true;
      if (level % 3 == 1 && ((r + c) & 1)) present = false;        // checker
      if (level % 3 == 2 && (c < 2 || c >= BRICK_COLS - 2)) present = false;
      brickHits[r][c] = present ? (r < 2 ? 2 : 1) : 0;
    }
  drawWall();
}

static void hud() {
  if (score == lastScore && lives == lastLives) return;
  lastScore = score; lastLives = lives;
  gfx.beginUpdate();
  gfx.fillRect(0, 0, W, HUD_H, 0xFF);
  gfx.setTextSize(3);
  gfx.setTextColor(0x00);
  gfx.setCursor(8, 6);
  gfx.print("SCORE ");
  gfx.print(score);
  gfx.setCursor(W - 260, 6);
  gfx.print("LIVES ");
  gfx.print(lives);
  gfx.setCursor(W / 2 - 60, 6);
  gfx.print("LVL ");
  gfx.print(level);
  gfx.endUpdate();
}

static void eraseBallAt(int x, int y) { if (x >= 0) gfx.fillRect(x, y, BALL, BALL, 0xFF); }

static void serveReset() {
  for (auto &b : balls) b.live = false;
  balls[0] = { padX + padW / 2.0f - BALL / 2.0f, (float)(padY - BALL - 1), 0, 0, true, -1, -1 };
  serving = true;
}

static void spawnCap(float x, float y) {
  for (auto &c : caps) {
    if (c.live) continue;
    static const char kinds[3] = { 'E', 'T', 'S' };
    c = { x, y, kinds[(millis() / 7) % 3], true, -1, -1 };
    return;
  }
}

static void applyCap(char k) {
  score += 50;
  if (k == 'E' && padW < 220) padW += 40;
  if (k == 'S') speed = max(3.0f, speed * 0.7f);
  if (k == 'T') {
    for (auto &nb : balls) {
      if (nb.live) continue;
      for (auto &b : balls) if (b.live) {
        nb = b; nb.vx = -b.vx; nb.vy = -fabsf(b.vy); nb.px = -1;
        break;
      }
      break;
    }
  }
}

static void newGame() {
  score = 0; lives = 3; level = 1; speed = 5.0f;
  padW = 140; padX = W / 2.0f - padW / 2;
  lastScore = -1;
  gameOver = false;
  for (auto &c : caps) c.live = false;
  gfx.fillScreen(0xFF);
  buildWall();
  hud();
  serveReset();
}

void setup() {
  Serial.begin(115200);
  delay(200);

  if (!gfx.begin()) { while (1) delay(1000); }
  static const uint8_t kGreys[16] =
    { 0, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 16, 26 };
  gfx.setPulseWindow(7000);
  gfx.setGreyPositions(kGreys);
  gfx.setTravelBoost(7);
  gfx.setWhiteRefresh(1);          // live deghosting under the game

  W = gfx.width(); H = gfx.height();
  padY = H - 40;
  brickW = (W - BRICK_PAD) / BRICK_COLS - BRICK_PAD;

  usbMouseBegin();                 // takes over the USB port — serial ends here
  newGame();
}

static void stepBall(Ball &b) {
  b.x += b.vx; b.y += b.vy;

  if (b.x <= 0)            { b.x = 0;            b.vx = fabsf(b.vx); }
  if (b.x + BALL >= W)     { b.x = W - BALL;     b.vx = -fabsf(b.vx); }
  if (b.y <= HUD_H)        { b.y = HUD_H;        b.vy = fabsf(b.vy); }

  // Paddle: english by contact point.
  if (b.vy > 0 && b.y + BALL >= padY && b.y + BALL <= padY + PAD_H + 8 &&
      b.x + BALL > padX && b.x < padX + padW) {
    const float rel = ((b.x + BALL / 2.0f) - (padX + padW / 2.0f)) / (padW / 2.0f);
    const float ang = rel * 1.05f;               // ±60 degrees
    b.vx = speed * sinf(ang);
    b.vy = -speed * cosf(ang);
    b.y = padY - BALL;
  }

  if (b.y > H) { b.live = false; return; }

  // Bricks.
  for (int r = 0; r < BRICK_ROWS; r++) {
    const int by = BRICK_TOP + r * (BRICK_H + BRICK_PAD);
    if (b.y + BALL <= by || b.y >= by + BRICK_H) continue;
    for (int c = 0; c < BRICK_COLS; c++) {
      if (!brickHits[r][c]) continue;
      const int bx = BRICK_PAD + c * (brickW + BRICK_PAD);
      if (b.x + BALL <= bx || b.x >= bx + brickW) continue;
      brickHits[r][c]--;
      drawBrick(r, c);
      score += brickHits[r][c] ? 5 : (10 + level * 5);
      if (!brickHits[r][c] && ((r * 7 + c * 13 + (int)score) % 6) == 0)
        spawnCap(bx + brickW / 2.0f, (float)(by + BRICK_H));
      const float ol = (b.x + BALL) - bx, orr = (bx + brickW) - b.x;
      const float ot = (b.y + BALL) - by, ob = (by + BRICK_H) - b.y;
      if (min(ol, orr) < min(ot, ob)) b.vx = -b.vx; else b.vy = -b.vy;
      return;
    }
  }
}

void loop() {
  gfx.waitFrame();                 // waitVBL(): draw in the blank, 50Hz lock

  // --- input: touch is absolute and wins; mouse deltas as a bonus ---
  int tx, ty;
  const bool touching =
    touchRead(gfx.driver().getConfig().i2c.wire, tx, ty);
  static bool wasTouching = false;
  const bool tap = touching && !wasTouching;
  wasTouching = touching;

  const int32_t dx = usbMouseTakeDX();
  const uint8_t btn = usbMouseButtons();
  const bool click = ((btn & 1) && !(prevBtn & 1)) || tap;
  prevBtn = btn;

  if (touching) {
#if TOUCH_FLIP_X
    padX = (W - 1 - tx) - padW / 2.0f;
#else
    padX = tx - padW / 2.0f;
#endif
  } else {
    padX += dx * 1.6f;
  }
  if (padX < 0) padX = 0;
  if (padX + padW > W) padX = W - padW;

  if (gameOver) {
    if (click) newGame();
    return;
  }

  gfx.beginUpdate();

  // paddle (redraw on move or resize)
  const int ppx = (int)padX;
  if (ppx != padPX || padW != padPW) {
    if (padPX >= 0) gfx.fillRect(padPX, padY, padPW, PAD_H, 0xFF);
    gfx.fillRect(ppx, padY, padW, PAD_H, 0x00);
    padPX = ppx; padPW = padW;
  }

  // balls
  bool anyLive = false;
  for (auto &b : balls) {
    if (!b.live) continue;
    if (serving) { b.x = padX + padW / 2.0f - BALL / 2.0f; b.y = padY - BALL - 1; }
    else stepBall(b);
    if (!b.live) { eraseBallAt(b.px, b.py); b.px = -1; continue; }
    anyLive = true;
    const int x = (int)b.x, y = (int)b.y;
    if (x != b.px || y != b.py) {
      eraseBallAt(b.px, b.py);
      gfx.fillRect(x, y, BALL, BALL, 0x00);
      b.px = x; b.py = y;
    }
  }
  if (serving && click) {
    serving = false;
    balls[0].vx = speed * 0.35f; balls[0].vy = -speed;
  }

  // capsules
  for (auto &c : caps) {
    if (!c.live) continue;
    c.y += 2.5f;
    const int x = (int)c.x - 12, y = (int)c.y;
    if (c.px >= 0) gfx.fillRect(c.px, c.py, 24, 20, 0xFF);
    if (y + 20 >= padY && x + 24 > padX && x < padX + padW) {
      c.live = false; c.px = -1; applyCap(c.kind); continue;
    }
    if (y > H) { c.live = false; c.px = -1; continue; }
    gfx.fillRect(x, y, 24, 20, 0x60);
    gfx.setTextSize(2); gfx.setTextColor(0xFF);
    gfx.setCursor(x + 7, y + 3); gfx.print(c.kind);
    c.px = x; c.py = y;
  }

  gfx.endUpdate();
  hud();

  // --- life / level flow ---
  if (!anyLive && !serving) {
    if (--lives <= 0) {
      gameOver = true;
      gfx.beginUpdate();
      gfx.setTextSize(6); gfx.setTextColor(0x00);
      gfx.setCursor(W / 2 - 260, H / 2 - 30);
      gfx.print("GAME OVER");
      gfx.setTextSize(2);
      gfx.setCursor(W / 2 - 130, H / 2 + 40);
      gfx.print("click to play again");
      gfx.endUpdate();
    } else {
      serveReset();
    }
  }

  bool wallLeft = false;
  for (int r = 0; r < BRICK_ROWS && !wallLeft; r++)
    for (int c = 0; c < BRICK_COLS && !wallLeft; c++)
      wallLeft = brickHits[r][c] != 0;
  if (!wallLeft) {
    level++; speed *= 1.12f;
    buildWall();
    serveReset();
  }
}
