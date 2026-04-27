// JHockey Combined Bringup, Tests, & XBee

#include <Arduino.h>
#include <Pixy2.h>
#include <DualMAX14870MotorShield.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <string.h>
#include <math.h>

// =====================================================
// XBee Configuration & Structs 
// =====================================================
#define MAX_ROBOTS 10
#define ROBOT_ID 'X'        
#define RX_TIMEOUT_MS 5

struct RobotEntry {
  char letter;
  int x;
  int y;
};

// =====================================================
// XBee Global Variables
// =====================================================
bool isRobotRunning = false;
int matchByte = 0;
int xPos = 0;
int yPos = 0;
RobotEntry robots[MAX_ROBOTS];
int numRobots = 0;

static int prevMatchByte = 0;
static char rxBuffer[128];
static int rxIndex = 0;
static unsigned long lastRxTime = 0;

// =====================================================
// Pin Definitions & Hardware Objects
// =====================================================
const int PIN_PING = 40;  // single-wire ultrasonic

Pixy2                   pixy;
DualMAX14870MotorShield motors;
Adafruit_BNO055         bno = Adafruit_BNO055(-1, 0x28, &Wire);

// =====================================================
// Robot Navigation Constants 
// =====================================================
const int16_t FORWARD_SPEED = 250;
const int16_t TURN_SPEED    = 200;

const float   IMU_STRAIGHT_KP      = 3.0;
const float   IMU_STRAIGHT_KI      = 0.1;
const float   IMU_STRAIGHT_KD      = 0.5;
const float   IMU_STRAIGHT_I_LIMIT = 30.0;
const int16_t MAX_IMU_CORRECTION   = 80;

const float   IMU_TURN_KP        = 10.0;
const float   IMU_TURN_KI        = 1.0;
const float   IMU_TURN_KD        = 0.3;
const float   IMU_TURN_I_LIMIT   = 100.0;
const float   TURN_TOLERANCE_DEG = 3.0;
const int     TURN_MIN_SPD       = 80;

const int     SIG_PUCK         = 1;
const int     PIXY_CENTER_X    = 158;
const float   PIXY_PX_PER_DEG  = 5.27;
const int     PUCK_CENTER_PX   = 40;
const int     PUCK_EXIT_PX     = 60;

const float   PUCK_KP          = 4;
const float   PUCK_KI          = 0.5;
const float   PUCK_KD          = 0.8;
const float   PUCK_I_LIMIT     = 60.0;
const float    PUCK_STOP_CM     = 4.0;
const float   APPROACH_HEADING_LIMIT_DEG  = 90.0;
const float   APPROACH_BEHIND_TRIGGER_CM  = 20.0;
const int16_t SEARCH_SPEED     = 160;
const int16_t RECOVERY_BACKUP_SPEED = 220;
const unsigned long BEHIND_PUCK_BACKUP_MS = 180;
const unsigned long TURN_SEARCH_WALL_TIMEOUT_MS = 500;
const float   TURN_SEARCH_MIN_YAW_CHANGE_DEG = 10.0;
const int RESET_WALL_CLEARANCE_MARGIN = 42;

const int GOAL_A_CX   = 55;
const int GOAL_A_Y    = 3;
const int GOAL_B_CX   = 55;
const int GOAL_B_Y    = 230;
const int FIELD_MID_Y = (GOAL_A_Y + GOAL_B_Y) / 2; 
const int FIELD_MIN_X = 0;
const int FIELD_MAX_X = 110;
const int FIELD_MIN_Y = 0;
const int FIELD_MAX_Y = 230;
const int WALL_MARGIN_FIELD = 20;
const int WALL_STUCK_PROGRESS = 3;
const int16_t WALL_STUCK_MIN_CMD = 130;
const unsigned long WALL_STUCK_TIMEOUT_MS = 450;
const unsigned long WALL_BACKUP_MS = 400;

const int     SHOOT_DIST_FIELD = 32;  
const int     GOAL_BRAKE_DIST_FIELD = 20;
const int     GOAL_BRAKE_RELEASE_DIST_FIELD = 24;
const int16_t GOAL_BRAKE_REVERSE_SPEED = 240;
const int     GOAL_BRAKE_REVERSE_MS = 500;
const int     GOAL_BRAKE_SETTLE_MS = 60;
const int16_t SHOOT_SPEED      = 400; 
const int     SHOOT_BURST_MS   = 800;
const int     REAIM_FIELD      = 80;

const int WAYPOINT_DIST = 20;
const int SHOOT_WAYPOINT_X_OFFSET = 22;
const int SHOOT_WAYPOINT_X_MARGIN = 18;
const int RESET_SIDE_X_MARGIN = 24;
const int RESET_WALL_Y_MARGIN = 20;
const int RESET_CORNER_Y_MARGIN = 18;
const int RESET_GOAL_STOP_DIST = 24;

// =====================================================
// Robot Navigation State Variables
// =====================================================
float         imuTargetHeading = 0.0;
float         imuPrevError     = 0.0;
float         imuIntegral      = 0.0;
unsigned long imuPrevMs        = 0;
float         bootYaw          = 0.0;

int           startY           = -1;
bool          startYLocked     = false;
int           targetGoalX      = GOAL_B_CX;
int           targetGoalY      = GOAL_B_Y;
int           ownGoalX         = GOAL_A_CX;
int           ownGoalY         = GOAL_A_Y;
bool          goalLocked       = false;

void turnTowardWorldPoint(int tx, int ty);
void driveToWorldPoint(int tx, int ty, int stopDist, bool reaim, bool keepPuck);

// =====================================================
// XBee Functions (Parsed from broadcast)
// =====================================================
static int extractDigits(const char* buf, int len, int& pos, int n) {
  int val = 0;
  for (int i = 0; i < n; i++) {
    if (pos >= len) return -1;
    char c = buf[pos++];
    if (c < '0' || c > '9') return -1;
    val = val * 10 + (c - '0');
  }
  return val;
}

static bool parseBroadcast(const char* buf) {
  int len = strlen(buf);
  if (len < 13) return false;
  if (buf[0] != '>' || buf[len - 1] != ';') return false;

  if (buf[len - 3] < '0' || buf[len - 3] > '9') return false;
  if (buf[len - 2] < '0' || buf[len - 2] > '9') return false;
  int txChk = (buf[len - 3] - '0') * 10 + (buf[len - 2] - '0');

  int calcChk = 0;
  for (int i = 0; i < len - 3; i++) calcChk += (unsigned char)buf[i];
  calcChk += ';';
  calcChk %= 64;
  if (calcChk != txChk) return false;

  int pos = 1;
  int mBit = extractDigits(buf, len, pos, 1);
  if (mBit < 0) return false;
  if (extractDigits(buf, len, pos, 4) < 0) return false;

  matchByte = mBit;

  int dataEnd = len - 3;
  numRobots = 0;

  while (pos + 7 <= dataEnd && numRobots < MAX_ROBOTS) {
    char letter = buf[pos++];
    int rx = extractDigits(buf, len, pos, 3);
    if (rx < 0) return false;
    int ry = extractDigits(buf, len, pos, 3);
    if (ry < 0) return false;
    robots[numRobots].letter = letter;
    robots[numRobots].x = rx;
    robots[numRobots].y = ry;

    numRobots++;

    if (letter == ROBOT_ID) {
      xPos = rx;
      yPos = ry;
    }
  }
  return (numRobots > 0);
}

static void processMessage() {
  if (rxIndex == 0) return;
  rxBuffer[rxIndex] = '\0';

  if (parseBroadcast(rxBuffer)) {
    if (matchByte == 1 && prevMatchByte == 0) {
      isRobotRunning = true;
      Serial.println(F(">>> START SIGNAL <<<"));
    }
    prevMatchByte = matchByte;
  }
  rxIndex = 0;
}

void xbeeSetup() {
  Serial1.begin(115200);   
  delay(200);
  Serial.print(F("XBee ready, Robot ID: "));
  Serial.println((char)ROBOT_ID);
  lastRxTime = millis();
}

void xbeeUpdate() {
  while (Serial1.available()) {
    char c = Serial1.read();
    lastRxTime = millis();

    if (c == ';') {
      if (rxIndex < (int)(sizeof(rxBuffer) - 1)) rxBuffer[rxIndex++] = c;
      processMessage();
    } else if (c == '>') {
      rxIndex = 0;
      rxBuffer[rxIndex++] = c;
    } else {
      if (rxIndex < (int)(sizeof(rxBuffer) - 1))
        rxBuffer[rxIndex++] = c;
      else
        rxIndex = 0;
    }
  }

  if (rxIndex > 0 && (millis() - lastRxTime >= RX_TIMEOUT_MS)) {
    processMessage();
  }
}

// =====================================================
// Core Navigation & Math Helpers 
// =====================================================
float getYaw() {
  sensors_event_t event;
  bno.getEvent(&event, Adafruit_BNO055::VECTOR_EULER);
  return event.orientation.x;
}

float normalizeAngle(float angle) {
  while (angle >  180.0) angle -= 360.0;
  while (angle < -180.0) angle += 360.0;
  return angle;
}

int16_t getHeadingCorrection() {
  float yaw   = getYaw();
  float error = normalizeAngle(imuTargetHeading - yaw);

  unsigned long now = millis();
  float dt = (now - imuPrevMs) / 1000.0;
  if (dt < 0.001) dt = 0.001;

  imuIntegral += error * dt;
  imuIntegral  = constrain(imuIntegral, -IMU_STRAIGHT_I_LIMIT, IMU_STRAIGHT_I_LIMIT);

  float derivative = (error - imuPrevError) / dt;
  imuPrevError = error;
  imuPrevMs    = now;

  float raw = IMU_STRAIGHT_KP * error + IMU_STRAIGHT_KI * imuIntegral + IMU_STRAIGHT_KD * derivative;
  return constrain((int16_t)raw, -MAX_IMU_CORRECTION, MAX_IMU_CORRECTION);
}

void executeTurn(float angleDeg) {
  imuTargetHeading = normalizeAngle(getYaw() + angleDeg);
  float targetHeading = imuTargetHeading;

  Serial.print(F("Turn ")); Serial.print(angleDeg);
  Serial.print(F(" deg -> target ")); Serial.println(targetHeading);

  float prevError    = normalizeAngle(targetHeading - getYaw());
  float turnIntegral = 0.0;
  unsigned long prevMs = millis();

  while (true) {
    float yaw   = getYaw();
    float error = normalizeAngle(targetHeading - yaw);
    if (abs(error) <= TURN_TOLERANCE_DEG) break;

    unsigned long now = millis();
    float dt = (now - prevMs) / 1000.0;
    if (dt < 0.001) dt = 0.001;

    turnIntegral += error * dt;
    turnIntegral  = constrain(turnIntegral, -IMU_TURN_I_LIMIT, IMU_TURN_I_LIMIT);

    float derivative = (error - prevError) / dt;
    prevError = error;
    prevMs    = now;

    float rawSpeed = IMU_TURN_KP * error + IMU_TURN_KI * turnIntegral + IMU_TURN_KD * derivative;
    int16_t spd = constrain((int16_t)rawSpeed, -TURN_SPEED, TURN_SPEED);

    if (spd != 0 && abs(spd) < TURN_MIN_SPD) {
      spd = (spd > 0) ? TURN_MIN_SPD : -TURN_MIN_SPD;
    }

    motors.setSpeeds(-spd, spd);
    delay(10);
  }
  motors.setSpeeds(0, 0);
}

float readPingCm() {
  pinMode(PIN_PING, OUTPUT);
  digitalWrite(PIN_PING, LOW);  delayMicroseconds(5);
  digitalWrite(PIN_PING, HIGH); delayMicroseconds(5);
  digitalWrite(PIN_PING, LOW);

  pinMode(PIN_PING, INPUT);
  unsigned long dur = pulseIn(PIN_PING, HIGH, 30000UL);
  if (dur == 0) return 999;
  int d = (int)(0.034f * (float)dur * 0.5f);
  return (d <= 0) ? 999 : d;
}

bool pixySeePuck(int* blockX = nullptr, long* blockArea = nullptr) {
  pixy.ccc.getBlocks();
  long bestArea = 0;
  int  bestX    = 0;
  bool found    = false;

  for (int i = 0; i < pixy.ccc.numBlocks; i++) {
    if (pixy.ccc.blocks[i].m_signature != SIG_PUCK) continue;
    long area = (long)pixy.ccc.blocks[i].m_width * (long)pixy.ccc.blocks[i].m_height;
    if (area > bestArea) {
      bestArea = area;
      bestX    = pixy.ccc.blocks[i].m_x;
      found    = true;
    }
  }
  if (blockX)    *blockX    = bestX;
  if (blockArea) *blockArea = bestArea;
  return found;
}

void stopMotors() { motors.setSpeeds(0, 0); }

void resetHeadingHold(float targetHeading = NAN) {
  imuTargetHeading = isnan(targetHeading) ? getYaw() : normalizeAngle(targetHeading);
  imuPrevError = 0.0;
  imuIntegral = 0.0;
  imuPrevMs = millis();
}

void driveForMs(int16_t leftCmd, int16_t rightCmd, unsigned long durationMs) {
  unsigned long t0 = millis();
  while (millis() - t0 < durationMs) {
    xbeeUpdate();
    motors.setSpeeds(leftCmd, rightCmd);
    delay(20);
  }
  stopMotors();
}

bool isNearWall() {
  return (xPos <= FIELD_MIN_X + WALL_MARGIN_FIELD) ||
         (xPos >= FIELD_MAX_X - WALL_MARGIN_FIELD) ||
         (yPos <= FIELD_MIN_Y + WALL_MARGIN_FIELD) ||
         (yPos >= FIELD_MAX_Y - WALL_MARGIN_FIELD);
}

bool isNearSideWall() {
  return (xPos <= FIELD_MIN_X + WALL_MARGIN_FIELD) ||
         (xPos >= FIELD_MAX_X - WALL_MARGIN_FIELD);
}

bool maybeRecoverFromWall(int16_t leftCmd, int16_t rightCmd) {
  static bool trackerInit = false;
  static int anchorX = 0;
  static int anchorY = 0;
  static unsigned long anchorMs = 0;

  unsigned long now = millis();
  bool translating = ((long)leftCmd * (long)rightCmd > 0) &&
                     (max(abs(leftCmd), abs(rightCmd)) >= WALL_STUCK_MIN_CMD);

  if (!trackerInit) {
    anchorX = xPos;
    anchorY = yPos;
    anchorMs = now;
    trackerInit = true;
    return false;
  }

  if (!translating || !isNearWall()) {
    anchorX = xPos;
    anchorY = yPos;
    anchorMs = now;
    return false;
  }

  long dx = (long)(xPos - anchorX);
  long dy = (long)(yPos - anchorY);
  int moved = (int)sqrt((double)(dx * dx + dy * dy));
  if (moved >= WALL_STUCK_PROGRESS) {
    anchorX = xPos;
    anchorY = yPos;
    anchorMs = now;
    return false;
  }

  if (now - anchorMs < WALL_STUCK_TIMEOUT_MS) return false;

  Serial.print(F("RECOVERY: wall backup @ ("));
  Serial.print(xPos);
  Serial.print(',');
  Serial.print(yPos);
  Serial.println(')');

  driveForMs(-RECOVERY_BACKUP_SPEED, -RECOVERY_BACKUP_SPEED, WALL_BACKUP_MS);
  resetHeadingHold();

  anchorX = xPos;
  anchorY = yPos;
  anchorMs = millis();
  return true;
}

bool commandDriveWithRecovery(int16_t leftCmd, int16_t rightCmd) {
  if (maybeRecoverFromWall(leftCmd, rightCmd)) return true;
  motors.setSpeeds(leftCmd, rightCmd);
  return false;
}

float getGoalHeadingError() {
  return normalizeAngle(bootYaw - getYaw());
}

bool shouldGoBehindPuck(float distCm) {
  if (distCm <= 0 || distCm > APPROACH_BEHIND_TRIGGER_CM) return false;
  return abs(getGoalHeadingError()) > APPROACH_HEADING_LIMIT_DEG;
}

int getResetWallY() {
  return constrain(yPos, FIELD_MIN_Y + RESET_WALL_Y_MARGIN, FIELD_MAX_Y - RESET_WALL_Y_MARGIN);
}

int getResetCornerY() {
  if (ownGoalY == GOAL_A_Y) return FIELD_MIN_Y + RESET_CORNER_Y_MARGIN;
  return FIELD_MAX_Y - RESET_CORNER_Y_MARGIN;
}

int getResetSideX() {
  if (xPos <= (FIELD_MIN_X + FIELD_MAX_X) / 2) return FIELD_MAX_X - RESET_SIDE_X_MARGIN;
  return FIELD_MIN_X + RESET_SIDE_X_MARGIN;
}

void getResetClearancePoint(int* cx, int* cy) {
  *cx = xPos;
  *cy = yPos;

  if (xPos <= FIELD_MIN_X + WALL_MARGIN_FIELD) *cx = FIELD_MIN_X + RESET_WALL_CLEARANCE_MARGIN;
  else if (xPos >= FIELD_MAX_X - WALL_MARGIN_FIELD) *cx = FIELD_MAX_X - RESET_WALL_CLEARANCE_MARGIN;

  if (yPos <= FIELD_MIN_Y + WALL_MARGIN_FIELD) *cy = FIELD_MIN_Y + RESET_WALL_CLEARANCE_MARGIN;
  else if (yPos >= FIELD_MAX_Y - WALL_MARGIN_FIELD) *cy = FIELD_MAX_Y - RESET_WALL_CLEARANCE_MARGIN;
}

bool maybeRecoverSearchTurnFromWall(int8_t& searchDir) {
  static bool timerInit = false;
  static unsigned long nearWallMs = 0;
  static int8_t lastDir = 0;
  static float startYaw = 0.0f;

  unsigned long now = millis();
  if (!isNearWall()) {
    timerInit = false;
    lastDir = searchDir;
    return false;
  }

  if (!timerInit || lastDir != searchDir) {
    nearWallMs = now;
    timerInit = true;
    lastDir = searchDir;
    startYaw = getYaw();
    return false;
  }

  if (now - nearWallMs < TURN_SEARCH_WALL_TIMEOUT_MS) return false;

  float yawDelta = abs(normalizeAngle(getYaw() - startYaw));
  if (yawDelta >= TURN_SEARCH_MIN_YAW_CHANGE_DEG) {
    nearWallMs = now;
    startYaw = getYaw();
    return false;
  }

  Serial.print(F("RECOVERY: search turn flip @ ("));
  Serial.print(xPos);
  Serial.print(',');
  Serial.print(yPos);
  Serial.println(')');
  Serial.print(F("  yaw change only "));
  Serial.println(yawDelta, 1);

  searchDir = -searchDir;
  nearWallMs = millis();
  lastDir = searchDir;
  startYaw = getYaw();
  return true;
}

void runBadHeadingResetRoute() {
  int resetSideX = getResetSideX();
  int resetWallY = getResetWallY();
  int resetCornerY = getResetCornerY();

  Serial.print(F("APPROACH: heading "));
  Serial.print(getGoalHeadingError(), 1);
  Serial.println(F(" deg, running reset route"));

  driveForMs(-RECOVERY_BACKUP_SPEED, -RECOVERY_BACKUP_SPEED, BEHIND_PUCK_BACKUP_MS);

  if (isNearWall()) {
    int clearX, clearY;
    getResetClearancePoint(&clearX, &clearY);
    Serial.print(F("RESET: clearance waypoint=("));
    Serial.print(clearX);
    Serial.print(',');
    Serial.print(clearY);
    Serial.println(')');
    turnTowardWorldPoint(clearX, clearY);
    driveToWorldPoint(clearX, clearY, 18, /*reaim=*/true, /*keepPuck=*/false);
  }

  Serial.print(F("RESET: wall waypoint=("));
  Serial.print(resetSideX);
  Serial.print(',');
  Serial.print(resetWallY);
  Serial.println(')');
  turnTowardWorldPoint(resetSideX, resetWallY);
  driveToWorldPoint(resetSideX, resetWallY, 25, /*reaim=*/true, /*keepPuck=*/false);

  Serial.print(F("RESET: corner waypoint=("));
  Serial.print(resetSideX);
  Serial.print(',');
  Serial.print(resetCornerY);
  Serial.println(')');
  turnTowardWorldPoint(resetSideX, resetCornerY);
  driveToWorldPoint(resetSideX, resetCornerY, 25, /*reaim=*/true, /*keepPuck=*/false);

  Serial.print(F("RESET: own goal center=("));
  Serial.print(ownGoalX);
  Serial.print(',');
  Serial.print(ownGoalY);
  Serial.println(')');
  turnTowardWorldPoint(ownGoalX, ownGoalY);
  driveToWorldPoint(ownGoalX, ownGoalY, RESET_GOAL_STOP_DIST, /*reaim=*/true, /*keepPuck=*/false);

  Serial.println(F("RESET: re-face shooting goal"));
  turnTowardWorldPoint(targetGoalX, targetGoalY);
  resetHeadingHold(bootYaw);
}

int16_t driveStraight() {
  int16_t corr = getHeadingCorrection();
  commandDriveWithRecovery(FORWARD_SPEED - corr, FORWARD_SPEED + corr);
  return corr;
}

void getOpponentGoal(int* gx, int* gy) {
  *gx = targetGoalX;
  *gy = targetGoalY;
}

bool getOtherRobotPosition(int* ox, int* oy) {
  for (int i = 0; i < numRobots; i++) {
    if (robots[i].letter == ROBOT_ID) continue;
    *ox = robots[i].x;
    *oy = robots[i].y;
    return true;
  }
  return false;
}

void getShotSetupWaypoint(int gx, int gy, int* wx, int* wy) {
  *wy = (gy == GOAL_A_Y) ? (GOAL_A_Y + WAYPOINT_DIST)
                         : (GOAL_B_Y - WAYPOINT_DIST);
  *wx = gx;

  int ox, oy;
  if (!getOtherRobotPosition(&ox, &oy)) return;

  if (ox <= gx) *wx = gx + SHOOT_WAYPOINT_X_OFFSET;
  else          *wx = gx - SHOOT_WAYPOINT_X_OFFSET;
  *wx = constrain(*wx, FIELD_MIN_X + SHOOT_WAYPOINT_X_MARGIN,
                       FIELD_MAX_X - SHOOT_WAYPOINT_X_MARGIN);

  Serial.print(F(">>> opponent=("));
  Serial.print(ox);
  Serial.print(',');
  Serial.print(oy);
  Serial.print(F(") setup waypoint=("));
  Serial.print(*wx);
  Serial.print(',');
  Serial.print(*wy);
  Serial.println(')');
}

int distanceToPoint(int tx, int ty) {
  long dx = (long)(tx - xPos);
  long dy = (long)(ty - yPos);
  return (int)sqrt((double)(dx*dx + dy*dy));
}

void brakeBeforeGoal(int gx, int gy) {
  Serial.println(F("PHASE: brake before goal"));
  stopMotors();
  delay(GOAL_BRAKE_SETTLE_MS);

  unsigned long t0 = millis();
  while (millis() - t0 < GOAL_BRAKE_REVERSE_MS) {
    xbeeUpdate();
    int distToGoal = distanceToPoint(gx, gy);
    if (distToGoal >= GOAL_BRAKE_RELEASE_DIST_FIELD) break;

    motors.setSpeeds(-GOAL_BRAKE_REVERSE_SPEED, -GOAL_BRAKE_REVERSE_SPEED);
    delay(20);
  }
  stopMotors();
}

float worldPointToYaw(int tx, int ty) {
  int dx = tx - xPos;
  int dy = ty - yPos;
  int refY = startYLocked ? startY : yPos;
  if (refY >= FIELD_MID_Y) { dx = -dx; dy = -dy; }
  float relYaw = atan2((float)dx, (float)dy) * 180.0 / PI;
  return normalizeAngle(bootYaw + relYaw); 
}

void lockGoalsFromBootFacing() {
  float errA = abs(normalizeAngle(worldPointToYaw(GOAL_A_CX, GOAL_A_Y) - bootYaw));
  float errB = abs(normalizeAngle(worldPointToYaw(GOAL_B_CX, GOAL_B_Y) - bootYaw));

  if (errA <= errB) {
    targetGoalX = GOAL_A_CX;
    targetGoalY = GOAL_A_Y;
    ownGoalX = GOAL_B_CX;
    ownGoalY = GOAL_B_Y;
  } else {
    targetGoalX = GOAL_B_CX;
    targetGoalY = GOAL_B_Y;
    ownGoalX = GOAL_A_CX;
    ownGoalY = GOAL_A_Y;
  }
  goalLocked = true;

  Serial.print(F(">>> target goal locked = ("));
  Serial.print(targetGoalX);
  Serial.print(',');
  Serial.print(targetGoalY);
  Serial.print(F(") own goal = ("));
  Serial.print(ownGoalX);
  Serial.print(',');
  Serial.print(ownGoalY);
  Serial.println(')');
}

void turnTowardWorldPoint(int tx, int ty) {
  float target = worldPointToYaw(tx, ty);
  float delta  = normalizeAngle(target - getYaw());
  executeTurn(delta);
}

void driveToWorldPoint(int tx, int ty, int stopDist, bool reaim = true, bool keepPuck = false) {
  int anchorX = xPos;
  int anchorY = yPos;
  long reaimDist2 = (long)REAIM_FIELD * REAIM_FIELD;

  float puckPrevErr = 0.0;
  float puckIntegral = 0.0;
  unsigned long puckPrevMs = millis();
  bool puckPidInit = false;
  int8_t puckSearchDir = 1;

  const int16_t PUCK_BIAS_LIMIT = MAX_IMU_CORRECTION / 2;

  while (true) {
    xbeeUpdate();
    if (distanceToPoint(tx, ty) <= stopDist) break;

    if (reaim) {
      long ddx = (long)(xPos - anchorX);
      long ddy = (long)(yPos - anchorY);
      if (ddx*ddx + ddy*ddy >= reaimDist2) {
        stopMotors();
        turnTowardWorldPoint(tx, ty);  
        anchorX = xPos;
        anchorY = yPos;
      }
    }

    if (keepPuck) {
      int bx; long ba;
      bool seen = pixySeePuck(&bx, &ba);
      if (seen) {
        int offPx = PIXY_CENTER_X - bx;
        if (offPx > 0)      puckSearchDir = 1;
        else if (offPx < 0) puckSearchDir = -1;
        float puckErr = offPx / PIXY_PX_PER_DEG;

        unsigned long now = millis();
        float dt = (now - puckPrevMs) / 1000.0;
        if (dt < 0.001) dt = 0.001;
        if (!puckPidInit) { puckPrevErr = puckErr; puckPidInit = true; }

        puckIntegral += puckErr * dt;
        puckIntegral = constrain(puckIntegral, -PUCK_I_LIMIT, PUCK_I_LIMIT);

        float puckDeriv = (puckErr - puckPrevErr) / dt;
        puckPrevErr = puckErr;
        puckPrevMs = now;

        float puckRaw = PUCK_KP * puckErr + PUCK_KI * puckIntegral + PUCK_KD * puckDeriv;
        int16_t puckBias = constrain((int16_t)puckRaw, (int16_t)-PUCK_BIAS_LIMIT, (int16_t) PUCK_BIAS_LIMIT);

        if (abs(puckBias) > 10) {
            imuTargetHeading = getYaw(); 
        }

        int16_t headingCorr = getHeadingCorrection();
        int16_t corr = constrain((int16_t)(headingCorr + puckBias), (int16_t)-MAX_IMU_CORRECTION, (int16_t) MAX_IMU_CORRECTION);
        if (commandDriveWithRecovery(FORWARD_SPEED - corr, FORWARD_SPEED + corr)) {
          puckPidInit = false;
          puckIntegral = 0.0;
          continue;
        }
      } else {
        //Check for the puck in the blind spot
        float dist = readPingCm();
        if (dist > 0 && dist <= (PUCK_STOP_CM)) {
            //Yes, goal for goal
            driveStraight(); 
        } else {
            // No it is gone, find it
            if (puckSearchDir < 0) motors.setSpeeds(SEARCH_SPEED, -SEARCH_SPEED);
            else                   motors.setSpeeds(-SEARCH_SPEED, SEARCH_SPEED);
            puckPidInit = false;
            puckIntegral = 0.0;
        }
      }
    } else {
        driveStraight();
    }
    delay(20);
  }
  stopMotors();
}

// =====================================================
// PID Control & Testing Helpers
// =====================================================
static int16_t applyFrictionKick(int16_t spd) {
  if (spd != 0 && abs(spd) < TURN_MIN_SPD) {
    return (spd > 0) ? TURN_MIN_SPD : -TURN_MIN_SPD;
  }
  return spd;
}

static void spinSearchByDir(int8_t searchDir) {
  if (searchDir < 0) motors.setSpeeds(SEARCH_SPEED, -SEARCH_SPEED);
  else               motors.setSpeeds(-SEARCH_SPEED, SEARCH_SPEED);
}

static int16_t puckPidStep(float error, int16_t outputLim,
                           float& prevErr, float& integral,
                           unsigned long& prevMs, bool& init) {
  unsigned long now = millis();
  float dt = (now - prevMs) / 1000.0;
  if (dt < 0.001) dt = 0.001;

  if (!init) { prevErr = error; init = true; }

  integral += error * dt;
  integral  = constrain(integral, -PUCK_I_LIMIT, PUCK_I_LIMIT);

  float deriv = (error - prevErr) / dt;
  prevErr = error;
  prevMs  = now;

  float raw = PUCK_KP * error + PUCK_KI * integral + PUCK_KD * deriv;
  return constrain((int16_t)raw, (int16_t)-outputLim, (int16_t)outputLim);
}

static void approachPuckBlocking() {
  Serial.println(F("PHASE: approach puck"));
  float prevErr = 0, integral = 0;
  unsigned long prevMs = millis();
  bool pidInit = false;
  bool centered = false;
  bool driving  = false;
  int8_t searchDir = 1;

  while (true) {
    xbeeUpdate();
    int  bx; long ba;
    bool seen = pixySeePuck(&bx, &ba);
    float  dist = readPingCm();

    if (driving && dist > 0 && dist <= PUCK_STOP_CM) {
      stopMotors();
      resetHeadingHold();

      Serial.print(F("  at puck, dist=")); Serial.print(dist);
      Serial.print(F(" cm yaw=")); Serial.println(getYaw(), 1);
      return;
    }

    if (!seen) {
      if (maybeRecoverSearchTurnFromWall(searchDir)) {
        centered = false;
        driving = false;
        pidInit = false; integral = 0;
        delay(10);
        continue;
      }
      spinSearchByDir(searchDir);
      centered = false;
      driving = false;
      pidInit = false; integral = 0;
      delay(10);
      continue;
    }

    int   offPx = PIXY_CENTER_X - bx;
    float err   = offPx / PIXY_PX_PER_DEG;
    if (offPx > 0)      searchDir = 1;
    else if (offPx < 0) searchDir = -1;

    if (centered) { if (abs(offPx) > PUCK_EXIT_PX) centered = false; }
    else if (abs(offPx) <= PUCK_CENTER_PX) {
      centered = true;
      driving  = true;
      Serial.print(F("ALIGN done, yaw=")); Serial.println(getYaw(), 1);
    }

    if (driving && shouldGoBehindPuck(dist)) {
      runBadHeadingResetRoute();
      centered = false;
      driving = false;
      pidInit = false;
      integral = 0.0;
      delay(10);
      continue;
    }

    if (!driving) {
      int16_t spd = applyFrictionKick(puckPidStep(err, TURN_SPEED, prevErr, integral, prevMs, pidInit));
      motors.setSpeeds(-spd, spd);
    } else {
      int16_t bias = puckPidStep(err, MAX_IMU_CORRECTION, prevErr, integral, prevMs, pidInit);
      if (commandDriveWithRecovery(FORWARD_SPEED - bias, FORWARD_SPEED + bias)) {
        centered = false;
        driving = false;
        pidInit = false;
        integral = 0.0;
        delay(10);
        continue;
      }
    }
    delay(10);
  }
}

// =====================================================
// The Target Dribble & Shoot Routine
// =====================================================
// =====================================================
// The Target Dribble & Shoot Routine (WITH WAYPOINT)
// =====================================================
void testDribbleAndShoot() {
  Serial.println(F("TEST: dribble and shoot"));
  Serial.println(F("(place robot facing the goal you want to shoot at)"));

  Serial.println(F("Waiting for XBee position..."));
  while (xPos == 0 && yPos == 0) {
    xbeeUpdate();
    delay(100);
  }

  int gx, gy; 
  getOpponentGoal(&gx, &gy);
  
  Serial.print(F("Start (")); Serial.print(xPos); Serial.print(',');
  Serial.print(yPos); Serial.print(F(")  yaw="));
  Serial.print(getYaw(), 1);
  Serial.print(F("  goal=(")); Serial.print(gx); Serial.print(',');
  Serial.print(gy); Serial.println(')');

  approachPuckBlocking();

  /*
  Serial.println(F("PHASE: secure puck"));
  {
    const unsigned long SECURE_PUCK_MS = 250;
    unsigned long t0 = millis();
    while (millis() - t0 < SECURE_PUCK_MS) {
      xbeeUpdate();
      driveStraight();
      delay(20);
    }
    stopMotors();
    delay(80);
  }
  } */

  // ---------------------------------------------------------
  // NEW WAYPOINT LOGIC
  // ---------------------------------------------------------
  int waypointX, waypointY;
  getShotSetupWaypoint(gx, gy, &waypointX, &waypointY);

  Serial.println(F("PHASE: drive to setup waypoint"));
  turnTowardWorldPoint(waypointX, waypointY);

  driveToWorldPoint(waypointX, waypointY, 35, /*reaim=*/true, /*keepPuck=*/true);

  // ---------------------------------------------------------
  // FINAL SQUARE-UP AND SHOOT
  // ---------------------------------------------------------
  Serial.println(F("PHASE: square up to goal"));
  turnTowardWorldPoint(gx, gy);

  Serial.println(F("PHASE: final drive to goal"));
  driveToWorldPoint(gx, gy, SHOOT_DIST_FIELD, /*reaim=*/true, /*keepPuck=*/true);

  Serial.println(F("PHASE: final re-aim to goal"));
  turnTowardWorldPoint(gx, gy);

  Serial.println(F("PHASE: controlled final push"));
  
  while (true) {
    xbeeUpdate();
    int distToGoal = distanceToPoint(gx, gy);

    if (distToGoal <= GOAL_BRAKE_DIST_FIELD) {
      break; 
    }

    driveStraight(); 
    delay(20);
  }

  brakeBeforeGoal(gx, gy);

  Serial.println(F("TEST: done"));
}

// =====================================================
// Main Setup & Loop
// =====================================================
void setup() {
  Serial.begin(115200);
  delay(500);

  if (!bno.begin()) {
    Serial.println(F("[FAIL] BNO055 not found"));
    while (1) ;
  }
  delay(500);
  bno.setExtCrystalUse(true);
  delay(500);   
  imuPrevMs = millis();
  Serial.print(F("[OK] IMU, current yaw="));
  Serial.println(getYaw(), 1);

  if (pixy.init() >= 0) Serial.println(F("[OK] Pixy"));
  else                  Serial.println(F("[WARN] Pixy not responding"));

  motors.enableDrivers();
  Serial.println(F("[OK] Motors enabled"));

  xbeeSetup();
  Serial.println(F("--- Init complete ---"));

  Serial.println(F("Waiting for start signal (matchByte 0->1)..."));
  {
    unsigned long lastPrint = 0;
    while (!isRobotRunning) {
      xbeeUpdate();
      unsigned long now = millis();
      if (now - lastPrint >= 1000) {
        Serial.println(F("  ... waiting for start ..."));
        lastPrint = now;
      }
      delay(20);
    }
  }

  if (!startYLocked) {
    while (xPos == 0 && yPos == 0) {
      xbeeUpdate();
      delay(20);
    }
    startY = yPos;
    startYLocked = true;
    Serial.print(F(">>> startY locked = "));
    Serial.println(startY);
  }

  bootYaw          = getYaw();
  imuTargetHeading = bootYaw;
  imuPrevMs        = millis();
  lockGoalsFromBootFacing();
  Serial.print(F(">>> START — bootYaw locked = "));
  Serial.print(bootYaw, 1);
  Serial.println(F(" (this yaw == locked shooting direction)"));

  

  Serial.println(F("Setup done. Loop running."));
}

void loop() {
  xbeeUpdate();
  testDribbleAndShoot();  
  if (!isRobotRunning) {
    stopMotors();
    delay(50);
    return;
  }
  delay(20);
}
