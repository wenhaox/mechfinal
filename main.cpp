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
#define HEADING_MIN_DIST 2  

struct RobotEntry {
  char letter;
  int x;
  int y;
  float heading;
  bool hasPrev;
};

struct HistEntry { 
  char letter; 
  int prevX, prevY; 
  float heading; 
  bool hasPrev; 
};

// =====================================================
// XBee Global Variables
// =====================================================
const char robotId = ROBOT_ID;
bool isRobotRunning = false;
int matchByte = 0;
long gameTime = 0;
int xPos = 0;
int yPos = 0;
RobotEntry robots[MAX_ROBOTS];
int numRobots = 0;

static HistEntry history[MAX_ROBOTS];
static int numHistory = 0;
static int prevMatchByte = 0;
static char rxBuffer[128];
static int rxIndex = 0;
static unsigned long lastRxTime = 0;
static unsigned long validCoords = 0;
static unsigned long totalResponses = 0;

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
const int     TURN_MIN_SPD       = 70;

const int     SIG_PUCK         = 1;
const int     PIXY_CENTER_X    = 158;
const float   PIXY_PX_PER_DEG  = 5.27;
const int     PUCK_CENTER_PX   = 40;
const int     PUCK_EXIT_PX     = 120;

const float   PUCK_KP          = 5.0;
const float   PUCK_KI          = 0.5;
const float   PUCK_KD          = 0.8;
const float   PUCK_I_LIMIT     = 60.0;
const int     PUCK_STOP_CM     = 6;
const int16_t SEARCH_SPEED     = 160;

const int GOAL_A_CX   = 55;
const int GOAL_A_Y    = 3;
const int GOAL_B_CX   = 55;
const int GOAL_B_Y    = 230;
const int FIELD_MID_Y = (GOAL_A_Y + GOAL_B_Y) / 2; 

const int     SHOOT_DIST_FIELD = 30;  
const int16_t SHOOT_SPEED      = 350; 
const int     SHOOT_BURST_MS   = 600;
const int     REAIM_FIELD      = 80;

const int WAYPOINT_DIST = 60;

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

// =====================================================
// XBee Functions (Parsed from broadcast)
// =====================================================
static HistEntry* findHistory(char letter) {
  for (int i = 0; i < numHistory; i++)
    if (history[i].letter == letter) return &history[i];
  if (numHistory < MAX_ROBOTS) {
    HistEntry* e = &history[numHistory++];
    e->letter = letter;
    e->prevX = 0;
    e->prevY = 0;
    e->heading = 0.0;
    e->hasPrev = false;
    return e;
  }
  return nullptr;
}

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
  int mTime = extractDigits(buf, len, pos, 4);
  if (mTime < 0) return false;

  matchByte = mBit;
  gameTime = mTime;

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
    robots[numRobots].heading = 0.0;
    robots[numRobots].hasPrev = false;

    HistEntry* h = findHistory(letter);
    if (h) {
      if (h->hasPrev) {
        int dx = rx - h->prevX;
        int dy = ry - h->prevY;
        if (abs(dx) >= HEADING_MIN_DIST || abs(dy) >= HEADING_MIN_DIST) {
          h->heading = atan2((float)dy, (float)dx) * 180.0 / M_PI;
        }
      }
      h->prevX = rx;
      h->prevY = ry;
      h->hasPrev = true;
      robots[numRobots].heading = h->heading;
      robots[numRobots].hasPrev = h->hasPrev;
    }

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
  totalResponses++;

  if (parseBroadcast(rxBuffer)) {
    validCoords++;
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

int readPingCm() {
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

int16_t driveStraight() {
  int16_t corr = getHeadingCorrection();
  motors.setSpeeds(FORWARD_SPEED - corr, FORWARD_SPEED + corr);
  return corr;
}

void getOpponentGoal(int* gx, int* gy) {
  int refY = startYLocked ? startY : yPos;
  if (refY < FIELD_MID_Y) {
    *gx = GOAL_B_CX;  *gy = GOAL_B_Y;
  } else {
    *gx = GOAL_A_CX;  *gy = GOAL_A_Y;
  }
}

int distanceToPoint(int tx, int ty) {
  long dx = (long)(tx - xPos);
  long dy = (long)(ty - yPos);
  return (int)sqrt((double)(dx*dx + dy*dy));
}

float worldPointToYaw(int tx, int ty) {
  int dx = tx - xPos;
  int dy = ty - yPos;
  int refY = startYLocked ? startY : yPos;
  if (refY >= FIELD_MID_Y) { dx = -dx; dy = -dy; }
  float relYaw = atan2((float)dx, (float)dy) * 180.0 / PI;
  return normalizeAngle(bootYaw + relYaw); 
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
        int offPx = bx - PIXY_CENTER_X;
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

        motors.setSpeeds(FORWARD_SPEED - corr, FORWARD_SPEED + corr);
      } else {
        //Check for the puck in the blind spot
        int dist = readPingCm();
        if (dist > 0 && dist <= (PUCK_STOP_CM + 2)) {
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
    int  dist = readPingCm();

    if (driving && dist > 0 && dist <= PUCK_STOP_CM) {
      stopMotors();
      imuTargetHeading = getYaw();

      Serial.print(F("  at puck, dist=")); Serial.print(dist);
      Serial.print(F(" cm yaw=")); Serial.println(getYaw(), 1);
      return;
    }

    if (!seen) {
      spinSearchByDir(searchDir);
      centered = false;
      driving = false;
      pidInit = false; integral = 0;
      delay(10);
      continue;
    }

    int   offPx = bx - PIXY_CENTER_X;
    float err   = offPx / PIXY_PX_PER_DEG;
    if (offPx > 0)      searchDir = 1;
    else if (offPx < 0) searchDir = -1;

    if (centered) { if (abs(offPx) > PUCK_EXIT_PX) centered = false; }
    else if (abs(offPx) <= PUCK_CENTER_PX) {
      centered = true;
      driving  = true;
      Serial.print(F("ALIGN done, yaw=")); Serial.println(getYaw(), 1);
    }

    if (!driving) {
      int16_t spd = applyFrictionKick(puckPidStep(err, TURN_SPEED, prevErr, integral, prevMs, pidInit));
      motors.setSpeeds(-spd, spd);
    } else {
      int16_t bias = puckPidStep(err, MAX_IMU_CORRECTION, prevErr, integral, prevMs, pidInit);
      motors.setSpeeds(FORWARD_SPEED - bias, FORWARD_SPEED + bias);
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
  Serial.println(F("(place robot facing opponent goal at boot)"));

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

  // ---------------------------------------------------------
  // NEW WAYPOINT LOGIC
  // ---------------------------------------------------------
  int waypointX = gx;
  int waypointY;
  
  if (gy == GOAL_A_Y) {
    waypointY = GOAL_A_Y + WAYPOINT_DIST; 
  } else {
    waypointY = GOAL_B_Y - WAYPOINT_DIST; 
  }

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

  Serial.println(F("PHASE: Smart drive into goal"));
  
  while (true) {
    xbeeUpdate();
    
    // Find breaking zone
    int distToGoal = distanceToPoint(gx, gy);
    
    if (distToGoal <= 20) {
      break; 
    }
    
    driveStraight(); 
    delay(20);
  }

  // Goal break
  motors.setSpeeds(-250, -250);
  delay(80);
  stopMotors();

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
  Serial.print(F(">>> START — bootYaw locked = "));
  Serial.print(bootYaw, 1);
  Serial.println(F(" (this yaw == facing opponent goal)"));

  testDribbleAndShoot();  

  Serial.println(F("Setup done. Loop running."));
}

void loop() {
  xbeeUpdate();

  if (!isRobotRunning) {
    stopMotors();
    delay(50);
    return;
  }
  delay(20);
}