#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <ESP32Servo.h>
#include "html510.h"
#include "site.h"
#include <stdint.h>
#include <string.h>
#include <queue>

#define MOTOR_ADDR 1
#define VIVE_ADDR 2
#define TOPHAT_ADDR 0x28
#define TOF_CLK 18
#define TOF_DAT 19

int TOF_ADDR[] = { 10, 11, 12, 13, 14 };

#define PI 3.1415926
#define DEG / 180.0 * 3.1415926
#define INCH * 0.0254

struct Node {
  int row;
  int col;
  float g;
  float h;
  float f;
  int parentRow;
  int parentCol;
};

enum CommandState {
  BUTTON,
  NAVIGATE,
  MANUAL,
  WALLFOLLOW,
  ALL3
};

CommandState currentState = MANUAL;

bool isblue = true;

enum ButtonLoc {
  NEXUS,
  LOWTOWER,
  HIGHTOWER,
};
ButtonLoc buttonLoc = NEXUS;

enum ButtonState {
  BUTTON_IDLE,
  BUTTON_ROTATE,
  BUTTON_PRESSING,
  BUTTON_HOLD,
  BUTTON_REVERSING,
  BUTTON_REVERSING_HOLD,
  BUTTON_COMPLETE
};

enum ButtonPressType {
  HOLD,
  RAPID
};

struct ButtonConfig {
  ButtonPressType type;
  float holdTime;
  int repeatCount;
};

enum WFState {
  WF_FORWARD,
  WF_STRAFE_RIGHT,
  WF_BACKWARD,
  WF_STRAFE_LEFT
};

struct PIDStateWF {
  float integral;
  float lastError;
  uint32_t lastTime;
};


Adafruit_VL53L0X tof[5] = {
  Adafruit_VL53L0X(),
  Adafruit_VL53L0X(),
  Adafruit_VL53L0X(),
  Adafruit_VL53L0X(),
  Adafruit_VL53L0X(),
};

HTML510Server h(80);
WiFiClient clientSSE;

Servo servo;

void serveBaseHTML(String in) {
  h.sendhtml(F(site));
  // delay(10000);
}

byte wifiPackets = 0;
byte health = 150;

int motorMax = 100;

int forwardvector[]  = { 1, -1, -1,  1 };
int strafevector[]   = { 1,  1,  1,  1 };
int rotationvector[] = { 1,  1, -1, -1 };

float forwardSpeed = 0;
float strafeSpeed = 0;
float rotationSpeed = 0;

float XLoc = 0;
float YLoc = 0;
float ThetaLoc = 0;

/*
+y

__________
|   O     |
|     |   |
|     |   |
|     |   |
|  O  |   |    0 heading (+ is CW)
|     |   |    ^
|     |   |    |
|     |   |     
|___O_____|    +x

*/

// ~~~ user set ~~~
float deadScales[] = { 0.3807, -0.4154, 0.4329 * 2 * PI / 1.18411 * 0.963309361};  // TODO: DONE?: calibrate; scale forward, strafe, and rotation to calibrate
float viveScale[] = { 1.0 / 4000.0, 1.0 / 4000.0 };                  // TODO: calibrate; scales X and Y axis from vive to field coordinates
float basicKnownStart[] = { (4.25 + 1) INCH, 5.25 INCH, 0.0 };  // starting coordinates
float knownStart[] = { (4.25 + 1) INCH, 5.25 INCH, 0.0 };  // starting coordinates

float fieldheight = 141.0 INCH;
float fieldwidth = 57.0 INCH;

float viveCenter = 0.055;

// structured as x1, y1, x2, y2
float wallDefinition[][4] = {
  { 0, 0, 0, fieldheight }, { 0, 0, fieldwidth, 0 }, { fieldwidth, 0, fieldwidth, fieldheight }, { 0, fieldheight, fieldwidth, fieldheight },  // outer walls
  { fieldwidth - 0.508, 0.381, fieldwidth - 0.508, fieldheight - 0.381 },                                                                      // ramp wall
};
//TODO: ADD button boxes

// float laserDefinitions[] = { 0 DEG, 90 DEG, 168 DEG, 192 DEG, 270 DEG };
float laserDefinitions[] = { 0 DEG, 90 DEG, 192 DEG, 168 DEG, 270 DEG };
float laserDists[] =       {  0.12, 0.205/2, 0.125,   0.125,  0.205/2 };


// ~~~ computer set ~~~
// x, y, rot (radians)
double viveOffset[] = { 0.0, 0.0, 0.0 };           // auto-zero based on known starting position

double deadReckonOffsetCoords[] = { 0.0, 0.0, 0.0 };  // dead reckon offset from vive coords
double viveCoords[] = { 0.0, 0.0, 0.0 };              // vive coords
int viveRaw[] = {16000, 16000, 16000, 16000};
double laserFixCoords[] = { 0.0, 0.0, 0.0 };

double finalCoords[] = { 0.0, 0.0, 0.0 };


// precompute some wall parameters
float xWallDiffs[sizeof(wallDefinition) / sizeof(wallDefinition[0])];  // x4 - x3
float yWallDiffs[sizeof(wallDefinition) / sizeof(wallDefinition[0])];  // y4 - y3
float laserMin = 0.05;
float laserMax = 2.0;
float laserReadings[] = { 0.0, 0.0, 0.0, 0.0, 0.0 };
float expectedReadings[] = { 0.0, 0.0, 0.0, 0.0, 0.0 };
int expectedWalls[] = { -1, -1, -1, -1 ,-1 };



#define CELL_SIZE 0.05f
#define ARENA_WIDTH 1.524f
#define ARENA_HEIGHT 3.658f

#define GRID_COLS 30
#define GRID_ROWS 73
#define GRID_BYTES ((GRID_COLS * GRID_ROWS + 7) * 4 / 8)

uint8_t occupancy_grid[GRID_BYTES];
uint8_t inflated_grid[GRID_BYTES];
float temp_grid[GRID_COLS * GRID_ROWS + 7];


int metersToCol(float x) {
  int c = round(x / CELL_SIZE);
  if (c < 0) c = 0;
  if (c >= GRID_COLS) c = GRID_COLS - 1;
  return c;
}

int metersToRow(float y) {
  int r = round(y / CELL_SIZE);
  if (r < 0) r = 0;
  if (r >= GRID_ROWS) r = GRID_ROWS - 1;
  return r;
}

float colToMeters(int col) {
  return col * CELL_SIZE + (CELL_SIZE / 2.0f);
}

float rowToMeters(int row) {
  return row * CELL_SIZE + (CELL_SIZE / 2.0f);
}

void setCell(int row, int col, byte val) {
  // stores each cell into one bit, 1 = occupied
  int idx = row * GRID_COLS + col;
  occupancy_grid[idx / 2] &= ~(0xF << ((1-(idx % 2)) * 4));
  occupancy_grid[idx / 2] |= (val << ((1-(idx % 2)) * 4));
}

byte getCell(int row, int col) {
  int idx = row * GRID_COLS + col;
  return (occupancy_grid[idx / 2] >> ((1-(idx % 2)) * 4)) & 0xF;
}

void setInflatedCell(int row, int col, byte val) {
  int idx = row * GRID_COLS + col;
  inflated_grid[idx / 2] &= ~(0xF << ((1-(idx % 2)) * 4));
  inflated_grid[idx / 2] |= (val << ((1-(idx % 2)) * 4));
}

byte getInflatedCell(int row, int col) {
  int idx = row * GRID_COLS + col;
  return (inflated_grid[idx / 2] >> ((1-(idx % 2)) * 4)) & 0xF;
}

void setTempCell(int row, int col, float val) {
  int idx = row * GRID_COLS + col;
  temp_grid[idx] = val;
}

float getTempCell(int row, int col) {
  int idx = row * GRID_COLS + col;
  return temp_grid[idx];
}



void initWalls() {
  for (int i = 0; i < sizeof(wallDefinition) / sizeof(wallDefinition[0]); i++) {
    xWallDiffs[i] = wallDefinition[i][2] - wallDefinition[i][0];
    yWallDiffs[i] = wallDefinition[i][3] - wallDefinition[i][1];
  }
}

float getparam(String in) {
  int http = in.indexOf("HTTP");
  in = in.substring(0, http);
  int i;
  while ((i = in.indexOf("/")) != -1) {
    in = in.substring(i + 1);
  }
  return in.toFloat();
}

void updateMotors() {
  float maxMotorReq = abs(forwardSpeed) + abs(strafeSpeed) + abs(rotationSpeed);
  if (maxMotorReq < 1.0) maxMotorReq = 1.0;

  Wire.beginTransmission(MOTOR_ADDR);
  Wire.write('M');
  if (health == 0) {
    for (int i = 0; i < 4; i++) {
      char motorspeed = (0 + 125);
      Wire.write(motorspeed);
    }
  } else {
    for (int i = 0; i < 4; i++) {
      char motorspeed = ((forwardSpeed * forwardvector[i] + rotationSpeed * rotationvector[i] + strafeSpeed * strafevector[i]) / maxMotorReq * motorMax + 125);
      Wire.write(motorspeed);
    }
  }
  Wire.endTransmission();
}

long long prevReqID = -1;  // reqID will be client date.now()

void allReq(String in) {
  // expecting format of GET /allReq/counter?forward?strafe?rotation HTTP/1.1
  int http = in.indexOf("HTTP");
  in = in.substring(0, http);
  int i;
  while ((i = in.indexOf("/")) != -1) {
    in = in.substring(i + 1);
  }

  // should just be counter?forward?strafe?rotation
  int ind = -1;
  ind = in.indexOf("?");
  if (ind == -1) return;  // malformed
  long long reqID = strtoll(in.substring(0, ind).c_str(), 0, 10);
  in = in.substring(ind + 1);
  if (reqID < prevReqID) return;  // outdated request! ignore
  prevReqID = reqID;

  ind = in.indexOf("?");
  if (ind == -1) return;  // malformed
  forwardSpeed = in.substring(0, ind).toFloat();
  in = in.substring(ind + 1);

  ind = in.indexOf("?");
  if (ind == -1) return;  // malformed
  strafeSpeed = in.substring(0, ind).toFloat();
  in = in.substring(ind + 1);

  rotationSpeed = in.toFloat();

  currentState = MANUAL;

  updateMotors();

  h.sendhtml("OK");
  wifiPackets++;
}

void locReq(String in) {
  // expecting format of GET /allReq/counter?X?Y HTTP/1.1
  int http = in.indexOf("HTTP");
  in = in.substring(0, http);
  int i;
  while ((i = in.indexOf("/")) != -1) {
    in = in.substring(i + 1);
  }

  // should just be counter?forward?strafe?rotation
  int ind = -1;
  ind = in.indexOf("?");
  if (ind == -1) return;  // malformed
  long long reqID = strtoll(in.substring(0, ind).c_str(), 0, 10);
  in = in.substring(ind + 1);
  if (reqID < prevReqID) return;  // outdated request! ignore
  prevReqID = reqID;

  ind = in.indexOf("?");
  if (ind == -1) return;  // malformed
  XLoc = in.substring(0, ind).toFloat();
  in = in.substring(ind + 1);
  YLoc = in.toFloat();

  currentState = NAVIGATE;
  resetNav();

  h.sendhtml("OK");
  wifiPackets++;
}

void resetLocReq(String in) {
  // expecting format of GET /allReq/counter?X?Y?ROT HTTP/1.1
  int http = in.indexOf("HTTP");
  in = in.substring(0, http);
  int i;
  while ((i = in.indexOf("/")) != -1) {
    in = in.substring(i + 1);
  }

  // should just be counter?forward?strafe?rotation
  int ind = -1;
  ind = in.indexOf("?");
  if (ind == -1) return;  // malformed
  long long reqID = strtoll(in.substring(0, ind).c_str(), 0, 10);
  in = in.substring(ind + 1);
  if (reqID < prevReqID) return;  // outdated request! ignore
  prevReqID = reqID;

  ind = in.indexOf("?");
  if (ind == -1) return;  // malformed
  knownStart[0] = in.substring(0, ind).toFloat();
  in = in.substring(ind + 1);

  ind = in.indexOf("?");
  if (ind == -1) return;  // malformed
  knownStart[1] = in.substring(0, ind).toFloat();
  in = in.substring(ind + 1);

  knownStart[2] = in.toFloat();

  viveCoords[0] = 0.0;
  viveCoords[1] = 0.0;
  viveCoords[2] = 0.0;
  deadReckonOffsetCoords[0] = 0.0;
  deadReckonOffsetCoords[1] = 0.0;
  deadReckonOffsetCoords[2] = 0.0;
  laserFixCoords[0] = 0.0;
  laserFixCoords[1] = 0.0;
  laserFixCoords[2] = 0.0;

  for (int row = 0; row < GRID_ROWS; row++) {
    for (int col = 0; col < GRID_COLS; col++) {
      setTempCell(row, col, 0);
    }
  }

  h.sendhtml("OK");
  wifiPackets++;
}

void wallReq(String in) {
  currentState = WALLFOLLOW;

  h.sendhtml("OK");
  wifiPackets++;
}

void teamReq(String in) {
  // expecting format of GET /allReq/counter?X?Y HTTP/1.1
  int http = in.indexOf("HTTP");
  in = in.substring(0, http-1);
  int i;
  while ((i = in.indexOf("/")) != -1) {
    in = in.substring(i + 1);
  }

  Serial.println(in);

  if (in.equals("0")) {
    isblue = false;
  } else {
    isblue = true;
  }

  h.sendhtml("OK");
  wifiPackets++;
}

void resetReq(String in) {
  // expecting format of GET /allReq/counter?X?Y HTTP/1.1
  knownStart[0] = basicKnownStart[0];
  knownStart[1] = basicKnownStart[1];
  knownStart[2] = basicKnownStart[2];
  viveCoords[0] = 0.0;
  viveCoords[1] = 0.0;
  viveCoords[2] = 0.0;
  deadReckonOffsetCoords[0] = 0.0;
  deadReckonOffsetCoords[1] = 0.0;
  deadReckonOffsetCoords[2] = 0.0;
  laserFixCoords[0] = 0.0;
  laserFixCoords[1] = 0.0;
  laserFixCoords[2] = 0.0;

  for (int row = 0; row < GRID_ROWS; row++) {
    for (int col = 0; col < GRID_COLS; col++) {
      setTempCell(row, col, 0);
    }
  }

  h.sendhtml("OK");
  wifiPackets++;
}

bool servoOn = false;
void servoReq(String in) {
  // if (servoDirection) {
  //   servo.write(180);
  // } else {
  //   servo.write(0);
  // }

  servoOn = !servoOn;

  h.sendhtml("OK");
  wifiPackets++;
}

bool servoDirection = false;
void swapServo () {
  if (servoOn) {
    if (servoDirection) {
      servo.write(180);
    } else {
      servo.write(0);
    }

    servoDirection = !servoDirection;
  } else {
    servo.write(180);
  }
  
}

void SSEReq(WiFiClient cSSE) {
  clientSSE = cSSE;
  // that's all that needs to be done
}

void sendSSE() {
  // "coords" event, 1s retry, with a data json in the format {'x': '<X>', 'y': '<Y>', 'r':'<R>'}
  h.sendplainSSE("event: coords\nretry:1000\ndata:{\"x\":" + String(finalCoords[0]) + ", \"y\":" + String(finalCoords[1]) + ", \"r\":" + String(finalCoords[2]) + ", \"l1\":" + String(laserReadings[0]) + ", \"l2\":" + String(laserReadings[1]) + ", \"l3\":" + String(laserReadings[2]) + ", \"l4\":" + String(laserReadings[3]) + ", \"l5\":" + String(laserReadings[4]) + "}");
}

void initLasers() {
  pinMode(TOF_CLK, OUTPUT);
  pinMode(TOF_DAT, OUTPUT);

  digitalWrite(TOF_DAT, LOW);  // all off for now
  for (int i = 0; i < 6; i++) {
    delay(10);
    digitalWrite(TOF_CLK, HIGH);
    delay(10);
    digitalWrite(TOF_CLK, LOW);
  }

  delay(10);
  digitalWrite(TOF_DAT, HIGH);  // turn lasers on one by one

  for (int i = 0; i < 5; i++) {
    delay(10);
    digitalWrite(TOF_CLK, HIGH);
    delay(10);
    digitalWrite(TOF_CLK, LOW);
    delay(10);
    Serial.println(tof[i].begin(TOF_ADDR[i], true, &Wire, Adafruit_VL53L0X::VL53L0X_SENSE_DEFAULT));
  }

  delay(10);

  // start continuous ranging
  for (int i = 0; i < 5; i++) {
    tof[i].startRangeContinuous();
  }
}


#define LASER_CORRECTION_FACTOR 0.8f
#define LASER_MAX_DIFF 0.1f

#define LASER_TEMP_STEP 0.03f
#define LASER_COST_RISE 1.0 / 1.0 // seconds per rise
#define LASER_COST_FALL 1.0 / 30.0 // seconds per fall

void decayTemp(float dt) {
  for (int row = 0; row < GRID_ROWS; row++) {
    for (int col = 0; col < GRID_COLS; col++) {
      float nextNum = max(getTempCell(row, col) - LASER_COST_FALL * dt / 1000.0, 0.0);
      
      setTempCell(row, col, nextNum);
    }
  }
}

// TODO: potential additions: LIDAR position measurement (use a 95% "realness" factor compared to expected values based on field position, plus )
// the assumption is that we measure at a high enough frequency that we won't be super horrible offset
// make sure to account for potential interference (other robot, etc)
// might need to do line tracing based on representation of the field
long laserDtLast = -1;
void readLasers() {
  int dt = millis() - laserDtLast;
  if (laserDtLast == -1) {
    dt = 250;
  }
  laserDtLast = millis();

  int ambad = 0;

  // add correction factor
  // first handle expected readings
  for (int i = 0; i < sizeof(laserDefinitions) / sizeof(laserDefinitions[0]); i++) {
    expectedReadings[i] = -1.0;                                // first reset to "undefined"
    expectedWalls[i] = -1;
    float lXdiff = sin(laserDefinitions[i] + finalCoords[2]);  // x2 - x1 (normalized to length 1)
    float lYdiff = cos(laserDefinitions[i] + finalCoords[2]);  // y2 - y1 (normalized to length 1)

    for (int j = 0; j < sizeof(wallDefinition) / sizeof(wallDefinition[0]); j++) {
      // check against every wall
      float bottom = lXdiff * yWallDiffs[j] - lYdiff * xWallDiffs[j];
      float xdist = wallDefinition[j][0] - finalCoords[0];  // = x3-x1
      float ydist = wallDefinition[j][1] - finalCoords[1];  // = y3-y1
      // u defines the wall line segment = -((x2-x1)(y3-y1) - (y2-y1)(x3-x1))
      float uTop = -(lXdiff * ydist - lYdiff * xdist);

      // now check intersection before continuing (if 0 < u / bottom < 1, valid (aka top and bottom have same sign, and top is smaller than bottom))
      if (bottom == 0 || abs(uTop) > abs(bottom) || ((uTop < 0) != (bottom < 0))) continue;
      // t defines the distance of intersection of laser - this is the number we want! (since laser definition is already normalized)
      // = (x3-x1)(y3-y4) - (y3-y1)(x4-x3)
      float t = (xdist * yWallDiffs[j] - ydist * xWallDiffs[j]) / bottom;
      if (t < 0) continue;  // laser needs to go backwards.. doesn't make sense
      if (expectedReadings[i] < 0 || expectedReadings[i] > t) {
        // found newer or closer entry!
        expectedReadings[i] = t;
        expectedWalls[i] = j;
      }
    }

    if (expectedReadings[i] > laserMax || expectedReadings[i] < laserMin) {
      // bad data, so ignore
      expectedReadings[i] = -1.0;
      expectedWalls[i] = -1;
    }

    // now get real laser information
    if (tof[i].isRangeComplete()) {
      int range = tof[i].readRangeResult();

      Wire.beginTransmission(TOF_ADDR[i]);
      byte error = Wire.endTransmission();

      if (error != 0) {
        ambad++;
      }

      if (range < 3000 && range > 30)
        laserReadings[i] = range / 1000.0 + laserDists[i];
      else
        laserReadings[i] = -1.0;
    }
  }

  if (ambad == 5) {
    initLasers();
  }
  
  decayTemp(dt);
    
  // use diff to compute position offset!
  float xCorrection = 0.0f;
  float yCorrection = 0.0f;
  float totalWeight = 0.0f;

  for (int k = 0; k < sizeof(laserDefinitions) / sizeof(laserDefinitions[0]); k++) {
    if (laserReadings[k] < 0) continue;

    // always do temp
    float normalizeXDiff = sin(laserDefinitions[k] + finalCoords[2]); // normalize to x direction laser is pointing in
    float normalizeYDiff = cos(laserDefinitions[k] + finalCoords[2]); // normalize to y direction laser is pointing in

    // do temp obstacles
    for (double dist = 0; dist <= laserReadings[k]; dist += LASER_TEMP_STEP) {
      int row = metersToRow(normalizeYDiff * laserReadings[k] + finalCoords[1]);
      int col = metersToCol(ARENA_WIDTH - (normalizeXDiff * laserReadings[k] + finalCoords[0]) - 0.07);
      if (row >= 0 && row < GRID_ROWS && col >= 0 && col < GRID_COLS) {
        setTempCell(row, col, max(getTempCell(row, col) - LASER_COST_FALL * dt / 1000.0, 0.0));
      }
    }
    int row = metersToRow(normalizeYDiff * laserReadings[k] + finalCoords[1]);
    int col = metersToCol(ARENA_WIDTH - (normalizeXDiff * laserReadings[k] + finalCoords[0]) - 0.07);
    if (row >= 0 && row < GRID_ROWS && col >= 0 && col < GRID_COLS) {
      setTempCell(row, col, min(getTempCell(row, col) + LASER_COST_RISE * dt / 1000.0, 1.0));
    }

    // don't do correction if no expected
    if(expectedReadings[k] < 0) continue;
    
    float diff = laserReadings[k] - expectedReadings[k];
    if (abs(diff) > LASER_MAX_DIFF) continue;

    // attempt to weight by level of perpendicularity to wall its facing

    float wallNormalX = yWallDiffs[expectedWalls[k]];
    float wallNormalY = -xWallDiffs[expectedWalls[k]];
    float wallLength = sqrt((wallNormalX * wallNormalX) + (wallNormalY * wallNormalY));
    wallNormalX /= wallLength;
    wallNormalY /= wallLength;

    float dot = abs((normalizeXDiff * wallNormalX) + (normalizeYDiff * wallNormalY));

    float weight = dot * dot;
    if (weight < 0.1f) continue; // skip if parallel

    xCorrection += diff * normalizeXDiff * weight;
    yCorrection += diff * normalizeYDiff * weight;
    totalWeight += weight;
  }

  if (totalWeight > 0.2f) {
    xCorrection /= totalWeight;
    yCorrection /= totalWeight;
    laserFixCoords[0] -= LASER_CORRECTION_FACTOR * xCorrection;
    laserFixCoords[1] -= LASER_CORRECTION_FACTOR * yCorrection;
  }
}

byte readVivebuffer[100];
void getViveRaw() {
  // reads dead reckoning data
  Wire.requestFrom(VIVE_ADDR, 2 * 4 + 1);

  int i = 0;
  while (Wire.available()) {
    byte c = Wire.read();
    readVivebuffer[i++] = c;
  }

  if (readVivebuffer[0] != 'V') return;  // malformed

  memcpy(&viveRaw[0], &readVivebuffer[1], 2);
  memcpy(&viveRaw[1], &readVivebuffer[1 + 2], 2);
  memcpy(&viveRaw[2], &readVivebuffer[1 + 2 + 2], 2);
  memcpy(&viveRaw[3], &readVivebuffer[1 + 2 + 2 + 2], 2);

  Serial.print(viveRaw[0]);
  Serial.print(" - ");
  Serial.print(viveRaw[1]);
  Serial.print(" - ");
  Serial.print(viveRaw[2]);
  Serial.print(" - ");
  Serial.println(viveRaw[3]);
}

bool activateVive = false;

void calibrateVive() {
  // first get vive coordinates
  // (yield until we get good coordinates, aka in range)
  int counter = 0;
  do {
    getViveRaw();
    delay(100); // let other thread work
    if (counter++ > 50) {
      return;
    }
  } while(viveRaw[0] > 8000);

  activateVive = true;

  double viveCenterX = (viveRaw[0] + viveRaw[2]) / 2.0 * viveScale[0];
  double viveCenterY = (viveRaw[1] + viveRaw[3]) / 2.0 * viveScale[1];
  double viveRot = atan2(viveRaw[0] - viveRaw[2], viveRaw[1] - viveRaw[3]);

  viveOffset[0] = viveCenterX;
  viveOffset[1] = viveCenterY;
  viveOffset[2] = viveRot;
}

void readVive() {
  if (!activateVive) return;
  // first get vive coordinates
  getViveRaw();
  if (viveRaw[0] > 8000) {
    // bad coordinates! ignore vive, don't reset dead reckoning
    return;
  }
  
  // then convert into field coordinates
  double viveCenterX = (viveRaw[0] + viveRaw[2]) / 2.0 * viveScale[0] - viveOffset[0];
  double viveCenterY = (viveRaw[1] + viveRaw[3]) / 2.0 * viveScale[1] - viveOffset[1];
  double viveRot = atan2(viveRaw[0] - viveRaw[2], viveRaw[1] - viveRaw[3]) - viveOffset[2];

  // compensate for vive rotation
  double realX = viveCenterX * cos(viveOffset[2]) - viveCenterY * sin(viveOffset[2]) + viveCenter * sin(viveRot);
  double realY = viveCenterX * sin(viveOffset[2]) + viveCenterY * cos(viveOffset[2]) - viveCenter * cos(viveRot);

  // commit into array
  viveCoords[0] = realX;
  viveCoords[1] = realY;
  viveCoords[2] = viveRot;
  
  deadReckonOffsetCoords[0] = 0.0;
  deadReckonOffsetCoords[1] = 0.0;
  deadReckonOffsetCoords[2] = 0.0;

  updateCoords();
}

byte readbuffer[100];
void readDeadReckon() {
  // reads dead reckoning data
  Wire.requestFrom(MOTOR_ADDR, 4 * 3 + 1);

  int i = 0;
  while (Wire.available()) {
    byte c = Wire.read();
    readbuffer[i++] = c;
  }

  if (readbuffer[0] != 'D') return;  // malformed

  // assume +strafe is right
  // assume rotation is CW
  float forwardsDead = 0.0;
  float strafeDead = 0.0;
  float rotateDead = 0.0;

  memcpy(&forwardsDead, &readbuffer[1], 4);
  memcpy(&strafeDead, &readbuffer[1 + 4], 4);
  memcpy(&rotateDead, &readbuffer[1 + 4 + 4], 4);

  double curRot = finalCoords[2];
  deadReckonOffsetCoords[0] += (forwardsDead * sin(curRot) * deadScales[0] - strafeDead * cos(curRot) * deadScales[1]);
  deadReckonOffsetCoords[1] += (forwardsDead * cos(curRot) * deadScales[0] + strafeDead * sin(curRot) * deadScales[1]);
  deadReckonOffsetCoords[2] += (rotateDead) * deadScales[2];

  updateCoords();
}

double deadReckonCheckpoint[] = {0.0, 0.0, 0.0};
double laserCheckpoint[] = {0.0, 0.0, 0.0};


bool saved = false;
void saveDeadReckon() {
  saved = true;
  deadReckonCheckpoint[0] = deadReckonOffsetCoords[0];
  deadReckonCheckpoint[1] = deadReckonOffsetCoords[1];
  deadReckonCheckpoint[2] = deadReckonOffsetCoords[2];
  laserCheckpoint[0] = laserFixCoords[0];
  laserCheckpoint[1] = laserFixCoords[1];
  laserCheckpoint[2] = laserFixCoords[2];
}

void loadDeadReckon() {
  if (saved) {
    deadReckonOffsetCoords[0] = deadReckonCheckpoint[0];
    deadReckonOffsetCoords[1] = deadReckonCheckpoint[1];
    deadReckonOffsetCoords[2] = deadReckonCheckpoint[2];
    laserFixCoords[0] = laserCheckpoint[0];
    laserFixCoords[1] = laserCheckpoint[1];
    laserFixCoords[2] = laserCheckpoint[2];
    saved = false;
  }
}

// just commit the coordinates to the array
void updateCoords() {
  finalCoords[0] = viveCoords[0] + deadReckonOffsetCoords[0] + laserFixCoords[0] + knownStart[0];
  finalCoords[1] = viveCoords[1] + deadReckonOffsetCoords[1] + laserFixCoords[1] + knownStart[1];
  finalCoords[2] = viveCoords[2] + deadReckonOffsetCoords[2] + laserFixCoords[2] + knownStart[2];
}

void tophat_handle() {
  // A: transmit wifi data
  Wire.beginTransmission(TOPHAT_ADDR);
  Wire.write((char)wifiPackets);
  wifiPackets = 0;  // reset counter
  Wire.endTransmission();

  // B: recieve health data
  Wire.requestFrom(TOPHAT_ADDR, 1);
  if (Wire.available()) {
    health = Wire.read();
    Serial.print("got health: ");
    Serial.println(health);
  }
  if (health == 0) {
    updateMotors();
  }
  // Serial.println(health);
}

void serve() {
  h.serve();
}



// ~~~~~~~~~~~~~~~~~~~ NAVIGATION ~~~~~~~~~~~~~~~~~~~~

// defining the top left corner of everything, x,y = 0 is top left corner
#define RAILING_X 0.508f
#define RAILING_Y 0.341f
#define RAILING_WIDTH 0.025f
#define RAILING_HEIGHT 2.8956f

#define TOWER_X 0.94f
#define TOWER_Y 1.6872f
#define TOWER_WIDTH 0.17f
#define TOWER_HEIGHT 0.17f

#define BUTTON_PRESS_DISTANCE 0.25f  //TODO: tune this to min pressing distance
#define BUTTON_PRESSING_DISTANCE 0.05f  //TODO: tune this to min pressing distance
#define BUTTON_RELEASE_DISTANCE 0.15f  //TODO: tune this to min pressing distance

#define NEXUS_BLUE_X 0.472f
#define NEXUS_BLUE_Y 0.0f + 0.127 + 0.115f
#define NEXUS_BLUE_WIDTH 0.127f
#define NEXUS_BLUE_HEIGHT 0.019f

#define NEXUS_BLUE_X_ASTAR 0.91f
#define NEXUS_BLUE_Y_ASTAR 0.0381f
#define NEXUS_BLUE_WIDTH_ASTAR 0.2032f
#define NEXUS_BLUE_HEIGHT_ASTAR 0.127f

#define NEXUS_BLUE_APPROACH_X colToMeters(19)  // center of nexus button
#define NEXUS_BLUE_APPROACH_Y rowToMeters(6)   // one cell below
#define NEXUS_BLUE_APPROACH_THETA PI

#define NEXUS_RED_X 0.472f
#define NEXUS_RED_Y 3.543f - 0.127f
#define NEXUS_RED_WIDTH 0.127f
#define NEXUS_RED_HEIGHT 0.019f

#define NEXUS_RED_X_ASTAR 0.91f
#define NEXUS_RED_Y_ASTAR 3.4425
#define NEXUS_RED_WIDTH_ASTAR 0.2032f
#define NEXUS_RED_HEIGHT_ASTAR 0.127f

#define RAMP_BUTTON_BLUE_X 1.443f
#define RAMP_BUTTON_BLUE_Y 2.015f
#define RAMP_BUTTON_BLUE_WIDTH 0.019f
#define RAMP_BUTTON_BLUE_HEIGHT 0.127f

#define RAMP_BUTTON_RED_X 1.443f
#define RAMP_BUTTON_RED_Y 1.605f
#define RAMP_BUTTON_RED_WIDTH 0.019f
#define RAMP_BUTTON_RED_HEIGHT 0.127f

#define TOWER_BUTTON_BLUE_X 0.472f
#define TOWER_BUTTON_BLUE_Y 1.867f + 0.25f;
#define TOWER_BUTTON_BLUE_WIDTH 0.127f
#define TOWER_BUTTON_BLUE_HEIGHT 0.019f

#define TOWER_BUTTON_RED_X 0.472f
#define TOWER_BUTTON_RED_Y 1.669f
#define TOWER_BUTTON_RED_WIDTH 0.127f
#define TOWER_BUTTON_RED_HEIGHT 0.019f

#define INFLATE_RADIUS 6
#define INFLATE_RADIUS_WALL 5
#define INFLATE_RADIUS_RAMP 6
#define INFLATE_COST 2.0f
#define INFLATE_HIGH_COST 6

#define TEMP_MAX_COST 10.0f

#define MAX_OPEN (GRID_COLS * GRID_ROWS)
#define MAX_PATH (GRID_COLS * GRID_ROWS)

#define WAYPOINT_THRESHOLD 0.05f

// typedef struct Node Node;

Node cameFrom[GRID_ROWS][GRID_COLS];

void setWalls() {
  for (int col = 0; col < GRID_COLS; col++) {
    // Set the top and bottom rows as occupied
    setCell(0, col, 0xF);
    setCell((GRID_ROWS - 1), col, 0xF);
  }
  // Set the left and right columnns as occupied
  for (int row = 0; row < GRID_ROWS; row++) {
    setCell(row, 0, 0xF);
    setCell(row, (GRID_COLS - 1), 0xF);
  }
}

void setObstacle(float x, float y, float width, float height, float val=1) {
  int col0 = metersToCol(x);
  int row0 = metersToRow(y);
  int col1 = metersToCol(x + width);
  int row1 = metersToRow(y + height);

  // guarantee at least 1 cell
  if (col1 < col0) col1 = col0;
  if (row1 < row0) row1 = row0;

  for (int row = row0; row <= row1; row++) {
    for (int col = col0; col <= col1; col++) {
      setCell(row, col, val);
    }
  }
}

void inflateObstacles() {
  // Copy already made grid to make inflated one
  memcpy(inflated_grid, occupancy_grid, GRID_BYTES);
  // For every occupied cell, create a boundary around it of 3 additional cells to ensure clearance
  // Gives about an extra 15cm of clearance
  for (int row = 0; row < GRID_ROWS; row++) {
    for (int col = 0; col < GRID_COLS; col++) {
      int cNum = getCell(row, col);
      if (cNum) {
        int rad = (cNum == 0xF)?INFLATE_RADIUS_WALL:((cNum == 0xE)?INFLATE_RADIUS_RAMP:INFLATE_RADIUS);
        for (int deltaRow = -rad; deltaRow <= rad; deltaRow++) {
          for (int deltaCol = -rad; deltaCol <= rad; deltaCol++) {
            int newRow = row + deltaRow;
            int newCol = col + deltaCol;
            if (newRow >= 0 && newRow < GRID_ROWS && newCol >= 0 && newCol < GRID_COLS) {
              byte infValue = round((1 - pow(deltaRow * deltaRow + deltaCol * deltaCol, 0.5)/rad) * INFLATE_HIGH_COST);
              if (deltaRow == 0 && deltaCol == 0) {
                infValue = 0xF;
              }
              if (infValue > getInflatedCell(newRow, newCol)) {
                setInflatedCell(newRow, newCol, infValue);
              }
            }
          }
        }
      }
    }
  }
}

// A* stuff
float heuristic(int row, int col, int goalRow, int goalCol) {
  return pow((float) ((row - goalRow)*(row - goalRow) + (col - goalCol)*(col - goalCol)), 0.5);
}

Node openSet[MAX_OPEN];
int openCount = 0;
bool closedSet[GRID_ROWS][GRID_COLS];

void addToOpenSet (Node n) {
  openSet[openCount] = n;
  openCount++;
}

Node getLowestF() {
  int bestIdx = 0;
  for (int i = 1; i < openCount; i++) {
    if (openSet[i].f < openSet[bestIdx].f) {
      bestIdx = i;
    }
  }
  Node best = openSet[bestIdx];

  openSet[bestIdx] = openSet[openCount - 1];
  openCount--;
  return best;
}

bool checkOpenSet(int row, int col) {
  for (int i = 0; i < openCount; i++) {
    if (openSet[i].row == row && openSet[i].col == col) {
      return true;
    }
  }
  return false;
}

int pathRow[MAX_PATH];
int pathCol[MAX_PATH];
int pathLength = 0;  //total number of waypoints in the path
int currentWaypoint = 0;

void aStar(int startRow, int startCol, int goalRow, int goalCol) {
  memset(closedSet, false, sizeof(closedSet));
  openCount = 0;
  pathLength = 0;
  currentWaypoint = 0;

  Node startNode;
  startNode.row = startRow;
  startNode.col = startCol;
  startNode.g = 0.0f;
  startNode.h = heuristic(startRow, startCol, goalRow, goalCol);
  startNode.f = startNode.g + startNode.h;
  startNode.parentRow = -1;
  startNode.parentCol = -1;

  addToOpenSet(startNode);
  cameFrom[startRow][startCol] = startNode;

  while (openCount > 0) {
    Node currentNode = getLowestF();
    if (currentNode.row == goalRow && currentNode.col == goalCol) {
      pathLength = 0;
      Node step = currentNode;
      while (step.parentRow != -1) {
        if (pathLength >= 2) {
          int rDiff = abs(step.row - pathRow[pathLength-2]);
          int cDiff = abs(step.col - pathCol[pathLength-2]);
          if ((rDiff <= 1 && cDiff <= 3) || (rDiff <= 3 && cDiff <= 1)) {
            // does a one-two! thus, cut the corner (limit of 3x1 or 1x3)
            pathLength--;
          }
        }
        pathRow[pathLength] = step.row;
        pathCol[pathLength] = step.col;
        pathLength++;
        step = cameFrom[step.parentRow][step.parentCol];
      }
      if (pathLength >= 2) {
        int rDiff = abs(startRow - pathRow[pathLength-2]);
        int cDiff = abs(startCol - pathCol[pathLength-2]);
        if ((rDiff <= 1 && cDiff <= 3) || (rDiff <= 3 && cDiff <= 1)) {
          // does a one-two! thus, cut the corner (limit of 3x1 or 1x3)
          pathLength--;
        }
      }
      pathRow[pathLength] = startRow;
      pathCol[pathLength] = startCol;
      pathLength++;

      // reverse so index = 0 is the start and last index is the goal
      for (int i = 0; i < pathLength / 2; i++) {
        int tmpRow = pathRow[i];
        int tmpCol = pathCol[i];
        pathRow[i] = pathRow[pathLength - 1 - i];
        pathCol[i] = pathCol[pathLength - 1 - i];
        pathRow[pathLength - 1 - i] = tmpRow;
        pathCol[pathLength - 1 - i] = tmpCol;
      }

      return;
    }
    closedSet[currentNode.row][currentNode.col] = true;

    int directionsRow[] = { -1, 1, 0, 0};
    int directionsCol[] = {  0, 0,-1, 1};

    for (int i = 0; i < 4; i++) {
      int neighborRow = currentNode.row + directionsRow[i];
      int neighborCol = currentNode.col + directionsCol[i];

      if (neighborRow < 0 || neighborRow >= GRID_ROWS || neighborCol < 0 || neighborCol >= GRID_COLS || getInflatedCell(neighborRow, neighborCol) == 0xF) {
        continue;
      }

      Node neighborNode;
      neighborNode.row = neighborRow;
      neighborNode.col = neighborCol;
      neighborNode.g = currentNode.g + 1.0f + getInflatedCell(neighborRow, neighborCol) * INFLATE_COST + getTempCell(neighborRow, neighborCol) * TEMP_MAX_COST;
      neighborNode.h = heuristic(neighborRow, neighborCol, goalRow, goalCol);
      neighborNode.f = neighborNode.g + neighborNode.h;
      neighborNode.parentRow = currentNode.row;
      neighborNode.parentCol = currentNode.col;

      if (!checkOpenSet(neighborRow, neighborCol)) {
        if (closedSet[neighborRow][neighborCol]) {
          if(cameFrom[neighborNode.row][neighborNode.col].f > neighborNode.f) {
            addToOpenSet(neighborNode);
            cameFrom[neighborNode.row][neighborNode.col] = neighborNode;
          }
        } else {
          addToOpenSet(neighborNode);
          cameFrom[neighborNode.row][neighborNode.col] = neighborNode;
        }
      }
    }
  }
}

auto cmp = [](Node left, Node right) { return (left.f) > (right.f); };

std::priority_queue<Node, std::vector<Node>, decltype(cmp)> lambda_pq(cmp);

void aStar2(int startRow, int startCol, int goalRow, int goalCol) {
  lambda_pq = std::priority_queue<Node, std::vector<Node>, decltype(cmp)>(cmp);

  memset(closedSet, false, sizeof(closedSet));
  openCount = 0;
  // pathLength = 0;
  // currentWaypoint = 0;

  Node startNode;
  startNode.row = startRow;
  startNode.col = startCol;
  startNode.g = 0.0f;
  startNode.h = heuristic(startRow, startCol, goalRow, goalCol);
  startNode.f = startNode.g + startNode.h;
  startNode.parentRow = -1;
  startNode.parentCol = -1;

  lambda_pq.push(startNode);

  // addToOpenSet(startNode);
  cameFrom[startRow][startCol] = startNode;

  while (!lambda_pq.empty()) {
    Node currentNode = lambda_pq.top();
    lambda_pq.pop();
    if (closedSet[currentNode.row][currentNode.col]) {
      // delete currentNode;
      continue; // ignore already-processed
    }

    if (currentNode.row == goalRow && currentNode.col == goalCol) {
      pathLength = 0;
      currentWaypoint = 0;
      Node step = currentNode;
      while (step.parentRow != -1) {
        if (pathLength >= 2) {
          int rDiff = abs(step.row - pathRow[pathLength-2]);
          int cDiff = abs(step.col - pathCol[pathLength-2]);
          if ((rDiff <= 1 && cDiff <= 3) || (rDiff <= 3 && cDiff <= 1)) {
            // does a one-two! thus, cut the corner (limit of 3x1 or 1x3)
            pathLength--;
          }
        }
        pathRow[pathLength] = step.row;
        pathCol[pathLength] = step.col;
        pathLength++;
        step = cameFrom[step.parentRow][step.parentCol];
      }
      if (pathLength >= 2) {
        int rDiff = abs(startRow - pathRow[pathLength-2]);
        int cDiff = abs(startCol - pathCol[pathLength-2]);
        if ((rDiff <= 1 && cDiff <= 3) || (rDiff <= 3 && cDiff <= 1)) {
          // does a one-two! thus, cut the corner (limit of 3x1 or 1x3)
          pathLength--;
        }
      }
      pathRow[pathLength] = startRow;
      pathCol[pathLength] = startCol;
      pathLength++;

      // reverse so index = 0 is the start and last index is the goal
      for (int i = 0; i < pathLength / 2; i++) {
        int tmpRow = pathRow[i];
        int tmpCol = pathCol[i];
        pathRow[i] = pathRow[pathLength - 1 - i];
        pathCol[i] = pathCol[pathLength - 1 - i];
        pathRow[pathLength - 1 - i] = tmpRow;
        pathCol[pathLength - 1 - i] = tmpCol;
      }

      // delete currentNode;
      return;
    }
    closedSet[currentNode.row][currentNode.col] = true;

    int directionsRow[] = { -1, 1, 0, 0};
    int directionsCol[] = {  0, 0,-1, 1};

    for (int i = 0; i < 4; i++) {
      int neighborRow = currentNode.row + directionsRow[i];
      int neighborCol = currentNode.col + directionsCol[i];

      if (neighborRow < 0 || neighborRow >= GRID_ROWS || neighborCol < 0 || neighborCol >= GRID_COLS || getInflatedCell(neighborRow, neighborCol) == 0xF) {
        // delete currentNode;
        continue;
      }

      Node neighborNode;
      neighborNode.row = neighborRow;
      neighborNode.col = neighborCol;
      neighborNode.g = currentNode.g + 1.0f + getInflatedCell(neighborRow, neighborCol) * INFLATE_COST + getTempCell(neighborRow, neighborCol) * TEMP_MAX_COST;
      neighborNode.h = heuristic(neighborRow, neighborCol, goalRow, goalCol);
      neighborNode.f = neighborNode.g + neighborNode.h;
      neighborNode.parentRow = currentNode.row;
      neighborNode.parentCol = currentNode.col;

      if (closedSet[neighborRow][neighborCol]) {
        if(cameFrom[neighborNode.row][neighborNode.col].f > neighborNode.f) {
          // addToOpenSet(neighborNode);
          lambda_pq.push(neighborNode);
          cameFrom[neighborNode.row][neighborNode.col] = neighborNode;
        }
      } else {
        // addToOpenSet(neighborNode);
        lambda_pq.push(neighborNode);
        cameFrom[neighborNode.row][neighborNode.col] = neighborNode;
      }
    }
  }
}

// Position PID (x,y,theta)
// float KP[] = { 50.0f, 50.0f, 5.0f };
float KP[] = { 5.0f, 8.0f, 2.0f };
float KI[] = { 0.001f, 0.0005f, 0.001f };
float KD[] = { 0.00f, 0.00f, 0.00f };

float positionIntegral[] = { 0.0f, 0.0f, 0.0f };
float positionLastError[] = { 0.0f, 0.0f, 0.0f };

float positionPID(float goalX, float goalY, float dt, float goalTheta = -999.0f, bool highprecision = true) {
  // if no theta provided, face the waypoint
  if (goalTheta == -999.0f) {
    goalTheta = -atan2(goalX - (ARENA_WIDTH-finalCoords[0]), goalY - finalCoords[1]);
  }
  // make it so target theta is always just angle from current heading to the waypoint

  float errorWorld[] = {
    (goalX - (ARENA_WIDTH-finalCoords[0])),
    (goalY - finalCoords[1]),
    (goalTheta - finalCoords[2])
  };


  Serial.println("goals: " + String(goalX) + ", " + String(goalY) + ", " + String(goalTheta));
  Serial.println("curs: " + String((ARENA_WIDTH-finalCoords[0])) + ", " + String(finalCoords[1]) + ", " + String(finalCoords[2]));

  // angle wrapping from -pi to pi
  while (errorWorld[2] > PI) errorWorld[2] -= 2.0f * PI;
  while (errorWorld[2] < -PI) errorWorld[2] += 2.0f * PI;

  // rotate errors into body frame
  float errorBody[] = {
    (-errorWorld[0] * cos(finalCoords[2]) - errorWorld[1] * sin(finalCoords[2])),
    (-errorWorld[0] * sin(finalCoords[2]) + errorWorld[1] * cos(finalCoords[2])),
    (errorWorld[2])
  };

  // actually do PID
  float velocity[] = { 0.0f, 0.0f, 0.0f };
  for (int i = 0; i < 3; i++) {
    float proportionalTerm = KP[i] * errorBody[i];

    positionIntegral[i] += errorBody[i] * dt;
    float integralTerm = KI[i] * positionIntegral[i];

    float derivativeTerm = KD[i] * (errorBody[i] - positionLastError[i]) / dt;

    velocity[i] = proportionalTerm + integralTerm + derivativeTerm;
    positionLastError[i] = errorBody[i];

    // clamp out of bounds values
    if (positionIntegral[i] > 10.0f) positionIntegral[i] = 10.0f;
    if (positionIntegral[i] < -10.0f) positionIntegral[i] = -10.0f;
    Serial.print(errorBody[i]); Serial.print(" "); 
  }
  Serial.println();

  double maxVel = abs(velocity[1]) + abs(velocity[0]); // don't consider rotation
  if (highprecision) maxVel = 1; // if high precision, don't do speed compensation

  forwardSpeed = velocity[1] / maxVel;
  strafeSpeed = velocity[0] / maxVel;
  rotationSpeed = velocity[2];
  Serial.println("driving with " + String(forwardSpeed) + ", " + String(strafeSpeed) + ", " + String(rotationSpeed));
  updateMotors();

  // skew it so strafe needs to be way closer to continue
  return pow(errorBody[0]*errorBody[0] + errorBody[1]*errorBody[1], 0.5);
}

void followPath(float dt) {
  // check if robot has reached the end of the path
  if (currentWaypoint >= pathLength) {
    return;  // stop if all waypoints have been reached
  }

  // get current waypoint
  float goalX = colToMeters(pathCol[currentWaypoint]);
  float goalY = rowToMeters(pathRow[currentWaypoint]);

  Serial.println("navigating to id " + String(currentWaypoint) + " at " + String(goalX) + ", " + String(goalY));
  float error;

  if (currentWaypoint > 0) {
    float lastX = colToMeters(pathCol[currentWaypoint-1]);
    float lastY = rowToMeters(pathRow[currentWaypoint-1]);
    float goaltheta = -atan2(goalX - lastX, goalY - lastY);
    error = positionPID(goalX, goalY, dt, goaltheta, currentWaypoint==pathLength-1);
  } else {
    error = positionPID(goalX, goalY, dt, currentWaypoint==pathLength-1);
  }

  // if close enough to waypoint based on threshold, proceed to next one
  float errorX = goalX - (ARENA_WIDTH - finalCoords[0]);
  float errorY = goalY - finalCoords[1];
  // if (sqrt(errorX * errorX + errorY * errorY) < WAYPOINT_THRESHOLD) {
  if (error < (currentWaypoint==pathLength-1? WAYPOINT_THRESHOLD/3 : WAYPOINT_THRESHOLD)) {
    currentWaypoint++;
  }
}

enum NavState {
  NAV_IDLE,
  NAV_NAVIGATING,
  NAV_COMPLETE
};

NavState navState = NAV_IDLE;

void resetNav() {
  navState = NAV_IDLE;
  pathLength = 0;
  currentWaypoint = 0;
}

int navigateTo(float dt, float goalX, float goalY) {
  switch (navState) {
    case NAV_IDLE:
      {
        int startRow = metersToRow(finalCoords[1]);
        int startCol = metersToCol(ARENA_WIDTH - finalCoords[0]);
        int goalRow = metersToRow(goalY);
        int goalCol = metersToCol(ARENA_WIDTH - goalX);
        Serial.println("astar from " + String(startRow) + ", " + String(startCol) + " to " + String(goalRow) + ", " + String(goalCol));
        aStar2(startRow, startCol, goalRow, goalCol);
        navState = NAV_NAVIGATING;
        return 0;
      }

    case NAV_NAVIGATING:
      {
        followPath(dt);
        if (currentWaypoint >= pathLength) {
          navState = NAV_COMPLETE;
        }
        return 1;
      }

    case NAV_COMPLETE:
      {
        forwardSpeed = 0;
        strafeSpeed = 0;
        rotationSpeed = 0;
        updateMotors();
        return 2;
      }
  }
}

void renav() {
  if (navState == NAV_NAVIGATING) {
    int startRow = metersToRow(finalCoords[1]);
    int startCol = metersToCol(ARENA_WIDTH - finalCoords[0]);
    int goalRow = metersToRow(YLoc);
    int goalCol = metersToCol(ARENA_WIDTH - XLoc);
    Serial.println("astar from " + String(startRow) + ", " + String(startCol) + " to " + String(goalRow) + ", " + String(goalCol));
    aStar2(startRow, startCol, goalRow, goalCol);
  }
}

#define tempgridMax 0xF

void sendMAP() {
  // "coords" event, 1s retry, with a data json in the format {'x': '<X>', 'y': '<Y>', 'r':'<R>'}
  String res = "";
  // for (int i = 0; i < GRID_BYTES; i++) {
  //   byte x1 = min((inflated_grid[i] >> 4) & 0xF + ((int) (temp_grid[2*i] * tempgridMax)), 0xF);
  //   byte x2 = min((inflated_grid[i]) & 0xF + ((int) (temp_grid[2*i + 1] * tempgridMax)), 0xF);
  //   byte x  = x1 << 4 + x2;
  //   if (x <= 0x0F) res += "0";
  //   res += String(x, HEX);
  // }

  for (int row = 0; row < GRID_ROWS; row++) {
    for (int col = 0; col < GRID_COLS; col++) {
      int inf = getInflatedCell(row, col);
      int temp = (int) (getTempCell(row, col) * tempgridMax);
      res += String(min(inf + temp, 0xF), HEX);
    }
  }

  String path = "[";

  // mark path cells
  bool onPath[GRID_ROWS][GRID_COLS] = {};
  for (int i = 0; i < pathLength; i++) {
    path += "[";
    path += String(pathCol[i]);
    path += ",";
    path += String(pathRow[i]);
    if (i == pathLength - 1) {
      path += "]";
    } else {
      path += "],";
    }
  }
  path += "]";

  h.sendplainSSE("event: map\nretry:1000\ndata:{\"mapdata\":\"" + res + "\", \"astar\":" + path + "}");
}


float pressStartX = -1.0f;
float pressStartY = -1.0f;
float reverseStartX = -1.0f;
float reverseStartY = -1.0f;

bool pressButton() {
  // mark starting position first time this is called
  if (pressStartX == -1.0f) {
    pressStartX = finalCoords[0];
    pressStartY = finalCoords[1];
  }

  // calculated distance travelled
  float distance = sqrt(pow(finalCoords[0] - pressStartX, 2) + pow(finalCoords[1] - pressStartY, 2));

  // drive forward into the button
  if (distance < BUTTON_PRESSING_DISTANCE) {
    forwardSpeed = 0.3f;
    strafeSpeed = 0.0f;
    rotationSpeed = 0.0f;
    updateMotors();
    return false;  // continue pressing button
  }

  else {
    forwardSpeed = 0.0f;
    strafeSpeed = 0.0f;
    rotationSpeed = 0.0f;
    updateMotors();
    pressStartX = -1.0f;
    pressStartY = -1.0f;
    return true;  // button is pressed
  }
}

bool reverseFromButton() {
  // since the robot ends in an occupied cell, the next A* call will fail to find a path, so need to reverse back into free space
  // basically same as pressButton, but in reverse
  if (reverseStartX == -1.0f) {
    reverseStartX = finalCoords[0];
    reverseStartY = finalCoords[1];
  }

  float distance = sqrt(pow(finalCoords[0] - reverseStartX, 2) + pow(finalCoords[1] - reverseStartY, 2));

  if (distance < BUTTON_PRESSING_DISTANCE * 0.7) {
    forwardSpeed = -0.3f;
    strafeSpeed = 0.0f;
    rotationSpeed = 0.0f;
    updateMotors();
    return false;  // continue reversing
  }

  else {
    forwardSpeed = 0.0f;
    strafeSpeed = 0.0f;
    rotationSpeed = 0.0f;
    // reset
    reverseStartX = -1.0f;
    reverseStartY = -1.0f;
    return true;  // reversing complete
  }
}

bool reverseFromButtonHold() {
  // since the robot ends in an occupied cell, the next A* call will fail to find a path, so need to reverse back into free space
  // basically same as pressButton, but in reverse
  if (reverseStartX == -1.0f) {
    reverseStartX = finalCoords[0];
    reverseStartY = finalCoords[1];
  }

  float distance = sqrt(pow(finalCoords[0] - reverseStartX, 2) + pow(finalCoords[1] - reverseStartY, 2));

  if (distance < BUTTON_RELEASE_DISTANCE) {
    forwardSpeed = -0.3f;
    strafeSpeed = 0.0f;
    rotationSpeed = 0.0f;
    updateMotors();
    return false;  // continue reversing
  }

  else {
    forwardSpeed = 0.0f;
    strafeSpeed = 0.0f;
    rotationSpeed = 0.0f;
    // reset
    reverseStartX = -1.0f;
    reverseStartY = -1.0f;
    return true;  // reversing complete
  }
}

ButtonState buttonState = BUTTON_IDLE;

float targetApproachX = 0.0f;
float targetApproachY = 0.0f;
float targetApproachTheta = 0.0f;

ButtonConfig currentButtonConfig;
int repeatsDone = 0;
long holdStartTime = -1;

void resetButton() {
  buttonState = BUTTON_IDLE;
  pressStartX = -1.0f;
  pressStartY = -1.0f;
  reverseStartX = -1.0f;
  reverseStartY = -1.0f;
  repeatsDone = 0;
  holdStartTime = -1;
}

void attackButton(float dt, float approachX, float approachY, float approachTheta) {
  switch (buttonState) {
    case BUTTON_IDLE:
      {
        targetApproachX = approachX;
        targetApproachY = approachY;
        targetApproachTheta = approachTheta;
        // currentButtonConfig = config;
        repeatsDone = 0;
        holdStartTime = -1;
        buttonState = BUTTON_ROTATE;
        break;
      }

    case BUTTON_ROTATE:
      {
        Serial.println("ROTATE");
        positionPID(ARENA_WIDTH - targetApproachX, targetApproachY, dt, targetApproachTheta);
        float errorTheta = targetApproachTheta - finalCoords[2];
        while (errorTheta > PI) errorTheta -= 2.0f * PI;
        while (errorTheta < -PI) errorTheta += 2.0f * PI;
        if (abs(errorTheta) < 0.1f) {
          buttonState = BUTTON_PRESSING;
          saveDeadReckon();
        }
        break;
      }

    case BUTTON_PRESSING:
      {
        Serial.println("PRESS");
        if (pressButton()) {
          if (currentButtonConfig.type == HOLD) {
            holdStartTime = millis();
            buttonState = BUTTON_HOLD;
          } else {
            repeatsDone++;
            buttonState = BUTTON_REVERSING;
          }
        }
        break;
      }

    case BUTTON_HOLD:
      {
        Serial.println("HOLD");
        forwardSpeed = 0.3f;
        strafeSpeed = 0.0f;
        rotationSpeed = 0.0f;
        updateMotors();
        if (millis() - holdStartTime >= currentButtonConfig.holdTime) {
          buttonState = BUTTON_REVERSING_HOLD;
        }
        break;
      }

    case BUTTON_REVERSING:
      {
        Serial.println("REV");
        if (reverseFromButton()) {
          if (currentButtonConfig.type == RAPID && repeatsDone < currentButtonConfig.repeatCount) {
            buttonState = BUTTON_PRESSING;
          } else {
            buttonState = BUTTON_COMPLETE;
          }
        }
        break;
      }
    
    case BUTTON_REVERSING_HOLD:
      {
        Serial.println("REV");
        if (reverseFromButtonHold()) {
          if (currentButtonConfig.type == RAPID && repeatsDone < currentButtonConfig.repeatCount) {
            buttonState = BUTTON_PRESSING;
          } else {
            buttonState = BUTTON_COMPLETE;
          }
        }
        break;
      }

    case BUTTON_COMPLETE:
      {
        loadDeadReckon();
        forwardSpeed = 0.0f;
        strafeSpeed = 0.0f;
        rotationSpeed = 0.0f;
        updateMotors();
        break;
      }
  }
}

bool buttonDone() {
  return buttonState == BUTTON_COMPLETE;
}

void setTower(String in) {
  ButtonConfig nexusAttack = {RAPID, 0.0f, 5};
  ButtonConfig towerAttack = {HOLD, 8000.0f, 0};

  if (in.equals("nexus")) {
    currentButtonConfig = nexusAttack;
    Serial.println("NEXUS!");
    buttonLoc = NEXUS;
    if (isblue) {
      // attack red nexus
      // XLoc = NEXUS_RED_X + NEXUS_RED_WIDTH / 2.0f;
      XLoc = NEXUS_RED_X - NEXUS_RED_WIDTH / 2.0f;
      YLoc = NEXUS_RED_Y - NEXUS_BLUE_HEIGHT ;
      ThetaLoc = 0;
    } else {
      // attack blue nexus
      // XLoc = NEXUS_BLUE_X + NEXUS_BLUE_WIDTH / 2.0f;
      XLoc = NEXUS_BLUE_X - NEXUS_BLUE_WIDTH / 2.0f;
      YLoc = NEXUS_BLUE_Y + NEXUS_BLUE_HEIGHT ;
      ThetaLoc = PI;
    }
  } else if (in.equals("high")) {
    currentButtonConfig = towerAttack;
    Serial.println("HIGH!");
    buttonLoc = HIGHTOWER;
    if (isblue) {
      // attack blue button to capture high tower for blue team
      XLoc = RAMP_BUTTON_BLUE_X - BUTTON_PRESS_DISTANCE/2;
      // YLoc = RAMP_BUTTON_BLUE_Y + RAMP_BUTTON_BLUE_HEIGHT / 2.0f;
      YLoc = RAMP_BUTTON_BLUE_Y;
      ThetaLoc = PI / 2.0f;
    } else {
      // attack red button to capture high tower for red team
      XLoc = RAMP_BUTTON_RED_X - BUTTON_PRESS_DISTANCE/2;
      // YLoc = RAMP_BUTTON_RED_Y + RAMP_BUTTON_RED_HEIGHT / 2.0f;
      YLoc = RAMP_BUTTON_RED_Y;
      ThetaLoc = PI / 2.0f;
    }
  } else if (in.equals("low")) {
    currentButtonConfig = towerAttack;
    Serial.println("LOW!");
    buttonLoc = LOWTOWER;
    if (isblue) {
      // XLoc = TOWER_BUTTON_BLUE_X + TOWER_BUTTON_BLUE_WIDTH / 2.0f;
      XLoc = TOWER_BUTTON_BLUE_X;
      YLoc = TOWER_BUTTON_BLUE_Y + TOWER_BUTTON_BLUE_HEIGHT + BUTTON_PRESS_DISTANCE;
      ThetaLoc = PI;
    } else {
      // XLoc = TOWER_BUTTON_RED_X + TOWER_BUTTON_RED_WIDTH / 2.0f;
      XLoc = TOWER_BUTTON_RED_X;
      YLoc = TOWER_BUTTON_RED_Y + TOWER_BUTTON_RED_HEIGHT - BUTTON_PRESS_DISTANCE;
      ThetaLoc = 0;
    }
  }

  resetNav();
  resetButton();
}

void buttonReq(String in) {
  // expecting format of GET /allReq/counter?X?Y HTTP/1.1
  int http = in.indexOf("HTTP");
  in = in.substring(0, http-1);
  int i;
  while ((i = in.indexOf("/")) != -1) {
    in = in.substring(i + 1);
  }
  Serial.println(in);

  setTower(in);

  currentState = BUTTON;

  h.sendhtml("OK");
  wifiPackets++;
}

void all3Req(String in) {
  // expecting format of GET /allReq/counter?X?Y HTTP/1.1
  // int http = in.indexOf("HTTP");
  // in = in.substring(0, http-1);
  // int i;
  // while ((i = in.indexOf("/")) != -1) {
  //   in = in.substring(i + 1);
  // }
  // Serial.println(in);

  setTower("high");
  currentState = ALL3;

  h.sendhtml("OK");
  wifiPackets++;
}

// ~~~~~~~~~ WALL FOLLOWING ~~~~~~~~~~


#define KP_DIST_WF 4.0f
#define KI_DIST_WF 0.05f
#define KD_DIST_WF 0.5f

#define KP_ANGLE_WF 4.0f
#define KI_ANGLE_WF 0.05f
#define KD_ANGLE_WF 0.5f

#define LASER_FRONT 0
#define LASER_RIGHT 1
#define LASER_BACK 2
#define LASER_LEFT 4

#define WF_THRESHOLD_FRONT 0.4f
#define WF_THRESHOLD_BACK 0.2f
#define WF_THRESHOLD_LEFT 0.2f
#define WF_THRESHOLD_RIGHT 0.3f
#define WF_SPEED 1.0f 
#define WF_MAX_LAT 1.0f
#define WF_MIN_STATE_TIME_FRONT 3000
#define WF_MIN_STATE_TIME_BACK 3000
#define WF_MIN_STATE_TIME_LEFT 250
#define WF_MIN_STATE_TIME_RIGHT 250
#define WF_CORRECTION_FRONT_BACK 0.1f
#define WF_CORRECTION_LEFT_RIGHT 0.3f 

#define WF_RIGHT_ADJ_TIME 2000
#define WF_RIGHT_TRANSITION_MARGIN 100

PIDStateWF PIDDist = { 0, 0, 0 };
PIDStateWF PIDAngle = { 0, 0, 0 };

float lastStrafeSpeed = 0.0f;
float lastForwardSpeed = 0.0f;
float lastAngularSpeed = 0.0f;

float StrafeSpeed = 0;
float ForwardSpeed = 0;
float angularSpeed = 0;

WFState wfState = WF_FORWARD;
uint32_t wfStateEnterTime = 0;

float WFPID(PIDStateWF& state, float error, float kp, float ki, float kd, float clamp) {
  uint32_t now = millis();
  float dt = (state.lastTime == 0) ? 0.02f : (now - state.lastTime) / 1000.0f;
  dt = constrain(dt, 0.005f, 0.2f);

  state.integral = constrain(state.integral + error * dt, -clamp / ki, clamp / ki);

  float derivative = (error - state.lastError) / dt;

  state.lastError = error;
  state.lastTime = now;

  return constrain((kp * error) + (ki * state.integral) + (kd * derivative), -clamp, clamp);
}
int rightTime = 0;

void setWFState(WFState newState) {
  wfState = newState;
  wfStateEnterTime = millis();
  // reset PID 
  PIDDist.integral  = 0.0f;
  PIDDist.lastError = 0.0f;
  PIDDist.lastTime  = 0;
  rightTime = 0;
}
// long lastmillis = 0;
void wallFollow(float& StrafeSpeed, float& ForwardSpeed, float& angularSpeed, float dt) {
  // if (millis() - lastmillis > 1000) lastmillis = millis();
  // int dttemp = millis() - lastmillis;
  // lastmillis = millis();
  float frontDist = laserReadings[LASER_FRONT];
  float rightDist = laserReadings[LASER_RIGHT];
  float leftDist  = laserReadings[LASER_LEFT];

  float backDist = laserReadings[LASER_BACK];

  bool forwardBlocked  = (frontDist > 0 && frontDist < WF_THRESHOLD_FRONT);
  bool rightBlocked    = (rightDist > 0 && rightDist < WF_THRESHOLD_RIGHT);
  bool backwardBlocked = (backDist  > 0 && backDist  < WF_THRESHOLD_BACK);
  bool leftBlocked     = (leftDist  > 0 && leftDist  < WF_THRESHOLD_LEFT);

  uint32_t timeInState = millis() - wfStateEnterTime;
  bool canSwitchStatesLeft = (timeInState >= WF_MIN_STATE_TIME_LEFT);
  bool canSwitchStatesRight = (timeInState >= WF_MIN_STATE_TIME_RIGHT);
  bool canSwitchStatesFront = (timeInState >= WF_MIN_STATE_TIME_FRONT);
  bool canSwitchStatesBack = (timeInState >= WF_MIN_STATE_TIME_BACK);

  // transitions
  switch (wfState) {
    case WF_FORWARD:
      Serial.println("WF_FORWARD");
      if (canSwitchStatesFront && forwardBlocked) setWFState(WF_STRAFE_RIGHT);
      break;

    case WF_STRAFE_RIGHT:
      Serial.println("WF_STRAFE_RIGHT");
      if (canSwitchStatesRight && rightBlocked && rightTime > WF_RIGHT_ADJ_TIME + WF_RIGHT_TRANSITION_MARGIN) setWFState(WF_BACKWARD);
      break;

    case WF_BACKWARD:
      Serial.println("WF_BACKWARD");
      if (canSwitchStatesBack && backwardBlocked) setWFState(WF_STRAFE_LEFT);
      break;

    case WF_STRAFE_LEFT:
      Serial.println("WF_STRAFE_LEFT");
      if (canSwitchStatesLeft && leftBlocked) setWFState(WF_FORWARD);
      break;
  }

  // outputs
  switch (wfState) {
    case WF_FORWARD: {
      float distError  = leftDist - WF_THRESHOLD_LEFT;
      float lateralCmd = (leftDist > 0) ? WFPID(PIDDist, distError, KP_DIST_WF, KI_DIST_WF, KD_DIST_WF, WF_MAX_LAT) : WF_MAX_LAT;
      if (abs(lateralCmd) > WF_CORRECTION_FRONT_BACK || leftDist == -1.0) {
        // lateral corrections
        StrafeSpeed = -lateralCmd;
        ForwardSpeed =  0.0f;
      } else {
        StrafeSpeed =  0.0f;
        ForwardSpeed = WF_SPEED;
      }
      angularSpeed = 0.0f;
      break;
    }

    case WF_STRAFE_RIGHT: {
      float distError  = frontDist - (rightTime > WF_RIGHT_ADJ_TIME ? WF_THRESHOLD_FRONT/2:WF_THRESHOLD_FRONT);
      float lateralCmd = (frontDist > 0) ? WFPID(PIDDist, distError, KP_DIST_WF, KI_DIST_WF, KD_DIST_WF, WF_MAX_LAT) : WF_SPEED;
      if (abs(lateralCmd) > WF_CORRECTION_FRONT_BACK) {
        StrafeSpeed =  0.0f;
        ForwardSpeed = lateralCmd;
      } else {
        StrafeSpeed =  WF_SPEED;
        ForwardSpeed =  0.0f;
        rightTime += dt;
      }
      angularSpeed = 0.0f;
      break;
    }

    case WF_BACKWARD: {
      float distError  = rightDist - WF_THRESHOLD_RIGHT;
      float lateralCmd = (rightDist > 0) ? WFPID(PIDDist, distError, KP_DIST_WF, KI_DIST_WF, KD_DIST_WF, WF_MAX_LAT) : WF_MAX_LAT;
      if (timeInState >= WF_MIN_STATE_TIME_BACK && abs(lateralCmd) > WF_CORRECTION_LEFT_RIGHT) {
        StrafeSpeed =  lateralCmd;
        ForwardSpeed =  0.0f;
      } else {
        StrafeSpeed =  0.0f;
        ForwardSpeed = -WF_SPEED;
      }
      angularSpeed = 0.0f;
      break;
    }

    case WF_STRAFE_LEFT: {
      float distError  = backDist - WF_THRESHOLD_BACK;
      float lateralCmd = (backDist > 0) ? WFPID(PIDDist, distError, KP_DIST_WF, KI_DIST_WF, KD_DIST_WF, WF_MAX_LAT) : WF_SPEED;
      if (abs(lateralCmd) > WF_CORRECTION_FRONT_BACK) {
        StrafeSpeed =  0.0f;
        ForwardSpeed = -lateralCmd;
      } else {
        StrafeSpeed = -WF_SPEED;
        ForwardSpeed =  0.0f;
      }
      angularSpeed = 0.0f;
      break;
    }
  }

  lastStrafeSpeed = StrafeSpeed;
  lastForwardSpeed = ForwardSpeed;
  lastAngularSpeed = angularSpeed;
}

void doWallFollow(float dt) {
  float StrafeSpeed, ForwardSpeed, angularSpeed;
  wallFollow(StrafeSpeed, ForwardSpeed, angularSpeed, dt);

  forwardSpeed  = ForwardSpeed;
  strafeSpeed   = StrafeSpeed;
  rotationSpeed = angularSpeed;

  Serial.print("x="); Serial.print(strafeSpeed, 3);
  Serial.print(" y="); Serial.print(forwardSpeed, 3);
  Serial.print(" w="); Serial.println(rotationSpeed, 3);

  updateMotors();
}

// ~~~~~~~~~ CENTRAL COMMAND ~~~~~~~~~~~

int wallAlignTime = 0;

long lastCommandTime = -1;
void centralCommand() {
  if (lastCommandTime == -1) {
    lastCommandTime = millis();
    return;
  }
  float dt = millis() - lastCommandTime;
  lastCommandTime = millis();

  switch(currentState) {
    case MANUAL:
      break;
    case NAVIGATE:
      navigateTo(dt, XLoc, YLoc);
      break;
    case WALLFOLLOW:
      doWallFollow(dt);
      break;
    case BUTTON:
      if (navigateTo(dt, XLoc, YLoc) == 2) {
        attackButton(dt, XLoc, YLoc, ThetaLoc);
      }
      break;
    case ALL3:
      if (navigateTo(dt, XLoc, YLoc) == 2) {
        attackButton(dt, XLoc, YLoc, ThetaLoc);
      }
      if (buttonDone()) {
        Serial.println("button done");
        if (buttonLoc == HIGHTOWER) {
          setTower("low");
        } else if (buttonLoc == LOWTOWER) {
          if (millis() - wallAlignTime < 3000) {
            forwardSpeed = 0.0;
            strafeSpeed = 1.0;
            rotationSpeed = 0.0;
            updateMotors();
            Serial.println(wallAlignTime);
            Serial.println(millis() - wallAlignTime);
            // wallAlignTime += dt;
          } else {
            Serial.println("stopping");
            setTower("nexus");
            viveCoords[0] = 0.0;
            viveCoords[1] = 75.0 INCH;
            viveCoords[2] = PI;
            deadReckonOffsetCoords[0] = 0.0;
            deadReckonOffsetCoords[1] = 0.0;
            deadReckonOffsetCoords[2] = 0.0;
            laserFixCoords[0] = 0.0;
            laserFixCoords[1] = 0.0;
            laserFixCoords[2] = 0.0;

            for (int row = 0; row < GRID_ROWS; row++) {
              for (int col = 0; col < GRID_COLS; col++) {
                setTempCell(row, col, 0);
              }
            }
          }
        }
      } else {
        wallAlignTime = millis();
      }
      break;
  }
}

void setup() {
  Wire.begin(0, 1, 40000);

  Serial.begin(115200);
  Serial.println("started!");

  WiFi.softAPConfig(IPAddress(192, 168, 1, 2), IPAddress(192, 168, 1, 2), IPAddress(255, 255, 255, 0));
  WiFi.softAP("TEAM 1 ESP", __null, 5);
  Serial.println(WiFi.localIP());

  h.begin(80);
  h.attachHandler("/setAll", allReq);
  h.attachHandler("/setLoc", locReq);
  h.attachHandler("/attack", servoReq);
  h.attachHandler("/startWallFollow", wallReq);
  h.attachHandler("/pressButton", buttonReq);
  h.attachHandler("/all3", all3Req);
  h.attachHandler("/changeTeam", teamReq);
  h.attachHandler("/resetPos", resetReq);
  h.attachHandler("/setResetLoc", resetLocReq);
  h.attachHandler("/ ", serveBaseHTML);

  h.attachHandlerSSE("/sse", SSEReq);

  servo.attach(4);

  updateMotors();

  initWalls();
  initLasers();
  calibrateVive();

  // Grid setup
  memset(occupancy_grid, 0, GRID_BYTES);
  memset(inflated_grid, 0, GRID_BYTES);

  setWalls();
  setObstacle(RAILING_X, RAILING_Y, RAILING_WIDTH, RAILING_HEIGHT, 0xE);
  setObstacle(TOWER_X, TOWER_Y, TOWER_WIDTH, TOWER_HEIGHT);
  setObstacle(NEXUS_BLUE_X_ASTAR, NEXUS_BLUE_Y_ASTAR, NEXUS_BLUE_WIDTH_ASTAR, NEXUS_BLUE_HEIGHT_ASTAR);
  setObstacle(NEXUS_RED_X_ASTAR, NEXUS_RED_Y_ASTAR, NEXUS_RED_WIDTH_ASTAR, NEXUS_RED_HEIGHT_ASTAR);
  inflateObstacles();
}

int delayTimesMS[] = {    10,        500,        200,    2000,        25,           100,       500,        100,          1000,    1000};
void (*handlers[])() = { serve, tophat_handle, sendSSE, sendMAP, readDeadReckon, readLasers, readVive, centralCommand, swapServo, renav};
long lastTimes[sizeof(delayTimesMS) / sizeof(delayTimesMS[0])];

void loop() {
  long time = millis();
  for (int i = 0; i < sizeof(delayTimesMS) / sizeof(delayTimesMS[0]); i++) {
    if (time - lastTimes[i] > delayTimesMS[i]) {
      lastTimes[i] = time;  // reset lastTimes
      // Serial.println(delayTimesMS[i]);
      (*handlers[i])();
      time = millis();  // refresh time in case handler took a long time
    }
  }
}