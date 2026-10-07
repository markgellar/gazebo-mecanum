#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <rcl/rcl.h>
#include <esp_timer.h>
#include <stdint.h>
#include <string.h>
#include <queue>

// Set to 1 to print a marker on every loop/publish (floods the serial monitor; only for crash hunting)
#define DEBUG_TRACE 0
// Set to 1 to print how long each loop() handler takes, every 5 s (to find what limits the rates)
#define DEBUG_TIMING 1
#if DEBUG_TRACE
#define TRACE(msg) Serial.println(msg)
#else
#define TRACE(msg)
#endif

#define MOTOR_ADDR 1
#define TOF_CLK 18
#define TOF_DAT 19

int TOF_ADDR[] = { 10, 11, 12, 13, 14 };
bool tofOk[] = { false, false, false, false, false };   // started successfully; dead ones aren't polled

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
  MANUAL,
  NAVIGATE,
  WALLFOLLOW
};

CommandState currentState = MANUAL;

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

#define IMU_ADDR 0x6A
#define MAG_ADDR 0x1C

int motorMax = 100;

int forwardvector[]  = { 1, -1, -1,  1 };
int strafevector[]   = { 1,  1,  1,  1 };
int rotationvector[] = { 1,  1, -1, -1 };

float forwardSpeed = 0;
float strafeSpeed = 0;
float rotationSpeed = 0;

float XLoc = 0;
float YLoc = 0;

float deadScales[] = { 0.3807, -0.4154, 0.4329 * 2 * PI / 1.18411 * 0.963309361 };

float knownStart[] = { 0.0, 0.0, 0.0 };

float laserDefinitions[] = { 0 DEG, 90 DEG, 192 DEG, 168 DEG, 270 DEG };
float laserDists[] =       { 0.12, 0.205/2, 0.125, 0.125, 0.205/2 };

float deadReckonOffsetCoords[] = { 0.0, 0.0, 0.0 };
float finalCoords[] = { 0.0, 0.0, 0.0 };   // x right, y forward (m), heading CLOCKWISE-positive (rad)

// Wheel motion since the last /odom publish, in ROS's robot frame: +x right (m), +y forward (m),
// rotation counterclockwise-positive (rad). Turned into velocities when published.
float odomAccum[] = { 0.0, 0.0, 0.0 };
uint32_t lastOdomPublishMs = 0;

float laserMin = 0.05;
float laserMax = 2.0;
float laserReadings[] = { 0.0, 0.0, 0.0, 0.0, 0.0 };   // from robot centre (sensor range + laserDists), for wall following
float rawRanges[] = { 0.0, 0.0, 0.0, 0.0, 0.0 };       // from the sensor face, for ROS (Range msgs use the sensor's frame)
int64_t rangeStampNs[] = { 0, 0, 0, 0, 0 };            // when each reading was taken (ROS time)
bool rangeFresh[] = { false, false, false, false, false };   // not yet published
int64_t rosTimeNs();

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

void updateCoords();

void updateMotors() {
  float maxMotorReq = abs(forwardSpeed) + abs(strafeSpeed) + abs(rotationSpeed);
  if (maxMotorReq < 1.0) maxMotorReq = 1.0;

  Wire.beginTransmission(MOTOR_ADDR);
  Wire.write('M');
  for (int i = 0; i < 4; i++) {
    char motorspeed = ((forwardSpeed * forwardvector[i] + rotationSpeed * rotationvector[i] + strafeSpeed * strafevector[i]) / maxMotorReq * motorMax + 125);
    Wire.write(motorspeed);
  }
  Wire.endTransmission();
}

void initLasers() {
  pinMode(TOF_CLK, OUTPUT);
  pinMode(TOF_DAT, OUTPUT);

  digitalWrite(TOF_DAT, LOW);
  for (int i = 0; i < 6; i++) {
    delay(10);
    digitalWrite(TOF_CLK, HIGH);
    delay(10);
    digitalWrite(TOF_CLK, LOW);
  }

  delay(10);
  digitalWrite(TOF_DAT, HIGH);

  for (int i = 0; i < 5; i++) {
    delay(10);
    digitalWrite(TOF_CLK, HIGH);
    delay(10);
    digitalWrite(TOF_CLK, LOW);
    delay(10);
    tofOk[i] = tof[i].begin(TOF_ADDR[i], true, &Wire, Adafruit_VL53L0X::VL53L0X_SENSE_DEFAULT);
    Serial.println(tofOk[i]);
  }

  delay(10);

  for (int i = 0; i < 5; i++) {
    if (tofOk[i]) tof[i].startRangeContinuous();
  }
  // A sensor that didn't start would cost an I2C timeout on every poll and stall the whole loop
  Serial.printf("ToF sensors started: %d %d %d %d %d (front, right, rear_right, rear_left, left)\n",
                tofOk[0], tofOk[1], tofOk[2], tofOk[3], tofOk[4]);
}

void readLasers() {
  int ambad = 0, active = 0;

  for (int i = 0; i < 5; i++) {
    if (!tofOk[i]) continue;
    active++;
    if (tof[i].isRangeComplete()) {
      int range = tof[i].readRangeResult();

      Wire.beginTransmission(TOF_ADDR[i]);
      byte error = Wire.endTransmission();

      if (error != 0) {
        ambad++;
      }

      if (range < 3000 && range > 30) {
        laserReadings[i] = range / 1000.0 + laserDists[i];
        rawRanges[i] = range / 1000.0;
      } else {
        laserReadings[i] = -1.0;
        rawRanges[i] = -1.0;
      }
      rangeStampNs[i] = rosTimeNs();
      rangeFresh[i] = true;
    }
  }

  if (active > 0 && ambad == active) {   // every working sensor stopped answering: restart them
    Serial.printf("ToF restart: all %d working sensors failed the I2C check (stalls the loop ~2 s)\n", active);
    initLasers();
  }
}

byte readbuffer[100];
void readDeadReckon() {
  Wire.requestFrom(MOTOR_ADDR, 4 * 3 + 1);

  int i = 0;
  while (Wire.available()) {
    byte c = Wire.read();
    readbuffer[i++] = c;
  }

  if (readbuffer[0] != 'D') return;

  float forwardsDead = 0.0;
  float strafeDead = 0.0;
  float rotateDead = 0.0;

  memcpy(&forwardsDead, &readbuffer[1], 4);
  memcpy(&strafeDead, &readbuffer[1 + 4], 4);
  memcpy(&rotateDead, &readbuffer[1 + 4 + 4], 4);

  float curRot = finalCoords[2];
  deadReckonOffsetCoords[0] += (forwardsDead * sin(curRot) * deadScales[0] - strafeDead * cos(curRot) * deadScales[1]);
  deadReckonOffsetCoords[1] += (forwardsDead * cos(curRot) * deadScales[0] + strafeDead * sin(curRot) * deadScales[1]);
  deadReckonOffsetCoords[2] += (rotateDead) * deadScales[2];

  // Same increments in the robot frame, for the ROS velocities. At heading 0 the update above moves
  // x by -strafe*s1 (right) and y by forward*s0; the firmware heading turns clockwise, ROS yaw doesn't.
  odomAccum[0] += -strafeDead * deadScales[1];
  odomAccum[1] += forwardsDead * deadScales[0];
  odomAccum[2] += -rotateDead * deadScales[2];

  updateCoords();
}

void updateCoords() {
  finalCoords[0] = deadReckonOffsetCoords[0] + knownStart[0];
  finalCoords[1] = deadReckonOffsetCoords[1] + knownStart[1];
  finalCoords[2] = deadReckonOffsetCoords[2] + knownStart[2];
}

// ~~~~~~~~~~~~~~~~~~~ NAVIGATION ~~~~~~~~~~~~~~~~~~~~

#define INFLATE_RADIUS 6
#define INFLATE_RADIUS_WALL 5
#define INFLATE_COST 2.0f
#define INFLATE_HIGH_COST 6

#define TEMP_MAX_COST 10.0f

#define MAX_OPEN (GRID_COLS * GRID_ROWS)
#define MAX_PATH (GRID_COLS * GRID_ROWS)

#define WAYPOINT_THRESHOLD 0.05f

Node cameFrom[GRID_ROWS][GRID_COLS];

void setWalls() {
  for (int col = 0; col < GRID_COLS; col++) {
    setCell(0, col, 0xF);
    setCell((GRID_ROWS - 1), col, 0xF);
  }
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

  if (col1 < col0) col1 = col0;
  if (row1 < row0) row1 = row0;

  for (int row = row0; row <= row1; row++) {
    for (int col = col0; col <= col1; col++) {
      setCell(row, col, val);
    }
  }
}

void inflateObstacles() {
  memcpy(inflated_grid, occupancy_grid, GRID_BYTES);
  for (int row = 0; row < GRID_ROWS; row++) {
    for (int col = 0; col < GRID_COLS; col++) {
      int cNum = getCell(row, col);
      if (cNum) {
        int rad = (cNum == 0xF) ? INFLATE_RADIUS_WALL : INFLATE_RADIUS;
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

// A* pathfinding
float heuristic(int row, int col, int goalRow, int goalCol) {
  return pow((float) ((row - goalRow)*(row - goalRow) + (col - goalCol)*(col - goalCol)), 0.5);
}

bool closedSet[GRID_ROWS][GRID_COLS];

int pathRow[MAX_PATH];
int pathCol[MAX_PATH];
int pathLength = 0;
int currentWaypoint = 0;

auto cmp = [](Node left, Node right) { return (left.f) > (right.f); };
std::priority_queue<Node, std::vector<Node>, decltype(cmp)> lambda_pq(cmp);

void aStar2(int startRow, int startCol, int goalRow, int goalCol) {
  lambda_pq = std::priority_queue<Node, std::vector<Node>, decltype(cmp)>(cmp);

  memset(closedSet, false, sizeof(closedSet));
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

  lambda_pq.push(startNode);
  cameFrom[startRow][startCol] = startNode;

  while (!lambda_pq.empty()) {
    Node currentNode = lambda_pq.top();
    lambda_pq.pop();
    if (closedSet[currentNode.row][currentNode.col]) {
      continue;
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
          pathLength--;
        }
      }
      pathRow[pathLength] = startRow;
      pathCol[pathLength] = startCol;
      pathLength++;

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

      if (closedSet[neighborRow][neighborCol]) {
        if(cameFrom[neighborNode.row][neighborNode.col].f > neighborNode.f) {
          lambda_pq.push(neighborNode);
          cameFrom[neighborNode.row][neighborNode.col] = neighborNode;
        }
      } else {
        lambda_pq.push(neighborNode);
        cameFrom[neighborNode.row][neighborNode.col] = neighborNode;
      }
    }
  }
}

// Position PID
float KP[] = { 5.0f, 8.0f, 2.0f };
float KI[] = { 0.001f, 0.0005f, 0.001f };
float KD[] = { 0.00f, 0.00f, 0.00f };

float positionIntegral[] = { 0.0f, 0.0f, 0.0f };
float positionLastError[] = { 0.0f, 0.0f, 0.0f };

float positionPID(float goalX, float goalY, float dt, float goalTheta = -999.0f, bool highprecision = true) {
  if (goalTheta == -999.0f) {
    goalTheta = -atan2(goalX - finalCoords[0], goalY - finalCoords[1]);
  }

  float errorWorld[] = {
    (goalX - finalCoords[0]),
    (goalY - finalCoords[1]),
    (goalTheta - finalCoords[2])
  };

  while (errorWorld[2] > PI) errorWorld[2] -= 2.0f * PI;
  while (errorWorld[2] < -PI) errorWorld[2] += 2.0f * PI;

  float errorBody[] = {
    (-errorWorld[0] * cos(finalCoords[2]) - errorWorld[1] * sin(finalCoords[2])),
    (-errorWorld[0] * sin(finalCoords[2]) + errorWorld[1] * cos(finalCoords[2])),
    (errorWorld[2])
  };

  float velocity[] = { 0.0f, 0.0f, 0.0f };
  for (int i = 0; i < 3; i++) {
    float proportionalTerm = KP[i] * errorBody[i];

    positionIntegral[i] += errorBody[i] * dt;
    float integralTerm = KI[i] * positionIntegral[i];

    float derivativeTerm = KD[i] * (errorBody[i] - positionLastError[i]) / dt;

    velocity[i] = proportionalTerm + integralTerm + derivativeTerm;
    positionLastError[i] = errorBody[i];

    if (positionIntegral[i] > 10.0f) positionIntegral[i] = 10.0f;
    if (positionIntegral[i] < -10.0f) positionIntegral[i] = -10.0f;
  }

  double maxVel = abs(velocity[1]) + abs(velocity[0]);
  if (highprecision) maxVel = 1;

  forwardSpeed = velocity[1] / maxVel;
  strafeSpeed = velocity[0] / maxVel;
  rotationSpeed = velocity[2];
  updateMotors();

  return pow(errorBody[0]*errorBody[0] + errorBody[1]*errorBody[1], 0.5);
}

void followPath(float dt) {
  if (currentWaypoint >= pathLength) {
    return;
  }

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

  if (error < (currentWaypoint==pathLength-1 ? WAYPOINT_THRESHOLD/3 : WAYPOINT_THRESHOLD)) {
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
        int startCol = metersToCol(finalCoords[0]);
        int goalRow = metersToRow(goalY);
        int goalCol = metersToCol(goalX);
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
  return -1;
}

void renav() {
  if (navState == NAV_NAVIGATING) {
    int startRow = metersToRow(finalCoords[1]);
    int startCol = metersToCol(finalCoords[0]);
    int goalRow = metersToRow(YLoc);
    int goalCol = metersToCol(XLoc);
    Serial.println("astar from " + String(startRow) + ", " + String(startCol) + " to " + String(goalRow) + ", " + String(goalCol));
    aStar2(startRow, startCol, goalRow, goalCol);
  }
}

// ~~~~~~~~~ WALL FOLLOWING ~~~~~~~~~~

#define KP_DIST_WF 4.0f
#define KI_DIST_WF 0.05f
#define KD_DIST_WF 0.5f

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
  PIDDist.integral  = 0.0f;
  PIDDist.lastError = 0.0f;
  PIDDist.lastTime  = 0;
  rightTime = 0;
}

void wallFollow(float& StrafeSpeed, float& ForwardSpeed, float& angularSpeed, float dt) {
  float frontDist = laserReadings[LASER_FRONT];
  float rightDist = laserReadings[LASER_RIGHT];
  float leftDist  = laserReadings[LASER_LEFT];
  float backDist  = laserReadings[LASER_BACK];

  bool forwardBlocked  = (frontDist > 0 && frontDist < WF_THRESHOLD_FRONT);
  bool rightBlocked    = (rightDist > 0 && rightDist < WF_THRESHOLD_RIGHT);
  bool backwardBlocked = (backDist  > 0 && backDist  < WF_THRESHOLD_BACK);
  bool leftBlocked     = (leftDist  > 0 && leftDist  < WF_THRESHOLD_LEFT);

  uint32_t timeInState = millis() - wfStateEnterTime;
  bool canSwitchStatesLeft = (timeInState >= WF_MIN_STATE_TIME_LEFT);
  bool canSwitchStatesRight = (timeInState >= WF_MIN_STATE_TIME_RIGHT);
  bool canSwitchStatesFront = (timeInState >= WF_MIN_STATE_TIME_FRONT);
  bool canSwitchStatesBack = (timeInState >= WF_MIN_STATE_TIME_BACK);

  switch (wfState) {
    case WF_FORWARD:
      if (canSwitchStatesFront && forwardBlocked) setWFState(WF_STRAFE_RIGHT);
      break;
    case WF_STRAFE_RIGHT:
      if (canSwitchStatesRight && rightBlocked && rightTime > WF_RIGHT_ADJ_TIME + WF_RIGHT_TRANSITION_MARGIN) setWFState(WF_BACKWARD);
      break;
    case WF_BACKWARD:
      if (canSwitchStatesBack && backwardBlocked) setWFState(WF_STRAFE_LEFT);
      break;
    case WF_STRAFE_LEFT:
      if (canSwitchStatesLeft && leftBlocked) setWFState(WF_FORWARD);
      break;
  }

  switch (wfState) {
    case WF_FORWARD: {
      float distError  = leftDist - WF_THRESHOLD_LEFT;
      float lateralCmd = (leftDist > 0) ? WFPID(PIDDist, distError, KP_DIST_WF, KI_DIST_WF, 0.5f, WF_MAX_LAT) : WF_MAX_LAT;
      if (abs(lateralCmd) > WF_CORRECTION_FRONT_BACK || leftDist == -1.0) {
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
      float distError  = frontDist - (rightTime > WF_RIGHT_ADJ_TIME ? WF_THRESHOLD_FRONT/2 : WF_THRESHOLD_FRONT);
      float lateralCmd = (frontDist > 0) ? WFPID(PIDDist, distError, KP_DIST_WF, KI_DIST_WF, 0.5f, WF_MAX_LAT) : WF_SPEED;
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
      float lateralCmd = (rightDist > 0) ? WFPID(PIDDist, distError, KP_DIST_WF, KI_DIST_WF, 0.5f, WF_MAX_LAT) : WF_MAX_LAT;
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
      float lateralCmd = (backDist > 0) ? WFPID(PIDDist, distError, KP_DIST_WF, KI_DIST_WF, 0.5f, WF_MAX_LAT) : WF_SPEED;
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

  updateMotors();
}

// ~~~~~~~~~ CENTRAL COMMAND ~~~~~~~~~~~

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
  }
}

#define IMU_ADDR 0x6A
#define MAG_ADDR 0x1C

// Madgwick filter state
float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
float beta = 0.04f;

// Latest IMU readings (for micro-ROS publishing)
float imu_ax, imu_ay, imu_az;
float imu_gx, imu_gy, imu_gz;

// Gyro bias (auto-calibrated at startup)
float gyro_bias_x = 0.0f;
float gyro_bias_y = 0.0f;
float gyro_bias_z = 0.0f;
bool gyro_calibrated = false;

// Mag hard-iron offsets (from calibration procedure)
float mag_offset_x = 66.53f;
float mag_offset_y = 6.99f;
float mag_offset_z = -3.19f;

// Set to true to use magnetometer for absolute heading
bool use_mag = true;

void writeReg(uint8_t addr, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

void initIMU() {
  writeReg(IMU_ADDR, 0x10, 0x48);  // Accel: 104Hz, ±4g
  writeReg(IMU_ADDR, 0x11, 0x40);  // Gyro: 104Hz, ±250dps
  writeReg(MAG_ADDR, 0x20, 0x1C);  // Mag: 155Hz, high-perf XY
  writeReg(MAG_ADDR, 0x21, 0x00);  // Mag: ±4 gauss
  writeReg(MAG_ADDR, 0x22, 0x00);  // Mag: continuous mode
  writeReg(MAG_ADDR, 0x23, 0x0C);  // Mag: high-perf Z
  Serial.println("IMU initialized");
}

void calibrateGyro() {
  Serial.println("Calibrating gyro - hold still...");
  float sum_x = 0, sum_y = 0, sum_z = 0;
  int samples = 500;

  for (int i = 0; i < samples; i++) {
    Wire.beginTransmission(IMU_ADDR);
    Wire.write(0x22);
    Wire.endTransmission();
    uint8_t received = Wire.requestFrom(IMU_ADDR, 6);
    if (received < 6) { i--; continue; }

    int16_t gx_raw = Wire.read() | (Wire.read() << 8);
    int16_t gy_raw = Wire.read() | (Wire.read() << 8);
    int16_t gz_raw = Wire.read() | (Wire.read() << 8);

    sum_x += gx_raw * 0.000153f;
    sum_y += gy_raw * 0.000153f;
    sum_z += gz_raw * 0.000153f;

    delay(2);
  }

  gyro_bias_x = sum_x / samples;
  gyro_bias_y = sum_y / samples;
  gyro_bias_z = sum_z / samples;
  gyro_calibrated = true;

  Serial.printf("Gyro bias: %.5f %.5f %.5f\n",
                gyro_bias_x, gyro_bias_y, gyro_bias_z);
}

void readMag(float &mx, float &my, float &mz) {
  Wire.beginTransmission(MAG_ADDR);
  Wire.write(0x28);
  Wire.endTransmission();

  uint8_t received = Wire.requestFrom(MAG_ADDR, 6);
  if (received < 6) { mx = my = mz = 0; return; }

  int16_t mx_raw = Wire.read() | (Wire.read() << 8);
  int16_t my_raw = Wire.read() | (Wire.read() << 8);
  int16_t mz_raw = Wire.read() | (Wire.read() << 8);

  // ±4 gauss: 6842 LSB/gauss, convert to microtesla
  mx = mx_raw * 0.0146f;
  my = my_raw * 0.0146f;
  mz = mz_raw * 0.0146f;
}

void madgwickUpdate6DOF(float gx, float gy, float gz,
                        float ax, float ay, float az,
                        float dt)
{
  float recipNorm;
  float s0, s1, s2, s3;
  float qDot1, qDot2, qDot3, qDot4;

  qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
  qDot2 = 0.5f * ( q0 * gx + q2 * gz - q3 * gy);
  qDot3 = 0.5f * ( q0 * gy - q1 * gz + q3 * gx);
  qDot4 = 0.5f * ( q0 * gz + q1 * gy - q2 * gx);

  float aNorm = ax * ax + ay * ay + az * az;
  if (aNorm > 0.01f) {
    recipNorm = 1.0f / sqrtf(aNorm);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    float _2q0 = 2.0f * q0, _2q1 = 2.0f * q1;
    float _2q2 = 2.0f * q2, _2q3 = 2.0f * q3;
    float _4q0 = 4.0f * q0, _4q1 = 4.0f * q1;
    float _4q2 = 4.0f * q2;
    float _8q1 = 8.0f * q1, _8q2 = 8.0f * q2;
    float q0q0 = q0 * q0, q1q1 = q1 * q1;
    float q2q2 = q2 * q2, q3q3 = q3 * q3;

    s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
    s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
    s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
    s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;

    recipNorm = 1.0f / sqrtf(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
    s0 *= recipNorm; s1 *= recipNorm;
    s2 *= recipNorm; s3 *= recipNorm;

    qDot1 -= beta * s0;
    qDot2 -= beta * s1;
    qDot3 -= beta * s2;
    qDot4 -= beta * s3;
  }

  q0 += qDot1 * dt;
  q1 += qDot2 * dt;
  q2 += qDot3 * dt;
  q3 += qDot4 * dt;

  recipNorm = 1.0f / sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;
}

void madgwickUpdate9DOF(float gx, float gy, float gz,
                        float ax, float ay, float az,
                        float mx, float my, float mz,
                        float dt)
{
  float recipNorm;
  float s0, s1, s2, s3;
  float qDot1, qDot2, qDot3, qDot4;
  float hx, hy;
  float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz;
  float _4bx, _4bz, _2q0, _2q1, _2q2, _2q3;
  float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;

  qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
  qDot2 = 0.5f * ( q0 * gx + q2 * gz - q3 * gy);
  qDot3 = 0.5f * ( q0 * gy - q1 * gz + q3 * gx);
  qDot4 = 0.5f * ( q0 * gz + q1 * gy - q2 * gx);

  float aNorm = ax * ax + ay * ay + az * az;
  if (aNorm < 0.01f) goto integrate;
  recipNorm = 1.0f / sqrtf(aNorm);
  ax *= recipNorm; ay *= recipNorm; az *= recipNorm;

  float mNorm;
  mNorm = mx * mx + my * my + mz * mz;
  if (mNorm < 0.01f) goto integrate;
  recipNorm = 1.0f / sqrtf(mNorm);
  mx *= recipNorm; my *= recipNorm; mz *= recipNorm;

  _2q0mx = 2.0f * q0 * mx; _2q0my = 2.0f * q0 * my; _2q0mz = 2.0f * q0 * mz;
  _2q1mx = 2.0f * q1 * mx;
  _2q0 = 2.0f * q0; _2q1 = 2.0f * q1; _2q2 = 2.0f * q2; _2q3 = 2.0f * q3;
  q0q0 = q0 * q0; q0q1 = q0 * q1; q0q2 = q0 * q2; q0q3 = q0 * q3;
  q1q1 = q1 * q1; q1q2 = q1 * q2; q1q3 = q1 * q3;
  q2q2 = q2 * q2; q2q3 = q2 * q3; q3q3 = q3 * q3;

  hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
  hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;
  _2bx = sqrtf(hx * hx + hy * hy);
  _2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
  _4bx = 2.0f * _2bx; _4bz = 2.0f * _2bz;

  s0 = -_2q2 * (2.0f * q1q3 - _2q0 * q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2 * q3 - ay) - _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
  s1 = _2q3 * (2.0f * q1q3 - _2q0 * q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2 * q3 - ay) - 4.0f * q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
  s2 = -_2q0 * (2.0f * q1q3 - _2q0 * q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2 * q3 - ay) - 4.0f * q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
  s3 = _2q1 * (2.0f * q1q3 - _2q0 * q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2 * q3 - ay) + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);

  recipNorm = 1.0f / sqrtf(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
  s0 *= recipNorm; s1 *= recipNorm; s2 *= recipNorm; s3 *= recipNorm;

  qDot1 -= beta * s0; qDot2 -= beta * s1;
  qDot3 -= beta * s2; qDot4 -= beta * s3;

integrate:
  q0 += qDot1 * dt; q1 += qDot2 * dt;
  q2 += qDot3 * dt; q3 += qDot4 * dt;

  recipNorm = 1.0f / sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= recipNorm; q1 *= recipNorm;
  q2 *= recipNorm; q3 *= recipNorm;
}

void readIMU() {
  TRACE("IMU1");
  if (!gyro_calibrated) return;

  // Read accel (0x28-0x2D)
  TRACE("IMU2");
  Wire.beginTransmission(IMU_ADDR);
  Wire.write(0x28);
  Wire.endTransmission();
  uint8_t received = Wire.requestFrom(IMU_ADDR, 6);
  if (received < 6) return;
  TRACE("IMU3");
  int16_t ax_raw = Wire.read() | (Wire.read() << 8);
  int16_t ay_raw = Wire.read() | (Wire.read() << 8);
  int16_t az_raw = Wire.read() | (Wire.read() << 8);
  TRACE("IMU4");
  Serial.printf("Free heap: %lu\n", esp_get_free_heap_size());

  // Read gyro (0x22-0x27)
  Serial.println("IMU4a");
  Wire.beginTransmission(IMU_ADDR);
  Serial.println("IMU4b");
  Wire.write(0x22);
  Serial.println("IMU4c");
  Wire.endTransmission();
  Serial.println("IMU4d");
  received = Wire.requestFrom(IMU_ADDR, 6);
  Serial.println("IMU4e");
  if (received < 6) return;
  TRACE("IMU5");
  int16_t gx_raw = Wire.read() | (Wire.read() << 8);
  int16_t gy_raw = Wire.read() | (Wire.read() << 8);
  int16_t gz_raw = Wire.read() | (Wire.read() << 8);

  TRACE("IMU6");
  imu_ax = ax_raw * 0.001197f;
  imu_ay = ay_raw * 0.001197f;
  imu_az = az_raw * 0.001197f;
  imu_gx = gx_raw * 0.000153f - gyro_bias_x;
  imu_gy = gy_raw * 0.000153f - gyro_bias_y;
  imu_gz = gz_raw * 0.000153f - gyro_bias_z;

  // High beta for first 3 seconds to converge heading, then lower for stability
  TRACE("IMU7");
  static int imuCount = 0;
  imuCount++;

  TRACE("IMU8");
  if (imuCount < 500) {
    beta = 0.5f;            // first 5 seconds: converge fast
  } else if (imuCount < 1000) {
    // ramp from 0.5 to 0.1 over next 5 seconds
    beta = 0.5f - (imuCount - 500) * 0.0008f;
  } else {
    beta = 0.1f;
  }

  if (use_mag) {
    float mx, my, mz;
    readMag(mx, my, mz);
    mx -= mag_offset_x;
    my -= mag_offset_y;
    mz -= mag_offset_z;

    float mx_corrected = my;
    float my_corrected = mx;
    float mz_corrected = mz;

    madgwickUpdate9DOF(imu_gx, imu_gy, imu_gz,
                       imu_ax, imu_ay, imu_az,
                       mx_corrected, my_corrected, mz_corrected,
                       0.01f);
  } else {
    madgwickUpdate6DOF(imu_gx, imu_gy, imu_gz,
                       imu_ax, imu_ay, imu_az,
                       0.01f);
  }
  TRACE("IMU9");

  static int printCount = 0;
  if (++printCount >= 50) {
    Serial.printf("Quat: w=%.3f x=%.3f y=%.3f z=%.3f  Heading: %.1f deg\n",
                  q0, q1, q2, q3,
                  atan2(2.0f * (q0 * q3 + q1 * q2),
                        1.0f - 2.0f * (q2 * q2 + q3 * q3)) * 180.0f / PI);
    printCount = 0;
  }
}

// =========== MICRO-ROS Publishing ===========

#include <micro_ros_utilities/type_utilities.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <sensor_msgs/msg/range.h>
#include <sensor_msgs/msg/imu.h>
#include <nav_msgs/msg/odometry.h>
#include <WiFi.h>
// #include <std_msgs/msg/float32_multi_array.h>

#define AP_AGENT_IP "192.168.4.2"   // the laptop, when it joins our own access point
#define AGENT_PORT 9999
#define AGENT_PORT_STR "9999"

// micro-ROS objects
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rclc_executor_t executor;

rcl_publisher_t imu_pub;
rcl_publisher_t tof_pub;
rcl_publisher_t odom_pub;

sensor_msgs__msg__Imu imu_msg;
sensor_msgs__msg__Range tof_msg;
nav_msgs__msg__Odometry odom_msg;

bool microros_connected = false;

const char* tof_frame_ids[] = {
  "front_tof_link",
  "right_tof_link",
  "rear_right_tof_link",
  "rear_left_tof_link",
  "left_tof_link"
};

const char* tof_topic_names[] = {
  "front_tof/range",
  "right_tof/range",
  "rear_right_tof/range",
  "rear_left_tof/range",
  "left_tof/range"
};

#include <esp_wifi.h>
#include <esp_event.h>
#include <nvs_flash.h>

// Home WiFi credentials and the laptop's address on it live in wifi_secrets.h (gitignored; copy
// wifi_secrets.h.example). Without that file the robot always runs its own access point, as before.
#if __has_include("wifi_secrets.h")
#include "wifi_secrets.h"
#endif

char agent_addr[16] = AP_AGENT_IP;   // micro-ROS agent (the laptop); set by initWiFi()

static EventGroupHandle_t wifi_events;
static const EventBits_t WIFI_GOT_IP = BIT0;
static bool home_wifi_mode = false;

static void wifi_event_handler(void* arg, esp_event_base_t base, int32_t id, void* data) {
  if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED && home_wifi_mode) {
    xEventGroupClearBits(wifi_events, WIFI_GOT_IP);
    esp_wifi_connect();   // keep trying: routers drop clients now and then
  } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
    xEventGroupSetBits(wifi_events, WIFI_GOT_IP);
  }
}

// Station mode: join the home network like any other device, so the laptop keeps its internet
bool joinHomeWifi(int timeout_ms) {
#ifdef HOME_WIFI_SSID
  esp_netif_t* sta = esp_netif_create_default_wifi_sta();
  esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL);
  esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, NULL);

  wifi_config_t wifi_config = {};
  strncpy((char*)wifi_config.sta.ssid, HOME_WIFI_SSID, sizeof(wifi_config.sta.ssid));
  strncpy((char*)wifi_config.sta.password, HOME_WIFI_PASSWORD, sizeof(wifi_config.sta.password));

  home_wifi_mode = true;
  esp_wifi_set_mode(WIFI_MODE_STA);
  esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
  esp_wifi_start();
  esp_wifi_set_ps(WIFI_PS_NONE);   // no power-save naps: they add ~100 ms latency to every message
  esp_wifi_connect();

  Serial.printf("Joining home WiFi \"%s\"...\n", HOME_WIFI_SSID);
  EventBits_t bits = xEventGroupWaitBits(wifi_events, WIFI_GOT_IP, pdFALSE, pdTRUE, pdMS_TO_TICKS(timeout_ms));
  if (bits & WIFI_GOT_IP) {
    esp_netif_ip_info_t ip;
    esp_netif_get_ip_info(sta, &ip);
    Serial.printf("Joined home WiFi as " IPSTR ", agent expected at %s\n", IP2STR(&ip.ip), HOME_AGENT_IP);
    return true;
  }
  home_wifi_mode = false;
  esp_wifi_disconnect();
  esp_wifi_stop();
  Serial.println("Home WiFi not available");
#endif
  return false;
}

// Access point mode: the robot runs its own network "MecanumRobot" (laptop joins it as 192.168.4.2)
void startAccessPoint() {
  esp_netif_create_default_wifi_ap();

  wifi_config_t wifi_config = {};
  strcpy((char*)wifi_config.ap.ssid, "MecanumRobot");
  strcpy((char*)wifi_config.ap.password, "robot12345");
  wifi_config.ap.ssid_len = strlen("MecanumRobot");
  wifi_config.ap.channel = 1;
  wifi_config.ap.authmode = WIFI_AUTH_WPA2_PSK;
  wifi_config.ap.max_connection = 4;

  esp_wifi_set_mode(WIFI_MODE_AP);
  esp_wifi_set_config(WIFI_IF_AP, &wifi_config);
  esp_wifi_start();

  strcpy(agent_addr, AP_AGENT_IP);
  Serial.println("AP \"MecanumRobot\" started at 192.168.4.1, agent expected at " AP_AGENT_IP);
}

// Home WiFi first (laptop keeps internet); our own access point if it isn't there (e.g. at a demo)
void initWiFi() {
  nvs_flash_init();
  esp_netif_init();
  esp_event_loop_create_default();
  wifi_events = xEventGroupCreate();

  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  esp_wifi_init(&cfg);

#ifdef HOME_WIFI_SSID
  if (joinHomeWifi(10000)) {
    strncpy(agent_addr, HOME_AGENT_IP, sizeof(agent_addr) - 1);
    return;
  }
#endif
  startAccessPoint();
}

void initMessages();

#include <WiFiUdp.h>
#include <rmw_microros/rmw_microros.h>
#include <rmw_microxrcedds_c/config.h>

WiFiUDP udp_client;
IPAddress agent_ip;

#include <lwip/sockets.h>
#include <uxr/client/profile/transport/custom/custom_transport.h>

int uros_sock = -1;
struct sockaddr_in uros_agent_addr;

extern "C" {

bool uros_open(uxrCustomTransport* transport) {
  uros_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if (uros_sock < 0) return false;

  struct sockaddr_in local = {};
  local.sin_family = AF_INET;
  local.sin_port = htons(0);
  local.sin_addr.s_addr = INADDR_ANY;
  bind(uros_sock, (struct sockaddr*)&local, sizeof(local));

  uros_agent_addr.sin_family = AF_INET;
  uros_agent_addr.sin_port = htons(AGENT_PORT);
  inet_aton(agent_addr, &uros_agent_addr.sin_addr);

  return true;
}

bool uros_close(uxrCustomTransport* transport) {
  if (uros_sock >= 0) close(uros_sock);
  uros_sock = -1;
  return true;
}

size_t uros_write(uxrCustomTransport* transport, const uint8_t* buf, size_t len, uint8_t* errcode) {
  int sent = sendto(uros_sock, buf, len, 0,
                    (struct sockaddr*)&uros_agent_addr, sizeof(uros_agent_addr));
  if (sent <= 0) { *errcode = 1; return 0; }
  return sent;
}

size_t uros_read(uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* errcode) {
  struct timeval tv;
  tv.tv_sec = timeout / 1000;
  tv.tv_usec = (timeout % 1000) * 1000;
  setsockopt(uros_sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

  int received = recvfrom(uros_sock, buf, len, 0, NULL, NULL);
  if (received <= 0) { *errcode = 1; return 0; }
  return received;
}

}

// Try to connect to the micro-ROS agent. Called every few seconds from loop() until it succeeds, so
// the agent and the PC's WiFi can come up in any order. A quick ping first keeps a failed attempt to
// ~100 ms instead of blocking the sensors for seconds.
void connectMicroROS() {
  static int attempts = 0;
  attempts++;

  allocator = rcl_get_default_allocator();

  rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
  rcl_ret_t rc2 __attribute__((unused)) = rcl_init_options_init(&init_options, allocator);
  rmw_init_options_t* rmw_options = rcl_init_options_get_rmw_init_options(&init_options);
  rmw_uros_options_set_udp_address(agent_addr, AGENT_PORT_STR, rmw_options);

  if (rmw_uros_ping_agent_options(100, 1, rmw_options) != RMW_RET_OK) {
    if (attempts % 10 == 1) {   // don't flood the log: report every 10th try (~30 s)
      Serial.printf("micro-ROS agent not reachable at %s:%s (attempt %d), retrying...\n", agent_addr, AGENT_PORT_STR, attempts);
    }
    rcl_init_options_fini(&init_options);
    return;
  }

  Serial.printf("Agent found (attempt %d), connecting...\n", attempts);
  rcl_ret_t ret;
  ret = rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator);
  if (ret != RCL_RET_OK) {
    Serial.printf("Support init failed: %ld\n", ret);
    rcl_init_options_fini(&init_options);
    return;
  }

  Serial.println("Creating node...");
  // ret = rclc_node_init_default(&node, "mecanum_robot", "", &support);
  node = rcl_get_zero_initialized_node();
  rcl_node_options_t node_options = rcl_node_get_default_options();
  node_options.enable_rosout = false;
  node_options.use_global_arguments = false;
  ret = rcl_node_init(&node, "mecanum_robot", "", &support.context, &node_options);
  if (ret != RCL_RET_OK) {
    Serial.printf("Node init failed: %ld\n", ret);
    rclc_support_fini(&support);
    return;
  }

  Serial.println("Creating publishers...");
  rcl_ret_t ret2;

  // Best effort for the sensor streams: no waiting for an ack per message (a lost one is replaced
  // a few ms later). Odometry stays reliable: at ~700 bytes it exceeds the 512-byte transport MTU,
  // and only reliable streams can fragment.
  ret2 = rclc_publisher_init_best_effort(&imu_pub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "imu/data");
  Serial.printf("IMU pub: %ld\n", ret2);

  ret2 = rclc_publisher_init_best_effort(&tof_pub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range), "tof/range");
  Serial.printf("ToF pub: %ld\n", ret2);

  ret2 = rclc_publisher_init_default(&odom_pub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry), "odom");
  Serial.printf("Odom pub: %ld\n", ret2);

  // Sync our clock with the agent so message stamps are in ROS time. sparse_slam looks up each
  // reading's pose in TF at its stamp; boot-clock stamps (1970) would get every reading dropped.
  if (rmw_uros_sync_session(1000) == RMW_RET_OK) {
    Serial.println("Time synced with agent");
  } else {
    Serial.println("Time sync FAILED: message stamps will be wrong");
  }

  initMessages();
  microros_connected = true;
  Serial.println("micro-ROS initialized!");
}

void initMessages() {
  // IMU message
  imu_msg.header.frame_id.data = (char*)"imu_link";
  imu_msg.header.frame_id.size = strlen("imu_link");
  imu_msg.header.frame_id.capacity = strlen("imu_link") + 1;

  imu_msg.orientation_covariance[0] = 0.001;
  imu_msg.orientation_covariance[4] = 0.001;
  imu_msg.orientation_covariance[8] = 0.01;
  imu_msg.angular_velocity_covariance[0] = 0.001;
  imu_msg.angular_velocity_covariance[4] = 0.001;
  imu_msg.angular_velocity_covariance[8] = 0.001;
  imu_msg.linear_acceleration_covariance[0] = 0.01;
  imu_msg.linear_acceleration_covariance[4] = 0.01;
  imu_msg.linear_acceleration_covariance[8] = 0.01;

  // ToF messages
  tof_msg.radiation_type = sensor_msgs__msg__Range__INFRARED;
  tof_msg.field_of_view = 0.44f;
  tof_msg.min_range = 0.03f;
  tof_msg.max_range = 2.0f;

  // Odom message
  odom_msg.header.frame_id.data = (char*)"odom";
  odom_msg.header.frame_id.size = 4;
  odom_msg.header.frame_id.capacity = 5;
  odom_msg.child_frame_id.data = (char*)"base_link";
  odom_msg.child_frame_id.size = 9;
  odom_msg.child_frame_id.capacity = 10;
}

// Current time in ROS (agent) time; boot time until the clock has been synced
int64_t rosTimeNs() {
  return rmw_uros_epoch_synchronized() ? rmw_uros_epoch_nanos() : esp_timer_get_time() * 1000LL;
}

// Each ToF reading as soon as it arrives, stamped when it was read: during a 0.5 rad/s spin the
// robot turns ~1 deg between readings, so late or batched stamps would smear the walls.
void publishRanges() {
  if (!microros_connected) return;
  for (int i = 0; i < 5; i++) {
    if (!rangeFresh[i]) continue;
    rangeFresh[i] = false;
    tof_msg.header.stamp.sec = rangeStampNs[i] / 1000000000LL;
    tof_msg.header.stamp.nanosec = rangeStampNs[i] % 1000000000LL;
    tof_msg.header.frame_id.data = (char*)tof_frame_ids[i];
    tof_msg.header.frame_id.size = strlen(tof_frame_ids[i]);
    tof_msg.header.frame_id.capacity = strlen(tof_frame_ids[i]) + 1;
    tof_msg.range = rawRanges[i] > 0 ? rawRanges[i] : INFINITY;
    rcl_ret_t rc __attribute__((unused)) = rcl_publish(&tof_pub, &tof_msg, NULL);
  }
}

void publishToROS() {
  if (!microros_connected) return;

  TRACE("PUB1");

  // Re-sync every minute: the ESP32's clock drifts against the PC's. Log how far each re-sync moves
  // the clock (expected: a few ms) so a bad sync can't go unnoticed.
  static uint32_t lastSync = millis();
  if (millis() - lastSync > 60000) {
    lastSync = millis();
    int64_t before = rmw_uros_epoch_nanos();
    int64_t local0 = esp_timer_get_time();
    rmw_ret_t sr = rmw_uros_sync_session(1000);
    int64_t elapsedNs = (esp_timer_get_time() - local0) * 1000LL;
    if (sr == RMW_RET_OK) {
      Serial.printf("Time resync: clock moved %+.1f ms (took %.0f ms)\n",
                    (rmw_uros_epoch_nanos() - before - elapsedNs) / 1e6, elapsedNs / 1e6);
    } else {
      Serial.println("Time resync FAILED (kept previous offset)");
    }
  }

  int64_t now_ns = rosTimeNs();
  int32_t sec = now_ns / 1000000000LL;
  uint32_t nsec = now_ns % 1000000000LL;

  TRACE("PUB2");

  // Publish IMU
  imu_msg.header.stamp.sec = sec;
  imu_msg.header.stamp.nanosec = nsec;
  imu_msg.orientation.w = q0;
  imu_msg.orientation.x = q1;
  imu_msg.orientation.y = q2;
  imu_msg.orientation.z = q3;
  imu_msg.angular_velocity.x = imu_gx;
  imu_msg.angular_velocity.y = imu_gy;
  imu_msg.angular_velocity.z = imu_gz;
  imu_msg.linear_acceleration.x = imu_ax;
  imu_msg.linear_acceleration.y = imu_ay;
  imu_msg.linear_acceleration.z = imu_az;
  
  TRACE("PUB3");
  rcl_ret_t rc __attribute__((unused));
  rc = rcl_publish(&imu_pub, &imu_msg, NULL);

  // Publish odometry
  odom_msg.header.stamp.sec = sec;
  odom_msg.header.stamp.nanosec = nsec;
  odom_msg.pose.pose.position.x = finalCoords[0];
  odom_msg.pose.pose.position.y = finalCoords[1];
  odom_msg.pose.pose.position.z = 0;
  // Heading to quaternion. ROS yaw is counterclockwise-positive, the firmware heading clockwise.
  float half_yaw = -finalCoords[2] / 2.0f;
  odom_msg.pose.pose.orientation.w = cos(half_yaw);
  odom_msg.pose.pose.orientation.x = 0;
  odom_msg.pose.pose.orientation.y = 0;
  odom_msg.pose.pose.orientation.z = sin(half_yaw);
  // Pose covariance
  odom_msg.pose.covariance[0] = 0.01;   // x
  odom_msg.pose.covariance[7] = 0.01;   // y
  odom_msg.pose.covariance[35] = 0.03;  // yaw

  // Velocities in the robot frame (what the EKF fuses): wheel motion since the last publish / time
  uint32_t nowMs = millis();
  float dt = (nowMs - lastOdomPublishMs) / 1000.0f;
  bool haveDt = lastOdomPublishMs != 0 && dt > 0.0f;
  odom_msg.twist.twist.linear.x = haveDt ? odomAccum[0] / dt : 0.0f;
  odom_msg.twist.twist.linear.y = haveDt ? odomAccum[1] / dt : 0.0f;
  odom_msg.twist.twist.angular.z = haveDt ? odomAccum[2] / dt : 0.0f;
  odomAccum[0] = odomAccum[1] = odomAccum[2] = 0.0f;
  lastOdomPublishMs = nowMs;
  odom_msg.twist.covariance[0] = 0.01;   // vx
  odom_msg.twist.covariance[7] = 0.01;   // vy
  odom_msg.twist.covariance[35] = 0.05;  // vyaw: mecanum wheels slip when turning
  
  TRACE("PUB5");
  rc = rcl_publish(&odom_pub, &odom_msg, NULL);
}

void setup() {
  Serial.begin(115200);
  Serial.println("Starting...");

  if (!Wire.begin(0, 1, 100000)) {
    Serial.println("Wire.begin FAILED");
    while(1) delay(100);
  }
  // A device that stops answering mid-run costs at most this per transaction (default is 50 ms)
  Wire.setTimeOut(10);

  initIMU();
  calibrateGyro();
  initLasers();

  memset(occupancy_grid, 0, GRID_BYTES);
  memset(inflated_grid, 0, GRID_BYTES);

  initWiFi();

  Serial.println("Setup complete (connecting to micro-ROS agent in the background)");
}

// Drop a session whose agent has gone away (e.g. the launch was restarted), so we reconnect to the
// new one instead of publishing into the void forever.
void disconnectMicroROS() {
  // Don't wait on the vanished agent while destroying entities
  rmw_context_t* rmw_context = rcl_context_get_rmw_context(&support.context);
  rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  rcl_ret_t rc __attribute__((unused));
  rc = rcl_publisher_fini(&imu_pub, &node);
  rc = rcl_publisher_fini(&tof_pub, &node);
  rc = rcl_publisher_fini(&odom_pub, &node);
  rc = rcl_node_fini(&node);
  rclc_support_fini(&support);
  microros_connected = false;
  Serial.println("micro-ROS agent lost; reconnecting...");
}

// Every 3 s: connect if we aren't, otherwise check the agent is still there (two failed checks in a
// row, ~6 s, means it's gone). Pings are quick (up to 3 x 100 ms) so the sensors barely notice.
void manageMicroROS() {
  static int missed = 0;
  if (!microros_connected) {
    missed = 0;
    connectMicroROS();
    return;
  }
  if (rmw_uros_ping_agent(100, 3) == RMW_RET_OK) {
    missed = 0;
  } else if (++missed >= 2) {
    disconnectMicroROS();
    missed = 0;
  }
}

int delayTimesMS[] = {     10,         25,             20,          100,            50,     1000,           3000,          20,};
void (*handlers[])() = { readIMU, readDeadReckon, readLasers, centralCommand, publishToROS, renav, manageMicroROS, publishRanges, };
long lastTimes[sizeof(delayTimesMS) / sizeof(delayTimesMS[0])];
const char* handlerNames[] = { "readIMU", "readDeadReckon", "readLasers", "centralCommand", "publishToROS", "renav", "manageMicroROS", "publishRanges" };
#define NUM_HANDLERS (sizeof(delayTimesMS) / sizeof(delayTimesMS[0]))

#if DEBUG_TIMING
int64_t handlerTotalUs[NUM_HANDLERS];
int64_t handlerMaxUs[NUM_HANDLERS];
uint32_t handlerCalls[NUM_HANDLERS];
uint32_t loopPasses = 0;
int64_t lastTimingReport = 0;

void reportTiming() {
  int64_t now = esp_timer_get_time();
  if (now - lastTimingReport < 5000000) return;
  float secs = (now - lastTimingReport) / 1e6f;
  Serial.printf("--- timing over %.1f s: %.1f loop passes/s\n", secs, loopPasses / secs);
  for (size_t i = 0; i < NUM_HANDLERS; i++) {
    if (handlerCalls[i] == 0) continue;
    Serial.printf("  %-15s %6.1f calls/s  avg %7.2f ms  max %7.2f ms  (%4.1f%% of time)\n", handlerNames[i],
                  handlerCalls[i] / secs, handlerTotalUs[i] / 1000.0f / handlerCalls[i], handlerMaxUs[i] / 1000.0f,
                  handlerTotalUs[i] / 10000.0f / secs);
    handlerTotalUs[i] = handlerMaxUs[i] = 0;
    handlerCalls[i] = 0;
  }
  loopPasses = 0;
  lastTimingReport = now;
}
#endif

void loop() {
  TRACE("LOOP");
  long time = millis();
  for (size_t i = 0; i < NUM_HANDLERS; i++) {
    if (time - lastTimes[i] > delayTimesMS[i]) {
      lastTimes[i] = time;
#if DEBUG_TIMING
      int64_t t0 = esp_timer_get_time();
      (*handlers[i])();
      int64_t dt = esp_timer_get_time() - t0;
      handlerTotalUs[i] += dt;
      if (dt > handlerMaxUs[i]) handlerMaxUs[i] = dt;
      handlerCalls[i]++;
#else
      (*handlers[i])();
#endif
      time = millis();
    }
  }
#if DEBUG_TIMING
  loopPasses++;
  reportTiming();
#endif
}
