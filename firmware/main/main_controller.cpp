#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <stdint.h>
#include <string.h>
#include <queue>

#define MOTOR_ADDR 1
#define TOF_CLK 18
#define TOF_DAT 19

int TOF_ADDR[] = { 10, 11, 12, 13, 14 };

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
float finalCoords[] = { 0.0, 0.0, 0.0 };

float laserMin = 0.05;
float laserMax = 2.0;
float laserReadings[] = { 0.0, 0.0, 0.0, 0.0, 0.0 };

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
    Serial.println(tof[i].begin(TOF_ADDR[i], true, &Wire, Adafruit_VL53L0X::VL53L0X_SENSE_DEFAULT));
  }

  delay(10);

  for (int i = 0; i < 5; i++) {
    tof[i].startRangeContinuous();
  }
}

void readLasers() {
  int ambad = 0;

  for (int i = 0; i < 5; i++) {
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

// Madgwick filter
float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
float beta = 0.1f;

void madgwickUpdate6DOF(float gx, float gy, float gz,
                        float ax, float ay, float az,
                        float dt)
{
  float recipNorm;
  float s0, s1, s2, s3;
  float qDot1, qDot2, qDot3, qDot4;
  float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2;
  float _8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

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

    _2q0 = 2.0f * q0; _2q1 = 2.0f * q1;
    _2q2 = 2.0f * q2; _2q3 = 2.0f * q3;
    _4q0 = 4.0f * q0; _4q1 = 4.0f * q1;
    _4q2 = 4.0f * q2;
    _8q1 = 8.0f * q1; _8q2 = 8.0f * q2;
    q0q0 = q0 * q0; q1q1 = q1 * q1;
    q2q2 = q2 * q2; q3q3 = q3 * q3;

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

// Store latest raw IMU data for micro-ROS publishing
float imu_ax, imu_ay, imu_az;
float imu_gx, imu_gy, imu_gz;

void writeReg(uint8_t addr, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

void initIMU() {
  // LSM6DS3TRC: 104Hz accel, ±4g
  writeReg(IMU_ADDR, 0x10, 0x48);
  // LSM6DS3TRC: 104Hz gyro, ±250dps
  writeReg(IMU_ADDR, 0x11, 0x40);
  // LIS3MDL: 155Hz, high performance XY
  writeReg(MAG_ADDR, 0x20, 0x1C);
  // LIS3MDL: ±4 gauss
  writeReg(MAG_ADDR, 0x21, 0x00);
  // LIS3MDL: continuous mode
  writeReg(MAG_ADDR, 0x22, 0x00);
  // LIS3MDL: high performance Z
  writeReg(MAG_ADDR, 0x23, 0x0C);

  Serial.println("IMU initialized (direct registers)");
}

void readIMU() {
  // Read accel (0x28-0x2D)
  Wire.beginTransmission(IMU_ADDR);
  Wire.write(0x28);
  Wire.endTransmission();
  Wire.requestFrom(IMU_ADDR, 6);
  int16_t ax_raw = Wire.read() | (Wire.read() << 8);
  int16_t ay_raw = Wire.read() | (Wire.read() << 8);
  int16_t az_raw = Wire.read() | (Wire.read() << 8);

  // Read gyro (0x22-0x27)
  Wire.beginTransmission(IMU_ADDR);
  Wire.write(0x22);
  Wire.endTransmission();
  Wire.requestFrom(IMU_ADDR, 6);
  int16_t gx_raw = Wire.read() | (Wire.read() << 8);
  int16_t gy_raw = Wire.read() | (Wire.read() << 8);
  int16_t gz_raw = Wire.read() | (Wire.read() << 8);

  // Convert to physical units
  // ±4g: 0.122 mg/LSB = 0.001197 m/s²/LSB
  imu_ax = ax_raw * 0.001197f;
  imu_ay = ay_raw * 0.001197f;
  imu_az = az_raw * 0.001197f;

  // ±250dps: 8.75 mdps/LSB = 0.000153 rad/s/LSB
  imu_gx = gx_raw * 0.000153f;
  imu_gy = gy_raw * 0.000153f;
  imu_gz = gz_raw * 0.000153f;

  madgwickUpdate6DOF(imu_gx, imu_gy, imu_gz,
                     imu_ax, imu_ay, imu_az,
                     0.01f);

  static int printCount = 0;
  if (++printCount >= 50) {
    Serial.printf("Quat: w=%.3f x=%.3f y=%.3f z=%.3f  Accel: %.2f %.2f %.2f\n",
                  q0, q1, q2, q3, imu_ax, imu_ay, imu_az);
    printCount = 0;
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("1 - Serial OK");

  if (!Wire.begin(0, 1, 100000)) {
    Serial.println("Wire.begin FAILED");
    while(1) delay(100);
  }
  Serial.println("2 - Wire OK");

  initIMU();

  memset(occupancy_grid, 0, GRID_BYTES);
  memset(inflated_grid, 0, GRID_BYTES);
  Serial.println("5 - Setup complete");
}

int delayTimesMS[] = {     10,     };
void (*handlers[])() = { readIMU, };
long lastTimes[sizeof(delayTimesMS) / sizeof(delayTimesMS[0])];

void loop() {
  long time = millis();
  for (int i = 0; i < sizeof(delayTimesMS) / sizeof(delayTimesMS[0]); i++) {
    if (time - lastTimes[i] > delayTimesMS[i]) {
      lastTimes[i] = time;
      (*handlers[i])();
      time = millis();
    }
  }
}
