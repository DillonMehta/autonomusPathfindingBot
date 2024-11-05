// Includes and Namespace
#include <Wire.h>
#include <Pololu3piPlus32U4.h>
#include <Pololu3piPlus32U4IMU.h>
#include <queue>
#include <vector>
using namespace Pololu3piPlus32U4;

// Hardware Initialization
OLED display;
Encoders encoders;
Motors motors;
IMU imu;

// Constants and Calibration Parameters
const int CLICKS_PER_ROTATION = 12;
const float GEAR_RATIO = 29.86F;
const float WHEEL_DIAMETER = 3.2; // in cm
const float WHEEL_CIRCUMFERENCE = 10.05; // in cm

// Gyro Variables
uint32_t turnAngle = 0;
int16_t turnRate;
int16_t gyroOffset;
uint16_t gyroLastUpdate = 0;

// Movement Parameters
double s1min = 80; 
double s2min = 85;
double Kpf = 1.0; // Proportional constant for forward movement
double Kps = 6.0; // Proportional constant for straight movement correction
double t1min = 25;
double t2min = 25;
double Kpt = 0.15; // Proportional constant for turning

// Grid Dimensions
const int GRID_ROWS = 5;
const int GRID_COLS = 4;
const float CELL_SIZE = 50.0; // Each cell is 50 cm (0.5 m x 0.5 m)

// Orientation Constants
enum Orientation { TOP = 0, RIGHT = 1, BOTTOM = 2, LEFT = 3 };

// User-Configurable Variables (Fill these out)
int orientation = 0; // T-0, R-1, B-2, L-3
int gateCount = 3;   // VERY IMPORTANT
int startNode = 18;  // Node numbers from 0 to 19
int G1 = 17;
int G2 = 13;
int G3 = 14;
int G4 = -1; // If less than gateCount, set to -1
int G5 = -1; // If less than gateCount, set to -1
int endNode = 18;

// Obstacle Representation
struct Cell {
    bool walls[4]; // {TOP, RIGHT, BOTTOM, LEFT}
};

Cell grid[GRID_ROWS][GRID_COLS];

// Robot State
struct Position {
    int row;
    int col;
};

Position robotPos;
Orientation robotOrient = (Orientation)orientation;

// Function Declarations
void initGrid();
void turnSensorSetup();
void turnSensorReset();
void turnSensorUpdate();
double ang();
void move_fwd(float distance);
void turn_left();
void turn_right();
void moveForwardOneCell();
void turnLeft90();
void turnRight90();
void updateRobotPosition();
bool isValid(Position pos);
bool isObstacle(Position current, Direction dir);
std::vector<Position> findPath(Position start, Position end);
void pathToMovements(std::vector<Position> path);
void executeMovements();

// Movement Function Array
typedef void (*MovementFunction)();
const int MAX_MOVEMENTS = 200;
MovementFunction movements[MAX_MOVEMENTS];
int movementCount = 0;

// Setup Function
void setup() {
    delay(1000);

    Serial.begin(9600);

    // Initialize hardware
    turnSensorSetup();
    turnSensorReset();

    // Initialize grid and robot position
    initGrid();
    robotPos = { startNode / GRID_COLS, startNode % GRID_COLS };

    // Plan the path
    std::vector<Position> completePath;
    Position currentPos = robotPos;

    // Collect Gate Positions
    Position gates[5];
    int gateNodes[] = { G1, G2, G3, G4, G5 };
    int gateIndices = 0;
    for (int i = 0; i < gateCount; i++) {
        if (gateNodes[i] >= 0) {
            gates[gateIndices++] = { gateNodes[i] / GRID_COLS, gateNodes[i] % GRID_COLS };
        }
    }

    // Final Target Position
    Position targetPos = { endNode / GRID_COLS, endNode % GRID_COLS };

    // Visit all gates
    for (int i = 0; i < gateIndices; i++) {
        std::vector<Position> pathSegment = findPath(currentPos, gates[i]);
        if (pathSegment.size() == 0) {
            // No path found
            Serial.println("No path to gate found.");
            while (1);
        }
        // Exclude the starting position to avoid duplication
        if (!completePath.empty()) {
            pathSegment.erase(pathSegment.begin());
        }
        completePath.insert(completePath.end(), pathSegment.begin(), pathSegment.end());
        currentPos = gates[i];
    }

    // Path to the end node
    std::vector<Position> pathToEnd = findPath(currentPos, targetPos);
    if (pathToEnd.size() == 0) {
        Serial.println("No path to end node found.");
        while (1);
    }
    // Exclude the starting position to avoid duplication
    if (!completePath.empty()) {
        pathToEnd.erase(pathToEnd.begin());
    }
    completePath.insert(completePath.end(), pathToEnd.begin(), pathToEnd.end());

    // Convert path to movements
    pathToMovements(completePath);

    // Execute movements
    executeMovements();

    // Stop the robot
    motors.setSpeeds(0, 0);

    // Wait indefinitely
    while (1) {
        delay(1000);
    }
}

// Main Loop
void loop() {
    // Empty loop
}

// Initialize the Grid and Obstacles
void initGrid() {
    // Initialize all cells with no walls
    for (int i = 0; i < GRID_ROWS; i++) {
        for (int j = 0; j < GRID_COLS; j++) {
            for (int k = 0; k < 4; k++) {
                grid[i][j].walls[k] = false;
            }
        }
    }

    // Example Obstacles (Adjust as needed)
    // Add walls between cells
    // For example, to add a wall between cell (1,1) and cell (1,2) on the right of (1,1):
    // grid[1][1].walls[RIGHT] = true;
    // grid[1][2].walls[LEFT] = true;

    // Obstacle between cell (1,1) and cell (1,2)
    grid[1][1].walls[RIGHT] = true;
    grid[1][2].walls[LEFT] = true;

    // Obstacle between cell (2,2) and cell (3,2)
    grid[2][2].walls[BOTTOM] = true;
    grid[3][2].walls[TOP] = true;

    // Add more obstacles as needed
}

// Gyro Functions
void turnSensorSetup() {
    Wire.begin();
    imu.init();
    imu.enableDefault();
    imu.configureForTurnSensing();

    display.clear();
    display.print(F("Gyro cal"));

    ledYellow(1);
    delay(500);

    // Calibrate the gyro.
    int32_t total = 0;
    for (uint16_t i = 0; i < 1024; i++) {
        while (!imu.gyroDataReady()) {}
        imu.readGyro();
        total += imu.g.z;
    }
    ledYellow(0);
    gyroOffset = total / 1024;

    display.clear();
    turnSensorReset();
}

void turnSensorReset() {
    cli(); // Disable interrupts
    gyroLastUpdate = micros();
    turnAngle = 0;
    sei(); // Enable interrupts
}

void turnSensorUpdate() {
    imu.readGyro();
    turnRate = imu.g.z - gyroOffset;

    uint16_t m = micros();
    uint16_t dt = m - gyroLastUpdate;
    gyroLastUpdate = m;

    int32_t d = (int32_t)turnRate * dt;
    turnAngle += (int64_t)d * 14680064 / 17578125;
}

double ang() {
    return ((int32_t)turnAngle) * (360.0 / 4294967296.0);
}

// Movement Functions
void move_fwd(float distance) {
    encoders.getCountsAndResetLeft();
    encoders.getCountsAndResetRight();

    float distanceTraveled = 0;
    while (distanceTraveled < distance) {
        turnSensorUpdate();
        int16_t countsLeft = encoders.getCountsLeft();
        int16_t countsRight = encoders.getCountsRight();

        float avgCounts = (countsLeft + countsRight) / 2.0;
        distanceTraveled = avgCounts / (CLICKS_PER_ROTATION * GEAR_RATIO) * WHEEL_CIRCUMFERENCE;

        float error = distance - distanceTraveled;
        float speed = error * Kpf + s1min;

        motors.setSpeeds(speed, speed - ang() * Kps);
    }
    motors.setSpeeds(0, 0);
    delay(100);
}

void turn_left() {
    turnSensorReset();
    while (ang() < 87.2) {
        turnSensorUpdate();
        motors.setSpeeds(-t1min - abs(90 - (ang())) * Kpt, t2min + abs(90 - (ang())) * Kpt);
    }
    motors.setSpeeds(0, 0);
    delay(100);
}

void turn_right() {
    turnSensorReset();
    while (ang() > -87.3) {
        turnSensorUpdate();
        motors.setSpeeds(t1min + abs(90 + (ang())) * Kpt, -t2min - abs(90 + (ang())) * Kpt);
    }
    motors.setSpeeds(0, 0);
    delay(100);
}

void moveForwardOneCell() {
    move_fwd(CELL_SIZE);
    updateRobotPosition();
}

void turnLeft90() {
    turn_left();
    robotOrient = (Orientation)((robotOrient + 3) % 4);
}

void turnRight90() {
    turn_right();
    robotOrient = (Orientation)((robotOrient + 1) % 4);
}

void updateRobotPosition() {
    switch (robotOrient) {
        case TOP:
            robotPos.row--;
            break;
        case RIGHT:
            robotPos.col++;
            break;
        case BOTTOM:
            robotPos.row++;
            break;
        case LEFT:
            robotPos.col--;
            break;
    }
}

// Pathfinding Functions
bool isValid(Position pos) {
    return (pos.row >= 0 && pos.row < GRID_ROWS && pos.col >= 0 && pos.col < GRID_COLS);
}

bool isObstacle(Position current, Direction dir) {
    return grid[current.row][current.col].walls[dir];
}

std::vector<Position> findPath(Position start, Position end) {
    bool visited[GRID_ROWS][GRID_COLS] = { false };
    Position parent[GRID_ROWS][GRID_COLS];

    std::queue<Position> queue;
    queue.push(start);
    visited[start.row][start.col] = true;

    bool found = false;

    while (!queue.empty()) {
        Position current = queue.front();
        queue.pop();

        if (current.row == end.row && current.col == end.col) {
            found = true;
            break;
        }

        // Explore neighbors (TOP, RIGHT, BOTTOM, LEFT)
        int dRow[] = { -1, 0, 1, 0 };
        int dCol[] = { 0, 1, 0, -1 };

        for (int i = 0; i < 4; i++) {
            Position neighbor = { current.row + dRow[i], current.col + dCol[i] };
            if (isValid(neighbor) && !isObstacle(current, (Direction)i) && !isObstacle(neighbor, (Direction)((i + 2) % 4)) && !visited[neighbor.row][neighbor.col]) {
                visited[neighbor.row][neighbor.col] = true;
                parent[neighbor.row][neighbor.col] = current;
                queue.push(neighbor);
            }
        }
    }

    std::vector<Position> path;
    if (found) {
        Position current = end;
        while (!(current.row == start.row && current.col == start.col)) {
            path.insert(path.begin(), current);
            current = parent[current.row][current.col];
        }
        path.insert(path.begin(), start);
    }
    return path;
}

void pathToMovements(std::vector<Position> path) {
    Position currentPos = robotPos;
    Orientation currentOrient = robotOrient;

    for (size_t i = 1; i < path.size(); i++) {
        Position nextPos = path[i];
        int dRow = nextPos.row - currentPos.row;
        int dCol = nextPos.col - currentPos.col;

        Orientation desiredOrient;
        if (dRow == -1 && dCol == 0) {
            desiredOrient = TOP;
        } else if (dRow == 0 && dCol == 1) {
            desiredOrient = RIGHT;
        } else if (dRow == 1 && dCol == 0) {
            desiredOrient = BOTTOM;
        } else if (dRow == 0 && dCol == -1) {
            desiredOrient = LEFT;
        } else {
            // Invalid movement
            continue;
        }

        // Calculate turn
        int turn = (desiredOrient - currentOrient + 4) % 4;

        // Add turn functions
        if (turn == 1) {
            movements[movementCount++] = &turnRight90;
        } else if (turn == 2) {
            movements[movementCount++] = &turnRight90;
            movements[movementCount++] = &turnRight90;
        } else if (turn == 3) {
            movements[movementCount++] = &turnLeft90;
        }

        // Move forward
        movements[movementCount++] = &moveForwardOneCell;

        // Update current orientation and position
        currentOrient = desiredOrient;
        currentPos = nextPos;
    }

    // Update robot's final orientation
    robotOrient = currentOrient;
}

void executeMovements() {
    for (int i = 0; i < movementCount; i++) {
        movements[i](); // Execute movement function
        delay(50); // Small delay between movements
    }
}
