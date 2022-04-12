# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, Camera, CameraRecognitionObject, InertialUnit, DistanceSensor, PositionSensor
import math
import random

WHEEL_DIST = 1.05
WHEEL_DIAMETER = 1.6
MAX_PHI = 2.5
MAX_SIMULATION_TIME = 30 * 1000
MAX_MEASURED_DISTANCE = 1.27
ACCEPTED_ERROR = 0.001
K = 10

NORTH = math.pi / 2
SOUTH = -math.pi / 2
WEST = math.pi
EAST = 0

DIRECTIONS = [WEST, NORTH, EAST, SOUTH]

# map of labyrinth
labyrinth = [(True, True, False, True),    # 01
             (False, True, False, True),   # 02
             (False, True, False, False),  # 03
             (False, True, True, False),   # 04
             (True, True, False, False),   # 05
             (False, True, True, False),   # 06
             (True, False, True, False),   # 07
             (True, False, True, False),   # 08
             (True, False, True, False),   # 09
             (True, False, False, True),   # 10
             (False, False, True, True),   # 11
             (True, False, True, False),   # 12
             (True, False, False, True),   # 13
             (False, True, False, True),   # 14
             (False, True, False, True),   # 15
             (False, False, True, True)    # 16
             ]

# init cell matrix
cells = [False] * 16

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getMotor('motorname')
#  ds = robot.getDistanceSensor('dsname')
#  ds.enable(timestep)

# enable distance sensors
frontDistanceSensor = robot.getDevice('front_ds')
leftDistanceSensor = robot.getDevice('left_ds')
rightDistanceSensor = robot.getDevice('right_ds')
frontDistanceSensor.enable(timestep)
leftDistanceSensor.enable(timestep)
rightDistanceSensor.enable(timestep)

# enable camera and recognition
camera = robot.getDevice('camera1')
camera.enable(timestep)
camera.recognitionEnable(timestep)

# enable imu
imu = robot.getDevice('inertial unit')
imu.enable(timestep)

# get handler to motors and set target position to infinity
leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0)
rightMotor.setVelocity(0)

time = 0


def inchesToMeters(x):
    return x / 39.37


def metersToInches(x):
    return x * 39.37


def getYawDegrees():
    Yaw = math.degrees(getYawRadians())

    if Yaw < 0:
        Yaw = Yaw + 360

    return Yaw


def getYawRadians():
    return imu.getRollPitchYaw()[2]


def getSensors():
    fdsVal = metersToInches(frontDistanceSensor.getValue())
    ldsVal = metersToInches(leftDistanceSensor.getValue())
    rdsVal = metersToInches(rightDistanceSensor.getValue())

    return fdsVal, ldsVal, rdsVal


def setSpeedsRPS(rpsLeft, rpsRight):
    if rpsLeft > MAX_PHI:
        leftMotor.setVelocity(MAX_PHI)
    elif rpsLeft < -MAX_PHI:
        leftMotor.setVelocity(-MAX_PHI)
    else:
        leftMotor.setVelocity(rpsLeft)

    if rpsRight > MAX_PHI:
        rightMotor.setVelocity(MAX_PHI)
    elif rpsRight < -MAX_PHI:
        rightMotor.setVelocity(-MAX_PHI)
    else:
        rightMotor.setVelocity(rpsRight)


# uses proportional control to move robot to a desired direction
def correctDirection(desiredDirection):
    global time
    error = getYawRadians() - desiredDirection

    while abs(error) > ACCEPTED_ERROR:
        speed = K * error

        setSpeedsRPS(speed, -speed)
        robot.step(timestep)
        time += timestep

        error = getYawRadians() - desiredDirection


def getWalls():
    MAX_DISTANCE_WALL = 7

    correctDirection(NORTH)
    north, west, east = getSensors()

    correctDirection(EAST)
    _, _, south = getSensors()

    walls = (west < MAX_DISTANCE_WALL, north < MAX_DISTANCE_WALL, east < MAX_DISTANCE_WALL, south < MAX_DISTANCE_WALL)

    return walls


def canReachCell(previousCell, cell, movement):
    testCell = -1

    if movement is None:
        return True

    elif movement == WEST:
        testCell = cell + 1
    elif movement == NORTH:
        testCell = cell + 4
    elif movement == EAST:
        testCell = cell - 1
    elif movement == SOUTH:
        testCell = cell - 4
    if previousCell == testCell:
        return True

    return False


def getPossiblePositions(movementList):
    possiblePositionsList = []
    previousPositionList = []

    for movement in range(len(movementList)):
        possiblePositionsMap = [False] * 16
        possiblePositionsList = []
        for i in range(len(labyrinth)):
            if movementList[movement][0] == labyrinth[i]:
                if movementList[movement][1] is None:
                    possiblePositionsMap[i] = True
                    possiblePositionsList.append(i)

                else:
                    for j in previousPositionList:
                        if canReachCell(j, i, movementList[movement][1]):
                            possiblePositionsMap[i] = True
                            possiblePositionsList.append(i)

        previousPositionList = possiblePositionsList

    return possiblePositionsList


def canMove(direction, walls):
    if direction == None:
        return True
    elif direction == WEST:
        return not walls[0]
    elif direction == NORTH:
        return not walls[1]
    elif direction == EAST:
        return not walls[2]
    else:
        return not walls[3]


def printData(map, cell, x, y, yaw):
    # print("Cell {0}".format(cell))
    # print("Position: ({0:.2f}, {1:.2f}) Yaw: {2:.2f})".format(x, y, yaw))
    for i in range(16):
        if map[i]:
            print("X", end="")
        else:
            print(".", end="")
        if (i + 1) % 4 == 0 and not i == 0:
            print("")
    print("({0:.2f}, {1:.2f}, {2}, {3:.2f})".format(x, y, cell, yaw))


def randomChooseDirection(walls):
    directions = []

    for i in range(len(DIRECTIONS)):
        if canMove(DIRECTIONS[i], walls):
            directions.append(DIRECTIONS[i])

    return random.choice(directions)


def move1Cell():
    global time
    distanceTraveled = 0
    front = getSensors()[0]

    setSpeedsRPS(MAX_PHI, MAX_PHI)

    while front > 3.5 and distanceTraveled < 10:
        frontPrevious = getSensors()[0]

        robot.step(timestep)
        time += timestep

        front = getSensors()[0]

        distanceTraveled += abs(frontPrevious - front)


def findPosition():
    movementList = []
    direction = None

    while True:
        walls = getWalls()

        movementList.append((walls, direction))

        positionsList = getPossiblePositions(movementList)

        if len(positionsList) == 1:
            return positionsList[0]

        direction = randomChooseDirection(walls)

        correctDirection(direction)

        move1Cell()


# First step to get sensor readings
robot.step(timestep)
time += timestep

a = findPosition()

print(a)

a = 1
