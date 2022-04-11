# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, Camera, CameraRecognitionObject, InertialUnit, DistanceSensor, PositionSensor
import math

WHEEL_DIST = 1.05
WHEEL_DIAMETER = 1.6
MAX_PHI = 2.5
MAX_SIMULATION_TIME = 30 * 1000
MAX_MEASURED_DISTANCE = 1.27
ACCEPTED_ERROR = 0.001
K = 10

RED = [1, 0, 0]
GREEN = [0, 1, 0]
BLUE = [0, 0, 1]
YELLOW = [1, 1, 0]

yellow_x = -20
yellow_y = 20

red_x = 20
red_y = 20

green_x = -20
green_y = -20

blue_x = 20
blue_y = -20

cylinder_radius = 3.14


def getXY(x1, y1, r1, x2, y2, r2, x3, y3, r3):
    A = 2 * x2 - 2 * x1
    B = 2 * y2 - 2 * y1
    C = r1 ** 2 - r2 ** 2 - x1 ** 2 + x2 ** 2 - y1 ** 2 + y2 ** 2
    D = 2 * x3 - 2 * x2
    E = 2 * y3 - 2 * y2
    F = r2 ** 2 - r3 ** 2 - x2 ** 2 + x3 ** 2 - y2 ** 2 + y3 ** 2
    x = (C * E - F * B) / (E * A - B * D)
    y = (C * D - A * F) / (B * D - A * E)
    return x, y


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


def getCylinderPosition(color):
    recognized_object_array = camera.getRecognitionObjects()

    for cylinder in range(len(recognized_object_array)):
        if recognized_object_array[cylinder].get_colors() == color:
            position = recognized_object_array[cylinder].get_position_on_image()[0]
            return position

    return 0


def getCylinderDirection(COLOR):
    global time

    error = 45 - getCylinderPosition(COLOR)

    while abs(error) > 1:
        rps = K * error
        setSpeedsRPS(-rps, rps)

        robot.step(timestep)
        time += timestep

        error = 45 - getCylinderPosition(COLOR)

    return getYawRadians()


def correctDirection(desiredDirection):
    global time
    error = getYawRadians() - desiredDirection

    while abs(error) > ACCEPTED_ERROR:
        speed = K * error

        setSpeedsRPS(speed, -speed)
        robot.step(timestep)
        time += timestep

        error = getYawRadians() - desiredDirection


def isCylinderColor(color):
    recognized_object_array = camera.getRecognitionObjects()

    for cylinder in range(len(recognized_object_array)):
        if recognized_object_array[cylinder].get_colors() == color:
            return True

    return False


def getCellEstimation(x, y):
    if y >= 10:
        if x <= -10:
            return 1
        elif -10 < x <= 0:
            return 2
        elif 0 < x <= 10:
            return 3
        elif 10 < x:
            return 4
    elif 10 > y >= 0:
        if x <= -10:
            return 5
        elif -10 < x <= 0:
            return 6
        elif 0 < x <= 10:
            return 7
        elif 10 < x:
            return 8
    elif 0 > y >= -10:
        if x <= -10:
            return 9
        elif -10 < x <= 0:
            return 10
        elif 0 < x <= 10:
            return 11
        elif 10 < x:
            return 12
    elif -10 > y:
        if x <= -10:
            return 13
        elif -10 < x <= 0:
            return 14
        elif 0 < x <= 10:
            return 15
        elif 10 < x:
            return 16

    return -1


def getCylindersDistances():
    redDir = getCylinderDirection(RED)
    redDis = getSensors()[0] + cylinder_radius

    yellowDir = getCylinderDirection(YELLOW)
    yellowDis = getSensors()[0] + cylinder_radius

    greenDir = getCylinderDirection(GREEN)
    greenDis = getSensors()[0] + cylinder_radius

    blueDir = getCylinderDirection(BLUE)
    blueDis = getSensors()[0] + cylinder_radius

    return redDis, yellowDis, greenDis, blueDis


def getPosition():
    redDis, yellowDis, greenDis, blueDis = getCylindersDistances()

    x, y = getXY(red_x, red_y, redDis, green_x, green_y, greenDis, blue_x, blue_y, blueDis)

    cell = getCellEstimation(x, y)

    return cell, x, y


def printData(cell, x, y, yaw):
    global cells
    # print("Cell {0}".format(cell))
    # print("Position: ({0:.2f}, {1:.2f}) Yaw: {2:.2f})".format(x, y, yaw))
    for i in range(16):
        if cells[i]:
            print("X", end="")
        else:
            print(".", end="")
        if (i + 1) % 4 == 0 and not i == 0:
            print("")
    print("({0:.2f}, {1:.2f}, {2}, {3:.2f})".format(x, y, cell, yaw))


def correctDistance(desiredDistance):
    global time
    error = getSensors()[0] - desiredDistance

    while abs(error) > ACCEPTED_ERROR:
        speed = K * error

        setSpeedsRPS(speed, speed)
        robot.step(timestep)
        time += timestep

        error = getSensors()[0] - desiredDistance


def isAllCellsCovered(cells):
    for i in cells:
        if i == False:
            return i
    return True


def move(x, y, frontPrevious, direction):
    global time

    robot.step(timestep)
    time += timestep

    front = getSensors()[0]

    distance = abs(frontPrevious - front)

    if direction == EAST:
        x += distance
    elif direction == WEST:
        x -= distance
    elif direction == NORTH:
        y += distance
    else:
        y -= distance

    cell = getCellEstimation(x, y)

    return cell, x, y


# First step to get sensor readings
robot.step(timestep)
time += timestep

# defining direction
NORTH = math.pi / 2
SOUTH = -math.pi / 2
WEST = math.pi
EAST = 0

# speed in rps
V = MAX_PHI

# init cell matrix
cells = [False] * 16

# use trilateration to get position
cell, x, y = getPosition()

# mark cell as visited
cells[cell - 1] = True

# init direction
hasReachedEastWall = False
sweepDirection = EAST
travelDirection = NORTH
correctDirection(travelDirection)

# print first map and pose
printData(cell, x, y, getYawRadians())

# execute until map is covered or 3 minutes
while not isAllCellsCovered(cells) and time < 3 * 60 * 1000:

    setSpeedsRPS(V, V)

    front = getSensors()[0]

    previousCell = cell

    # navigate until north wall
    while front > 3.5:
        # move and update pose and cell
        cell, x, y = move(x, y, front, travelDirection)

        front = getSensors()[0]

        # detect cell change
        if not cell == previousCell:
            previousCell = cell

            # mark cell as visited
            cells[cell - 1] = True
            # print map and pose
            printData(cell, x, y, getYawRadians())

    # if not covered all map
    if not isAllCellsCovered(cells):
        # calibrate y position
        correctDistance(3.5)
        if travelDirection == NORTH:
            y = 15
        else:
            y = -15

        # if already has navigated to the east, move to the west
        if hasReachedEastWall:
            sweepDirection = WEST

        correctDirection(sweepDirection)

        setSpeedsRPS(V, V)

        frontPrevious = getSensors()[0]
        front = frontPrevious
        previousCell = cell

        # navigate until next cell or close to wall
        while front > 3.5 and previousCell == cell:
            # move and update pose and cell
            cell, x, y = move(x, y, front, sweepDirection)

            front = getSensors()[0]

        # detect cell change
        if not previousCell == cell:
            # mark cell as visited
            cells[cell - 1] = True
            # print map and pose
            printData(cell, x, y, getYawRadians())

        # if not covered all map
        if not isAllCellsCovered(cells):
            # if encountered most eastern cells
            # mark east wall as reached
            if cell == 4 or cell == 16:
                hasReachedEastWall = True

                # calibrate x
                correctDistance(3.5)
                if sweepDirection == EAST:
                    x = 15
                else:
                    x = -15

            # change direction
            if travelDirection == NORTH:
                travelDirection = SOUTH
            else:
                travelDirection = NORTH

            correctDirection(travelDirection)

setSpeedsRPS(0, 0)
