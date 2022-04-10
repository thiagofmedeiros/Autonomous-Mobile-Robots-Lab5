# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, Camera, CameraRecognitionObject, InertialUnit, DistanceSensor, PositionSensor
import math

WHEEL_DIST = 1.05
WHEEL_DIAMETER = 1.6
MAX_PHI = 3.2
MAX_SIMULATION_TIME = 30 * 1000
MAX_MEASURED_DISTANCE = 1.27
ACCEPTED_ERROR = 0.001
K = 10

RED = [1, 0, 0]
GREEN = [0, 1, 0]
BLUE = [0, 0, 1]
YELLOW = [1, 1, 0]

yellow_x = -20
yellow_Y = 20

red_x = 20
red_Y = 20

green_x = -20
green_Y = -20

blue_x = 20
blue_Y = -20

cylinder_radius = 3.14


def getA(x1, x2):
    return -2 * x1 + 2 * x2


def getB(y1, y2):
    return -2 * y1 + 2 * y2


def getC(r1, r2, x1, x2, y1, y2):
    r1 = r1 ** 2
    r2 = r2 ** 2
    x1 = x1 ** 2
    x2 = x2 ** 2
    y1 = y1 ** 2
    y2 = y2 ** 2

    return r1 - r2 - x1 + x2 - y1 + y2


def getD(x2, x3):
    return -2 * x2 + 2 * x3


def getE(y2, y3):
    return -2 * y2 + 2 * y3


def getF(r2, r3, x2, x3, y2, y3):
    r2 = r2 ** 2
    r3 = r3 ** 2
    x2 = x2 ** 2
    x3 = x3 ** 2
    y2 = y2 ** 2
    y3 = y3 ** 2

    return r2 - r3 - x2 + x3 - y2 + y3


def getXY(x1, x2, x3, y1, y2, y3, r1, r2, r3):
    A = getA(x1, x2)
    B = getB(y1, y2)
    C = getC(r1, r2, x1, x2, y1, y2)
    D = getD(x2, x3)
    E = getE(y2, y3)
    F = getF(r2, r3, x2, x3, y2, y3)

    x = getX(A, B, C, D, E, F)
    y = getY(A, B, C, D, E, F)

    return x, y


def getX(A, B, C, D, E, F):
    numerator = (C * E) - (F * B)

    denominator = (E * A) - (B * D)

    x = numerator / denominator

    return x


def getY(A, B, C, D, E, F):
    numerator = (C * D) - (A * F)

    denominator = (B * D) - (A * E)

    y = numerator / denominator

    return y


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

    print("FrontDist {0:.3f} inches".format(fdsVal))
    print("LeftDist {0:.3f} inches".format(ldsVal))
    print("RightDist {0:.3f} inches\n".format(rdsVal))

    return fdsVal, ldsVal, rdsVal


def setSpeedsRPS(rpsLeft, rpsRight):
    if rpsLeft > MAX_PHI:
        print("Saturating left speed to {0:.3f} rad/s".format(MAX_PHI))
        leftMotor.setVelocity(MAX_PHI)
    elif rpsLeft < -MAX_PHI:
        print("Saturating left speed to {0:.3f} rad/s".format(-MAX_PHI))
        leftMotor.setVelocity(-MAX_PHI)
    else:
        print("Left motor velocity: {0:.3f} rad/s".format(rpsLeft))
        leftMotor.setVelocity(rpsLeft)

    if rpsRight > MAX_PHI:
        print("Saturating right speed to {0:.3f} rad/s".format(MAX_PHI))
        rightMotor.setVelocity(MAX_PHI)
    elif rpsRight < -MAX_PHI:
        print("Saturating right speed to {0:.3f} rad/s".format(-MAX_PHI))
        rightMotor.setVelocity(-MAX_PHI)
    else:
        print("Right motor velocity: {0:.3f} rad/s\n".format(rpsRight))
        rightMotor.setVelocity(rpsRight)


def rotateUntilObject(find, clockwise):
    global time

    recognized_object_array = camera.getRecognitionObjects()

    if clockwise:
        setSpeedsRPS(MAX_PHI, -MAX_PHI)
    else:
        setSpeedsRPS(-MAX_PHI, MAX_PHI)

    if find:
        while len(recognized_object_array) < 1:
            robot.step(timestep)
            time += timestep

            recognized_object_array = camera.getRecognitionObjects()
    else:
        while len(recognized_object_array) > 0:
            robot.step(timestep)
            time += timestep

            recognized_object_array = camera.getRecognitionObjects()


def getObjectDirection():
    recognized_object_array = camera.getRecognitionObjects()

    # If There is no object rotates until finds it
    # then rotates until it is no longer on view
    if len(recognized_object_array) < 1:
        rotateUntilObject(find=True, clockwise=True)
        firstYaw = getYawRadians()

        rotateUntilObject(find=False, clockwise=True)
        secondYaw = getYawRadians()
    # If There is an object rotates until it is no longer on view
    # Rotates in the opposite direction until finds it again
    # than rotates until it is no longer on view again
    else:
        rotateUntilObject(find=False, clockwise=True)
        firstYaw = getYawRadians()

        rotateUntilObject(find=True, clockwise=False)

        rotateUntilObject(find=False, clockwise=False)
        secondYaw = getYawRadians()

    # The direction of the object is the average between both angles
    direction = (firstYaw + secondYaw) / 2

    return direction


def correctDirection(desiredDirection):
    global time
    error = getYawRadians() - desiredDirection

    print("\nCorrecting Direction\n")

    while abs(error) > ACCEPTED_ERROR:
        speed = K * error

        setSpeedsRPS(speed, -speed)
        robot.step(timestep)
        time += timestep

        error = getYawRadians() - desiredDirection

    print("\nDirection Corrected\n")


def isCylinderColor(color):
    recognized_object_array = camera.getRecognitionObjects()

    for cylinder in range(len(recognized_object_array)):
        if recognized_object_array[cylinder].get_colors() == color:
            return True

    return False


# First step to get sensor readings
robot.step(timestep)
time += timestep

yellow = isCylinderColor(YELLOW)
red = isCylinderColor(RED)
blue = isCylinderColor(BLUE)
green = isCylinderColor(GREEN)

dire = getObjectDirection()
