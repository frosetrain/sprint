"""spRInt.

For IDE Series 2025
"""

from pybricks.hubs import PrimeHub
from pybricks.parameters import Direction, Port, Stop
from pybricks.pupdevices import ColorSensor, Motor
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch

# Line tracking constants
WHITE = 60
BLACK = 8
SPEED = 700
SENSOR_POSITIONS = (-3, -1, 1, 3)

hub = PrimeHub()
left_motor = Motor(Port.A, Direction.COUNTERCLOCKWISE)
right_motor = Motor(Port.B, Direction.CLOCKWISE)
color_sensors = (
    ColorSensor(Port.E),
    ColorSensor(Port.C),
    ColorSensor(Port.F),
    ColorSensor(Port.D),
)
db = DriveBase(left_motor, right_motor, 88, 164)
db.settings(SPEED, straight_acceleration=1800, turn_rate=100, turn_acceleration=400)
# 307, 1152, 183, 825


# 0mm
# 62.7776, 40.7231, 43.7788, 60.9610

# 10mm
# 62.7526, 58.3105, 8.3695, 58.3890

def reflection_to_line(reflection: int) -> float:
    """Convert reflection to line amount."""
    line_amount = 1 - (reflection - BLACK) / (WHITE - BLACK)
    return max(min(line_amount, 1), 0)


# sum = [0, 0, 0, 0]
# tally = 0
# stopwatch = StopWatch()
# print(db.settings())
# for i in range(4):
    # db.curve(82, 90)
# db.straight(1000)
while True:
    db.drive(70, 0)
    # if db.state()[1] >= 700:
        # print(stopwatch.time())
        # db.stop()
        # break
    # for i in range(4):
        # sum[i] += color_sensors[i].reflection()
    # tally += 1
    print([sensor.reflection() for sensor in color_sensors])
    # line_amounts = [reflection_to_line(sensor.reflection()) for sensor in color_sensors]
    # error = sum(reflection * position for reflection, position in zip(line_amounts, SENSOR_POSITIONS))
    # print(error)

# print(sum, tally)
