"""Some functions to calibrate the line tracking robot."""

from pybricks.hubs import PrimeHub
from pybricks.parameters import Direction, Port, Stop
from pybricks.pupdevices import ColorSensor, Motor
from pybricks.robotics import DriveBase
from pybricks.tools import wait

# Line tracking constants
WHITES = (67, 62, 60, 64)
BLACK = 8
SPEED = 700
SENSOR_POSITIONS = (-3, -1, 1, 3)

hub = PrimeHub()
left_motor = Motor(Port.F, Direction.COUNTERCLOCKWISE)
right_motor = Motor(Port.E, Direction.CLOCKWISE)
color_sensors = (
    ColorSensor(Port.D),
    ColorSensor(Port.B),
    ColorSensor(Port.A),
    ColorSensor(Port.C),
)
db = DriveBase(left_motor, right_motor, 88, 164)
db.settings(SPEED, straight_acceleration=1800, turn_rate=100, turn_acceleration=400)
# Default: 307, 1152, 183, 825


def process_reflections(reflections: tuple[int, int, int, int]) -> tuple[float, float, float, float]:
    """Convert reflection to line amount."""
    out = []
    for white, ref in zip(WHITES, reflections):
        line_amount = 1 - (ref - BLACK) / (white - BLACK)
        line_amount = max(min(line_amount, 1), 0)
        line_amount = curve(line_amount)
        out.append(line_amount)
    return tuple(out)


def curve(x: float) -> float:
    """Curve the line amount to keep total consistent."""
    if x < 0.1:
        return x
    elif x < 0.2:
        return 4 * x - 0.3
    elif x < 0.8:
        return x / 2 + 0.4
    else:
        return x


def average_reflection() -> tuple[int, int, int, int]:
    """Get the average reflection for calibration."""
    totals = [0, 0, 0, 0]
    for i in range(10000):
        for i, sensor in enumerate(color_sensors):
            totals[i] += sensor.reflection()
    return tuple(total // 10000 for total in totals)


# print(average_reflection())

while True:
    db.reset()
    while db.distance() < 2000:
        line_amounts = process_reflections(sensor.reflection() for sensor in color_sensors)
        error = sum(reflection * position for reflection, position in zip(line_amounts, SENSOR_POSITIONS))
        # if error > 0:
        # error = 1
        # else:
        # error = -1
        # print(error)
        db.drive(SPEED, error * 28)
    db.stop()
    db.turn(170)
