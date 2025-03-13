"""spRInt.

For IDE Series 2025
"""

from pybricks.hubs import PrimeHub
from pybricks.parameters import Direction, Port, Stop
from pybricks.pupdevices import ColorSensor, Motor
from pybricks.robotics import DriveBase
from pybricks.tools import wait

# Line tracking constants
WHITES = (71, 66, 62, 65)
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
    if x < 0.3:
        return (2 / 3) * x
    elif x < 0.4:
        return 3 * x - 0.7
    elif x < 0.7:
        return (2 / 3) * x + (7 / 30)
    else:
        return x


while True:
    db.drive(70, 0)
    print(process_reflections(sensor.reflection() for sensor in color_sensors))
