"""spRInt.

For IDE Series 2025
"""

from pybricks.hubs import PrimeHub
from pybricks.parameters import Direction, Port, Stop
from pybricks.pupdevices import ColorSensor, Motor
from pybricks.robotics import DriveBase
from pybricks.tools import wait

# Line tracking constants
WHITE = 100
BLACK = 0
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
db = DriveBase(left_motor, right_motor, 88, 160)
db.settings(straight_speed=SPEED, straight_acceleration=69420, turn_acceleration=69420, turn_rate=69420)


def reflection_to_line(reflection: int) -> float:
    """Convert reflection to line amount."""
    return 1 - (reflection - BLACK) / (WHITE - BLACK)


while True:
    db.drive(60, 0)
    line_amounts = (reflection_to_line(sensor.reflection()) for sensor in color_sensors)
    error = sum(reflection * position for reflection, position in zip(line_amounts, SENSOR_POSITIONS))
    print(error)
