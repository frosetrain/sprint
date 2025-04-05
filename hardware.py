"""SPIKE Prime hardware."""

from pybricks.hubs import PrimeHub
from pybricks.parameters import Direction, Port, Side
from pybricks.pupdevices import ColorSensor, Motor
from pybricks.robotics import DriveBase

hub = PrimeHub()
hub.display.orientation(Side.BOTTOM)
left_motor = Motor(Port.F, Direction.COUNTERCLOCKWISE)
right_motor = Motor(Port.E, Direction.CLOCKWISE)
color_sensors = (
    ColorSensor(Port.D),
    ColorSensor(Port.B),
    ColorSensor(Port.A),
    ColorSensor(Port.C),
)
db = DriveBase(left_motor, right_motor, 88, 163)
db.settings(700, straight_acceleration=2000, turn_rate=183, turn_acceleration=825)
