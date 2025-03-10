from pybricks.hubs import PrimeHub
from pybricks.parameters import Direction, Port, Stop
from pybricks.pupdevices import ColorSensor, Motor
from pybricks.robotics import DriveBase
from pybricks.tools import wait

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
db.settings(straight_speed=450, straight_acceleration=1000, turn_acceleration=1000, turn_rate=400)

while True:
    db.drive(60, 0)
    reflections = []
    for sensor in color_sensors:
        reflections.append(sensor.reflection())
    print(" ".join([str(r) for r in reflections]), sum(reflections))
