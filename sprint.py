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


def linetrack(junction_type, start, stretch):
    slow = True
    db.reset()
    while True:
        if slow and db.distance() > start:
            slow = False
        if not slow and db.distance() > stretch:
            slow = True
        left_reflection = left_color_sensor.reflection()
        right_reflection = right_color_sensor.reflection()
        print(left_reflection, right_reflection)
        if junction_type == "left":
            hit = left_reflection < 25 and right_reflection > 60
        elif junction_type == "both":
            hit = left_reflection < 25 and right_reflection < 25
        elif junction_type == "right":
            hit = left_reflection > 60 and right_reflection < 25
        elif junction_type == "stop":
            hit = True
        if hit and db.distance() > stretch:
            db.straight(112, then=Stop.BRAKE)
            break
        difference = left_reflection - right_reflection
        if slow:
            #original:200
            db.drive(200, difference * 0.6)
        else:
            db.drive(469, difference * 0.4)


linetrack("left", 100, 100)
db.turn(-90)
linetrack("both", 300, 300)
db.turn(90)
linetrack("both", 100, 100)
db.turn(45)
linetrack("both", 5, 5)
db.turn(-45)
linetrack("both", 100, 100)
db.turn(180)

