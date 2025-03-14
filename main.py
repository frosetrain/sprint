"""IDE Series 2025—SpRInt."""

from pybricks.hubs import PrimeHub
from pybricks.parameters import Direction, Port, Side, Stop
from pybricks.pupdevices import ColorSensor, Motor
from pybricks.robotics import DriveBase
from pybricks.tools import wait

# Line tracking constants
WHITES = (67, 62, 60, 64)
BLACK = 8
SPEED = 700
SENSOR_POSITIONS = (-3, -1, 1, 3)
K_P = 24.955
K_I = 33.417
K_D = 6.9888
INTEGRAL_MAX = 300
DERIVATIVE_WINDOW = 10
JUNCTION_SKIP_FRAMES = 50

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
    if x < 0.1:
        return x
    elif x < 0.2:
        return 4 * x - 0.3
    elif x < 0.8:
        return x / 2 + 0.4
    else:
        return x


def linetrack(min_distance: int, *, direction: str = "both", junctions: int = 1) -> None:
    """Line track using PID."""
    integral = 0
    previous_error = 0  # NOTE: For one-frame derivative
    error = 100
    rolling_errors = [0] * DERIVATIVE_WINDOW
    error_pointer = 0
    junctions_crossed = 0
    junction_distance = -20
    distance_indicator = False
    junction_indicator = False
    db.reset()
    hub.display.pixel(0, 0, 0)
    hub.display.pixel(0, 1, 0)

    while True:
        line_amounts = process_reflections(sensor.reflection() for sensor in color_sensors)

        # Detect junction
        distance_reached = db.distance() > min_distance
        skipping_junction = junction_distance <= db.distance() < junction_distance + 20
        if direction == "both":
            junction_reached = sum(line_amounts) > 3
        elif direction == "left":
            junction_reached = line_amounts[0] > 0.8 and line_amounts[1] > 0.5
        elif direction == "right":
            junction_reached = line_amounts[3] > 0.8 and line_amounts[2] > 0.5
        elif direction == "none":
            junction_reached = True
        else:
            raise ValueError("Invalid direction")

        # Show distance reached indicator
        if distance_reached and not distance_indicator:
            hub.display.pixel(0, 0, 100)
            hub.display.pixel(0, 1, 100)
            distance_indicator = True

        # Hide junction skipping indicator
        if junction_indicator and not skipping_junction:
            hub.display.pixel(0, 3, 0)
            hub.display.pixel(0, 4, 0)
            junction_indicator = False

        # Return if all conditions met, otherwise update junction state
        if junction_reached and distance_reached and not skipping_junction:
            junctions_crossed += 1
            if junctions_crossed >= junctions:
                return  # End line tracking
            else:
                junction_distance = db.distance()
                # Show junction skipping indicator
                hub.display.pixel(0, 3, 100)
                hub.display.pixel(0, 4, 100)

        # PID controller
        error = sum(reflection * position for reflection, position in zip(line_amounts, SENSOR_POSITIONS))
        p_term = K_P * error
        integral += error
        integral = max(min(integral, INTEGRAL_MAX), -INTEGRAL_MAX)
        i_term = K_I * integral
        d_term = K_D * (error - previous_error)  # NOTE
        d_term = K_D * (error - rolling_errors[error_pointer])

        # Update errors
        previous_error = error  # NOTE
        rolling_errors[error_pointer] = error
        error_pointer += 1
        error_pointer %= DERIVATIVE_WINDOW

        # Drive based on PID output
        output = p_term + i_term + d_term
        db.drive(SPEED, output)


def turn_left():
    """Turn left."""
    db.curve(80, -90)


def turn_right():
    """Turn right."""
    db.curve(80, 90)


hub.display.orientation(Side.BOTTOM)
hub.display.icon(
    [
        [0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0],
        [0, 0, 50, 0, 0],
        [0, 50, 0, 50, 0],
        [50, 0, 0, 0, 50],
    ]
)

# Lap 1
linetrack(1300)
turn_right()
linetrack(450)
turn_right()
linetrack(2500, junctions=2)  # Lap 2
turn_right()
linetrack(150)
turn_right()
linetrack(70, direction="right")
turn_right()
linetrack(450)
turn_right()
linetrack(80)
turn_left()
linetrack(1150, direction="none")
db.turn(12.87)
db.straight(436)
db.turn(-12.87)
linetrack(850, direction="right")
turn_right()
linetrack(150, direction="left")
turn_left()
linetrack(90)
turn_left()
linetrack(450)
turn_right()
linetrack(986, direction="none")
