"""IDE Series 2025—SpRInt."""

from pybricks.hubs import PrimeHub
from pybricks.parameters import Color, Direction, Port, Side
from pybricks.pupdevices import ColorSensor, Motor
from pybricks.robotics import DriveBase
from pybricks.tools import StopWatch, wait
from umath import pi, sin
from usys import version

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
db = DriveBase(left_motor, right_motor, 62.4, 160)
db.settings(straight_speed=530, straight_acceleration=3000, turn_rate=250, turn_acceleration=750)  # max turn acceleration 3000

SENSOR_POSITIONS = (-3, -1, 1, 3)


class PIController:
    """Proportional-integral controller for line tracking."""

    def __init__(self, k_p: float, k_i: float):
        """Init."""
        self.K_P = k_p
        self.K_I = k_i
        self.integral = 0
        self.integral_max = 100
        self.stopwatch = StopWatch()
        self.ticks = 0
        self.error_positive = True

    def update(self, error: float) -> float:
        """Get the PID output for a new error value."""
        # Proportional term
        p_term = self.K_P * error

        # Integral term
        # self.integral += error
        # self.integral = max(min(self.integral, self.integral_max), -self.integral_max)
        # if self.error_positive and error < 0:
        #     self.integral = 0
        #     self.error_positive = False
        # elif not self.error_positive and error > 0:
        #     self.integral = 0
        #     self.error_positive = True
        # i_term = self.K_I * self.integral

        i_term = 0

        # Output readings
        # if self.ticks >= 100:
        # print(error, p_term, i_term)
        # self.ticks = 0
        # self.ticks += 1

        return p_term + i_term


def process_reflections(reflections: tuple[int, int, int, int]) -> tuple[float, float, float, float]:
    """Convert reflection to line amount."""
    whites = (85, 82, 81, 81)
    black = 9
    out = []
    for white, ref in zip(whites, reflections):
        line_amount = 1 - (ref - black) / (white - black)
        line_amount = max(min(line_amount, 1), 0)
        out.append(line_amount)
    return tuple(out)


def linetrack(
    min_distance: int | float,
    speed: int | float,
    *,
    direction: str = "none",
    junctions: int = 1,
    intercept_direction: str = "right",
) -> None:
    """Line track using PID."""
    junctions_crossed = 0
    junction_size = 20
    junction_distance = -junction_size
    pid_controller = PIController(50, 1)  # FIXME: ...
    db.reset()
    hub.display.pixel(0, 0, 0)

    intercept = False
    slow = False

    while True:
        line_amounts = process_reflections(tuple(sensor.reflection() for sensor in color_sensors))

        # Detect junction
        distance_reached = db.distance() > min_distance
        skipping_junction = junction_distance <= db.distance() < junction_distance + junction_size
        if direction == "both":
            junction_reached = sum(line_amounts) > 3
        elif direction == "left":
            junction_reached = line_amounts[0] > 0.8 and line_amounts[1] > 0.5
        elif direction == "right":
            junction_reached = line_amounts[3] > 0.8 and line_amounts[2] > 0.5
        elif direction == "none":
            junction_reached = True
        elif direction == "white":
            junction_reached = sum(line_amounts) < 0.2
        else:
            raise ValueError("Invalid direction")

        # Return if all conditions met, otherwise update junction state
        if junction_reached and distance_reached and not skipping_junction:
            junctions_crossed += 1
            if junctions_crossed >= junctions:
                if direction not in ("none", "white"):
                    db.straight(36)
                return  # End line tracking
            else:
                junction_distance = db.distance()

        linear_error = sum(reflection * position for reflection, position in zip(line_amounts, SENSOR_POSITIONS))
        sined_error = 3 * sin(linear_error * pi / 6)
        turn_rate = pid_controller.update(sined_error)
        drive_speed = speed

        # Reduce speed if the robot is too far from the line
        if abs(sined_error) > 1.5:
            drive_speed = speed - speed * ((abs(sined_error) - 1.5) / 2)
            if not slow:
                slow = True
                hub.display.pixel(0, 0, 100)
        if slow and abs(sined_error) <= 1:
            slow = False
            hub.display.pixel(0, 0, 0)

        # Turn back to the line if the robot ran off the line
        if sum(line_amounts) < 0.2 and not intercept and direction != "white":
            intercept = True
            hub.display.pixel(0, 2, 100)
        elif intercept and abs(sined_error) < 1.5 and sum(line_amounts) > 0.8:
            intercept = False
            hub.display.pixel(0, 2, 0)
        if intercept:
            turn_rate = 100 if intercept_direction == "right" else -100
            drive_speed = 100

        # print(drive_speed)

        db.drive(drive_speed, turn_rate)


def turn_left() -> None:
    """Turn left."""
    db.curve(80, -90)


def turn_right() -> None:
    """Turn right."""
    db.curve(80, 90)


def c_major_scale():
    """A generator for notes in the C major scale."""
    saved = []
    for element in (262, 294, 330, 349, 392, 440, 494, 523):
        yield element
        saved.append(element)

    while saved:
        for element in saved:
            yield element


def play_song(notes: list[tuple[int, int]]) -> None:
    """Play a song from a list of tuples of frequency and duration."""
    for freq, duration in notes:
        if freq > 0:
            hub.speaker.beep(freq, duration)
        else:
            wait(duration)


def lap_1() -> None:
    """Lap 1 (the given path)."""
    linetrack(453, 530)  # PS-T1i
    linetrack(313, 300)  # T1i-T1o
    linetrack(302, 530)  # T1o-T2i
    linetrack(276, 250)  # T2i-T2o
    linetrack(123, 530, direction="both")  # T2o-J1
    turn_right()  # J1
    linetrack(121, 530)  # J1-T3i
    linetrack(290, 250, intercept_direction="left")  # T3i-T3o
    linetrack(0, 530, direction="both", intercept_direction="left")  # T3o-J2
    turn_right()  # J2
    linetrack(362, 530)  # J2-T4i
    linetrack(267, 300)  # T4i-T4o


def lap_2() -> None:
    """Lap 2."""
    linetrack(453, 530)  # PS-T1i
    linetrack(313, 300)  # T1i-T1o
    linetrack(302, 530)  # T1o-T2i
    linetrack(276, 250)  # T2i-T2o
    linetrack(123, 400, junctions=2, direction="both")  # T2o-J1-J3
    turn_left()  # J3
    linetrack(670, 300)  # J3-T5o
    linetrack(300, 150, direction="white")  # T5o-T6o
    db.turn(35)
    db.straight(200)


def main() -> None:
    """Main function."""
    print(version)
    hub.speaker.volume(100)
    notes = c_major_scale()
    hub.display.icon(
        [
            [0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0],
            [0, 0, 100, 0, 0],
            [0, 100, 0, 100, 0],
            [100, 0, 0, 0, 100],
        ]
    )

    # Airbus beacon and strobe lights
    strobe_beacon_colors = [Color.NONE] * 20
    strobe_beacon_colors[0] = Color.WHITE
    strobe_beacon_colors[2] = Color.WHITE
    strobe_beacon_colors[10] = Color.RED
    strobe_beacon_colors[11] = Color.RED
    hub.light.animate(strobe_beacon_colors, 75)

    wait(500)

    # Keep doing lap 1
    # while True:
    # lap_1()

    lap_2()

    # To test distances: Make robot stop after each linetrack


if __name__ == "__main__":
    main()
