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
db.settings(straight_speed=500, straight_acceleration=3000, turn_rate=350, turn_acceleration=3000)

SENSOR_POSITIONS = (-3, -1, 1, 3)


class PD:
    """Proportional-derivative controller for line tracking."""

    def __init__(self, k_p: float, k_d: float):
        """Init."""
        self.K_P = k_p
        self.K_D = k_d
        self.DERIVATIVE_WINDOW = 10
        self.rolling_errors = [0] * self.DERIVATIVE_WINDOW
        self.rolling_times = [0] * self.DERIVATIVE_WINDOW
        self.error_pointer = 0
        self.rolling_filled = False
        self.stopwatch = StopWatch()
        self.ticks = 0

    def update(self, error: float) -> float:
        """Get the PID output for a new error value."""
        # stopwatch_time = self.stopwatch.time()

        # Proportional term
        p_term = self.K_P * error

        # # Derivative term
        # if stopwatch_time - self.rolling_times[self.error_pointer] <= 0:
        #     d_term = 0
        # elif not self.rolling_filled:
        #     d_term = 0
        # else:
        #     d_term = (
        #         self.K_D
        #         * (error - self.rolling_errors[self.error_pointer])
        #         / (stopwatch_time - self.rolling_times[self.error_pointer])
        #     )

        # Update errors
        # self.rolling_errors[self.error_pointer] = error
        # self.rolling_times[self.error_pointer] = stopwatch_time
        # self.error_pointer = (self.error_pointer + 1) % self.DERIVATIVE_WINDOW
        # if not self.rolling_filled and self.error_pointer == 0:
        #     self.rolling_filled = True

        # Output readings
        # if self.ticks >= 100:
        #     print(stopwatch_time, error, p_term, d_term)
        #     self.ticks = 0
        # self.ticks += 1

        d_term = 0

        return p_term + d_term


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


def linetrack(min_distance: int | float, speed: int | float, *, direction: str = "none", junctions: int = 1) -> None:
    """Line track using PID."""
    junctions_crossed = 0
    junction_size = 20
    junction_distance = -junction_size
    distance_indicator = False
    junction_indicator = False
    ran_off = False
    off_distance = 0
    hub.display.pixel(0, 0, 0)
    hub.display.pixel(0, 1, 0)
    pid_controller = PD(40, 0)  # FIXME: ...
    db.reset()

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
                if direction != "none":
                    db.straight(36)
                return  # End line tracking
            else:
                junction_distance = db.distance()
                # Show junction skipping indicator
                hub.display.pixel(0, 3, 100)
                hub.display.pixel(0, 4, 100)

        linear_error = sum(reflection * position for reflection, position in zip(line_amounts, SENSOR_POSITIONS))
        sined_error = 3 * sin(linear_error * pi / 6)
        turn_rate = pid_controller.update(sined_error)
        drive_speed = speed
        if abs(sined_error) > 1:
            drive_speed = speed - speed * ((abs(sined_error) - 1) / 3)

        # Turn back to the line if the robot ran off the line
        if sum(line_amounts) < 0.5 and False:  # FIXME: ...
            if not ran_off:
                ran_off = True
                off_distance = db.distance()
            turn_rate = 100
            drive_speed = 100
        elif ran_off:
            ran_off = False
            db.reset(off_distance, 0)

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

    while True:
        linetrack(1600, 500)

    # Keep doing lap 1
    while True:
        linetrack(533 - 40, 700)  # PS-T1i
        hub.speaker.beep(next(notes), 50)
        linetrack(213 + 40 + 50 - 80, 400)  # HACK: T1i-T1o
        hub.speaker.beep(next(notes), 50)
        linetrack(482 - 50 - 23, 700)  # T1o-T2i
        hub.speaker.beep(next(notes), 50)
        linetrack(123 + 23, 500)  # T2i-T2o
        hub.speaker.beep(next(notes), 50)
        linetrack(323 - 150, 700, direction="both")  # T2o-J1
        hub.speaker.beep(next(notes), 50)
        turn_right()
        linetrack(397 - 36 - 40, 700)  # J1-T3i
        hub.speaker.beep(next(notes), 50)
        linetrack(0 + 40 + 50, 200)  # T3i-T3o
        hub.speaker.beep(next(notes), 50)
        linetrack(0, 700, direction="both")  # FIXME: T3o-J2
        hub.speaker.beep(next(notes), 50)
        turn_right()
        linetrack(538 - 36 - 40, 700)  # J2-T4i
        hub.speaker.beep(next(notes), 50)
        linetrack(217, 400)  # FIXME: T4i-T4o
        hub.speaker.beep(next(notes), 50)
        # linetrack(306 - 50, 700)  # T4o-S/F


if __name__ == "__main__":
    main()
