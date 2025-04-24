"""IDE Series 2025—SpRInt."""

from pybricks.hubs import PrimeHub
from pybricks.parameters import Color, Direction, Port, Side
from pybricks.pupdevices import ColorSensor, Motor
from pybricks.robotics import DriveBase
from pybricks.tools import StopWatch, wait
from umath import pi, sin

from music.carry_that_weight import BEEPS as CARRY_THAT_WEIGHT
from music.golden_slumbers import BEEPS as GOLDEN_SLUMBERS

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
db.settings(straight_speed=500, straight_acceleration=2000, turn_rate=130, turn_acceleration=585)

SENSOR_POSITIONS = (-3, -1, 1, 3)


class PID:
    """Proportional-integral-derivative controller for line tracking."""

    def __init__(self, k_p: float, k_i: float, k_d: float):
        """Init."""
        self.K_P = k_p
        self.K_I = k_i
        self.K_D = k_d
        self.INTEGRAL_MAX = self.K_I / 300
        # self.INTEGRAL_MAX = 1000
        self.DERIVATIVE_WINDOW = 10
        self.integral = 0
        self.rolling_errors = [0] * self.DERIVATIVE_WINDOW
        self.rolling_times = [0] * self.DERIVATIVE_WINDOW
        self.error_pointer = 0
        self.rolling_filled = False
        self.stopwatch = StopWatch()
        self.stopwatch.reset()
        self.previous_time = 0
        self.output_count = 0

    def update(self, error: float) -> float:
        """Get the PID output for a new error value."""
        # Update times
        stopwatch_time = self.stopwatch.time()
        dt = (stopwatch_time - self.previous_time) / 1000
        self.previous_time = stopwatch_time

        # Proportional term
        p_term = self.K_P * error

        # Integral term
        self.integral += error * dt
        self.integral = max(min(self.integral, self.INTEGRAL_MAX), -self.INTEGRAL_MAX)
        i_term = self.K_I * self.integral

        # Derivative term
        if stopwatch_time - self.rolling_times[self.error_pointer] <= 0:
            d_term = 0
        elif not self.rolling_filled:
            d_term = 0
        else:
            d_term = (
                self.K_D
                * (error - self.rolling_errors[self.error_pointer])
                / (stopwatch_time - self.rolling_times[self.error_pointer])
            )

        # Update errors
        self.rolling_errors[self.error_pointer] = error
        self.rolling_times[self.error_pointer] = stopwatch_time
        self.error_pointer = (self.error_pointer + 1) % self.DERIVATIVE_WINDOW
        if not self.rolling_filled and self.error_pointer == 0:
            self.rolling_filled = True

        # Output reading
        if stopwatch_time >= self.output_count * 100:
            print(error, p_term, i_term, d_term)
            self.output_count += 1

        return p_term + i_term + d_term


def process_reflections(reflections: tuple[int, int, int, int]) -> tuple[float, float, float, float]:
    """Convert reflection to line amount."""
    whites = (67, 62, 60, 64)
    black = 8
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
    pid_controller = PID(50, 0, 2000)
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

        # Turn back to the line if the robot ran off the line
        if sum(line_amounts) < 0.5:
            if not ran_off:
                ran_off = True
                off_distance = db.distance()
            turn_rate = 100
            drive_speed = 100
        elif ran_off:
            ran_off = False
            db.reset(off_distance)

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


def main():
    """Main function."""
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

    # play_song(GOLDEN_SLUMBERS)
    # play_song(CARRY_THAT_WEIGHT)
    wait(500)

    # Lap 1
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
    linetrack(0 + 40 + 50, 400)  # T3i-T3o
    hub.speaker.beep(next(notes), 50)
    # linetrack(287 - 150 - 50, 700, direction="both")  # T3o-J2
    hub.speaker.beep(next(notes), 50)
    turn_right()
    linetrack(538 - 36 - 40, 700)  # J2-T4i
    hub.speaker.beep(next(notes), 50)
    linetrack(217 + 40 + 50, 400)  # T4i-T4o
    hub.speaker.beep(next(notes), 50)
    linetrack(306 - 50, 700)  # T4o-S/F
    db.stop()


if __name__ == "__main__":
    main()
