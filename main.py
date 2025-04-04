"""IDE Series 2025—SpRInt."""

from pybricks.hubs import PrimeHub
from pybricks.iodevices import XboxController
from pybricks.parameters import Direction, Port, Side
from pybricks.pupdevices import ColorSensor, Motor
from pybricks.robotics import DriveBase
from pybricks.tools import StopWatch, wait
from umath import pi, sin

hub = PrimeHub()
left_motor = Motor(Port.F, Direction.COUNTERCLOCKWISE)
right_motor = Motor(Port.E, Direction.CLOCKWISE)
color_sensors = (
    ColorSensor(Port.D),
    ColorSensor(Port.B),
    ColorSensor(Port.A),
    ColorSensor(Port.C),
)
db = DriveBase(left_motor, right_motor, 88, 163)
db.settings(700, straight_acceleration=1400, turn_rate=183, turn_acceleration=825)
# Default: 307, 1152, 183, 825

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


def linetrack(min_distance: int, speed: int, *, direction: str = "none", junctions: int = 1) -> None:
    """Line track using PID."""
    junctions_crossed = 0
    junction_size = 20
    junction_distance = -junction_size
    distance_indicator = False
    junction_indicator = False
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
                db.straight(36)
                return  # End line tracking
            else:
                junction_distance = db.distance()
                # Show junction skipping indicator
                hub.display.pixel(0, 3, 100)
                hub.display.pixel(0, 4, 100)

        error = sum(reflection * position for reflection, position in zip(line_amounts, SENSOR_POSITIONS))
        db.drive(350, pid_controller.update(error))  # TODO: Variable speed


def simple_linetrack(p: int, i: int, d: int, distance: int = 0) -> None:
    """Simple line tracking."""
    pid_controller = PID(p, i, d)
    db.reset()
    frame_count = 0
    stopwatch = StopWatch()
    stopwatch.reset()
    while True:
        if distance and db.distance() > distance:
            db.stop()
            break
        line_amounts = process_reflections(tuple(sensor.reflection() for sensor in color_sensors))
        linear_error = sum(reflection * position for reflection, position in zip(line_amounts, SENSOR_POSITIONS))
        sined_error = 3 * sin(linear_error * pi / 6)
        turn_rate = pid_controller.update(sined_error)
        db.drive(700, turn_rate)
        frame_count += 1
    print("fps:", frame_count / stopwatch.time() * 1000)


def turn_left():
    """Turn left."""
    db.curve(80, -90)


def turn_right():
    """Turn right."""
    db.curve(80, 90)


def average_reflection() -> tuple[int, int, int, int]:
    """Get the average reflection for calibration."""
    totals = [0, 0, 0, 0]
    for i in range(10000):
        for i, sensor in enumerate(color_sensors):
            totals[i] += sensor.reflection()
    return tuple(total // 10000 for total in totals)


def print_reflections():
    """Continuously print space-separated reflection values."""
    while True:
        print(" ".join(sensor.reflection() for sensor in color_sensors))


def controller_drive():
    """Drive the robot using a controller (hehehehaw)."""
    controller = XboxController()
    while True:
        steering = controller.joystick_left()[0]
        throttle = controller.triggers()[1]
        db.drive(throttle * 7, steering)


def main():
    """Main function."""
    hub.display.orientation(Side.BOTTOM)
    hub.display.icon(
        [
            [0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0],
            [0, 0, 100, 0, 0],
            [0, 100, 0, 100, 0],
            [100, 0, 0, 0, 100],
        ]
    )

    wait(500)

    # db.turn(180)
    while True:
        simple_linetrack(50, 0, 2000, 1600)  # TODO: Tune PID
        db.brake()
        wait(500)
        db.turn(180)
        wait(500)

    # Lap 1
    linetrack(300, 700)
    hub.speaker.beep()
    linetrack(180, 300)
    hub.speaker.beep()
    linetrack(540, 700)
    hub.speaker.beep()
    linetrack(130, 400)
    hub.speaker.beep()
    linetrack(310, 700, direction="both")
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


if __name__ == "__main__":
    main()
