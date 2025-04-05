"""IDE Series 2025—SpRInt."""

from pybricks.tools import StopWatch, wait
from umath import pi, sin

from hardware import color_sensors, db, hub

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
        db.drive(700, turn_rate)


def turn_left():
    """Turn left."""
    db.curve(80, -90)


def turn_right():
    """Turn right."""
    db.curve(80, 90)


def main():
    """Main function."""
    hub.display.icon(
        [
            [0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0],
            [0, 0, 100, 0, 0],
            [0, 100, 0, 100, 0],
            [100, 0, 0, 0, 100],
        ]
    )
    # hub.speaker.play_notes(test)

    wait(500)

    # Lap 1
    linetrack(533 - 83, 700)
    hub.speaker.beep(262, 50)
    linetrack(213 + 83 + 50, 400)
    hub.speaker.beep(294, 50)
    linetrack(482 - 50 - 60, 700)
    hub.speaker.beep(330, 50)
    linetrack(123 + 60, 500)
    hub.speaker.beep(349, 50)
    linetrack(323 - 50, 700, direction="both")
    hub.speaker.beep(392, 50)
    turn_right()
    linetrack(397 - 36 - 83, 700)
    hub.speaker.beep(440, 50)
    linetrack(0 + 83 + 50, 400)
    hub.speaker.beep(494, 50)
    linetrack(287 - 50 - 50, 700, direction="both")
    hub.speaker.beep(523, 50)
    turn_right()
    linetrack(-999, 700)
    hub.speaker.beep(262, 50)

    # Lap 2
    # turn_right()
    # linetrack(150)
    # turn_right()
    # linetrack(70, direction="right")
    # turn_right()
    # linetrack(450)
    # turn_right()
    # linetrack(80)
    # turn_left()
    # linetrack(1150, direction="none")
    # db.turn(12.87)
    # db.straight(436)
    # db.turn(-12.87)
    # linetrack(850, direction="right")
    # turn_right()
    # linetrack(150, direction="left")
    # turn_left()
    # linetrack(90)
    # turn_left()
    # linetrack(450)
    # turn_right()
    # linetrack(986, direction="none")


if __name__ == "__main__":
    main()
