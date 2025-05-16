"""Graph some light sensor data."""

import click
import matplotlib.pyplot as plt

SENSOR_POSITIONS = (-3, -1, 1, 3)


def process_reflections(reflections: tuple[int, int, int, int]) -> tuple[float, float, float, float]:
    """Convert reflection to line amount."""
    whites = (58, 53, 53, 55)
    black = 7
    out = []
    for white, ref in zip(whites, reflections):
        line_amount = 1 - (ref - black) / (white - black)
        line_amount = max(min(line_amount, 1), 0)
        out.append(line_amount)
    return tuple(out)


@click.command()
@click.argument("file", type=click.File("r"))
def plot(file):
    values = [[], [], [], []]
    total_values = []
    errors = []

    data = file.readlines()
    for i, row in enumerate(data):
        # line_amounts = process_reflections(tuple(map(int, row.split())))
        line_amounts = tuple(map(float, row.split()))
        for j, val in enumerate(line_amounts):
            values[j].append(val)
        # error = sum(reflection * position for reflection, position in zip(line_amounts, SENSOR_POSITIONS))
        # total = sum(line_amounts)
        # total_values.append(total)
        # errors.append(error)

    plt.plot(values[0], label="Sensor 1")
    plt.plot(values[1], label="Sensor 2")
    plt.plot(values[2], label="Sensor 3")
    plt.plot(values[3], label="Sensor 4")
    # plt.plot(total_values, label="Total")
    # plt.plot(errors, label="Error")
    plt.legend()
    plt.show()


if __name__ == "__main__":
    plot()
