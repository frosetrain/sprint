"""Graph some light sensor data."""

import matplotlib.pyplot as plt

WHITES = (67, 62, 60, 64)
BLACK = 8
SENSOR_POSITIONS = (-3, -1, 1, 3)


def process_reflections(reflections: tuple[int, int, int, int]) -> tuple[float, float, float, float]:
    """Convert reflection to line amount."""
    out = []
    for white, ref in zip(WHITES, reflections):
        line_amount = 1 - (ref - BLACK) / (white - BLACK)
        line_amount = max(min(line_amount, 1), 0)
        out.append(line_amount)
    return tuple(out)


values = [[], [], [], []]
total_values = []
errors = []

with open("data/pd0", "r") as f:
    data = f.readlines()
    for i, row in enumerate(data):
        # line_amounts = process_reflections(tuple(map(int, row.split())))
        line_amounts = tuple(map(float, row.split()))
        for j, val in enumerate(line_amounts):
            if j == 0:
                val *= 100
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
