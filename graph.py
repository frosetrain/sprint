import matplotlib.pyplot as plt

WHITES = (71, 66, 62, 65)
BLACK = 8
SENSOR_POSITIONS = (-3, -1, 1, 3)


def process_reflections(reflections: tuple[int, int, int, int]) -> tuple[float, float, float, float]:
    """Convert reflection to line amount."""
    out = []
    for white, ref in zip(WHITES, reflections):
        line_amount = 1 - (ref - BLACK) / (white - BLACK)
        line_amount = max(min(line_amount, 1), 0)
        line_amount = curve(line_amount)
        out.append(line_amount)
    return tuple(out)


# 0.5 * (sin(pi * (reflection - 0.5)) + 1)


def curve(x: float) -> float:
    if x < 0.3:
        return (2 / 3) * x
    elif x < 0.4:
        return 3 * x - 0.7
    elif x < 0.7:
        return (2 / 3) * x + (7 / 30)
    else:
        return x


values = [[], [], [], []]
total_values = []
errors = []

with open("log3", "r") as f:
    data = f.readlines()
    for i, row in enumerate(data):
        line_amounts = process_reflections(tuple(map(int, row.split())))
        for j, val in enumerate(line_amounts):
            values[j].append(val)
        error = sum(reflection * position for reflection, position in zip(line_amounts, SENSOR_POSITIONS))
        total = sum(line_amounts)
        total_values.append(total)
        errors.append(error)


plt.plot(values[0], label="Sensor 1")
plt.plot(values[1], label="Sensor 2")
plt.plot(values[2], label="Sensor 3")
plt.plot(values[3], label="Sensor 4")
# plt.plot(total_values, label="Total")
plt.plot(errors, label="Error")
plt.legend()
plt.show()
