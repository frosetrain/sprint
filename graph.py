import matplotlib.pyplot as plt

values_1 = []
values_2 = []
values_3 = []
values_4 = []
total_values = []

with open("out", "r") as f:
    data = f.readlines()
    for line in data:
        v1, v2, v3, v4, total = map(int, line.split())
        values_1.append(v1)
        values_2.append(v2)
        values_3.append(v3)
        values_4.append(v4)
        total_values.append(total)

plt.plot(values_1, label="Sensor 1")
plt.plot(values_2, label="Sensor 2")
plt.plot(values_3, label="Sensor 3")
plt.plot(values_4, label="Sensor 4")
plt.plot(total_values, label="Total")
plt.legend()
plt.show()
