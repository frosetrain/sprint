"""Functions for calibration."""

from ustruct import calcsize, pack_into, unpack_from

from hardware import color_sensors, hub


def average_reflection(samples: int = 10000) -> tuple[int, int, int, int]:
    """Get the average reflection for calibration."""
    totals = [0, 0, 0, 0]
    for i in range(samples):
        for i, sensor in enumerate(color_sensors):
            totals[i] += sensor.reflection()

    averages = tuple(total // samples for total in totals)
    buf = bytearray(calcsize("4B"))
    pack_into("4B", buf, 0, *averages)
    hub.system.storage(0, write=buf)
    return averages


def read_storage() -> tuple[int, int, int, int]:
    """Read data from hub storage."""
    dump = hub.system.storage(0, read=calcsize("4B"))
    unpacked_averages = unpack_from("4B", dump, 0)
    return unpacked_averages


def print_reflections():
    """Continuously print space-separated reflection values."""
    while True:
        print(" ".join(sensor.reflection() for sensor in color_sensors))
