"""Calculates the distance travelled during acceleration."""

import click

ACCELERATION = 2000


@click.command()
@click.argument("initial", type=float)
@click.argument("final", type=float)
def acceleration_distance(initial, final):
    """Calculate the distance travelled during acceleration from INITIAL to FINAL."""
    direction = 1 if final > initial else -1
    distance = (final**2 - initial**2) / (2 * direction * ACCELERATION)
    click.echo(f"Distance: {distance:.1f} mm")


if __name__ == "__main__":
    acceleration_distance()
