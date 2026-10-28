"""The unchanged 8 speed bins x 8 absolute courses."""

DIRECTIONS = ("north", "northeast", "east", "southeast", "south", "southwest", "west", "northwest")

ACTION_CHOICES = {
    f"speed_{i}__{name}": (i, j) for i in range(8) for j, name in enumerate(DIRECTIONS)
}
