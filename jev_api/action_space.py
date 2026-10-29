"""Eight speed bins and sixteen absolute compass courses (128 Jev choices)."""

DIRECTIONS = ("north", "north_northeast", "northeast", "east_northeast",
              "east", "east_southeast", "southeast", "south_southeast",
              "south", "south_southwest", "southwest", "west_southwest",
              "west", "west_northwest", "northwest", "north_northwest")
COURSES_DEG = tuple(j * 22.5 for j in range(len(DIRECTIONS)))
SPEED_FRACTIONS = (0, .05, .1, .2, .35, .5, .75, 1)

ACTION_CHOICES = {
    f"speed_{i}__{name}": (i, j) for i in range(8) for j, name in enumerate(DIRECTIONS)
}
