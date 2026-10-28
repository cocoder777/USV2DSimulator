"""Shared geographic coordinates and speed conversion constant."""

import math

EARTH_RADIUS = 6378137.0
KNOT_MPS = 1852 / 3600


def xy(lon_lat, origin):
    lon, lat = lon_lat
    lon0, lat0 = origin
    return (
        (lon - lon0) * math.pi / 180 * EARTH_RADIUS * math.cos(math.radians(lat0)),
        (lat - lat0) * math.pi / 180 * EARTH_RADIUS,
    )
