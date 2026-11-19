"""Reproject the contact footprint of a metric obstacle into the rectified view."""
import math


def contact_point(position, center_offset):
    z, x = position
    if center_offset:
        distance = math.hypot(z, x)
        if distance <= center_offset:
            return 0., 0.
        scale = 1. - center_offset / distance
        return z * scale, x * scale
    return z, x
