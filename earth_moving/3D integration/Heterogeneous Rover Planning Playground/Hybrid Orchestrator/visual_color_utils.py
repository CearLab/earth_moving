"""Shared conversion utilities for consistent PyBullet/PyGame rover colors."""

from __future__ import annotations


def pygame_rgb(color, fallback=(128, 128, 128)):
    """Convert RGB/RGBA values in either 0..1 or 0..255 form to PyGame RGB."""
    if color is None:
        return tuple(int(value) for value in fallback[:3])
    try:
        values = [float(value) for value in color[:3]]
    except (TypeError, ValueError, IndexError):
        return tuple(int(value) for value in fallback[:3])
    if all(0.0 <= value <= 1.0 for value in values):
        values = [round(255.0 * value) for value in values]
    return tuple(max(0, min(255, int(round(value)))) for value in values)
