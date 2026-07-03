import numpy as np


def calculate_distance(point1: np.ndarray, point2: np.ndarray) -> float:
    """Haversine formula to calculate distance between two GPS coordinates in meters."""
    R = 6371000
    lat1, lon1 = np.radians(point1)
    lat2, lon2 = np.radians(point2)

    inner = np.sin((lat2 - lat1) / 2) ** 2 + np.cos(lat1) * np.cos(lat2) * np.sin((lon2 - lon1) / 2) ** 2
    return 2 * R * np.arcsin(np.sqrt(inner))
