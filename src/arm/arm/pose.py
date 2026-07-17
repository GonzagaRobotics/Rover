import numpy as np
from auto_msgs.msg import Aruco

# Distances between the centers of the 4 markers
MARKERS_W = 0.2
MARKERS_H = 0.1


def get_poses(msg: Aruco) -> np.ndarray:
    out = np.zeros((4, 3), dtype=np.float32)
    found = set()

    for i in range(len(msg.ids)):
        if msg.ids[i] > 3:
            continue

        found.add(msg.ids[i])
        out[msg.ids[i], 0] = msg.translations[i].x
        out[msg.ids[i], 1] = msg.translations[i].y
        out[msg.ids[i], 2] = msg.translations[i].z

    # We need all 4 markers to compute the pose
    # TODO: Do we really?
    if len(found) < 4:
        return None

    return out


def find_pose(msg: Aruco) -> np.ndarray:
    poses = get_poses(msg)
    if poses is None:
        return None

    # TODO: Double check these
    origins = np.array([
        [-MARKERS_W / 2, -MARKERS_H / 2, 0],
        [MARKERS_W / 2, -MARKERS_H / 2, 0],
        [MARKERS_W / 2,  MARKERS_H / 2, 0],
        [-MARKERS_W / 2,  MARKERS_H / 2, 0]
    ], dtype=np.float32)

    # TODO: Use a more robust method to compute the pose
    origins[:] -= poses[:]

    pose = origins.mean(axis=0)

    return pose
