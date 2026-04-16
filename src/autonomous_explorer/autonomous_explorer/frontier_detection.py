"""frontier_detection.py — Pure-Python frontier detection for occupancy grids.

This module implements Canny-edge-based frontier detection on 2-D occupancy
grids. It is intentionally free of any ROS dependency so that every function
can be unit-tested without a running ROS graph.

Pipeline overview
-----------------
1. Convert the flattened ``OccupancyGrid.data`` array into a three-level
   grayscale image (unknown → 127, free → 255, occupied → 0).
2. Extract edges of free space with the Canny detector.
3. Intersect those edges with dilated unknown space → **frontier cells**.
4. Cluster frontier cells with connected-components analysis.
5. Rank clusters by a cost–utility function and pick the best goal.

Why Canny instead of Wavefront Frontier Detection (WFD)?
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Classic WFD (Yamauchi 1997) requires a BFS from the robot cell outward,
which is simple but couples detection to the robot's pose and scales
poorly on large grids.  Treating the occupancy grid as a grayscale image
lets us delegate edge extraction to OpenCV's highly-optimised Canny
implementation and reuse the same image-processing primitives already
employed by the ``obstacle_detector`` node from Phase 2.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import NamedTuple

import cv2
import numpy as np
from numpy.typing import NDArray


# ---------------------------------------------------------------------------
# Data structures
# ---------------------------------------------------------------------------

class Pose2D(NamedTuple):
    """Minimal 2-D pose used for grid ↔ world conversions.

    Attributes
    ----------
    x : float
        X-coordinate of the map origin in world frame (metres).
    y : float
        Y-coordinate of the map origin in world frame (metres).
    """

    x: float
    y: float


@dataclass(frozen=True)
class FrontierCluster:
    """A group of connected frontier pixels.

    Attributes
    ----------
    centroid_px : tuple[int, int]
        Centroid of the cluster in pixel coordinates ``(col, row)``.
    size : int
        Number of frontier pixels in the cluster.
    bbox : tuple[int, int, int, int]
        Bounding box as ``(x, y, width, height)`` in pixel coordinates.
    """

    centroid_px: tuple[int, int]
    size: int
    bbox: tuple[int, int, int, int]


# ---------------------------------------------------------------------------
# Coordinate transforms
# ---------------------------------------------------------------------------

def grid_to_world(
    px: tuple[int, int],
    origin: Pose2D,
    resolution: float,
) -> tuple[float, float]:
    """Convert pixel coordinates to world-frame metric coordinates.

    Parameters
    ----------
    px : tuple[int, int]
        Pixel coordinates as ``(col, row)``.
    origin : Pose2D
        The origin of the occupancy grid in world frame.
    resolution : float
        Grid resolution in metres per pixel.

    Returns
    -------
    tuple[float, float]
        World coordinates ``(x, y)`` in metres.

    Notes
    -----
    The ROS ``OccupancyGrid`` stores its origin as the real-world pose of
    cell ``(0, 0)``.  Each cell spans ``resolution`` metres, so
    ``world = origin + pixel * resolution``.  We add half a cell so that
    the returned point sits at the **centre** of the pixel rather than its
    corner.
    """
    col, row = px
    world_x: float = origin.x + (col + 0.5) * resolution
    world_y: float = origin.y + (row + 0.5) * resolution
    return (world_x, world_y)


def world_to_grid(
    xy: tuple[float, float],
    origin: Pose2D,
    resolution: float,
) -> tuple[int, int]:
    """Convert world-frame metric coordinates to pixel coordinates.

    Parameters
    ----------
    xy : tuple[float, float]
        World coordinates ``(x, y)`` in metres.
    origin : Pose2D
        The origin of the occupancy grid in world frame.
    resolution : float
        Grid resolution in metres per pixel.

    Returns
    -------
    tuple[int, int]
        Pixel coordinates ``(col, row)``.

    Notes
    -----
    Inverse of :func:`grid_to_world`.  The result is truncated to the
    nearest integer (floor), which is the standard convention for
    continuous-to-discrete mapping in occupancy grids.
    """
    x, y = xy
    col: int = int((x - origin.x) / resolution)
    row: int = int((y - origin.y) / resolution)
    return (col, row)


# ---------------------------------------------------------------------------
# Occupancy-grid image conversion
# ---------------------------------------------------------------------------

def occupancy_grid_to_image(
    grid: NDArray[np.int8],
    width: int,
    height: int,
) -> NDArray[np.uint8]:
    """Convert ROS ``OccupancyGrid.data`` to a three-level grayscale image.

    Parameters
    ----------
    grid : NDArray[np.int8]
        Flat array of occupancy values from the ROS message.  Expected
        values are ``-1`` (unknown), ``0`` (free), and ``100`` (occupied).
    width : int
        Number of columns in the occupancy grid.
    height : int
        Number of rows in the occupancy grid.

    Returns
    -------
    NDArray[np.uint8]
        Grayscale image of shape ``(height, width)`` with pixel values:

        * ``127`` — unknown
        * ``255`` — free
        * ``0``   — occupied

    Notes
    -----
    The three-level quantisation produces clean, high-contrast boundaries
    that are ideal input for the subsequent Canny detector.  Any value
    outside ``{-1, 0, 100}`` is mapped to ``127`` (unknown) as a safe
    default.
    """
    image = np.full(width * height, 127, dtype=np.uint8)
    image[grid == 0] = 255
    image[grid == 100] = 0
    return image.reshape((height, width))


# ---------------------------------------------------------------------------
# Frontier detection
# ---------------------------------------------------------------------------

def detect_frontier_cells(
    image: NDArray[np.uint8],
    canny_low_threshold: int = 100,
    canny_high_threshold: int = 200,
) -> NDArray[np.uint8]:
    """Detect frontier pixels via Canny edge detection on free space.

    Parameters
    ----------
    image : NDArray[np.uint8]
        Three-level grayscale image produced by
        :func:`occupancy_grid_to_image`.
    canny_low_threshold : int
        Lower hysteresis threshold for the Canny detector.  With the
        three-level quantisation (0 / 127 / 255) any pair where
        ``low < 127`` and ``high > 127`` works reliably.
    canny_high_threshold : int
        Upper hysteresis threshold for the Canny detector.

    Returns
    -------
    NDArray[np.uint8]
        Binary image of the same shape where ``255`` marks frontier cells
        and ``0`` marks everything else.

    Notes
    -----
    A *frontier* is a free cell adjacent to unknown space.  The detection
    pipeline proceeds as follows:

    1. **Free-space mask** — threshold at 255 to isolate free cells.
    2. **Canny edges** — extract edges of the free-space region.  Because
       the image contains only three discrete levels, any reasonable
       threshold pair (here 100 / 200) reliably finds the boundary
       between free and non-free cells.
    3. **Dilated unknown mask** — expand unknown cells (value 127) by one
       pixel so that they overlap with edges lying exactly on the
       boundary.
    4. **Intersection** — ``frontier = edges ∩ dilated_unknown``.  This
       removes edges between free and *occupied* space, keeping only
       those adjacent to *unknown* space.
    5. **Morphological closing** — a 3 × 3 closing pass consolidates
       thin, fragmented frontier strips into solid regions that cluster
       more reliably downstream.

    Using Canny on the occupancy grid (treating the map as a grayscale
    image) rather than the classic Wavefront Frontier Detection
    (WFD, Yamauchi 1997) leverages mature, fast image-processing
    primitives and integrates naturally with the camera-based
    ``obstacle_detector`` from Phase 2.
    """
    # 1. Free-space mask
    free_mask = (image == 255).astype(np.uint8) * 255

    # 2. Canny edge detection on free-space mask
    edges = cv2.Canny(free_mask, canny_low_threshold, canny_high_threshold)

    # 3. Dilated unknown-space mask
    unknown_mask = (image == 127).astype(np.uint8) * 255
    kernel_dilate = np.ones((3, 3), np.uint8)
    dilated_unknown = cv2.dilate(unknown_mask, kernel_dilate, iterations=1)

    # 4. Intersection: edges that touch unknown space
    frontier = cv2.bitwise_and(edges, dilated_unknown)

    # 5. Morphological closing to consolidate thin frontier strips
    kernel_close = np.ones((3, 3), np.uint8)
    frontier = cv2.morphologyEx(frontier, cv2.MORPH_CLOSE, kernel_close)

    return frontier


# ---------------------------------------------------------------------------
# Clustering
# ---------------------------------------------------------------------------

def cluster_frontiers(
    frontier_image: NDArray[np.uint8],
    min_cluster_size: int,
) -> list[FrontierCluster]:
    """Group connected frontier pixels into clusters.

    Parameters
    ----------
    frontier_image : NDArray[np.uint8]
        Binary image where ``255`` marks frontier cells (output of
        :func:`detect_frontier_cells`).
    min_cluster_size : int
        Minimum number of pixels for a cluster to be retained.  Clusters
        smaller than this are discarded as noise.

    Returns
    -------
    list[FrontierCluster]
        List of valid frontier clusters, sorted by descending size.

    Notes
    -----
    Uses ``cv2.connectedComponentsWithStats`` with 8-connectivity.  The
    background label (0) is always skipped.  Filtering by
    ``min_cluster_size`` removes sensor noise and single-pixel artefacts
    that would otherwise produce jittery, unreachable goals.
    """
    num_labels, _labels, stats, centroids = cv2.connectedComponentsWithStats(
        frontier_image, connectivity=8
    )

    clusters: list[FrontierCluster] = []
    # Label 0 is background — skip it
    for label in range(1, num_labels):
        area: int = int(stats[label, cv2.CC_STAT_AREA])
        if area < min_cluster_size:
            continue

        cx = int(round(centroids[label][0]))
        cy = int(round(centroids[label][1]))

        bbox = (
            int(stats[label, cv2.CC_STAT_LEFT]),
            int(stats[label, cv2.CC_STAT_TOP]),
            int(stats[label, cv2.CC_STAT_WIDTH]),
            int(stats[label, cv2.CC_STAT_HEIGHT]),
        )

        clusters.append(FrontierCluster(
            centroid_px=(cx, cy),
            size=area,
            bbox=bbox,
        ))

    # Largest clusters first — a useful default for debugging
    clusters.sort(key=lambda c: c.size, reverse=True)
    return clusters


# ---------------------------------------------------------------------------
# Frontier selection (cost–utility)
# ---------------------------------------------------------------------------

def select_best_frontier(
    clusters: list[FrontierCluster],
    robot_position_px: tuple[int, int],
    info_gain_weight: float,
    distance_weight: float,
    blacklist: set[tuple[int, int]],
    min_frontier_distance_px: int = 0,
) -> FrontierCluster | None:
    """Select the frontier maximising a cost–utility score.

    Parameters
    ----------
    clusters : list[FrontierCluster]
        Candidate frontier clusters (output of :func:`cluster_frontiers`).
    robot_position_px : tuple[int, int]
        Current robot position in pixel coordinates ``(col, row)``.
    info_gain_weight : float
        Weight ``α`` for information gain (normalised cluster size).
    distance_weight : float
        Weight ``β`` for distance penalty (normalised Euclidean distance).
    blacklist : set[tuple[int, int]]
        Set of centroid pixel coordinates that have been marked as
        unreachable and should be skipped.
    min_frontier_distance_px : int
        Minimum Euclidean distance in pixels between the robot and a
        candidate centroid.  Clusters closer than this are filtered out
        before utility scoring.  This prevents selecting goals that fall
        inside Nav2's ``xy_goal_tolerance``, which would cause the
        controller to declare "goal reached" without moving the robot.

    Returns
    -------
    FrontierCluster | None
        The highest-scoring cluster, or ``None`` if every candidate is
        blacklisted, too close, or the input list is empty.

    Notes
    -----
    The utility function is:

    .. math::

        U = \\alpha \\cdot \\hat{g} - \\beta \\cdot \\hat{d}

    where :math:`\\hat{g}` is the cluster size normalised to ``[0, 1]``
    by dividing by the maximum size across all candidates, and
    :math:`\\hat{d}` is the Euclidean distance from the robot to the
    centroid, similarly normalised.

    A higher ``info_gain_weight`` biases the robot toward large unexplored
    regions; a higher ``distance_weight`` biases it toward nearby
    frontiers, reducing travel time but potentially leaving distant
    pockets unmapped until late in the run.
    """
    # Filter out blacklisted clusters
    candidates = [c for c in clusters if c.centroid_px not in blacklist]

    # Filter out clusters too close to the robot (inside Nav2 goal tolerance)
    if min_frontier_distance_px > 0:
        candidates = [
            c for c in candidates
            if np.hypot(
                c.centroid_px[0] - robot_position_px[0],
                c.centroid_px[1] - robot_position_px[1],
            ) >= min_frontier_distance_px
        ]

    if not candidates:
        return None

    # Pre-compute raw values
    max_size = max(c.size for c in candidates)
    distances = [
        np.hypot(
            c.centroid_px[0] - robot_position_px[0],
            c.centroid_px[1] - robot_position_px[1],
        )
        for c in candidates
    ]
    max_distance = max(distances) if max(distances) > 0.0 else 1.0

    best_cluster: FrontierCluster | None = None
    best_utility: float = -float("inf")

    for cluster, dist in zip(candidates, distances):
        info_gain_norm = cluster.size / max_size
        distance_norm = dist / max_distance

        utility = info_gain_weight * info_gain_norm - distance_weight * distance_norm

        if utility > best_utility:
            best_utility = utility
            best_cluster = cluster

    return best_cluster
