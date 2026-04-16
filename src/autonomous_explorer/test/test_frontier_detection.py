"""test_frontier_detection.py — Unit tests for the pure-Python frontier module.

Run with:
    python3 -m pytest test/test_frontier_detection.py -v

No ROS runtime required.
"""

from __future__ import annotations

import numpy as np
import pytest

from autonomous_explorer.frontier_detection import (
    FrontierCluster,
    Pose2D,
    cluster_frontiers,
    detect_frontier_cells,
    grid_to_world,
    occupancy_grid_to_image,
    select_best_frontier,
    world_to_grid,
)


# -----------------------------------------------------------------------
# Helpers
# -----------------------------------------------------------------------

def _make_grid(*rows: list[int]) -> tuple[np.ndarray, int, int]:
    """Build a flat int8 grid from row lists.

    Each value should be -1 (unknown), 0 (free), or 100 (occupied).
    Returns ``(flat_array, width, height)``.
    """
    arr = np.array(rows, dtype=np.int8)
    height, width = arr.shape
    return arr.flatten(), width, height


# -----------------------------------------------------------------------
# occupancy_grid_to_image
# -----------------------------------------------------------------------

class TestOccupancyGridToImage:
    """Test the three-level mapping on a synthetic 5×5 grid."""

    def test_three_level_mapping(self) -> None:
        grid, w, h = _make_grid(
            [-1,   0, 100,   0, -1],
            [ 0,   0,   0,   0,  0],
            [100, 100,   0, -1, -1],
            [-1,   0, 100,   0, -1],
            [-1, -1,  -1,  -1, -1],
        )
        image = occupancy_grid_to_image(grid, w, h)

        assert image.shape == (5, 5)
        # unknown → 127
        assert image[0, 0] == 127
        # free → 255
        assert image[0, 1] == 255
        # occupied → 0
        assert image[0, 2] == 0

    def test_all_unknown(self) -> None:
        grid = np.full(25, -1, dtype=np.int8)
        image = occupancy_grid_to_image(grid, 5, 5)
        assert np.all(image == 127)

    def test_all_free(self) -> None:
        grid = np.zeros(25, dtype=np.int8)
        image = occupancy_grid_to_image(grid, 5, 5)
        assert np.all(image == 255)


# -----------------------------------------------------------------------
# detect_frontier_cells
# -----------------------------------------------------------------------

class TestDetectFrontierCells:
    """Frontier detection on hand-crafted grids."""

    def test_simple_case(self) -> None:
        """A free region bordered by unknown space must produce frontiers.

        Grid layout (10×10):
          - Top-left 5×5 quadrant is FREE (0).
          - Everything else is UNKNOWN (-1).

        The boundary between free and unknown should be detected.
        """
        grid_2d = np.full((10, 10), -1, dtype=np.int8)
        grid_2d[:5, :5] = 0  # free quadrant

        image = occupancy_grid_to_image(grid_2d.flatten(), 10, 10)
        frontier = detect_frontier_cells(image)

        assert frontier.shape == (10, 10)
        # Frontier pixels must exist along the boundary of the free quadrant
        assert np.any(frontier > 0), "Expected frontier cells at free/unknown boundary"
        # No frontier should appear deep inside the free region
        assert frontier[1, 1] == 0, "Interior free cell should not be a frontier"

    def test_no_frontier_fully_known(self) -> None:
        """A fully known (free + occupied) map has no unknown border → no frontiers."""
        grid_2d = np.zeros((10, 10), dtype=np.int8)
        grid_2d[0, :] = 100  # top wall occupied
        grid_2d[9, :] = 100
        grid_2d[:, 0] = 100
        grid_2d[:, 9] = 100

        image = occupancy_grid_to_image(grid_2d.flatten(), 10, 10)
        frontier = detect_frontier_cells(image)

        assert np.sum(frontier) == 0, "Fully known map should have no frontiers"

    def test_no_frontier_all_unknown(self) -> None:
        """An entirely unknown grid has no free edges → no frontiers."""
        grid = np.full(100, -1, dtype=np.int8)
        image = occupancy_grid_to_image(grid, 10, 10)
        frontier = detect_frontier_cells(image)

        assert np.sum(frontier) == 0

    def test_custom_canny_thresholds(self) -> None:
        """Verify that custom Canny thresholds are accepted without error."""
        grid_2d = np.full((10, 10), -1, dtype=np.int8)
        grid_2d[:5, :5] = 0

        image = occupancy_grid_to_image(grid_2d.flatten(), 10, 10)
        frontier = detect_frontier_cells(
            image, canny_low_threshold=50, canny_high_threshold=150
        )
        # Should still detect frontiers with different thresholds
        assert np.any(frontier > 0)


# -----------------------------------------------------------------------
# cluster_frontiers
# -----------------------------------------------------------------------

class TestClusterFrontiers:
    """Clustering and noise filtering."""

    def test_filters_small_clusters(self) -> None:
        """Clusters below min_cluster_size must be rejected."""
        # Create an image with one large blob and one tiny blob
        img = np.zeros((50, 50), dtype=np.uint8)
        # Large region (well above threshold)
        img[10:20, 10:20] = 255   # 100 pixels
        # Tiny region (below threshold)
        img[40, 40] = 255          # 1 pixel

        clusters = cluster_frontiers(img, min_cluster_size=5)

        # Only the large cluster should survive
        assert len(clusters) == 1
        assert clusters[0].size >= 5

    def test_multiple_clusters(self) -> None:
        """Two separated blobs should produce two clusters."""
        img = np.zeros((50, 50), dtype=np.uint8)
        img[5:10, 5:10] = 255    # 25 px
        img[30:35, 30:35] = 255  # 25 px

        clusters = cluster_frontiers(img, min_cluster_size=1)
        assert len(clusters) == 2

    def test_empty_image(self) -> None:
        """No frontier pixels → empty list."""
        img = np.zeros((20, 20), dtype=np.uint8)
        clusters = cluster_frontiers(img, min_cluster_size=1)
        assert clusters == []


# -----------------------------------------------------------------------
# select_best_frontier
# -----------------------------------------------------------------------

class TestSelectBestFrontier:
    """Cost–utility ranking and blacklist filtering."""

    @pytest.fixture()
    def two_clusters(self) -> list[FrontierCluster]:
        """Two clusters: one close-small, one far-large."""
        close_small = FrontierCluster(
            centroid_px=(10, 10), size=20, bbox=(5, 5, 10, 10)
        )
        far_large = FrontierCluster(
            centroid_px=(90, 90), size=200, bbox=(80, 80, 20, 20)
        )
        return [close_small, far_large]

    def test_utility_tradeoff_prefers_large(
        self, two_clusters: list[FrontierCluster]
    ) -> None:
        """With high α and low β, the large far cluster wins."""
        result = select_best_frontier(
            clusters=two_clusters,
            robot_position_px=(5, 5),
            info_gain_weight=2.0,
            distance_weight=0.1,
            blacklist=set(),
        )
        assert result is not None
        assert result.size == 200, "High info_gain_weight should pick the large cluster"

    def test_utility_tradeoff_prefers_close(
        self, two_clusters: list[FrontierCluster]
    ) -> None:
        """With low α and high β, the close small cluster wins."""
        result = select_best_frontier(
            clusters=two_clusters,
            robot_position_px=(5, 5),
            info_gain_weight=0.1,
            distance_weight=2.0,
            blacklist=set(),
        )
        assert result is not None
        assert result.size == 20, "High distance_weight should pick the close cluster"

    def test_respects_blacklist(
        self, two_clusters: list[FrontierCluster]
    ) -> None:
        """A blacklisted cluster is never selected, even if it has the best utility."""
        # Blacklist the far-large cluster
        blacklist = {(90, 90)}
        result = select_best_frontier(
            clusters=two_clusters,
            robot_position_px=(5, 5),
            info_gain_weight=2.0,
            distance_weight=0.1,
            blacklist=blacklist,
        )
        assert result is not None
        assert result.centroid_px == (10, 10), "Blacklisted cluster must be skipped"

    def test_all_blacklisted_returns_none(
        self, two_clusters: list[FrontierCluster]
    ) -> None:
        """If every cluster is blacklisted, return None."""
        blacklist = {(10, 10), (90, 90)}
        result = select_best_frontier(
            clusters=two_clusters,
            robot_position_px=(5, 5),
            info_gain_weight=1.0,
            distance_weight=0.5,
            blacklist=blacklist,
        )
        assert result is None

    def test_empty_clusters_returns_none(self) -> None:
        """Empty input → None."""
        result = select_best_frontier(
            clusters=[],
            robot_position_px=(0, 0),
            info_gain_weight=1.0,
            distance_weight=0.5,
            blacklist=set(),
        )
        assert result is None

    def test_filters_too_close(self) -> None:
        """Frontiers inside min_frontier_distance_px are excluded."""
        close = FrontierCluster(
            centroid_px=(15, 15), size=50, bbox=(10, 10, 10, 10)
        )
        far = FrontierCluster(
            centroid_px=(60, 60), size=50, bbox=(50, 50, 20, 20)
        )
        result = select_best_frontier(
            clusters=[close, far],
            robot_position_px=(10, 10),
            info_gain_weight=1.0,
            distance_weight=0.5,
            blacklist=set(),
            min_frontier_distance_px=20,
        )
        assert result is not None
        assert result.centroid_px == (60, 60), (
            "Close cluster should be filtered by min_frontier_distance_px"
        )

    def test_returns_none_when_all_too_close(self) -> None:
        """If every cluster is too close, return None (normal termination)."""
        close = FrontierCluster(
            centroid_px=(12, 12), size=50, bbox=(10, 10, 5, 5)
        )
        result = select_best_frontier(
            clusters=[close],
            robot_position_px=(10, 10),
            info_gain_weight=1.0,
            distance_weight=0.5,
            blacklist=set(),
            min_frontier_distance_px=20,
        )
        assert result is None


# -----------------------------------------------------------------------
# Coordinate transforms
# -----------------------------------------------------------------------

class TestCoordinateTransforms:
    """Round-trip grid ↔ world conversions."""

    def test_grid_to_world(self) -> None:
        origin = Pose2D(x=-5.0, y=-5.0)
        resolution = 0.05
        # Pixel (100, 100) → world (-5 + 100.5*0.05, -5 + 100.5*0.05)
        wx, wy = grid_to_world((100, 100), origin, resolution)
        assert abs(wx - 0.025) < 1e-6
        assert abs(wy - 0.025) < 1e-6

    def test_world_to_grid(self) -> None:
        origin = Pose2D(x=-5.0, y=-5.0)
        resolution = 0.05
        col, row = world_to_grid((0.0, 0.0), origin, resolution)
        assert col == 100
        assert row == 100

    def test_round_trip(self) -> None:
        origin = Pose2D(x=-10.0, y=-10.0)
        resolution = 0.05
        original_px = (50, 75)
        world = grid_to_world(original_px, origin, resolution)
        recovered = world_to_grid(world, origin, resolution)
        assert recovered == original_px
