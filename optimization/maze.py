"""
2D occupancy-grid generators for the swarm coverage testbed.

Convention:
    grid[y, x] = 0  -> free
    grid[y, x] = 1  -> wall
"""

from __future__ import annotations

from collections import deque
from pathlib import Path
from typing import Optional

import numpy as np

FREE = 0
WALL = 1


def _free_cells(grid: np.ndarray) -> list[tuple[int, int]]:
    ys, xs = np.where(grid == FREE)
    return list(zip(xs.tolist(), ys.tolist()))


def _bfs_reachable(grid: np.ndarray, start: tuple[int, int]) -> set[tuple[int, int]]:
    h, w = grid.shape
    visited: set[tuple[int, int]] = {start}
    queue = deque([start])
    while queue:
        x, y = queue.popleft()
        for dx, dy in ((-1, 0), (1, 0), (0, -1), (0, 1)):
            nx, ny = x + dx, y + dy
            if 0 <= nx < w and 0 <= ny < h and grid[ny, nx] == FREE and (nx, ny) not in visited:
                visited.add((nx, ny))
                queue.append((nx, ny))
    return visited


def is_fully_connected(grid: np.ndarray) -> bool:
    """All free cells reachable from any single free cell."""
    free = _free_cells(grid)
    if not free:
        return False
    return len(_bfs_reachable(grid, free[0])) == len(free)


def find_components(grid: np.ndarray) -> list[set[tuple[int, int]]]:
    """
    All connected components of free cells, sorted by size descending.
    The first entry is the main reachable area; subsequent entries are
    isolated pockets that an editor should highlight as invalid.
    """
    remaining = set(_free_cells(grid))
    components: list[set[tuple[int, int]]] = []
    while remaining:
        seed = next(iter(remaining))
        comp = _bfs_reachable(grid, seed)
        components.append(comp)
        remaining -= comp
    components.sort(key=len, reverse=True)
    return components


def disconnected_cells(grid: np.ndarray) -> set[tuple[int, int]]:
    """Free cells that are NOT in the largest connected component."""
    comps = find_components(grid)
    if len(comps) <= 1:
        return set()
    return set().union(*comps[1:])


def boundary_breaches(grid: np.ndarray) -> set[tuple[int, int]]:
    """Free cells that touch the outer border (boundary walls are missing)."""
    h, w = grid.shape
    breaches: set[tuple[int, int]] = set()
    for x in range(w):
        if grid[0, x] == FREE:
            breaches.add((x, 0))
        if grid[h - 1, x] == FREE:
            breaches.add((x, h - 1))
    for y in range(h):
        if grid[y, 0] == FREE:
            breaches.add((0, y))
        if grid[y, w - 1] == FREE:
            breaches.add((w - 1, y))
    return breaches


def save_map(grid: np.ndarray, path: str | Path) -> None:
    """Write to .npy (binary) or .txt (0/1 chars, one row per line). Creates parent dirs."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    if path.suffix == ".txt":
        with path.open("w") as f:
            for row in grid:
                f.write("".join(str(int(v)) for v in row))
                f.write("\n")
    else:
        np.save(path, grid.astype(np.int8))


def load_map(path: str | Path) -> np.ndarray:
    path = Path(path)
    if path.suffix == ".txt":
        rows = [list(line.strip()) for line in path.read_text().splitlines() if line.strip()]
        return np.array(rows, dtype=np.int8)
    return np.load(path).astype(np.int8)


def open_arena(size: int) -> np.ndarray:
    """Just boundary walls; everything else free."""
    grid = np.zeros((size, size), dtype=np.int8)
    grid[0, :] = WALL
    grid[-1, :] = WALL
    grid[:, 0] = WALL
    grid[:, -1] = WALL
    return grid


def random_obstacles(
    size: int,
    density: float = 0.25,    # fraction of (size-2)² interior cells made into walls
    seed: Optional[int] = None,
    max_attempts: int = 20,   # BFS retries before falling back to an open arena
) -> np.ndarray:
    """
    Boundary walls plus randomly scattered interior wall cells.
    Retries until the free space is fully connected, up to max_attempts.
    Falls back to an open arena if no valid layout is found.
    """
    rng = np.random.default_rng(seed)
    # The (size-2)² gap excludes the always-walled outer border.
    # n_walls = density × interior_cell_count, so density=0.25 fills 25% of the inside.
    interior = (size - 2) ** 2
    n_walls = int(round(density * interior))

    for _ in range(max_attempts):
        grid = open_arena(size)
        idx = rng.choice(interior, size=n_walls, replace=False)
        ys = 1 + idx // (size - 2)
        xs = 1 + idx % (size - 2)
        grid[ys, xs] = WALL
        if is_fully_connected(grid):
            return grid

    return open_arena(size)


def recursive_backtracker(size: int, seed: Optional[int] = None) -> np.ndarray:
    """
    Classic perfect-maze generator on a grid where corridors are 1 cell wide.
    `size` should be odd; if even, it is decreased by 1 internally.

    Algorithm: starting from cell (1, 1), pick a random unvisited neighbor two
    cells away, knock down the wall between, recurse. The "two cells away" step
    is what makes the corridor pattern: cells at odd indices are corridor, cells
    at even indices are walls (or knocked-down passages). That's why size must
    be odd — to make both border rows and columns walls.
    """
    if size % 2 == 0:
        size -= 1
    rng = np.random.default_rng(seed)
    grid = np.ones((size, size), dtype=np.int8)

    def carve(x: int, y: int) -> None:
        grid[y, x] = FREE
        # Steps of 2 in each cardinal direction — see docstring.
        dirs = [(2, 0), (-2, 0), (0, 2), (0, -2)]
        rng.shuffle(dirs)
        for dx, dy in dirs:
            nx, ny = x + dx, y + dy
            if 0 < nx < size - 1 and 0 < ny < size - 1 and grid[ny, nx] == WALL:
                # Knock down the wall midway between (x, y) and (nx, ny).
                grid[y + dy // 2, x + dx // 2] = FREE
                carve(nx, ny)

    carve(1, 1)
    return grid


def random_free_positions(
    grid: np.ndarray,
    n: int,
    seed: Optional[int] = None,
    min_separation: float = 1.5,   # in cell units; 1.5 ≈ "more than one cell apart"
) -> np.ndarray:
    """
    Sample `n` distinct cell-center positions from free cells, in (x, y) order.
    Uses min_separation (in cell units) to avoid spawning drones on top of each
    other or sharing the same coverage footprint at t=0. The default 1.5 keeps
    drones from starting inside the same DroneConfig.sensor_range=1.6 wedge.
    """
    rng = np.random.default_rng(seed)
    free = _free_cells(grid)
    rng.shuffle(free)

    chosen: list[tuple[float, float]] = []
    for x, y in free:
        cx, cy = x + 0.5, y + 0.5
        if all((cx - px) ** 2 + (cy - py) ** 2 >= min_separation ** 2 for px, py in chosen):
            chosen.append((cx, cy))
            if len(chosen) == n:
                break

    if len(chosen) < n:
        raise RuntimeError(f"Could not place {n} drones with separation {min_separation}")
    return np.array(chosen, dtype=np.float64)
