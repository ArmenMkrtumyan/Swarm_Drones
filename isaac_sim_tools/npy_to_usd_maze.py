#!/usr/bin/env python3
"""
Convert a 2D NumPy maze map into a 3D Isaac Sim USD/USD-A building world.

Input:
    lab/swarm/2d_to_3d_maps/maps/building_maze.npy

Output:
    lab/swarm/2d_to_3d_maps/worlds/building_maze.usda

Default convention:
    grid value 0       -> free floor
    grid value nonzero -> wall

Why:
    This keeps your 2D coverage map and Isaac 3D environment identical.
    One 2D cell becomes one 5m x 5m Isaac tile by default.

Run with Isaac Sim Python, not your normal venv Python.

Windows example:
    & "C:\\Users\\user1811\\Desktop\\armen-capstone\\isaac-sim\\python.bat" `
        .\\isaac_sim_tools\\npy_to_usd_maze.py `
        --map .\\lab\\swarm\\2d_to_3d_maps\\maps\\building_maze.npy `
        --out .\\lab\\swarm\\2d_to_3d_maps\\worlds\\building_maze.usda `
        --spawn-count 4

Linux/WSL Isaac example:
    ./python.sh isaac_sim_tools/npy_to_usd_maze.py \
        --map lab/swarm/2d_to_3d_maps/maps/building_maze.npy \
        --out lab/swarm/2d_to_3d_maps/worlds/building_maze.usda \
        --spawn-count 4
"""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Iterable, List, Tuple

import numpy as np

from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux, UsdPhysics, UsdShade


Rect = Tuple[int, int, int, int]  # row0, col0, height_cells, width_cells


# ---------------------------------------------------------------------
# Basic USD helpers
# ---------------------------------------------------------------------


def ensure_parent_dir(path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)


def make_material(
    stage: Usd.Stage,
    path: str,
    color_rgb: Tuple[float, float, float],
    opacity: float = 1.0,
    roughness: float = 0.75,
    metallic: float = 0.0,
) -> UsdShade.Material:
    """
    Create a simple USD Preview Surface material.
    """
    material = UsdShade.Material.Define(stage, path)

    shader = UsdShade.Shader.Define(stage, f"{path}/PreviewSurface")
    shader.CreateIdAttr("UsdPreviewSurface")

    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(
        Gf.Vec3f(*color_rgb)
    )
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(roughness)
    shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(metallic)

    if opacity < 1.0:
        shader.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(opacity)

    material.CreateSurfaceOutput().ConnectToSource(
        shader.ConnectableAPI(), "surface"
    )
    return material


def bind_material(prim: Usd.Prim, material: UsdShade.Material) -> None:
    UsdShade.MaterialBindingAPI.Apply(prim).Bind(material)


def add_collision(prim: Usd.Prim) -> None:
    """
    Static collider. Do NOT add RigidBodyAPI for walls/floor/ceiling,
    because these should stay fixed.
    """
    UsdPhysics.CollisionAPI.Apply(prim)


def add_box(
    stage: Usd.Stage,
    path: str,
    center_xyz: Tuple[float, float, float],
    size_xyz: Tuple[float, float, float],
    material: UsdShade.Material | None = None,
    collision: bool = True,
) -> Usd.Prim:
    """
    Add a cube scaled to exact dimensions.

    USD cube with size=1 spans [-0.5, 0.5], so scaling by size_xyz gives
    a box with dimensions size_xyz.
    """
    cube = UsdGeom.Cube.Define(stage, path)
    cube.CreateSizeAttr(1.0)

    cube.AddTranslateOp().Set(Gf.Vec3f(*center_xyz))
    cube.AddScaleOp().Set(Gf.Vec3f(*size_xyz))

    prim = cube.GetPrim()

    if material is not None:
        bind_material(prim, material)

    if collision:
        add_collision(prim)

    return prim


def add_cylinder_marker(
    stage: Usd.Stage,
    path: str,
    center_xyz: Tuple[float, float, float],
    radius: float,
    height: float,
    material: UsdShade.Material | None = None,
) -> Usd.Prim:
    """
    Small visible spawn marker. No collision.
    """
    cylinder = UsdGeom.Cylinder.Define(stage, path)
    cylinder.CreateRadiusAttr(radius)
    cylinder.CreateHeightAttr(height)
    cylinder.AddTranslateOp().Set(Gf.Vec3f(*center_xyz))

    prim = cylinder.GetPrim()

    if material is not None:
        bind_material(prim, material)

    return prim


# ---------------------------------------------------------------------
# Grid / geometry helpers
# ---------------------------------------------------------------------


def load_grid(path: Path) -> np.ndarray:
    """
    Load .npy or text grid.
    """
    if not path.exists():
        raise FileNotFoundError(f"Map file does not exist: {path}")

    if path.suffix.lower() == ".npy":
        grid = np.load(path, allow_pickle=False)
    elif path.suffix.lower() in {".txt", ".csv"}:
        grid = np.loadtxt(path, delimiter="," if path.suffix.lower() == ".csv" else None)
    else:
        raise ValueError(
            f"Unsupported map extension: {path.suffix}. Use .npy, .txt, or .csv."
        )

    if grid.ndim != 2:
        raise ValueError(f"Expected 2D grid, got shape {grid.shape}")

    return grid


def make_wall_mask(
    grid: np.ndarray,
    wall_value: int | float | None,
    invert: bool,
) -> np.ndarray:
    """
    Convert arbitrary numeric grid into boolean wall mask.
    """
    if wall_value is None:
        wall_mask = grid != 0
    else:
        wall_mask = grid == wall_value

    if invert:
        wall_mask = ~wall_mask

    return wall_mask.astype(bool)


def grid_rect_to_world_box(
    row0: int,
    col0: int,
    height_cells: int,
    width_cells: int,
    rows: int,
    cols: int,
    cell_size: float,
    wall_height: float,
) -> Tuple[Tuple[float, float, float], Tuple[float, float, float]]:
    """
    Convert a rectangle of occupied grid cells into one Isaac box.

    Grid:
        row increases downward
        col increases rightward

    Isaac:
        x increases rightward
        y increases upward/opposite of image row
        z is height

    Maze is centered around world origin.
    """
    center_col = col0 + width_cells / 2.0
    center_row = row0 + height_cells / 2.0

    x = (center_col - cols / 2.0) * cell_size
    y = -(center_row - rows / 2.0) * cell_size
    z = wall_height / 2.0

    size_x = width_cells * cell_size
    size_y = height_cells * cell_size
    size_z = wall_height

    return (x, y, z), (size_x, size_y, size_z)


def grid_cell_to_world_xy(
    row: int,
    col: int,
    rows: int,
    cols: int,
    cell_size: float,
) -> Tuple[float, float]:
    """
    Convert one grid cell center to Isaac x/y.
    """
    x = (col + 0.5 - cols / 2.0) * cell_size
    y = -(row + 0.5 - rows / 2.0) * cell_size
    return x, y


def greedy_merge_wall_rectangles(wall_mask: np.ndarray) -> List[Rect]:
    """
    Merge adjacent wall cells into larger rectangles.

    This reduces USD prim count:
        instead of one cube per wall cell,
        long wall runs become one larger box.

    Simple greedy algorithm:
        1. Find first unvisited wall cell.
        2. Expand width along the row.
        3. Expand height while the full width remains wall and unvisited.
        4. Mark rectangle visited.
    """
    rows, cols = wall_mask.shape
    visited = np.zeros_like(wall_mask, dtype=bool)
    rects: List[Rect] = []

    for r in range(rows):
        for c in range(cols):
            if not wall_mask[r, c] or visited[r, c]:
                continue

            # Expand width.
            width = 1
            while (
                c + width < cols
                and wall_mask[r, c + width]
                and not visited[r, c + width]
            ):
                width += 1

            # Expand height while the whole row segment is available.
            height = 1
            while r + height < rows:
                segment = wall_mask[r + height, c : c + width]
                segment_visited = visited[r + height, c : c + width]

                if np.all(segment) and not np.any(segment_visited):
                    height += 1
                else:
                    break

            visited[r : r + height, c : c + width] = True
            rects.append((r, c, height, width))

    return rects


def unmerged_wall_rectangles(wall_mask: np.ndarray) -> List[Rect]:
    """
    One rectangle per wall cell. Useful for debugging exact grid geometry.
    """
    rows, cols = wall_mask.shape
    rects: List[Rect] = []

    for r in range(rows):
        for c in range(cols):
            if wall_mask[r, c]:
                rects.append((r, c, 1, 1))

    return rects


def choose_spawn_cells(free_mask: np.ndarray, count: int) -> List[Tuple[int, int]]:
    """
    Choose reasonably spread-out free cells for spawn markers.

    This is only for visual markers / convenience.
    You can still spawn your actual drone wherever you want later.
    """
    if count <= 0:
        return []

    free_cells = np.argwhere(free_mask)
    if len(free_cells) == 0:
        return []

    # First spawn: free cell closest to top-left interior.
    first_idx = np.argmin(free_cells[:, 0] + free_cells[:, 1])
    selected = [tuple(map(int, free_cells[first_idx]))]

    # Next spawns: farthest from already selected cells.
    while len(selected) < count and len(selected) < len(free_cells):
        best_cell = None
        best_dist = -1.0

        for cell in free_cells:
            cell_tuple = tuple(map(int, cell))
            if cell_tuple in selected:
                continue

            min_dist_to_selected = min(
                float((cell_tuple[0] - s[0]) ** 2 + (cell_tuple[1] - s[1]) ** 2)
                for s in selected
            )

            if min_dist_to_selected > best_dist:
                best_dist = min_dist_to_selected
                best_cell = cell_tuple

        if best_cell is None:
            break

        selected.append(best_cell)

    return selected


# ---------------------------------------------------------------------
# Scene construction
# ---------------------------------------------------------------------


def add_lights(
    stage: Usd.Stage,
    rows: int,
    cols: int,
    cell_size: float,
    ceiling_bottom_height: float,
    spacing_cells: int = 6,
    intensity: float = 450.0,
) -> None:
    """
    Add ceiling lights inside the building.

    Rect lights are placed slightly below the ceiling so the interior is visible.
    """
    UsdGeom.Xform.Define(stage, "/World/Lights")

    dome = UsdLux.DomeLight.Define(stage, "/World/Lights/DomeLight")
    dome.CreateIntensityAttr(80.0)

    spacing_cells = max(1, spacing_cells)
    light_z = max(0.5, ceiling_bottom_height - 0.15)

    light_index = 0
    for r in range(spacing_cells // 2, rows, spacing_cells):
        for c in range(spacing_cells // 2, cols, spacing_cells):
            x, y = grid_cell_to_world_xy(r, c, rows, cols, cell_size)

            light = UsdLux.RectLight.Define(
                stage, f"/World/Lights/CeilingLight_{light_index:03d}"
            )
            light.AddTranslateOp().Set(Gf.Vec3f(x, y, light_z))
            light.CreateWidthAttr(cell_size * 0.7)
            light.CreateHeightAttr(cell_size * 0.7)
            light.CreateIntensityAttr(intensity)
            light.CreateColorAttr(Gf.Vec3f(1.0, 0.96, 0.88))

            # Point the light downward.
            light.AddRotateXYZOp().Set(Gf.Vec3f(180.0, 0.0, 0.0))

            light_index += 1


def add_overview_camera(
    stage: Usd.Stage,
    rows: int,
    cols: int,
    cell_size: float,
    ceiling_bottom_height: float,
) -> None:
    """
    Add a debug camera above the world.
    Helpful if you open the USD directly.
    """
    camera = UsdGeom.Camera.Define(stage, "/World/DebugCamera")

    world_size = max(rows, cols) * cell_size
    camera_height = max(ceiling_bottom_height + 20.0, world_size * 0.9)

    camera.AddTranslateOp().Set(Gf.Vec3f(0.0, -world_size * 0.55, camera_height))
    camera.AddRotateXYZOp().Set(Gf.Vec3f(60.0, 0.0, 0.0))
    camera.CreateFocalLengthAttr(24.0)


def build_maze_usd(
    map_path: Path,
    output_path: Path,
    cell_size: float,
    wall_height: float,
    wall_value: int | float | None,
    invert: bool,
    merge_walls: bool,
    floor_thickness: float,
    ceiling: bool,
    ceiling_bottom_height: float,
    ceiling_thickness: float,
    ceiling_opacity: float,
    ceiling_collision: bool,
    spawn_count: int,
    spawn_marker_radius: float,
    spawn_marker_height: float,
    spawn_altitude: float,
    add_debug_camera_flag: bool,
    add_lights_flag: bool,
    light_spacing_cells: int,
) -> None:
    grid = load_grid(map_path)
    wall_mask = make_wall_mask(grid, wall_value=wall_value, invert=invert)
    free_mask = ~wall_mask

    rows, cols = wall_mask.shape

    if rows < 2 or cols < 2:
        raise ValueError(f"Map too small: {rows}x{cols}")

    ensure_parent_dir(output_path)

    stage = Usd.Stage.CreateNew(str(output_path))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)

    world = UsdGeom.Xform.Define(stage, "/World")
    stage.SetDefaultPrim(world.GetPrim())

    # Useful metadata on the Maze root.
    maze_xform = UsdGeom.Xform.Define(stage, "/World/Maze")
    maze_prim = maze_xform.GetPrim()
    maze_prim.CreateAttribute("source_map", Sdf.ValueTypeNames.String).Set(str(map_path))
    maze_prim.CreateAttribute("rows", Sdf.ValueTypeNames.Int).Set(int(rows))
    maze_prim.CreateAttribute("cols", Sdf.ValueTypeNames.Int).Set(int(cols))
    maze_prim.CreateAttribute("cell_size_m", Sdf.ValueTypeNames.Double).Set(float(cell_size))

    # Physics scene.
    physics_scene = UsdPhysics.Scene.Define(stage, "/World/physicsScene")
    physics_scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
    physics_scene.CreateGravityMagnitudeAttr().Set(9.81)

    # Groups.
    UsdGeom.Xform.Define(stage, "/World/Maze/Floor")
    UsdGeom.Xform.Define(stage, "/World/Maze/Walls")
    UsdGeom.Xform.Define(stage, "/World/Maze/Ceiling")
    UsdGeom.Xform.Define(stage, "/World/Maze/SpawnMarkers")

    # Materials.
    floor_mat = make_material(
        stage,
        "/World/Looks/FloorConcrete",
        color_rgb=(0.36, 0.36, 0.34),
        roughness=0.9,
    )
    wall_mat = make_material(
        stage,
        "/World/Looks/WallConcrete",
        color_rgb=(0.68, 0.68, 0.64),
        roughness=0.85,
    )
    ceiling_mat = make_material(
        stage,
        "/World/Looks/Ceiling",
        color_rgb=(0.82, 0.82, 0.80),
        opacity=ceiling_opacity,
        roughness=0.9,
    )
    spawn_mat = make_material(
        stage,
        "/World/Looks/SpawnMarker",
        color_rgb=(0.1, 0.8, 0.2),
        opacity=0.85,
        roughness=0.5,
    )

    # Floor: one large slab.
    floor_size_x = cols * cell_size
    floor_size_y = rows * cell_size

    add_box(
        stage,
        "/World/Maze/Floor/FloorSlab",
        center_xyz=(0.0, 0.0, -floor_thickness / 2.0),
        size_xyz=(floor_size_x, floor_size_y, floor_thickness),
        material=floor_mat,
        collision=True,
    )

    # Walls.
    if merge_walls:
        wall_rects = greedy_merge_wall_rectangles(wall_mask)
    else:
        wall_rects = unmerged_wall_rectangles(wall_mask)

    for i, (r0, c0, h, w) in enumerate(wall_rects):
        center, size = grid_rect_to_world_box(
            row0=r0,
            col0=c0,
            height_cells=h,
            width_cells=w,
            rows=rows,
            cols=cols,
            cell_size=cell_size,
            wall_height=wall_height,
        )

        add_box(
            stage,
            f"/World/Maze/Walls/WallBlock_{i:04d}_r{r0}_c{c0}_h{h}_w{w}",
            center_xyz=center,
            size_xyz=size,
            material=wall_mat,
            collision=True,
        )

    # Ceiling: one large slab.
    if ceiling:
        add_box(
            stage,
            "/World/Maze/Ceiling/CeilingSlab",
            center_xyz=(0.0, 0.0, ceiling_bottom_height + ceiling_thickness / 2.0),
            size_xyz=(floor_size_x, floor_size_y, ceiling_thickness),
            material=ceiling_mat,
            collision=ceiling_collision,
        )

    # Spawn markers.
    spawn_cells = choose_spawn_cells(free_mask, spawn_count)

    for i, (r, c) in enumerate(spawn_cells):
        x, y = grid_cell_to_world_xy(r, c, rows, cols, cell_size)

        marker = add_cylinder_marker(
            stage,
            f"/World/Maze/SpawnMarkers/Spawn_{i}_r{r}_c{c}",
            center_xyz=(x, y, spawn_marker_height / 2.0),
            radius=spawn_marker_radius,
            height=spawn_marker_height,
            material=spawn_mat,
        )

        marker.CreateAttribute("spawn_index", Sdf.ValueTypeNames.Int).Set(i)
        marker.CreateAttribute("grid_row", Sdf.ValueTypeNames.Int).Set(int(r))
        marker.CreateAttribute("grid_col", Sdf.ValueTypeNames.Int).Set(int(c))
        marker.CreateAttribute("spawn_x_m", Sdf.ValueTypeNames.Double).Set(float(x))
        marker.CreateAttribute("spawn_y_m", Sdf.ValueTypeNames.Double).Set(float(y))
        marker.CreateAttribute("spawn_z_m", Sdf.ValueTypeNames.Double).Set(
            float(spawn_altitude)
        )

    if add_lights_flag:
        add_lights(
            stage=stage,
            rows=rows,
            cols=cols,
            cell_size=cell_size,
            ceiling_bottom_height=ceiling_bottom_height,
            spacing_cells=light_spacing_cells,
        )

    if add_debug_camera_flag:
        add_overview_camera(
            stage=stage,
            rows=rows,
            cols=cols,
            cell_size=cell_size,
            ceiling_bottom_height=ceiling_bottom_height,
        )

    stage.GetRootLayer().Save()

    wall_cells = int(np.count_nonzero(wall_mask))
    free_cells = int(np.count_nonzero(free_mask))

    print()
    print("USD maze generated successfully.")
    print(f"  Input map:        {map_path}")
    print(f"  Output USD:       {output_path}")
    print(f"  Grid:             {rows} rows x {cols} cols")
    print(f"  Cell size:        {cell_size:.2f} m")
    print(f"  World size:       {floor_size_x:.2f} m x {floor_size_y:.2f} m")
    print(f"  Wall cells:       {wall_cells}")
    print(f"  Free cells:       {free_cells}")
    print(f"  Wall blocks:      {len(wall_rects)}")
    print(f"  Wall height:      {wall_height:.2f} m")
    print(f"  Ceiling:          {'yes' if ceiling else 'no'}")
    if ceiling:
        print(f"  Ceiling bottom:   {ceiling_bottom_height:.2f} m")
        print(f"  Ceiling collision:{'yes' if ceiling_collision else 'no'}")
    print(f"  Spawn markers:    {len(spawn_cells)}")
    for i, (r, c) in enumerate(spawn_cells):
        x, y = grid_cell_to_world_xy(r, c, rows, cols, cell_size)
        print(f"    Spawn {i}: grid=({r}, {c}), xyz=({x:.2f}, {y:.2f}, {spawn_altitude:.2f})")
    print()


# ---------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Convert a 2D .npy/.txt/.csv maze grid into an Isaac Sim USD world."
    )

    parser.add_argument(
        "--map",
        required=True,
        type=Path,
        help="Input map path: .npy, .txt, or .csv",
    )
    parser.add_argument(
        "--out",
        required=True,
        type=Path,
        help="Output USD/USD-A path, for example lab/swarm/2d_to_3d_maps/worlds/building_maze.usda",
    )

    parser.add_argument(
        "--cell-size",
        type=float,
        default=5.0,
        help="Meters per grid cell. Keep 5.0 to match your 2D simulator.",
    )
    parser.add_argument(
        "--wall-height",
        type=float,
        default=4.0,
        help="Wall height in meters.",
    )
    parser.add_argument(
        "--wall-value",
        type=float,
        default=None,
        help=(
            "Specific grid value treated as wall. "
            "Default: any non-zero value is wall."
        ),
    )
    parser.add_argument(
        "--invert",
        action="store_true",
        help="Invert wall/free interpretation.",
    )

    parser.add_argument(
        "--no-merge-walls",
        action="store_true",
        help="Disable wall merging and create one cube per wall cell.",
    )

    parser.add_argument(
        "--floor-thickness",
        type=float,
        default=0.20,
        help="Floor slab thickness in meters.",
    )

    parser.add_argument(
        "--no-ceiling",
        action="store_true",
        help="Do not create ceiling slab. Useful for debugging top-down view.",
    )
    parser.add_argument(
        "--ceiling-bottom-height",
        type=float,
        default=4.0,
        help="Height of the bottom face of the ceiling slab.",
    )
    parser.add_argument(
        "--ceiling-thickness",
        type=float,
        default=0.20,
        help="Ceiling slab thickness in meters.",
    )
    parser.add_argument(
        "--ceiling-opacity",
        type=float,
        default=0.35,
        help="Ceiling material opacity. Use 1.0 for opaque final scene.",
    )
    parser.add_argument(
        "--no-ceiling-collision",
        action="store_true",
        help="Make ceiling visual-only, without collision.",
    )

    parser.add_argument(
        "--spawn-count",
        type=int,
        default=4,
        help="Number of visible spawn markers to create.",
    )
    parser.add_argument(
        "--spawn-marker-radius",
        type=float,
        default=0.35,
        help="Spawn marker radius in meters.",
    )
    parser.add_argument(
        "--spawn-marker-height",
        type=float,
        default=0.05,
        help="Spawn marker disk height in meters.",
    )
    parser.add_argument(
        "--spawn-altitude",
        type=float,
        default=1.5,
        help="Suggested drone spawn altitude stored as metadata.",
    )

    parser.add_argument(
        "--no-lights",
        action="store_true",
        help="Do not add interior lights.",
    )
    parser.add_argument(
        "--light-spacing-cells",
        type=int,
        default=6,
        help="Spacing between ceiling lights, in grid cells.",
    )

    parser.add_argument(
        "--debug-camera",
        action="store_true",
        help="Add a simple overview camera.",
    )

    return parser.parse_args()


def main() -> None:
    args = parse_args()

    if args.cell_size <= 0:
        raise ValueError("--cell-size must be positive")

    if args.wall_height <= 0:
        raise ValueError("--wall-height must be positive")

    if args.ceiling_bottom_height <= 0:
        raise ValueError("--ceiling-bottom-height must be positive")

    if args.ceiling_opacity < 0.0 or args.ceiling_opacity > 1.0:
        raise ValueError("--ceiling-opacity must be between 0 and 1")

    build_maze_usd(
        map_path=args.map,
        output_path=args.out,
        cell_size=args.cell_size,
        wall_height=args.wall_height,
        wall_value=args.wall_value,
        invert=args.invert,
        merge_walls=not args.no_merge_walls,
        floor_thickness=args.floor_thickness,
        ceiling=not args.no_ceiling,
        ceiling_bottom_height=args.ceiling_bottom_height,
        ceiling_thickness=args.ceiling_thickness,
        ceiling_opacity=args.ceiling_opacity,
        ceiling_collision=not args.no_ceiling_collision,
        spawn_count=args.spawn_count,
        spawn_marker_radius=args.spawn_marker_radius,
        spawn_marker_height=args.spawn_marker_height,
        spawn_altitude=args.spawn_altitude,
        add_debug_camera_flag=args.debug_camera,
        add_lights_flag=not args.no_lights,
        light_spacing_cells=args.light_spacing_cells,
    )


if __name__ == "__main__":
    main()