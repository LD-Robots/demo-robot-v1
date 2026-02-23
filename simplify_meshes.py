#!/usr/bin/env python3
"""
Reduce mesh complexity (decimate) for all STL files referenced in a URDF.

Uses Open3D's quadric decimation to reduce triangle count while preserving
shape quality. Outputs simplified meshes to a separate directory and
generates a new URDF pointing to them.

For collision meshes, can generate convex hulls which are much faster
for physics engines like Gazebo (ODE/Bullet/DART).

Usage:
    python simplify_meshes.py                           # defaults
    python simplify_meshes.py -r 0.1                    # keep 10% of triangles
    python simplify_meshes.py -i robot.urdf -r 0.05     # custom input, 5%
    python simplify_meshes.py --convex-collision         # convex hulls for collision
"""

import argparse
import os
import shutil
import struct
import xml.etree.ElementTree as ET

import numpy as np
import open3d as o3d


def count_stl_triangles(path: str) -> int:
    """Quick triangle count from binary STL header (no full parse needed)."""
    with open(path, "rb") as f:
        f.read(80)
        return struct.unpack("<I", f.read(4))[0]


def decimate_mesh(input_path: str, output_path: str, ratio: float) -> dict:
    """
    Decimate a mesh using quadric error metric.

    Returns dict with stats: original_triangles, final_triangles, reduction_pct.
    """
    mesh = o3d.io.read_triangle_mesh(input_path)
    original = len(mesh.triangles)

    if original == 0:
        shutil.copy2(input_path, output_path)
        return {"original": 0, "final": 0, "reduction_pct": 0.0}

    target = max(int(original * ratio), 50)

    simplified = mesh.simplify_quadric_decimation(
        target_number_of_triangles=target
    )

    simplified.compute_vertex_normals()

    o3d.io.write_triangle_mesh(output_path, simplified)
    final = len(simplified.triangles)

    return {
        "original": original,
        "final": final,
        "reduction_pct": (1 - final / original) * 100 if original > 0 else 0,
    }


def convex_hull_mesh(input_path: str, output_path: str) -> dict:
    """
    Compute convex hull of a mesh - ideal for collision geometry.

    Returns dict with stats.
    """
    mesh = o3d.io.read_triangle_mesh(input_path)
    original = len(mesh.triangles)

    if original == 0:
        shutil.copy2(input_path, output_path)
        return {"original": 0, "final": 0, "reduction_pct": 0.0}

    hull, _ = mesh.compute_convex_hull()
    hull.compute_vertex_normals()

    o3d.io.write_triangle_mesh(output_path, hull)
    final = len(hull.triangles)

    return {
        "original": original,
        "final": final,
        "reduction_pct": (1 - final / original) * 100 if original > 0 else 0,
    }


def find_urdf_meshes(urdf_path: str) -> list[str]:
    """Extract unique mesh filenames from URDF (both visual and collision)."""
    tree = ET.parse(urdf_path)
    root = tree.getroot()
    meshes = set()
    for mesh_el in root.iter("mesh"):
        filename = mesh_el.get("filename", "")
        # Strip package:// prefix
        if filename.startswith("package://"):
            filename = filename[len("package://"):]
        if filename.endswith(".stl"):
            meshes.add(filename)
    return sorted(meshes)


def create_output_urdf(
    input_urdf: str,
    output_urdf: str,
    new_visual_dir: str,
    new_collision_dir: str | None,
):
    """
    Copy URDF, replacing mesh paths to point to simplified directories.

    Handles any source directory in mesh paths (assets/, assets_visual/, etc).
    """
    tree = ET.parse(input_urdf)
    root = tree.getroot()

    for link in root.iter("link"):
        # Visual meshes -> visual dir
        for visual in link.findall("visual"):
            for mesh_el in visual.iter("mesh"):
                fn = mesh_el.get("filename", "")
                if fn.startswith("package://"):
                    fn = fn[len("package://"):]
                if fn.endswith(".stl"):
                    basename = os.path.basename(fn)
                    mesh_el.set("filename", f"package://{new_visual_dir}{basename}")

        # Collision meshes -> collision dir (or visual dir)
        col_dir = new_collision_dir or new_visual_dir
        for collision in link.findall("collision"):
            for mesh_el in collision.iter("mesh"):
                fn = mesh_el.get("filename", "")
                if fn.startswith("package://"):
                    fn = fn[len("package://"):]
                if fn.endswith(".stl"):
                    basename = os.path.basename(fn)
                    mesh_el.set("filename", f"package://{col_dir}{basename}")

    tree.write(output_urdf, xml_declaration=True, encoding="utf-8")
    print(f"\nURDF written: {output_urdf}")


def main():
    parser = argparse.ArgumentParser(
        description="Decimate STL meshes referenced by a URDF for faster simulation."
    )
    parser.add_argument(
        "-i", "--input",
        default="robot_simplified.urdf",
        help="Input URDF file (default: robot_simplified.urdf)",
    )
    parser.add_argument(
        "-o", "--output",
        default="robot_sim.urdf",
        help="Output URDF file (default: robot_sim.urdf)",
    )
    parser.add_argument(
        "-r", "--ratio",
        type=float,
        default=0.1,
        help="Target ratio of triangles to keep for visual meshes (default: 0.1 = 10%%)",
    )
    parser.add_argument(
        "--source-dir",
        default="assets/",
        help="Directory containing the original STL meshes (default: assets/). "
             "Meshes are always read from here regardless of URDF paths.",
    )
    parser.add_argument(
        "--convex-collision",
        action="store_true",
        help="Use convex hulls for collision meshes (recommended for Gazebo). "
             "Much faster physics than decimated meshes.",
    )
    parser.add_argument(
        "--collision-ratio",
        type=float,
        default=None,
        help="Ratio for collision meshes if not using --convex-collision (default: same as --ratio).",
    )
    parser.add_argument(
        "--visual-dir",
        default="meshes/visual/",
        help="Output directory for visual meshes (default: meshes/visual/)",
    )
    parser.add_argument(
        "--collision-dir",
        default="meshes/collision/",
        help="Output directory for collision meshes (default: meshes/collision/)",
    )
    parser.add_argument(
        "--min-triangles",
        type=int,
        default=50000,
        help="Skip decimation for meshes with fewer triangles than this (default: 50000). "
             "Small meshes are copied as-is.",
    )
    args = parser.parse_args()

    base_dir = os.path.dirname(os.path.abspath(args.input)) or "."
    urdf_path = args.input

    # Always read originals from source-dir
    source_dir = os.path.join(base_dir, args.source_dir)

    # Resolve output directories
    visual_out = os.path.join(base_dir, args.visual_dir)
    separate_collision = args.convex_collision or args.collision_ratio is not None
    collision_ratio = args.collision_ratio or args.ratio
    collision_out = os.path.join(base_dir, args.collision_dir) if separate_collision else None

    os.makedirs(visual_out, exist_ok=True)
    if collision_out:
        os.makedirs(collision_out, exist_ok=True)

    # Find all meshes in URDF
    mesh_files = find_urdf_meshes(urdf_path)
    print(f"Found {len(mesh_files)} unique STL meshes in {urdf_path}")
    print(f"Reading originals from: {source_dir}")
    if args.convex_collision:
        print(f"Collision mode: CONVEX HULL")
    elif separate_collision:
        print(f"Collision mode: decimate at {collision_ratio:.0%}")
    print()

    total_original = 0
    total_visual = 0
    total_collision = 0
    total_orig_size = 0
    total_new_size = 0

    for mesh_rel in mesh_files:
        basename = os.path.basename(mesh_rel)
        # Always read from source dir
        mesh_path = os.path.join(source_dir, basename)

        if not os.path.exists(mesh_path):
            print(f"  SKIP  {basename} (not found in {args.source_dir})")
            continue

        orig_size = os.path.getsize(mesh_path)
        total_orig_size += orig_size

        tri_count = count_stl_triangles(mesh_path)
        visual_path = os.path.join(visual_out, basename)

        # Skip decimation for small meshes
        if tri_count < args.min_triangles:
            if os.path.abspath(mesh_path) != os.path.abspath(visual_path):
                shutil.copy2(mesh_path, visual_path)
            total_original += tri_count
            total_visual += tri_count
            total_new_size += orig_size
            print(
                f"  {basename:40s}  {tri_count:>8,} tri  "
                f"COPY (under {args.min_triangles:,} threshold)"
            )
            # Collision for small meshes
            if collision_out:
                col_path = os.path.join(collision_out, basename)
                if args.convex_collision:
                    stats_c = convex_hull_mesh(mesh_path, col_path)
                    total_collision += stats_c["final"]
                    print(
                        f"  {'  (collision convex)':40s}  "
                        f"{tri_count:>8,} -> {stats_c['final']:>8,} tri"
                    )
                else:
                    shutil.copy2(mesh_path, col_path)
                    total_collision += tri_count
            continue

        # Visual mesh
        stats_v = decimate_mesh(mesh_path, visual_path, args.ratio)
        total_original += stats_v["original"]
        total_visual += stats_v["final"]

        new_size = os.path.getsize(visual_path)
        total_new_size += new_size

        print(
            f"  {basename:40s}  {stats_v['original']:>8,} -> {stats_v['final']:>8,} tri  "
            f"({stats_v['reduction_pct']:5.1f}% reduction)  "
            f"{orig_size/1024/1024:.1f}MB -> {new_size/1024/1024:.1f}MB"
        )

        # Collision mesh
        if collision_out:
            col_path = os.path.join(collision_out, basename)
            if args.convex_collision:
                stats_c = convex_hull_mesh(mesh_path, col_path)
            else:
                stats_c = decimate_mesh(mesh_path, col_path, collision_ratio)
            total_collision += stats_c["final"]
            mode = "convex" if args.convex_collision else "decimate"
            print(
                f"  {'  (collision ' + mode + ')':40s}  "
                f"{stats_c['original']:>8,} -> {stats_c['final']:>8,} tri  "
                f"({stats_c['reduction_pct']:5.1f}% reduction)"
            )

    # Summary
    print(f"\n{'='*80}")
    print(f"Total original triangles:   {total_original:>12,}")
    print(f"Total visual triangles:     {total_visual:>12,}")
    if collision_out:
        print(f"Total collision triangles:  {total_collision:>12,}")
    print(f"Overall visual reduction:   {(1 - total_visual / total_original) * 100:.1f}%")
    if collision_out:
        print(f"Overall collision reduction: {(1 - total_collision / total_original) * 100:.1f}%")
    print(f"Total size:  {total_orig_size/1024/1024:.1f} MB -> {total_new_size/1024/1024:.1f} MB")

    # Create output URDF
    create_output_urdf(
        urdf_path,
        args.output,
        args.visual_dir,
        args.collision_dir if collision_out else None,
    )
    print(f"\nDone! Use '{args.output}' for simulation.")


if __name__ == "__main__":
    main()
