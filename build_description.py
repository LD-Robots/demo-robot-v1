#!/usr/bin/env python3
"""
Master script: runs the full URDF pipeline and deploys to a robot_description package.

Runs all 4 steps:
  1. urdf_simplify.py    — simplify URDF structure from Onshape export
  2. apply_joint_limits.py — apply joint limits from YAML config
  3. simplify_meshes.py   — decimate visual meshes + convex hull collision
  4. split_urdf.py        — split into joints/links xacro files

Then copies the results (xacro files + meshes) to the target robot_description package.

Usage:
    python build_description.py /path/to/my_robot_description
    python build_description.py /path/to/dual_arm_description -r 0.2
    python build_description.py /path/to/dual_arm_description --skip-simplify
"""

import argparse
import os
import shutil
import subprocess
import sys


def run(cmd: list[str], desc: str) -> None:
    """Run a command, printing it and checking for errors."""
    print(f"\n{'='*70}")
    print(f"  {desc}")
    print(f"  $ {' '.join(cmd)}")
    print(f"{'='*70}\n")
    result = subprocess.run(cmd, check=False)
    if result.returncode != 0:
        print(f"\nERROR: {desc} failed (exit code {result.returncode})")
        sys.exit(1)


def main():
    parser = argparse.ArgumentParser(
        description="Run full URDF pipeline and deploy to a robot_description package."
    )
    parser.add_argument(
        "target",
        help="Path to target robot_description package "
             "(e.g. /path/to/dual_arm_description)",
    )
    parser.add_argument(
        "--source-urdf",
        default="robot.urdf",
        help="Source URDF from Onshape export (default: robot.urdf)",
    )
    parser.add_argument(
        "-r", "--ratio",
        type=float,
        default=0.2,
        help="Visual mesh decimation ratio (default: 0.2 = 20%%)",
    )
    parser.add_argument(
        "--skip-simplify",
        action="store_true",
        help="Skip step 1 (urdf_simplify.py), use existing robot_simplified.urdf",
    )
    parser.add_argument(
        "--skip-limits",
        action="store_true",
        help="Skip step 2 (apply_joint_limits.py), use existing robot_with_limits.urdf",
    )
    parser.add_argument(
        "--fixed-legs",
        action="store_true",
        help="Add xacro support for fixed_legs argument",
    )
    parser.add_argument(
        "--damping",
        type=float,
        default=0.5,
        help="Joint damping (default: 0.5)",
    )
    parser.add_argument(
        "--friction",
        type=float,
        default=0.1,
        help="Joint friction (default: 0.1)",
    )
    args = parser.parse_args()

    # Resolve paths
    script_dir = os.path.dirname(os.path.abspath(__file__))
    target = os.path.abspath(args.target)
    package_name = os.path.basename(target)

    print(f"Target package: {package_name}")
    print(f"Target path:    {target}")

    if not os.path.isdir(target):
        print(f"ERROR: Target directory does not exist: {target}")
        sys.exit(1)

    # Step 1: Simplify URDF structure
    if not args.skip_simplify:
        run(
            [sys.executable, os.path.join(script_dir, "urdf_simplify.py"),
             args.source_urdf],
            "Step 1/4: Simplify URDF structure",
        )
    else:
        print("\n-- Skipping step 1 (urdf_simplify.py)")

    # Step 2: Apply joint limits
    if not args.skip_limits:
        run(
            [sys.executable, os.path.join(script_dir, "apply_joint_limits.py")],
            "Step 2/4: Apply joint limits",
        )
    else:
        print("\n-- Skipping step 2 (apply_joint_limits.py)")

    # Step 3: Simplify meshes
    run(
        [sys.executable, os.path.join(script_dir, "simplify_meshes.py"),
         "-o", "robot_gazebo.urdf",
         "-i", "robot_with_limits.urdf",
         "-r", str(args.ratio),
         "--convex-collision"],
        "Step 3/4: Decimate meshes (visual + convex collision)",
    )

    # Step 4: Split into xacro files
    split_cmd = [
        sys.executable, os.path.join(script_dir, "split_urdf.py"),
        "-i", "robot_gazebo.urdf",
        "-p", package_name,
        "--damping", str(args.damping),
        "--friction", str(args.friction),
    ]
    if args.fixed_legs:
        split_cmd.append("--fixed-legs")
    run(split_cmd, "Step 4/4: Split into joints/links xacro files")

    # Deploy to target package
    print(f"\n{'='*70}")
    print(f"  Deploying to {target}")
    print(f"{'='*70}\n")

    # Derive xacro base name (e.g. "dual_arm" from "dual_arm_description")
    xacro_name = package_name.replace("_description", "")

    # Copy xacro files
    joints_src = os.path.join(script_dir, "urdf", "joints", "gazebo_joints.xacro")
    links_src = os.path.join(script_dir, "urdf", "links", "gazebo_links.xacro")
    joints_dst = os.path.join(target, "urdf", "joints", f"{xacro_name}_joints.xacro")
    links_dst = os.path.join(target, "urdf", "links", f"{xacro_name}_links.xacro")

    os.makedirs(os.path.dirname(joints_dst), exist_ok=True)
    os.makedirs(os.path.dirname(links_dst), exist_ok=True)

    shutil.copy2(joints_src, joints_dst)
    print(f"  Copied: {joints_dst}")
    shutil.copy2(links_src, links_dst)
    print(f"  Copied: {links_dst}")

    # Copy meshes
    for mesh_type in ["visual", "collision"]:
        src_dir = os.path.join(script_dir, "meshes", mesh_type)
        dst_dir = os.path.join(target, "meshes", mesh_type)
        os.makedirs(dst_dir, exist_ok=True)

        count = 0
        for f in sorted(os.listdir(src_dir)):
            if f.endswith(".stl"):
                shutil.copy2(os.path.join(src_dir, f), os.path.join(dst_dir, f))
                count += 1
        print(f"  Copied: {count} {mesh_type} meshes -> {dst_dir}")

    print(f"\nDone! Package '{package_name}' updated.")
    print(f"  Xacro files:  {xacro_name}_joints.xacro, {xacro_name}_links.xacro")
    print(f"  Mesh paths:   package://{package_name}/meshes/visual|collision/*.stl")


if __name__ == "__main__":
    main()
