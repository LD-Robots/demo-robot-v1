# URDF Pipeline: Onshape to Gazebo

Pipeline for converting Onshape CAD exports into simulation-ready URDF files. Three scripts, run in order:

```
robot.urdf (Onshape export)
    │
    ▼  urdf_simplify.py
robot_simplified.urdf (clean frames, merged links)
    │
    ▼  apply_joint_limits.py
robot_with_limits.urdf (joint limits from YAML)
    │
    ▼  simplify_meshes.py
robot_gazebo.urdf (decimated visual + convex collision meshes)
```

---

## Step 1: Simplify URDF structure — `urdf_simplify.py`

Simplifies and reorients URDF files exported from Onshape (via onshape-to-robot).

### What it does

1. **Analyzes the URDF** — identifies main links, sub-links (duplicate/fixed joints), displays kinematic tree
2. **Waits for confirmation** (`yes`) before modifying anything
3. **Simplifies the structure**:
   - Removes sub-links (duplicate joints, fixed joints)
   - Merges sub-link masses into their parent main links
   - Strips motor hardware meshes (rotors, stators, bearings, nuts, spacers) from visual/collision
4. **Reorients all frames** to standard convention:
   - Z up, X forward, Y left
   - Pitch / knee / elbow → `axis="0 1 0"` (Y)
   - Roll → `axis="1 0 0"` (X)
   - Yaw → `axis="0 0 1"` (Z)
5. **Cleans rpy values** — joints get rpy close to `0 0 0`
6. **Preserves physics** — world-frame axes are identical (or flipped with limits swapped)

### How it works

**Pass 1** — Computes the world-frame rotation for each link:
```
R_world[child] = R_world[parent] @ R_joint_origin
```

**Pass 2** — For each joint, independently:
1. Computes the physical axis in the new world frame: `v_world = R_GLOBAL @ R_world[child] @ axis_local`
2. Builds a canonical frame (aligned to world axes) with the joint axis as one of its axes
3. Computes `rpy = mat_to_rpy(R_canon_parent^T @ R_canon_child)` → small values
4. Transforms link elements (visual, collision, inertial) from old frame to new one

Each frame is computed directly from the world-frame axis, not propagated through the chain. This eliminates twist accumulation.

A global rotation `R_GLOBAL = Rz(-90°)` converts from Onshape convention (X=right, Y=forward) to standard (X=forward, Y=left, Z=up).

### Usage

```bash
python urdf_simplify.py robot.urdf
python urdf_simplify.py robot.urdf -o robot_clean.urdf
echo "yes" | python urdf_simplify.py robot.urdf    # non-interactive
```

| Argument | Description |
|----------|-------------|
| `input_urdf` | Path to the input URDF file |
| `-o`, `--output` | Output file (default: `<input>_simplified.urdf`) |

**Dependencies:** None (standard Python only).

---

## Step 2: Apply joint limits — `apply_joint_limits.py`

Reads joint limits from a YAML config and writes them into the URDF.

### Usage

```bash
python apply_joint_limits.py
python apply_joint_limits.py -c custom_limits.yaml
python apply_joint_limits.py -i in.urdf -o out.urdf
```

| Argument | Default | Description |
|----------|---------|-------------|
| `-i`, `--input` | `robot_simplified.urdf` | Input URDF |
| `-c`, `--config` | `joint_limits.yaml` | YAML config with joint limits |
| `-o`, `--output` | `robot_with_limits.urdf` | Output URDF |

### YAML format

```yaml
joints:
  waist_yaw_joint:
    lower: -1.5708
    upper: 1.5708
    effort: 100
    velocity: 100
```

**Dependencies:** `pyyaml`

---

## Step 3: Simplify meshes — `simplify_meshes.py`

Decimates STL meshes for faster simulation. Uses Open3D's quadric decimation for visual meshes and convex hulls for collision meshes.

### Why

Onshape exports high-poly CAD meshes (6.4M triangles, 306 MB total). This is too heavy for real-time simulation. The script:
- **Visual meshes**: reduces triangle count by a ratio (default 10%) — good enough for rendering
- **Collision meshes**: computes convex hulls — much faster for physics engines (ODE/Bullet/DART)
- **Small meshes** (under `--min-triangles` threshold): copied as-is, no decimation

### Usage

```bash
# Recommended for Gazebo (visual at 20%, convex hull collision):
python simplify_meshes.py -o robot_gazebo.urdf -r 0.2 --convex-collision

# Visual only, no separate collision:
python simplify_meshes.py -r 0.1

# Custom source and output:
python simplify_meshes.py -i robot_with_limits.urdf -o robot_gazebo.urdf -r 0.1 --convex-collision

# Separate decimated collision (not convex):
python simplify_meshes.py -r 0.1 --collision-ratio 0.02
```

| Argument | Default | Description |
|----------|---------|-------------|
| `-i`, `--input` | `robot_simplified.urdf` | Input URDF |
| `-o`, `--output` | `robot_sim.urdf` | Output URDF |
| `-r`, `--ratio` | `0.1` | Fraction of triangles to keep for visual meshes |
| `--convex-collision` | off | Use convex hulls for collision (recommended for Gazebo) |
| `--collision-ratio` | same as `-r` | Separate decimation ratio for collision meshes |
| `--source-dir` | `assets/` | Directory with original STL meshes |
| `--visual-dir` | `meshes/visual/` | Output directory for visual meshes |
| `--collision-dir` | `meshes/collision/` | Output directory for collision meshes |
| `--min-triangles` | `50000` | Meshes below this threshold are copied without decimation |

### Typical results

| | Original | Visual (20%) | Collision (convex) |
|---|---|---|---|
| Triangles | 6,426,130 | ~1,285,000 | ~210,000 |
| Size | 306 MB | ~61 MB | ~10 MB |

**Dependencies:** `open3d`

---

## Full pipeline example

```bash
# 1. Simplify URDF structure (from Onshape export)
echo "yes" | python urdf_simplify.py robot.urdf

# 2. Apply joint limits
python apply_joint_limits.py

# 3. Decimate meshes for Gazebo
python simplify_meshes.py -o robot_gazebo.urdf -r 0.2 --convex-collision
```

Output: `robot_gazebo.urdf` with meshes in `meshes/visual/` and `meshes/collision/`.
