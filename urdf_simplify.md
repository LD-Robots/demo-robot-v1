# urdf_simplify.py

Python script for simplifying and reorienting URDF files exported from Onshape (via onshape-to-robot).

## What it does

1. **Analyzes the URDF** - identifies main links, sub-links (duplicate/fixed joints) and displays a kinematic tree
2. **Waits for confirmation** (`yes`) before modifying anything
3. **Simplifies the structure**:
   - Removes sub-links (duplicate joints, fixed joints)
   - Merges sub-link masses into their parent main links
   - Strips motor hardware meshes (rotors, stators, bearings, nuts, spacers) from visual/collision, keeping only structural parts
4. **Reorients all frames** to the standard convention:
   - **Z up, X forward, Y left**
   - Pitch / knee / elbow → `axis="0 1 0"` (Y)
   - Roll → `axis="1 0 0"` (X)
   - Yaw → `axis="0 0 1"` (Z)
5. **Cleans rpy values** - joints get rpy close to `0 0 0` (like G1)
6. **Preserves physics** - world-frame axes are identical (or flipped with limits swapped)

## How it works

### Two-Pass Canonical Frame Algorithm

**Pass 1** - Computes the world-frame rotation for each link in the original URDF:
```
R_world[child] = R_world[parent] @ R_joint_origin
```

**Pass 2** - For each joint, independently:
1. Computes the physical axis in the new world frame: `v_world = R_GLOBAL @ R_world[child] @ axis_local`
2. Builds a **canonical frame** (as closely aligned to the world frame as possible) that has the joint axis as one of its axes
3. Computes `rpy = mat_to_rpy(R_canon_parent^T @ R_canon_child)` → small values
4. Transforms link elements (visual, collision, inertial) from the old frame to the new one

The advantage: each frame is computed directly from the world-frame axis, not propagated through the chain. This eliminates twist accumulation through the kinematic chain.

### Global rotation

The Onshape URDF has the base frame with X=right, Y=forward, Z=up.
The script applies `R_GLOBAL = Rz(-90°)` to obtain X=forward, Y=left, Z=up.

### Axis fallback

If a joint's physical axis doesn't match the name-based classification (e.g. `right_ankle_pitch` has its axis along X instead of Y due to CAD asymmetry), the script automatically falls back to the closest standard axis.

## Usage

```bash
# Basic usage (output: robot_simplified.urdf)
python urdf_simplify.py robot.urdf

# With specified output file
python urdf_simplify.py robot.urdf -o robot_clean.urdf

# Non-interactive (pipe "yes")
echo "yes" | python urdf_simplify.py robot.urdf -o output.urdf
```

### Arguments

| Argument | Description |
|----------|-------------|
| `input_urdf` | Path to the input URDF file |
| `--output`, `-o` | Output URDF file (default: `<input>_simplified.urdf`) |

### Dependencies

None - uses only standard Python modules (`xml.etree`, `math`, `argparse`).

## Example output

```
Result joint origins:
  waist_yaw_joint           axis=[0 0 1] rpy=[0 0 0]
  left_hip_pitch_joint      axis=[0 1 0] rpy=[0 0 0]
  left_hip_roll_joint       axis=[1 0 0] rpy=[0 0 0]
  left_shoulder_yaw_joint   axis=[0 0 1] rpy=[0.174533 0 0]
  ...

Done!
  Output: robot_simplified.urdf
  Links: 26 -> 24
  Joints: 25 -> 23
  Mass preserved: 57.342 kg
```
