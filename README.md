# Trajectory-Aware MoCaS Camera Layout Optimization

MATLAB implementation of camera-layout optimization for tracking a robot along a prescribed six-degree-of-freedom trajectory, accounting for robot self-occlusion.

This branch supports the ICARA 2027 study **Camera Layout Optimization for Trajectory-Aware MoCaS under Robot Self-Occlusion**. It extends full-workspace volumetric coverage optimization by concentrating a fixed camera budget on the motion of body-fixed markers. The OmniOcta aerial robot is used as the case study.

## Method

The optimizer selects each camera's three position coordinates, yaw, and pitch. The trajectory and marker configuration remain fixed. With eight cameras, the decision vector contains 40 variables.

For every trajectory sample, the implementation:

1. Transforms the body-fixed markers into the world frame.
2. Checks camera sensing range and conical field of view.
3. Tests the surviving camera-to-marker rays against the robot triangle mesh.
4. Counts valid markers and determines whether the sample satisfies the tracking requirement.

The objective combines a graded meanâ€“worst-case tracking-deficiency penalty with camera-spacing, stereo-angle, and inward-looking orientation penalties. MATLAB's `particleswarm` performs the optimization, with independent particle evaluations distributed across parallel workers.

Self-occlusion is included **during optimization**. A bounding-volume hierarchy (BVH), built once in the body frame, accelerates intersection queries without decimating the occlusion mesh. The implementation uses range/FOV culling, rayâ€“box rejection, and early termination of rayâ€“triangle tests.

## Tracking metrics

- **Valid marker:** observed by at least `k_req` cameras after range, FOV, and self-occlusion checks.
- **Trackable sample:** contains at least `M_req` valid markers.
- **Trackability (%):** percentage of evaluated trajectory samples that are trackable.
- **Minimum/maximum valid markers:** smallest/largest valid-marker count over all trajectory samples.

The default requirements are `k_req = 4` and `M_req = 3`. A minimum count of zero means no marker meets the four-camera requirement; individual cameras may still see markers. A reported trackability of 100% applies to the evaluated samples, rather than guaranteeing visibility between samples.

## Repository contents

| Path | Purpose |
|---|---|
| `new/main_new.m` | Configure and run trajectory-aware optimization; evaluate, plot, and save results |
| `new/optimizeTrajectoryCameraLayout.m` | Decision bounds, swarm initialization, and PSO execution |
| `new/trajectoryLayoutObjective.m` | Tracking and geometric objective terms |
| `new/evaluateTrajectoryVisibility.m` | Cameraâ€“marker visibility and view counts |
| `new/evaluateTrackability.m` | Valid-marker counts and sample trackability |
| `new/plottingnew.m` | Load and visualize a saved trajectory-aware run |
| `baseline/main.m` | Run volumetric camera-layout optimization |
| `baseline/plotold.m` | Evaluate a saved volumetric layout on a saved trajectory |
| `common/` | Trajectory generators, geometry, visibility acceleration, and plotting helpers |
| `common/omni.STL` | OmniOcta geometry |
| `common/markers.mat` | Body-frame marker coordinates |
| `common/saved_layouts/` | Saved optimization runs and baseline layouts |

## Requirements

- MATLAB with `stlread` and `triangulation` support.
- Global Optimization Toolbox for `particleswarm`.
- Parallel Computing Toolbox for the default parallel configuration.

The default script starts eight local workers and uses 3,000 particles. Adjust the worker count and optimization settings for the available machine. Reducing swarm size or iteration limits changes the search configuration and may change the results.

## Quick start

### 1. Download the branch

```bash
git clone --branch icara2027 --single-branch https://github.com/RISC-NYUAD/MoCaS-camera-optimization.git
cd MoCaS-camera-optimization
```

### 2. Configure local paths

The entry-point scripts currently contain machine-specific Windows paths. In `new/main_new.m`, replace the `common_folder` assignment with:

```matlab
script_folder = fileparts(mfilename('fullpath'));
repo_folder = fileparts(script_folder);
common_folder = fullfile(repo_folder, 'common');
```

Use the same replacement in `new/plottingnew.m` and `baseline/plotold.m` before running those scripts. In `baseline/plotold.m`, also replace the `new_folder` assignment with:

```matlab
new_folder = fullfile(repo_folder, 'new');
```

### 3. Select a trajectory

In `new/main_new.m`, enable exactly one trajectory assignment:

```matlab
% Diagonal pitch-up
problem.trajectory = generateTrajectory(duration, dt);

% Helical descent
% problem.trajectory = generateHelicalDescentTrajectory(duration, dt);

% Forward weave
% problem.trajectory = generateForwardOscillatingTrajectory(duration, dt);
```

The committed main script selects the helical descent by default.

### 4. Run optimization

From the repository root in MATLAB:

```matlab
cd new
main_new
```

If a parallel pool already exists, replace the script's pool-creation block with the following to reuse it:

```matlab
pool = gcp('nocreate');
if isempty(pool)
    pool = parpool('local', 8);
end
```

For serial execution, omit pool creation and set `'UseParallel', false` in `new/optimizeTrajectoryCameraLayout.m`.

## Default configuration

These values are specified in `new/main_new.m`.

| Parameter | Value |
|---|---|
| Workspace bounds (m) | x: [-8, 8], y: [-8, 8], z: [0, 6] |
| Cameras | 8 |
| Views required per valid marker | 4 |
| Valid markers required per trackable sample | 3 |
| Full conical FOV angle | 75Â° (solid angle approximately 1.30 sr) |
| Sensing range | 0.1â€“10 m |
| Camera yaw bounds | -180° to 180° |
| Camera pitch bounds | -60° to 60° |
| Camera-spacing penalty threshold | 4 m |
| Stereo-angle penalty interval | 10°-170° |
| Trajectory duration / sampling period | 5 s / 0.05 s |
| Trajectory samples | 101 |
| Marker-neighborhood exclusion distance | 0.025 m |
| Meanâ€“worst-case mixing coefficient | 0.5 |
| Tracking weight | 100 |
| Each geometric penalty weight | 0.1 |
| PSO swarm size | 3,000 |
| Maximum PSO iterations | 1,000 |
| Random seed | 1 |
| Parallel workers | 8 |

Spacing and stereo geometry are penalized in the objective; their listed thresholds are not hard feasibility guarantees. The PSO objective limit is `1e-12`; unspecified solver options retain MATLAB defaults.

## Prescribed trajectories

All positions are in metres and angles below are in radians. Let `s = t/T`, with `0 <= s <= 1`.

| Maneuver | Position `[x, y, z]` | Attitude `[roll, pitch, yaw]` |
|---|---|---|
| Diagonal pitch-up | `[-6.5 + 13s, -6.5 + 13s, 2 - s]` | `[0, (pi/2)s, 0]` |
| Helical descent | `[3cos(4pi s), 3sin(4pi s), 4.5 - 2.5s]` | `[(pi/6)sin(4pi s), (pi/3)sin(8pi s), 0]` |
| Forward weave | `[3sin(6pi s), -5.5 + 11s, 3]` | `[0, -(pi/2)cos(6pi s), 0]` |

The helix completes two turns. The forward weave completes three lateral oscillations and three pitch cycles. These expressions follow the executable generator definitions; some comments in the source describe earlier settings.

Use the rotation matrices returned by the generators to preserve their body-to-world rotation convention.

## Geometry and markers

The main script loads `common/omni.STL` and divides its vertex coordinates by 1,000 to convert millimetres to metres. `markers.mat` must contain an `M x 3` array named `markers_body`, expressed in metres in the same body frame. If that file is absent, the main script creates a six-marker configuration from its embedded coordinates.

The marker configuration used in the study was generated using the software described in:

> M. Hamandi, A. M. Ali, N. Evangeliou, A. Tzes, and F. Khorrami, â€œGenerating distinctive marker configurations for robot detection in motion capture systems,â€ 2025 11th International Conference on Automation, Robotics, and Applications (ICARA), pp. 247â€“251, 2025.

The full occlusion mesh is retained. The default BVH leaf capacity is 12 triangles. Intersections within 0.025 m of the marker endpoint are excluded to avoid treating its mounting surface as an occluder.

## Saved results and visualization

`main_new.m` saves a timestamped file under:

```text
common/saved_layouts/camera_layout_YYYYMMDD_HHMMSS.mat
```

The file contains `saved_run`, including optimized and initial layouts, trajectory plotting data, trackability arrays, valid-marker counts, solver output, elapsed time, and tracking thresholds.

Each camera layout is an `N x 5` matrix with columns `[x, y, z, yaw, pitch]`, using metres and radians.

To visualize an existing run, configure `results_file` in `new/plottingnew.m`, then run:

```matlab
plottingnew
```

Saved plotting data are nested under `saved_run.plot_problem`. This is not a complete optimization checkpoint: reevaluating self-occlusion requires loading the body-frame markers and mesh and reconstructing the visibility configuration.

## Volumetric baseline comparison

Run `baseline/main.m` from the `baseline` directory to optimize volumetric coverage. To evaluate a saved volumetric layout against a trajectory-aware layout, configure the paths in `baseline/plotold.m` and choose:

```matlab
trajectory_results_name = 'hellicalhpsuccess.mat';
base_results_name = 'base2.mat';
use_self_occlusion = true;
```

Then run `plotold` from the `baseline` directory. Repeat with the desired trajectory result files while keeping the same volumetric layout.

## License

This repository is distributed under the [MIT License](LICENSE).
