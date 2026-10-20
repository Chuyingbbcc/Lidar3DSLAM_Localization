
# LiDAR-Inertial SLAM System

A ROS 2 LiDAR-inertial SLAM system combining incremental Normal Distributions Transform (IncNDT) scan registration with an Error-State Kalman Filter (ESKF). A g2o back end incorporates GPS/RTK observations and loop closures to reduce accumulated drift and improve global consistency.

The map is organized into keyframes, submaps, and a global map. An OpenGL viewer displays point clouds and trajectories during mapping, replay, and optimization. The current workflow processes recorded ROS 2 bags offline.

## Demos

KITTI `2011_10_03_drive_0027`: optimized map, LiDAR visualization, and frontend/backend comparisons.

### Optimized map — top view

<img src="output/kitti_output/opti_overall_map.png" alt="Top view of the optimized LiDAR map" width="900">

### LiDAR views

| Single LiDAR scan | Driving view |
| --- | --- |
| <img src="output/kitti_output/single_lidar.png" alt="A single LiDAR point cloud" width="440"> | <img src="output/kitti_output/driving_view.gif" alt="Animated driving view through the LiDAR map" width="440"> |

### Loop-closure comparison

Frontend on the left; after optimization on the right. Red and green identify the two saved loop-frame groups; gray shows the surrounding map.

<img src="output/kitti_output/loop_comparison_aligned.png" alt="Side-by-side frontend and optimized loop-frame clouds with a shared legend" width="1000">

### Route comparison

Orange shows the frontend route; cyan shows the optimized route. The second view makes vertical differences visible.

| Route overview | Z-direction view |
| --- | --- |
| <img src="output/kitti_output/route_comparison_labeled.png" alt="Frontend and optimized route overview, with orange and cyan legend" width="440"> | <img src="output/kitti_output/z_direction_route_comparison_labeled.png" alt="Side view comparing frontend and optimized route heights" width="440"> |

## System architecture

![System architecture](output/read_me_dialog/systems_diagram.png)

`mapping_node` runs the front end or back end and publishes keyframe metadata through `lio_msgs/FrameData`. `opengl_vis_node` loads the saved clouds and renders the selected pose layers.

## Technical overview

### Point cloud preprocessing

Voxel downsampling replaces points within each voxel with their centroid, reducing density while retaining scene geometry. The current implementation filters points outside an axis-aligned region: X and Y within ±40 m, and Z between −1 m and 5 m. These bounds reflect the code; the draft's proposed 1–20 m radial filter is not the current implementation.

### Incremental NDT

The local map stores a mean and covariance for each sufficiently populated voxel. A transformed scan point is associated with its voxel and, optionally, the six neighboring voxels. Registration minimizes the Mahalanobis residual between the point and voxel distribution, rejecting associations above an outlier threshold.

For a source point $p$, pose $(R,t)$, and voxel mean $\mu$:

$$
q = Rp+t, \qquad e=q-\mu, \qquad E=e^T\Sigma^{-1}e.
$$

Gauss–Newton estimates a six-dimensional pose correction. With correction ordering $\Delta x=[\Delta\theta^T,\Delta t^T]^T$, the implementation uses:

$$
J=\begin{bmatrix}-R[p]_\times & R\end{bmatrix}.
$$

The pose is updated on SE(3) until convergence or the iteration limit. Aligned scans update only affected map voxels. Covariance singular values are clamped before inversion, and a fixed-capacity LRU cache bounds the active voxel map.

### LiDAR–IMU fusion

The ESKF maintains position, velocity, orientation, accelerometer bias, gyroscope bias, and gravity, with an 18-dimensional error state. Bias-corrected IMU measurements propagate the nominal state and covariance. NDT pose observations supply measurement corrections, which are injected into the nominal state before resetting the error state.

### Keyframes and submaps

Translation and rotation thresholds select keyframes. Their point clouds and pose metadata are saved locally, then grouped into submaps for back-end registration and optimization. Configuration controls keyframe spacing, submap size, and registration settings.

### GPS/RTK alignment

GDAL projects latitude and longitude into UTM coordinates. The local odometry route is aligned with the GPS/RTK route using a robust planar rigid transform: SVD estimates yaw and XY translation, residual-based rejection refines the correspondences, and the median height difference estimates the vertical offset.

### Pose graph and loop closure

The g2o graph combines relative odometry edges, GPS position constraints, and validated loop-closure edges. Loop candidates are selected by spatial proximity; a local search evaluates nearby keyframe pairs using NDT registration. The graph uses Levenberg–Marquardt optimization and Huber robust kernels to reduce the influence of outlier constraints.

## Dataset and evaluation

The completed pipeline was evaluated on the configured KITTI `2011_10_03_drive_0027` recording using saved trajectories, keyframe clouds, and accepted loop measurements.

| Run statistic | Value |
| --- | ---: |
| Keyframes / available cloud files | 2,239 / 2,239 |
| Trajectory duration | 403.84 s |
| Estimated horizontal distance traveled | 3,073.27 m |
| Trajectory XY extent | 462.72 × 568.94 m |
| Median horizontal keyframe spacing | 0.977 m |
| Accepted loop constraints | 12 |

Evaluation focuses on **relative-pose and point-cloud consistency**, without comparing trajectory positions against GNSS.

| Consistency metric | Frontend | Before loop optimization | After loop optimization |
| --- | ---: | ---: | ---: |
| Accepted-loop translation RMSE ↓ | 10.618 m | 2.355 m | 0.713 m |
| Accepted-loop rotation RMSE ↓ | 3.457° | 2.938° | 3.080° |
| Mean cloud matching coverage ↑ | 90.41% | 89.82% | 89.78% |
| Cloud pairs with sufficient matches | 224 / 224 | 223 / 224 | 223 / 224 |

The run produced a complete set of saved keyframe clouds over a 3.07 km estimated route. Loop optimization reduced translation residual RMSE against 12 accepted loop measurements by **69.7%**, from 2.355 m to 0.713 m. Sampled adjacent-cloud matching retained approximately **90% coverage** after optimization. These results support operation of the mapping, saving, and loop-optimization pipeline; rotation residuals show no improvement from the loop stage. Cloud checks sample every tenth adjacent pair and use saved relative poses without rerunning registration. One optimized pair has insufficient matches. These are internal consistency checks, not independent accuracy measurements.

See [evaluation details and commands](src/lio_viz/docs/evaluation.md).

## Main challenges

- **Limited motion compensation:** the draft's datasets lack per-point timestamps, preventing point-level LiDAR deskewing and leaving within-scan motion distortion.
- **Intermittent IMU data:** missing measurements weaken prediction, particularly during sharp turns and rapid orientation changes.
- **Parameter sensitivity:** NDT, keyframe selection, loop search, and graph weights require tuning across sensors and environments.
- **Weak vertical constraints:** street-driving geometry provides less Z-axis information, allowing vertical drift to accumulate.
- **Loosely coupled GPS correction:** GPS acts primarily through the back end, improving global consistency while front-end drift can still accumulate.

## Future work

- Fuse camera observations with the LiDAR-inertial pipeline.
- Associate fitted planes with meaningful objects and structures.
- Explore learned visual geometry, including VGGT, for reconstruction and correspondence.
- Remove dynamic objects before registration and map integration.

## Repository layout

```text
src/
├── lio_msgs/                  Custom keyframe visualization messages
└── lio_viz/
    ├── include/               Estimation, mapping, and visualization interfaces
    ├── src/
    │   ├── Config/            YAML configuration
    │   ├── io/                Offline ROS bag reader
    │   ├── Lio/               LiDAR–IMU processing pipeline
    │   └── Slam/              Front end, back end, and pose graph optimization
    ├── docs/                  Evaluation documentation
    └── test/                  Core and evaluation tests
