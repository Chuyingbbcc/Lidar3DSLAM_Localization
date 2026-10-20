
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

## Technical details

The pipeline combines point-cloud preprocessing, incremental scan registration, LiDAR–IMU fusion, keyframe and submap management, GPS alignment, and global pose graph optimization.

### 1. Point cloud preprocessing

**Voxel downsampling.** Each input point is assigned to a spatial voxel. Points in the same voxel are replaced by their centroid, reducing point-cloud density while preserving the scene's overall geometry.

**Geometric filtering.** Before scan matching, the current implementation retains points with X and Y within ±40 m and Z between −1 m and 5 m.

### 2. Incremental NDT — scan registration and local-map update

#### Voxel map

The local map is divided into fixed-size voxels. Each sufficiently populated voxel stores the mean $\mu$ and covariance $\Sigma$ of its points. The inverse covariance acts as an information matrix, weighting registration errors according to local geometry.

#### Scan registration

Starting from an initial pose $(R,t)$, a source point $p$ is transformed into the map frame. The corresponding voxel and, optionally, its six neighboring voxels provide candidate associations:

$$
q = Rp+t, \qquad e=q-\mu, \qquad E=e^T\Sigma^{-1}e.
$$

Associations whose Mahalanobis error $E$ exceeds the outlier threshold are rejected.

#### Gauss–Newton optimization

Registration estimates a six-dimensional pose correction. For the ordering $\Delta x=[\Delta\theta^T,\Delta t^T]^T$, the implementation uses:

$$
J=\begin{bmatrix}-R[p]_\times & R\end{bmatrix}.
$$

Valid associations contribute to the normal equations:

$$
H=\sum J^T\Sigma^{-1}J, \qquad
b=-\sum J^T\Sigma^{-1}e, \qquad
H\Delta x=b.
$$

The correction updates the pose on SE(3). Association and optimization repeat until the correction falls below the convergence threshold or the iteration limit is reached.

#### Incremental map update

After registration, the aligned scan is inserted into the local map. Only affected voxels have their statistics updated. Small covariance singular values are clamped before inversion for numerical stability, and a fixed-capacity LRU cache bounds the active voxel map.

### 3. ESKF sensor fusion — LiDAR–IMU state estimation

The Error-State Kalman Filter combines high-frequency IMU propagation with pose corrections from NDT registration. Its nominal state contains position, velocity, orientation, accelerometer bias, gyroscope bias, and gravity:

$$
x=\{p,v,R,b_a,b_g,g\}, \qquad
\delta x=[\delta p^T,\delta v^T,\delta\theta^T,\delta b_a^T,\delta b_g^T,\delta g^T]^T.
$$

The error state has 18 dimensions.

#### IMU prediction

Bias-corrected measurements $\hat a=a_m-b_a$ and $\hat\omega=\omega_m-b_g$ propagate the nominal state over a time step $\Delta t$:

$$
\begin{aligned}
R_{k+1} &= R_k\mathrm{Exp}(\hat\omega\Delta t), \\
v_{k+1} &= v_k+(R_k\hat a+g)\Delta t, \\
p_{k+1} &= p_k+v_k\Delta t+\tfrac12(R_k\hat a+g)\Delta t^2.
\end{aligned}
$$

The covariance is propagated using the linearized error dynamics and process noise.

#### LiDAR correction and error reset

The registered LiDAR pose is compared with the predicted pose to form a position and orientation residual $r$. With measurement Jacobian $H$ and measurement-noise covariance $R_m$, the Kalman update is:

$$
K=P^-H^T(HP^-H^T+R_m)^{-1}, \qquad \delta x=Kr.
$$

Position, velocity, biases, and gravity receive additive corrections; orientation receives a rotation correction through $R\leftarrow R\mathrm{Exp}(\delta\theta)$. The filter updates the covariance and resets the error state around zero before the next prediction cycle.

### 4. Keyframe and submap management

Translation and rotation thresholds select keyframes from the estimated trajectory. Keyframe point clouds and pose metadata are saved locally, then grouped into submaps for back-end registration and optimization. This hierarchy supports map management at keyframe, submap, and global-map levels.

### 5. GPS integration — coordinate conversion and route alignment

GDAL projects GPS/RTK latitude and longitude into UTM coordinates. A robust planar rigid transform aligns the local odometry trajectory with the GPS/RTK route, estimating yaw and XY translation separately from the vertical offset.

#### Planar alignment

Paired odometry and GPS/RTK positions are centered. SVD of their cross-covariance estimates the planar rotation, with translation recovered from the trajectory centroids:

$$
p_{xy}^{\mathrm{gps}}=R_{xy}p_{xy}^{\mathrm{odom}}+t_{xy}.
$$

#### Robust refinement and height alignment

A minimum-motion threshold removes spatially redundant correspondences. Alignment residuals identify outliers; the transform is re-estimated from the remaining inliers. The vertical offset is the median paired height difference:

$$
\Delta z=\mathrm{median}_i(z_i^{\mathrm{gps}}-z_i^{\mathrm{odom}}).
$$

The aligned GPS observations provide global position constraints for the back end.

### 6. Pose graph optimization and loop closure

Each keyframe is a pose vertex $X_i\in SE(3)$. The g2o graph combines three constraint types:

- **Odometry:** relative transformations between consecutive keyframes.
- **GPS:** global position observations associated with keyframes.
- **Loop closure:** relative transformations between revisited keyframes.

#### Loop candidate validation

Spatial proximity identifies potential revisits. A local sliding-window search evaluates nearby keyframe pairs using NDT registration, and a validated pair supplies an additional loop-closure edge.

#### Global optimization

The graph minimizes weighted residuals across the odometry, GPS, and loop constraints:

$$
X^*=\arg\min_X\sum_{c\in\mathcal C}\rho(e_c^T\Omega_c e_c),
$$

where $\mathcal C$ is the constraint set, $e_c$ is a constraint residual, $\Omega_c$ is its information matrix, and $\rho$ denotes a Huber robust kernel. Levenberg–Marquardt optimization adjusts the keyframe poses to reduce accumulated drift and improve global consistency.

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
