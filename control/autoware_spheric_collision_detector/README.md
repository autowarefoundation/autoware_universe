# AUTOWARE_SPHERIC_COLLISION_DETECTOR

## PURPOSE

A new collision checker, referred to as Spheric Collision Detector (SCD), is proposed. The method approximates the ego-vehicle and its surrounding objects using a set of spheres, thereby simplifying collision detection to a mere distance evaluation.

The proposed algorithm is similar to OCC; However unlike OCC it does not rely on point cloud data for object representation. Instead it uses the length and width of objects to construct the spherical approximation. This approach has proven effective for detecting collisions, particularly in emergency situations.

Experimental results were simulated using CARLA(0.9.15), Autoware(2024.01) and ROS 2 bridges. They indicate that the proposed approach is at least 200 times faster than OCC.

To top all this, the algorithm was also tested in PixKit 2.0, an autonomous driving and development vehicle using a 5-foot inflated jumbo beach ball.

## ALGORITHM

### Compute ego's stopping distance

We start by computing the braking distance, $s_b$, which is the distance the vehicle travels after the brakes are applied. It is defined as $|v|^2/2.0 * a$ - deduced from the equation of motion, where $v$ is the initial velocity of the ego-vehicle and $a$, the maximum deceleration required for the ego to stop. The thinking distance, $s_t$, is the distance the ego travels in the time it takes for the brakes to be applied after realizing that there is a need to stop. It is defined as $v * t_d$, with $t_d$ as the delay time. The stopping distance is the sum of the thinking distance and the braking distance.

### Resample input trajectory

Next, we select consecutive trajectory points spaced at one sphere diameter apart. This reduces computation cost as collision checks are performed only on these sampled points.

### Cut resampled trajectory

In this step only the segment of the resampled trajectory within the stopping distance is considered.

### Create ego's footprints

For each of the trajectory points, their respective footprints are computed using the below expressions:
(1) x*{front} ={}& h*{front} + wb \\
(2) x*{center} ={}& wb/2.0 \\
(3) x*{rear} ={}& -(h*{rear}) \\
(4) y*{left} ={}& wt/2.0 + h*{left} \\
(5) y*{right} ={}& -(wt/2.0 + h\_{right})

### Create ego's passing areas

Subsequently, we generate spheres, and place them on the ego's footprints.

### Represent objects with spheres

We also approximate the objects surrounding the ego-vehicle using spheres. Each object's footprint is derived from its length and width. Uniform spheres are then generated and placed along an object's center-line, with their relative centers defined as follows:
$(x_{front}, 0)$, $(x_{front}/2, 0)$, $(0,0)$,
$(x_{rear}/2, 0)$, $(x_{rear}, 0)$.

We set the radius of each sphere to half the vehicle's width.

### Check collision

To perform a collision check we execute the following steps:

- We loop through each distinct sphere pair, $(s_{i}, s_{j})$ of the ego-vehicle and the obstacle, where $1 \leq i \leq m$, $1 \leq j \leq n$. $m$ and $n$ denote the number of spheres of the obstacle and the ego-vehicle respectively.
- At each iteration, the Euclidean distance, $\varepsilon_{ij}$, between the centers of the spheres, $c_i(x,y,z)$ and $c_j(x,y,z)$, is calculated.
- We then compare $\varepsilon_{ij}$ with the sum of the radii of the respective spheres, $ \sum r_i r_j$. If this distance is less, the diagnostic status is set to ERROR as a collision is imminent; otherwise, we repeat \textit{step 2} for the next pair of spheres. The diagnostic status is set to OK if no collision is detected.

#### INPUT(S)

| Name                       | Type                                                  | Description                                                                                                           |
| -------------------------- | ----------------------------------------------------- | --------------------------------------------------------------------------------------------------------------------- |
| `input/odometry`           | `nav_msgs::msg::Odometry`                             | Current Position, orientation, and velocity of the ego-vehicle.                                                       |
| `input/trajectory`         | `autoware_auto_planning_msgs::msg::Trajectory`        | Predicted trajectory (sequence of poses of the ego-vehicle over time with corresponding velocities and accelerations) |
| `input/object_recognition` | `autoware_auto_perception_msgs::msg::DetectedObjects` | Length, width, and height of objects surrounding the ego-vehicle                                                      |

### OUTPUT(S)

| Name           | Type                                   | Description              |
| -------------- | -------------------------------------- | ------------------------ |
| `debug/marker` | `visualization_msgs::msg::MarkerArray` | Marker for visualization |

### PARAMETERS

| Name               | Type     | Description                                               | Default value |
| :----------------- | :------- | :-------------------------------------------------------- | :------------ |
| `delay_time`       | `double` | Delay time of the ego-vehicle [s]                         | 0.3           |
| `max_deceleration` | `double` | Max deceleration of the ego-vehicle when stopping [m/s^2] | 2.0           |
