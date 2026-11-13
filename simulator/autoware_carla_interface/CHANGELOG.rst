^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_carla_interface
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.53.0 (2026-09-29)
-------------------
* Merge remote-tracking branch 'origin/main' into prepare-0.53.0-changelog
* feat(autoware_carla_interface): allow cameras to publish bgr8 (`#13411 <https://github.com/autowarefoundation/autoware_universe/issues/13411>`_)
  CARLA renders BGRA and fills the alpha channel with 255. Nothing
  downstream reads it, so publishing bgra8 spends a quarter of every camera
  message on a constant.
  Accept bgr8 as an image_encoding alongside bgra8 and mono8: it drops the
  alpha channel and nothing else, so a consumer that wants colour gets the
  same picture for three quarters of the bytes -- 4,320,000 instead of
  5,760,000 for a 1600x900 frame. Unlike mono8 it costs no information, so
  it is the cheaper default for any colour consumer. The default stays
  bgra8.
  Claude-Session: https://claude.ai/code/session_011TMdPd8VUDxUrFtckxg2AQ
  Co-authored-by: Masaya Kataoka <ms.kataoka@gmail.com>
* fix(autoware_carla_interface): stamp each sensor measurement with the frame it was captured on (`#13408 <https://github.com/autowarefoundation/autoware_universe/issues/13408>`_)
* feat(autoware_carla_interface): let a sensor mapping set the CARLA capture rate (`#13407 <https://github.com/autowarefoundation/autoware_universe/issues/13407>`_)
* feat(autoware_carla_interface): publish CARLA vehicles as ground truth detections (`#13321 <https://github.com/autowarefoundation/autoware_universe/issues/13321>`_)
  * feat(autoware_carla_interface): publish CARLA vehicles as ground truth detections
  Adds an opt-in parameter, publish_ground_truth_objects (default: false, no
  behavior change), that publishes every CARLA vehicle except the ego to
  /perception/object_recognition/detection/objects.
  This feeds the perception stack from simulator truth instead of from sensor
  data, so tracking and prediction keep running on the real Autoware nodes
  downstream. It is useful when the vehicles come from an external traffic
  simulator that the sensor pipeline cannot see reliably, and when the goal is to
  exercise planning rather than detection.
  Publishing happens on a world.on_tick() callback, so the objects follow CARLA's
  cadence rather than the sensor loop, which only turns once every sensor has
  delivered its frame. The poses come out of the tick snapshot, so a tick costs no
  round trip to the server and every object in a message belongs to one frame. A
  snapshot carries only ids and poses, so what never changes about an actor -
  whether it is a vehicle to report, its class and its size - is looked up the
  first time that id appears and kept; in steady state a tick asks the server for
  nothing.
  Messages carry the bridge's clock rather than snapshot.timestamp. The two count
  from different starts, and stamping from the snapshot puts the objects far
  enough ahead of /clock that the tracker's output rate collapses.
  Classification comes from the CARLA blueprint base_type attribute, so the
  mapping needs no hand maintained table of blueprint ids. Velocity is left to the
  tracker: publishing it would need a world to object frame rotation that is easy
  to get wrong, and a wrong twist is worse for prediction than no twist.
  Pedestrians are not covered yet.
  * chore(autoware_carla_interface): keep the ground truth publisher's comments in the file's style
  Shorten the docstrings to one line each, replace the multi-line rationale
  comments with one-liners and drop a noqa the repository's flake8 does not need.
  No behavior change; the rationale stays in the pull request.
  * feat(autoware_carla_interface): apply the review suggestions to the ground truth publisher
  Add the bus label (CARLA spells the base_type "Bus") and publish the bounding
  box center instead of the actor origin, as suggested in review. The suggestions
  were applied through the review UI without a sign-off, so they are folded into
  this signed commit; the content is unchanged.
  Co-authored-by: Mete Fatih Cırıt <mfc@autoware.org>
  Co-authored-by: Masaya Kataoka <ms.kataoka@gmail.com>
  ---------
  Co-authored-by: Mete Fatih Cırıt <mfc@autoware.org>
  Co-authored-by: Masaya Kataoka <ms.kataoka@gmail.com>
* feat(autoware_carla_interface): add a tick follower mode for co-simul… (`#13286 <https://github.com/autowarefoundation/autoware_universe/issues/13286>`_)
* feat(autoware_carla_interface): publish CARLA traffic-light states, matched to the map by position (`#13327 <https://github.com/autowarefoundation/autoware_universe/issues/13327>`_)
  * feat(autoware_carla_interface): publish CARLA traffic-light states, matched to the map by position
  Bridge the CARLA server's traffic-light states into Autoware's perception
  output so a CARLA closed loop can run without camera-based recognition.
  The key problem is associating a CARLA traffic light with an Autoware
  `traffic_light_group_id` (a `traffic_light` regulatory-element id in the
  lanelet2 map). Instead of assuming the CARLA OpenDRIVE signal id equals the
  regulatory-element id (true only for maps auto-generated from the same
  OpenDRIVE) or hand-writing an id table, the bridge discovers the mapping
  geometrically: each CARLA light head is matched to the nearest lanelet2 light
  head and its state is published under every regulatory element that references
  that head. This works for hand-authored / Vector Map Builder maps too.
  - New `modules/traffic_light_matcher.py`: parses the lanelet2 `.osm` directly
  (reads `local_x`/`local_y`, i.e. the map frame; no lanelet2/projector
  dependency), keys physical heads by their `refers` way, and matches CARLA
  heads conservatively (distance threshold + a disjoint-group ambiguity ratio),
  dropping and logging ambiguous / too-far lights rather than guessing.
  - `carla_ros.py`: publishes `TrafficLightGroupArray` on
  /perception/traffic_light_recognition/traffic_signals, aggregated per group.
  - `carla_autoware.py`: `force_green` freezes all lights green for camera-less
  runs.
  - Parameters grouped under the `traffic_light.` namespace; resolution order is
  id-map override -> position match -> OpenDRIVE-id fallback.
  - Unit tests for the matcher; README documents the feature.
  Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>
  * fix(autoware_carla_interface): address Codex review on traffic-light matching
  - Use the resolved map origin (`_current_map_origin()`) instead of the raw
  `map_origin_x/y` parameters when placing CARLA light heads in the map frame,
  so georeferenced maps (origin derived from the OpenDRIVE geoReference in
  `on_world_ready`, parameters left at zero) match correctly instead of falling
  outside the distance threshold and publishing nothing. (P1)
  - Treat a candidate head as a genuine alternative for the ambiguity test unless
  its group set is exactly equal to the winner's, replacing the `isdisjoint`
  check. Overlapping-but-unequal sets (e.g. {500, 501} vs {501}) would otherwise
  be accepted by arbitrary ranking and publish a missing or spurious group. Adds
  a regression test. (P2)
  Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>
  ---------
  Co-authored-by: Masaya Kataoka <cld-masaya.kataoka@tier4.jp>
  Co-authored-by: Claude Opus 4.8 (1M context) <noreply@anthropic.com>
* fix(autoware_carla_interface): synthesize the steering report when CARLA reports no wheel angle (`#13276 <https://github.com/autowarefoundation/autoware_universe/issues/13276>`_)
  * fix(autoware_carla_interface): synthesize the steering report when CARLA reports no wheel angle
  CARLA 0.10 (Chaos physics) always returns 0 from get_wheel_steer_angle(),
  so /vehicle/status/steering_status reported a constant 0 rad steering angle
  regardless of the applied control. Controllers that consume the steering
  state (e.g. the MPC lateral controller) then operate on a vehicle model
  whose steering never responds.
  When the reported wheel angle is exactly 0, fall back to synthesizing the
  steering report from the applied VehicleControl.steer scaled by the max
  wheel steer angle. On CARLA 0.9.x, where the API works, the measured wheel
  angle keeps taking precedence.
  Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>
  * fix(autoware_carla_interface): fold the steering curve into the synthesized report
  The CARLA 0.10 fallback synthesized the steering report from
  get_control().steer * max wheel angle, but that fraction is the value
  requested BEFORE the server applies the vehicle's speed-based
  steering_curve. Whenever the curve attenuates steering at speed the
  report overstated the wheel angle CARLA actually produced, feeding
  controllers (e.g. the MPC lateral controller) an inconsistent state.
  Fold the same steering_curve back into the synthesized value so the
  report tracks the produced angle. With flatten_steering_curve the cached
  curve is the identity curve, so the factor is ~1.0 and the report is
  unchanged.
  Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>
  * refactor(autoware_carla_interface): dispatch the steering report on the CARLA version
  Select the measured vs. synthesized steering report from the CARLA server
  version instead of treating a reported 0 wheel angle as the trigger. The
  server version is read once at world load (set_carla_version): CARLA 0.10+
  (Chaos) synthesizes from the applied control, 0.9.x uses the measured wheel
  angle. This avoids mistaking a genuinely centered wheel on 0.9.x for the
  broken 0.10 API, and an unparsable version keeps the 0.9.x behavior.
  Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>
  ---------
  Co-authored-by: Masaya Kataoka <cld-masaya.kataoka@tier4.jp>
  Co-authored-by: Claude Fable 5 <noreply@anthropic.com>
* feat(autoware_carla_interface): allow overriding the wheel max steer angle for steer calibration (`#13278 <https://github.com/autowarefoundation/autoware_universe/issues/13278>`_)
  * feat(autoware_carla_interface): allow overriding the wheel max steer angle for steer calibration
  CARLA 0.10 (Chaos) reports max_steer_angle = 70 deg for the front wheels but
  only achieves roughly a third of it: with a sustained steer input of 0.33-0.41
  the yaw-rate-derived tire angle saturates around 0.11-0.16 rad, i.e. an
  effective full-steer angle of about 22 deg. The normalized steer command and
  the synthesized steering report both use the reported 70 deg, so the command
  is 3x weaker than intended and the reported steering is 3x larger than what
  the vehicle actually does.
  Add a max_wheel_steer_angle_deg parameter (default 0 = use the physics value)
  that overrides the angle used for both conversions, so it can be calibrated
  to the measured full-steer angle of the simulated vehicle.
  Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>
  * fix(autoware_carla_interface): scale the steering report by the calibrated max wheel angle
  Apply max_wheel_steer_angle_deg to the steering feedback as well, not just
  the command normalization. ego_status() published the raw CARLA
  get_wheel_steer_angle() on the physics full-steer scale, so an override left
  the reported tire angle ~3x larger than the command convention and preserved
  the command/report mismatch the parameter is meant to remove.
  Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>
  ---------
  Co-authored-by: Masaya Kataoka <cld-masaya.kataoka@tier4.jp>
  Co-authored-by: Claude Fable 5 <noreply@anthropic.com>
  Co-authored-by: Max-Bin <vborisw@gmail.com>
* docs(autoware_carla_interface): document CARLA 0.10 opt-in launch parameters (`#13222 <https://github.com/autowarefoundation/autoware_universe/issues/13222>`_)
  * docs(autoware_carla_interface): document CARLA 0.10 opt-in launch parameters
  Add the opt-in CARLA 0.10 launch parameters to the "Configurable Parameters
  for World Loading" table (carla_map, no_rendering_mode, force_load_world,
  map_origin_x/y, spawn_point_ground_snap, spawn_point_ground_offset_z,
  initial_pose_ground_offset_z), and add a "Ground snapping" subsection that
  explains the neighborhood ground-projection sampling (9-point cross probe,
  highest-hit selection) and the has_attribute fallback.
  Documentation only; each parameter defaults to the current behavior.
  Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>
  * style(pre-commit): autofix
  * docs(autoware_carla_interface): correct no_rendering and ground-snap wording
  Address review feedback on the documented behavior:
  - no_rendering_mode is applied unconditionally on world load, so the
  default False (re-)enables rendering rather than leaving an existing
  headless server unchanged.
  - Ground snapping only logs a warning on the spawn-point fallback; the
  RViz initial-pose fallback is silent, and the default random spawn does
  not exercise the spawn-point path at all.
  Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>
  * style(pre-commit): autofix
  ---------
  Co-authored-by: Claude Opus 4.8 <noreply@anthropic.com>
  Co-authored-by: pre-commit-ci-lite[bot] <117423508+pre-commit-ci-lite[bot]@users.noreply.github.com>
  Co-authored-by: Masaya Kataoka <cld-masaya.kataoka@tier4.jp>
* fix(autoware_carla_interface): derive the map origin from the OpenDRIVE geoReference (`#13303 <https://github.com/autowarefoundation/autoware_universe/issues/13303>`_)
  * fix(autoware_carla_interface): derive the map origin from the OpenDRIVE geoReference
  The GNSS pose and RViz initialpose paths convert between the CARLA world
  frame and the Autoware map frame with the hand-set map_origin_x/y
  parameters. For georeferenced maps (e.g. converted from lanelet2) such
  hand-maintained constants can silently disagree with the map's own
  OpenDRIVE geoReference by sub-metre amounts (observed: 0.44 m on a real
  map), shifting GNSS/initialpose against everything that derives its
  offset from the map itself.
  Resolve the origin from a single source of truth instead: an explicit
  non-zero parameter still wins, otherwise the offset is derived once from
  the OpenDRIVE <geoReference> +lat_0/+lon_0 as the origin's in-cell MGRS
  coordinates. Stock CARLA towns (no usable geoReference, or lat_0/lon_0 at
  0/0) keep the plain 0/0 behavior.
  Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>
  * style(pre-commit): autofix
  ---------
  Co-authored-by: Masaya Kataoka <cld-masaya.kataoka@tier4.jp>
  Co-authored-by: Claude Fable 5 <noreply@anthropic.com>
  Co-authored-by: pre-commit-ci-lite[bot] <117423508+pre-commit-ci-lite[bot]@users.noreply.github.com>
* fix(autoware_carla_interface): wake the sleeping physics body when launching from standstill (`#13305 <https://github.com/autowarefoundation/autoware_universe/issues/13305>`_)
  * fix(autoware_carla_interface): wake the sleeping physics body when launching from standstill
  CARLA 0.10 (UE5/Chaos) puts a stationary vehicle's physics body to sleep,
  and VehicleControl throttle does not wake it. A vehicle that has been
  stopped for a while (waiting for a route, holding at an intersection,
  pausing mid-mission) can then never launch again: the commanded throttle
  is applied (verified via get_control(): throttle > 0, brake 0, first
  gear) but velocity stays exactly 0 until an external set_target_velocity
  kick wakes the body.
  Nudge the body awake with a small forward set_target_velocity whenever
  the stack is trying to pull away from a standstill (throttle commanded,
  no brake, speed < 0.05 m/s). Once the vehicle is rolling this no-ops.
  Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>
  * fix(autoware_carla_interface): gate the standstill wake nudge behind a parameter
  The wake nudge fired on every launch from standstill (throttle > 0, no
  brake, speed < 0.05 m/s), so on the supported CARLA 0.9.15 environment —
  whose physics bodies never sleep — it would inject a 0.3 m/s
  set_target_velocity at every ordinary start and override the throttle-
  driven launch dynamics.
  Gate it behind a new wake_sleeping_physics parameter (default false),
  matching the opt-in pattern used for the other CARLA 0.10 workarounds
  (flatten_steering_curve). 0.9.15 is now unaffected; 0.10 users enable it
  explicitly.
  Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>
  * style(pre-commit): autofix
  ---------
  Co-authored-by: Masaya Kataoka <cld-masaya.kataoka@tier4.jp>
  Co-authored-by: Claude Fable 5 <noreply@anthropic.com>
  Co-authored-by: pre-commit-ci-lite[bot] <117423508+pre-commit-ci-lite[bot]@users.noreply.github.com>
  Co-authored-by: Max-Bin <vborisw@gmail.com>
* fix(autoware_carla_interface): publish the E2E kinematic state directly from the CARLA ground truth (`#13280 <https://github.com/autowarefoundation/autoware_universe/issues/13280>`_)
  * fix(autoware_carla_interface): publish the E2E kinematic state directly from the CARLA ground truth
  The E2E planning group synthesized /localization/kinematic_state and the
  map->base_link TF through a GNSS round-trip: the interface published the GT
  GNSS pose, vehicle_velocity_converter turned the velocity report into a
  twist, and carla_state_publisher merged the two back into an Odometry. Apart
  from the extra hops and latency, the twist leg carried the heading-rate unit
  bug fixed in `#13274 <https://github.com/autowarefoundation/autoware_universe/issues/13274>`_, and any other consumer stack publishing localization
  topics alongside it ends up with duplicate publishers.
  Publish the kinematic state and TF directly from the ego ground-truth
  transform in the interface node (opt-in publish_ground_truth_localization
  parameter, enabled by the E2E launch group), drop carla_state_publisher and
  vehicle_velocity_converter from the launch, and feed twist2accel from the
  ground-truth odometry (use_odom) instead of the converter twist.
  Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>
  * fix(autoware_carla_interface): rotate the angular velocity into the odometry child frame
  get_angular_velocity() is expressed in CARLA's world frame, while
  Odometry.twist must be expressed in child_frame_id (base_link). Negating
  only the world-frame z component misstates the yaw rate on slopes or
  banked roads and discards the body-frame x/y rates. Rotate the vector
  with the inverse ego rotation, as already done for the linear velocity,
  then apply the deg/s left-handed to rad/s right-handed conversion with
  the axis signs used by the official ros-bridge (x, -y, -z).
  Addresses the Codex P2 review comment on the PR.
  Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>
  * refactor(autoware_carla_interface): extract the per-sensor dispatch out of run_step
  CodeScene flagged run_step for rising cyclomatic complexity (10 -> 11,
  threshold 9) after the ground-truth localization branch was added. Move
  the enable check into _publish_ground_truth_odometry as a guard clause
  and extract the sensor-type dispatch into _publish_sensor_data, which
  brings run_step down to complexity 2 and keeps every touched method
  under the threshold.
  Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>
  ---------
  Co-authored-by: Masaya Kataoka <cld-masaya.kataoka@tier4.jp>
  Co-authored-by: Claude Fable 5 <noreply@anthropic.com>
  Co-authored-by: Max-Bin <vborisw@gmail.com>
* feat(autoware_carla_interface): optionally flatten the corrupt CARLA 0.10 steering curve (`#13277 <https://github.com/autowarefoundation/autoware_universe/issues/13277>`_)
  * feat(autoware_carla_interface): optionally flatten the corrupt CARLA 0.10 steering curve
  CARLA 0.10 ships corrupt per-vehicle steering-curve data (duplicated,
  unsorted points such as (10 m/s, 0.5)), and the simulator applies that curve
  internally, attenuating the achievable wheel angle at driving speeds. Add an
  opt-in flatten_steering_curve parameter (default false) that writes an
  identity curve back with apply_physics_control() right after the ego spawn,
  so the commanded steer fraction maps directly to the wheel angle.
  Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>
  * style(pre-commit): autofix
  ---------
  Co-authored-by: Masaya Kataoka <cld-masaya.kataoka@tier4.jp>
  Co-authored-by: Claude Fable 5 <noreply@anthropic.com>
  Co-authored-by: pre-commit-ci-lite[bot] <117423508+pre-commit-ci-lite[bot]@users.noreply.github.com>
* feat(autoware_carla_interface): tolerate maps without parseable OpenDRIVE metadata (`#13233 <https://github.com/autowarefoundation/autoware_universe/issues/13233>`_)
* fix(autoware_carla_interface): normalize the steer command by the wheel max steer angle (`#13275 <https://github.com/autowarefoundation/autoware_universe/issues/13275>`_)
  * fix(autoware_carla_interface): normalize the steer command by the wheel max steer angle
  The steer command from raw_vehicle_cmd_converter (convert_steer_cmd: false)
  is a tire angle in radians, but it was written to VehicleControl.steer as-is,
  which CARLA interprets as a fraction of the wheel's max steer angle. On top
  of that it was multiplied by the physics steering_curve, which the simulator
  already applies internally — and which CARLA 0.10 returns as corrupt data
  (duplicated, unsorted points such as (10 m/s, 0.5), halving the steering
  around 36 km/h).
  Normalize the commanded tire angle by the max wheel steer angle from the
  vehicle physics (radians(max(wheels[].max_steer_angle))), clamp to [-1, 1],
  and drop the steering-curve multiplication so the speed-based limit is only
  applied once, inside the simulator.
  Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>
  * style(pre-commit): autofix
  ---------
  Co-authored-by: Masaya Kataoka <cld-masaya.kataoka@tier4.jp>
  Co-authored-by: Claude Fable 5 <noreply@anthropic.com>
  Co-authored-by: pre-commit-ci-lite[bot] <117423508+pre-commit-ci-lite[bot]@users.noreply.github.com>
* feat(autoware_carla_interface): make world loading version-tolerant with force_load_world (`#13262 <https://github.com/autowarefoundation/autoware_universe/issues/13262>`_)
* fix(autoware_carla_interface): convert the heading rate to rad/s with a right-handed sign (`#13274 <https://github.com/autowarefoundation/autoware_universe/issues/13274>`_)
  VelocityReport.heading_rate forwarded CARLA's angular velocity unchanged,
  but carla.Actor.get_angular_velocity() returns deg/s with a left-handed
  (CW-positive) convention, while the report expects rad/s CCW-positive. Every
  consumer of the heading rate (e.g. vehicle_velocity_converter feeding the
  E2E localization) therefore received a value ~57x too large with an inverted
  sign, which was measured to destabilize the lateral control loop in a
  closed-loop CARLA run.
  Co-authored-by: Masaya Kataoka <cld-masaya.kataoka@tier4.jp>
  Co-authored-by: Claude Fable 5 <noreply@anthropic.com>
* feat(autoware_carla_interface): add min_positive_throttle for standstill starts (`#13263 <https://github.com/autowarefoundation/autoware_universe/issues/13263>`_)
  Heavy CARLA vehicles (e.g. vehicle.taxi.ford) do not creep and never
  start moving on the small throttle the actuation map yields at low
  target accelerations, so the ego gets stuck at standstill.
  Add a min_positive_throttle parameter (default 0.0 = disabled) that
  enforces a lower bound on the commanded throttle while accelerating
  from (near) standstill: it applies only when the command is positive,
  no brake is requested, and the current speed is below
  min_positive_throttle_speed_threshold (default 0.8 m/s; negative =
  always).
  Also pin the vehicle to first gear with manual shifting in the control
  command so heavy vehicles respond to throttle immediately instead of
  idling in neutral.
  With the default parameters the throttle floor is disabled and the
  commanded throttle passes through unchanged.
  Co-authored-by: Claude Opus 4.8 <noreply@anthropic.com>
  Co-authored-by: Masaya Kataoka <cld-masaya.kataoka@tier4.jp>
  Co-authored-by: Max-Bin <vborisw@gmail.com>
* fix(autoware_carla_interface): raise a descriptive error when the ego vehicle fails to spawn (`#13264 <https://github.com/autowarefoundation/autoware_universe/issues/13264>`_)
* feat(autoware_carla_interface): optionally snap spawn and initial pose to ground (`#13232 <https://github.com/autowarefoundation/autoware_universe/issues/13232>`_)
  Add an opt-in `spawn_point_ground_snap` mode that snaps the ego spawn
  point and the RViz "2D Pose Estimate" initial pose onto the CARLA map
  geometry using `world.ground_projection` (sampling a few nearby points
  and taking the highest ground hit). The Z offset above the projected
  ground is configurable via `spawn_point_ground_offset_z` (0.5) and
  `initial_pose_ground_offset_z` (1.0).
  This is useful on maps where the map-frame z does not match the terrain
  (e.g. some CARLA 0.10 levels), where a fixed z offset can spawn the
  vehicle significantly above/below the road surface.
  The feature is gated behind `spawn_point_ground_snap` (default False) and
  `ground_projection` is guarded with `hasattr`, so behavior is unchanged
  on CARLA 0.9.x and older APIs.
  This is one of a series of small enabler PRs toward CARLA 0.10.0 support.
  Co-authored-by: Claude Opus 4.8 <noreply@anthropic.com>
* feat(autoware_carla_interface): add optional no_rendering_mode world setting (`#13210 <https://github.com/autowarefoundation/autoware_universe/issues/13210>`_)
* feat(autoware_carla_interface): sort steering curve before interpolation (`#13195 <https://github.com/autowarefoundation/autoware_universe/issues/13195>`_)
  `control_callback` feeds the vehicle's `steering_curve` into
  `numpy.interp`, which requires the sample x-coordinates to be
  monotonically increasing. CARLA 0.10 can return the steering-curve
  points out of order, producing an incorrect steer ratio.
  Sort the curve points by x before interpolating. On CARLA 0.9.x the
  curve is already sorted, so this is a behavioral no-op.
  Co-authored-by: Claude Opus 4.8 <noreply@anthropic.com>
* feat(autoware_carla_interface): guard sensor blueprint attributes with has_attribute (`#13196 <https://github.com/autowarefoundation/autoware_universe/issues/13196>`_)
  Camera and LiDAR blueprint configuration called `set_attribute`
  unconditionally. CARLA sensor blueprints expose different attribute
  sets across versions (e.g. 0.9.15 vs 0.10), so an absent attribute
  raised during sensor setup.
  Add a `_set_attribute_if_supported` helper that checks `has_attribute`
  before setting, mirroring the existing `_set_noise_attribute` guard used
  for GNSS/IMU, and route the camera and LiDAR attributes through it.
  On CARLA 0.9.x these attributes are all present, so behavior is
  unchanged.
  Co-authored-by: Claude Opus 4.8 <noreply@anthropic.com>
* feat(autoware_carla_interface): allow explicit carla_map override in launch (`#13193 <https://github.com/autowarefoundation/autoware_universe/issues/13193>`_)
* feat(autoware_carla_interface): add map_origin_x/y offset between CARLA and map frame (`#13198 <https://github.com/autowarefoundation/autoware_universe/issues/13198>`_)
  Add optional `map_origin_x` / `map_origin_y` parameters that translate
  between CARLA's local world origin and the Autoware map frame origin.
  The offset is applied in `carla_location_to_ros_point` (CARLA -> ROS)
  and its inverse `ros_pose_to_carla_transform` (ROS -> CARLA), and is
  threaded through from the ego `pose()` publisher and the
  `initialpose_callback`.
  This is needed for CARLA levels authored with their own local origin
  instead of matching the lanelet2/PCD map frame (e.g. a
  real-world-georeferenced custom map). Stock CARLA towns are natively
  aligned, so the offsets default to 0.0.
  With the default 0.0 offsets the arithmetic is the identity, so the
  published pose and the applied initial pose are byte-for-byte identical
  to the previous behavior.
  Co-authored-by: Claude Opus 4.8 <noreply@anthropic.com>
  Co-authored-by: Max-Bin <vborisw@gmail.com>
* fix(autoware_carla_interface): stop dropping frames when the publish rate matches the sensor rate (`#13149 <https://github.com/autowarefoundation/autoware_universe/issues/13149>`_)
  should_publish compared the elapsed time against the period exactly.
  A sensor whose sensor_tick equals the configured publish period lands
  on time_diff values that fall a float rounding step short of it, so
  those frames are dropped and the next one only arrives a whole period
  later. The sensor then publishes at a fraction of the rate it was
  configured for.
  Counting publications over 200 source frames, with the timestamps
  accumulated the way a simulation clock accumulates them:
  fixed_delta_seconds  frequency_hz  before  after
  0.05                 20               134    200
  0.1                  10               134    200
  0.02                 50               193    200
  1/30                 30               115    200
  Compare against the period less a tolerance far below any simulation
  step, which cannot admit a genuinely early frame.
* feat: relay CAMERA_FRONT to traffic_light namespace regardless of use_light_weight_sensor_mapping options (`#13166 <https://github.com/autowarefoundation/autoware_universe/issues/13166>`_)
  * feat: relay CAMERA_FRONT to traffic_light namespace regardless of use_light_weight_sensor_mapping options
  * style(pre-commit): autofix
  ---------
  Co-authored-by: Ryohsuke Mitsudome <ryoshuke.mitsudome@tier4.jp>
  Co-authored-by: pre-commit-ci-lite[bot] <117423508+pre-commit-ci-lite[bot]@users.noreply.github.com>
* feat(autoware_carla_interface): make IMU and GNSS noise configurable (`#13154 <https://github.com/autowarefoundation/autoware_universe/issues/13154>`_)
  Both sensors were spawned with every noise attribute pinned to zero and
  no way to change it. Noise-free measurements are the right default for
  reproducing a run, but they are not what any hardware produces, and a
  localisation or odometry stack scored against a perfect gyro reports an
  accuracy it will not reach on a vehicle.
  Take the noise attributes from the sensor's parameters in the sensor
  mapping, keeping zero as the default so an existing configuration
  behaves exactly as before. The IMU also gains the gyro bias attributes,
  which were never set at all, and each attribute is checked against the
  blueprint before it is written so a CARLA version that lacks one is not
  a failure.
* feat(autoware_carla_interface): allow cameras to publish mono8 (`#13151 <https://github.com/autowarefoundation/autoware_universe/issues/13151>`_)
* chore(carla_interface): add Bin Wang as maintainer (`#13063 <https://github.com/autowarefoundation/autoware_universe/issues/13063>`_)
* feat(carla_interface): launch shift_decider as standalone node when agnocast (`#13015 <https://github.com/autowarefoundation/autoware_universe/issues/13015>`_)
  * refactor: apply_agnocast
  * refactor: revert comment
  ---------
* Contributors: Masaya Kataoka, Maxime CLEMENT, Ryohsuke Mitsudome, Yutaro Kobayashi, inf, 張 智輝

0.52.0 (2026-06-30)
-------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* fix(autoware_carla_interface): decouple sensor publishing from the synchronous tick loop (`#12748 <https://github.com/autowarefoundation/autoware_universe/issues/12748>`_)
  The bridge converts and publishes sensor data inline in the same thread
  that paces world.tick(). Publishing a large message can block on DDS
  flow control (a 1600x900 BGRA image with reliable QoS is ~5.8 MB,
  far above typical writer watermarks), so transport conditions stall
  the simulation clock itself. Measured on a 28-vCPU cloud VM with one
  subscribed camera: 7 ms loop iterations inflate to ~200 ms and the
  simulation drops from 20 fps to ~7 fps, which trips Autoware's
  data-freshness gates.
  Changes:
  - Add SensorPublishWorker: one daemon thread per heavy sensor with a
  bounded latest-wins queue. Camera and lidar conversion/publishing run
  on these workers, so the tick thread never blocks on serialization or
  DDS flow control. Stale frames are dropped instead of stalling the
  simulation (sensor data is perishable). With this change the same
  one-camera scenario keeps loop iterations at 12-28 ms.
  - Skip conversion and publishing entirely for sensors with no
  subscribers (checked per frame, so late subscribers still start
  receiving). Spawned-but-unconsumed sensors now cost nothing on the
  ROS side.
  - Gate the always-on CAM_FRONT traffic-light relays and republish node
  behind use_light_weight_sensor_mapping, consistent with the other
  camera republish nodes, so camera consumers are opt-out in
  performance-sensitive setups.
  Frequency gating and registry bookkeeping stay on the tick thread, so
  the sensor registry is never accessed concurrently; messages are
  stamped with the timestamp captured when their frame was enqueued.
  Note: with cameras spawned, CARLA's own per-tick render/stream cost on
  server-class GPUs remains the next bottleneck; that layer is outside
  this interface.
* fix(autoware_carla_interface): skip redundant world reload (`#12616 <https://github.com/autowarefoundation/autoware_universe/issues/12616>`_)
* feat(autoware_carla_interface): add spectator follow script (`#12526 <https://github.com/autowarefoundation/autoware_universe/issues/12526>`_)
* feat(autoware_carla_interface): add light weight sensor config (`#12525 <https://github.com/autowarefoundation/autoware_universe/issues/12525>`_)
  * feat(autoware_carla_interface): add light weight sensor config
  * update README
  * update comment
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci-lite[bot] <117423508+pre-commit-ci-lite[bot]@users.noreply.github.com>
* feat(autoware_carla_interface): add maintainer (`#12549 <https://github.com/autowarefoundation/autoware_universe/issues/12549>`_)
  Add masaya.kataoka@tier4.jp as a maintainer of autoware_carla_interface.
  Co-authored-by: Claude Opus 4.6 <noreply@anthropic.com>
* Contributors: Masaya Kataoka, Ryohsuke Mitsudome, github-actions, oguzkaganozt

0.51.0 (2026-05-01)
-------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* feat(carla_interface): implement turn_indicators (`#12527 <https://github.com/mitsudome-r/autoware_universe/issues/12527>`_)
  Implemented turn_indicators
* feat: default artifact paths to ~/autoware_data/ml_models (`#12523 <https://github.com/mitsudome-r/autoware_universe/issues/12523>`_)
  feat(launches,configs): default artifact paths to ~/autoware_data/ml_models
  Roll every per-package `data_path` / `model_path` launch-arg default
  from `$(env HOME)/autoware_data[/...]` to
  `$(env HOME)/autoware_data/ml_models[/...]` so standalone universe
  launches resolve artifacts under the new `~/autoware_data/ml_models/`
  layout (`autowarefoundation/autoware#7068 <https://github.com/autowarefoundation/autoware/issues/7068>`_).
  When invoked through autoware_launch the parent overrides cascade and
  already pin the new root (`autowarefoundation/autoware_launch#1835 <https://github.com/autowarefoundation/autoware_launch/issues/1835>`_); this
  commit closes the gap for users who launch a perception / localization /
  sensing / planning component directly with `ros2 launch <pkg>`.
  22 launch files updated (one-line default change each):
  - e2e/autoware_tensorrt_vad/launch/vad_carla_tiny.launch.xml
  - localization/yabloc/yabloc_pose_initializer/launch/yabloc_pose_initializer.launch.xml
  - perception/autoware_bevfusion/launch/bevfusion.launch.xml
  - perception/autoware_camera_streampetr/launch/streampetr.launch.xml
  - perception/autoware_image_projection_based_fusion/launch/pointpainting_fusion.launch.xml
  - perception/autoware_lidar_apollo_instance_segmentation/launch/lidar_apollo_instance_segmentation.launch.xml
  - perception/autoware_lidar_centerpoint/launch/lidar_centerpoint.launch.xml
  - perception/autoware_lidar_frnet/launch/lidar_frnet.launch.xml
  - perception/autoware_lidar_transfusion/launch/lidar_transfusion.launch.xml
  - perception/autoware_ptv3/launch/ptv3.launch.xml
  - perception/autoware_shape_estimation/launch/shape_estimation.launch.xml
  - perception/autoware_simpl_prediction/launch/simpl.launch.xml
  - perception/autoware_tensorrt_bevdet/launch/tensorrt_bevdet.launch.xml
  - perception/autoware_tensorrt_bevformer/launch/bevformer.launch.xml
  - perception/autoware_tensorrt_yolox/launch/{yolox_traffic_light_detector,yolox_tiny,yolox_s_plus_opt}.launch.xml
  - perception/autoware_traffic_light_classifier/launch/{car,pedestrian}_traffic_light_classifier.launch.xml
  - perception/autoware_traffic_light_fine_detector/launch/traffic_light_fine_detector.launch.xml
  - planning/autoware_diffusion_planner/launch/diffusion_planner.launch.xml
  - sensing/autoware_calibration_status_classifier/launch/calibration_status_classifier.launch.xml
  Drive-by README and test fixes:
  - e2e/autoware_tensorrt_vad/{README.md,docs/design.md}: also migrate the
  `$HOME/autoware_map/Town01` examples to `$HOME/autoware_data/maps/Town01`.
  - localization/yabloc/{README.md,yabloc_pose_initializer/README.md}: also
  migrate `$HOME/autoware_map/sample-map-rosbag` to
  `$HOME/autoware_data/maps/demos/sample-map-rosbag`.
  - control/autoware_smart_mpc_trajectory_follower/README.md: migrate the
  `map_path:=$HOME/autoware_map/sample-map-planning` example to
  `$HOME/autoware_data/maps/demos/sample-map-planning`.
  - simulator/autoware_carla_interface/README.md: migrate every
  `$HOME/autoware_map/Town01/...` reference to
  `$HOME/autoware_data/maps/Town01/...`.
  - perception/{autoware_bevfusion,autoware_image_projection_based_fusion,autoware_lidar_centerpoint,autoware_tensorrt_bevformer}/README.md: copy-paste examples updated to `~/autoware_data/ml_models/<pkg>`.
  - perception/autoware_camera_streampetr/config/ml_package_camera_streampetr.param.yaml: header comment updated.
  - planning/autoware_diffusion_planner/README.md: prerequisites snippet updated.
  - sensing/autoware_calibration_status_classifier/test/{test_model_inference,test_calibration_status_classifier}.cpp: hardcoded fallback ONNX path updated.
  Users on the legacy layout can pin the old root with
  `data_path:=$HOME/autoware_data` (or the per-package equivalent) on the
  command line.
  Refs: https://github.com/autowarefoundation/autoware/issues/7068
* fix(autoware_carla_interface): remove autoware_launch exec_depend to break circular dependency (`#12112 <https://github.com/mitsudome-r/autoware_universe/issues/12112>`_)
  fix(autoware_carla_interface): break circular dependency with autoware_launch
  Use an indirect variable reference for autoware_launch in the launch file
  to prevent check-package-depends from auto-adding exec_depend, which
  creates a circular dependency:
  autoware_carla_interface -> autoware_launch -> autoware_carla_interface
  The check-package-depends hook scans for find-pkg-share patterns and
  auto-adds exec_depend entries. By using $(var launch_config_pkg) instead
  of a literal package name, the hook's filter (skips names containing $)
  prevents the auto-addition while runtime behavior remains identical.
* feat: carla interface e2e planning (`#11706 <https://github.com/mitsudome-r/autoware_universe/issues/11706>`_)
  * fix(autoware_carla_interface): correct config file installation paths
  Fix setup.py to install sensor_mapping.yaml in config/ subdirectory
  instead of root share directory. This ensures the package works correctly
  in production/deployment scenarios where source files are not available.
  - raw_vehicle_cmd_converter.param.yaml: installed to share root (correct)
  - sensor_mapping.yaml: installed to share/config/ (matches expected path)
  Without this fix, the package relies on fallback to source directory which
  fails in Docker containers and binary package deployments.
  * fix(autoware_carla_interface): complete sensor mapping example in README
  Update camera sensor mapping example to include all required fields:
  - Add topic_info field for camera_info topic
  - Add qos_profile field for ROS2 QoS configuration
  - Remove unnecessary quotes from YAML values
  These fields are essential for proper camera sensor configuration and were
  missing from the documentation example, potentially causing confusion for
  users trying to configure custom sensors.
  * docs(autoware_carla_interface): remove misleading LiDAR concatenation note
  Remove the note about uncommenting LiDAR concatenation relay from Known Issues
  section. The single LiDAR configuration may still require the concatenated topic
  for coordinate transformation, which will be tested separately.
  The relay in launch file remains commented out pending further testing.
  * refactor(autoware_carla_interface): remove unnecessary lidar concatenation relay
  Remove commented-out lidar concatenation relay from launch file. Testing confirms
  that the main Autoware sensing pipeline already provides the concatenated pointcloud
  topic through the mirror_cropped pipeline, making this relay redundant.
  The /sensing/lidar/concatenated/pointcloud topic is successfully published by
  the main sensing stack and consumed by localization and perception modules.
  * feat(autoware_carla_interface): add E2E planning support and VAD integration
  Add comprehensive end-to-end (E2E) planning infrastructure to enable
  neural planners like VAD to directly control vehicles in CARLA simulation.
  Key features:
  - E2E planning components (velocity converter, state publisher, TF, controls)
  - CARLA utility C++ nodes (state publisher, operation mode publisher)
  - Multi-camera combiner for temporal synchronization
  - Conditional E2E activation via use_e2e_planning flag
  - Automatic CARLA map name derivation from map_path
  - Mixed Python/C++ package architecture
  Related to autoware_tensorrt_vad integration for end-to-end autonomous driving.
  * chore(autoware_carla_interface): simplify CHANGELOG for 0.48.0
  Simplify 0.48.0 CHANGELOG section to keep it concise for PR submission.
  * refactor(autoware_carla_interface): remove E2E control stack from CARLA interface
  Move E2E control stack (trajectory follower, shift decider, external cmd selector,
  vehicle command gate) from autoware_carla_interface.launch.xml to e2e_simulator.launch.xml
  for better modularity and separation of concerns.
  The control stack is now managed by the main E2E simulator launch file.
  * chore(autoware_carla_interface): remove E2E planning entry from CHANGELOG
  * chore(autoware_carla_interface): remove unused exec dependencies
  Remove autoware_external_cmd_selector and autoware_launch from exec_depend.
  These packages are not used by autoware_carla_interface:
  - autoware_external_cmd_selector is not referenced anywhere in the codebase
  - autoware_launch is a top-level launcher that includes this package, not a dependency of it
  * feat(autoware_carla_interface): add E2E planning support and VAD integration
  Add comprehensive end-to-end (E2E) planning infrastructure to enable
  neural planners like VAD to directly control vehicles in CARLA simulation.
  Key features:
  - E2E planning components (velocity converter, state publisher, TF, controls)
  - CARLA utility C++ nodes (state publisher, operation mode publisher)
  - Multi-camera combiner for temporal synchronization
  - Conditional E2E activation via use_e2e_planning flag
  - Automatic CARLA map name derivation from map_path
  - Mixed Python/C++ package architecture
  Related to autoware_tensorrt_vad integration for end-to-end autonomous driving.
  * fix(autoware_carla_interface): enhance map loading and world initialization
  Improve reliability and debugging of CARLA world initialization:
  Launch configuration:
  - Fix map path extraction to handle trailing slashes properly
  - Add debug logging to display map path resolution
  World initialization:
  - Add 2-second synchronization delay after load_world() to ensure full map loading
  - Verify world readiness by attempting tick() before accessing settings
  - Handle tick() failure gracefully in asynchronous mode
  These changes prevent race conditions when loading non-default maps that
  require additional initialization time.
  * refactor(autoware_carla_interface): reduce cyclomatic complexity in load_world
  Extract spawn point parsing and traffic manager setup into separate
  methods to improve code maintainability and reduce complexity.
  - Add _parse_spawn_point() method to handle spawn point string parsing
  - Add _setup_traffic_manager() method to configure traffic manager
  - Reduce load_world() cyclomatic complexity from 12 to 3
  * fix(autoware_carla_interface): format launch file to comply with prettier
  * fix(autoware_carla_interface): add cspell ignore for trafficmanager
  Fix CI spell-check failure by adding inline cspell ignore comment
  for CARLA API method name `get_trafficmanager`.
  ---------
* Contributors: Max-Bin, Mete Fatih Cırıt, SakodaShintaro, github-actions

0.50.0 (2026-02-14)
-------------------

0.49.0 (2025-12-30)
-------------------

0.48.0 (2025-11-18)
-------------------
* Merge remote-tracking branch 'origin/main' into humble
* fix: carla interface config and docs (`#11571 <https://github.com/autowarefoundation/autoware_universe/issues/11571>`_)
  * fix(autoware_carla_interface): correct config file installation paths
  Fix setup.py to install sensor_mapping.yaml in config/ subdirectory
  instead of root share directory. This ensures the package works correctly
  in production/deployment scenarios where source files are not available.
  - raw_vehicle_cmd_converter.param.yaml: installed to share root (correct)
  - sensor_mapping.yaml: installed to share/config/ (matches expected path)
  Without this fix, the package relies on fallback to source directory which
  fails in Docker containers and binary package deployments.
  * fix(autoware_carla_interface): complete sensor mapping example in README
  Update camera sensor mapping example to include all required fields:
  - Add topic_info field for camera_info topic
  - Add qos_profile field for ROS2 QoS configuration
  - Remove unnecessary quotes from YAML values
  These fields are essential for proper camera sensor configuration and were
  missing from the documentation example, potentially causing confusion for
  users trying to configure custom sensors.
  * docs(autoware_carla_interface): remove misleading LiDAR concatenation note
  Remove the note about uncommenting LiDAR concatenation relay from Known Issues
  section. The single LiDAR configuration may still require the concatenated topic
  for coordinate transformation, which will be tested separately.
  The relay in launch file remains commented out pending further testing.
  * refactor(autoware_carla_interface): remove unnecessary lidar concatenation relay
  Remove commented-out lidar concatenation relay from launch file. Testing confirms
  that the main Autoware sensing pipeline already provides the concatenated pointcloud
  topic through the mirror_cropped pipeline, making this relay redundant.
  The /sensing/lidar/concatenated/pointcloud topic is successfully published by
  the main sensing stack and consumed by localization and perception modules.
  ---------
* feat(autoware_carla_interface): sensor kit integration with multi-camera support (`#11471 <https://github.com/autowarefoundation/autoware_universe/issues/11471>`_)
  * feat(autoware_carla_interface): add sensor kit integration with multi-camera support
  - Add sensor_mapping.yaml for sensor kit configuration loader
  - Implement coordinate transformer for ROS base_link to CARLA vehicle frame
  - Add sensor kit loader with Autoware calibration parsing
  - Create multi-camera combiner node for 6-camera grid visualization
  - Add topic relays and image compression for all 6 cameras
  - Implement modular sensor manager with ROS publisher management
  - Refactor carla_ros.py to use new sensor kit architecture
  This enables dynamic sensor configuration from Autoware sensor kit YAML files
  and provides compressed image streams for bandwidth optimization.
  * refactor(autoware_carla_interface): remove legacy objects.json and simplify sensor configuration
  - Remove objects.json and objects_definition_file parameter (replaced by sensor_mapping.yaml)
  - Remove use_autoware_sensor_kit parameter (always use sensor kit now)
  - Simplify sensor loading logic in carla_ros.py
  - Update sensor_kit_loader to always attempt sensor kit calibration first
  - Update launch file and documentation to reflect new configuration
  - Make wheelbase configurable via sensor_mapping.yaml
  This simplifies the configuration by removing redundant parameters and
  making the sensor kit approach the standard method.
  * refactor(autoware_carla_interface): move multi_camera_combiner to proper module structure
  - Move scripts/multi_camera_combiner.py to src/autoware_carla_interface/multi_camera_combiner_node.py
  - Register as entry point in setup.py for standard ROS2 node deployment
  - Update launch file to use node instead of executable
  - Remove empty scripts directory
  This follows ROS2 Python package best practices by having all executable
  nodes as entry points rather than standalone scripts.
  * fix(autoware_carla_interface): prevent silent sensor misconfiguration at origin
  Critical fixes for sensor kit loading:
  1. Fix default sensor_kit_name pointing to wrong package
  - Changed from "carla_sensor_kit_launch" to "carla_sensor_kit_description"
  - The _description package contains the required calibration files
  - Launch package does not have sensor transforms
  2. Remove dangerous silent fallback to mapping-only configuration
  - Mapping file has NO transform data, would spawn all sensors at (0,0,0)
  - Now fails fast with clear error message instead of broken sensor layout
  - Prevents silent sensor misconfiguration that appears to work but is broken
  3. Add validation for sensor transforms
  - Verify all enabled sensors have transform calibration data
  - Fail with descriptive error if transforms are missing
  - Prevents degenerate sensor configurations
  4. Remove unused _create_configs_from_mapping() method
  - No longer needed as mapping-only mode is not supported
  - Mapping file is only used for topics/QoS/parameters, not poses
  Without these fixes, sensor kit lookup failures would silently create a broken
  configuration with all cameras/LiDAR at the vehicle origin looking forward.
  * refactor(autoware_carla_interface): improve code quality and sensor configuration
  This commit enhances the overall code quality, error handling, and
  documentation of the CARLA-Autoware interface. Key improvements include:
  Core Refactoring:
  - Migrate all sensors to registry-based publishing (GNSS/pose included)
  - Remove legacy frequency tracking in favor of simulation-time based system
  - Delete obsolete sensor_kit_parser.py module
  - Convert SensorWrapper._sensors_list from class to instance variable
  Error Handling & Validation:
  - Add comprehensive YAML validation with detailed error messages
  - Improve sensor kit package discovery with better fallback logic
  - Add safe error handling in sensor setup with graceful degradation
  - Enhance sensor ID validation to prevent silent misconfigurations
  Code Quality:
  - Add proper shebangs (#!/usr/bin/env python3) to Python modules
  - Fix corrupted license comments
  - Add comprehensive docstrings to all major methods
  - Document thread safety issues in ROS spin thread
  - Improve shutdown procedure to prevent publisher/thread leaks
  Configuration & Standards:
  - Standardize angle units (radians) per Autoware conventions
  - Remove angle unit auto-detection heuristic
  - Update default use_traffic_manager to False
  - Clarify GNSS covariance matrix documentation
  Documentation:
  - Add detailed sensor configuration guide to README
  - Document sensor kit calibration file structure
  - Add CARLA sensor parameter references
  - Remove completed TODO items from known limitations
  * fix(autoware_carla_interface): implement thread safety with proper locking
  Add threading.Lock to protect shared state accessed by both ROS spin thread
  and main simulation loop. This fixes critical race conditions that could cause:
  - Control commands being partially applied
  - Stale pose data being used
  - Potential crashes from concurrent actor access
  Changes:
  - Add self._state_lock (threading.Lock) to protect shared variables
  - Protect control_callback: locks when writing current_control
  - Protect initialpose_callback: locks when accessing/modifying ego_actor
  - Protect pose(): locks when reading ego_actor transform
  - Protect ego_status(): locks when reading all ego_actor state
  - Protect run_step(): locks when reading current_control
  - Update documentation to reflect thread-safe implementation
  - Remove outdated TODO comment about adding synchronization
  Protected shared state:
  - self.current_control (written by ROS callback, read by simulation loop)
  - self.ego_actor (written by initialpose callback, read everywhere)
  - self.physics_control (accessed by control callback)
  - self.timestamp (read/written from both threads)
  * fix(autoware_carla_interface): add robust cleanup and exception handling
  Implement comprehensive resource cleanup to prevent actor leaks and ensure
  proper shutdown even when exceptions occur or signals are received.
  Changes:
  Main Entry Point (carla_autoware.py):
  - Add try/finally block to ensure cleanup on all exit paths
  - Register both SIGINT and SIGTERM signal handlers
  - Add exception handling for KeyboardInterrupt
  - Improve _cleanup() with individual try/except blocks
  - Cleanup resources in reverse initialization order
  - Continue cleanup even if individual steps fail
  Sensor Cleanup (carla_wrapper.py):
  - Improve SensorWrapper.cleanup() robustness
  - Separate stop() and destroy() calls with individual error handling
  - Collect and log cleanup errors without failing entire cleanup
  - Ensure all sensors are cleaned up even if some fail
  - Clear sensors list after cleanup
  Benefits:
  - Prevents CARLA actor leaks on crashes or Ctrl+C
  - Ensures ROS shutdown happens properly
  - No hanging processes or zombie actors
  - Cleaner error messages during shutdown
  - Resources freed even on partial failures
  * fix(autoware_carla_interface): protect timestamp write with lock in run_step
  Fix critical race condition where main thread writes self.timestamp without
  lock while ROS callback thread reads it (via first_order_steering) with lock.
  Issue:
  - run_step() wrote self.timestamp = timestamp WITHOUT acquiring _state_lock
  - control_callback() calls first_order_steering() WHILE holding _state_lock
  - first_order_steering() reads self.timestamp to calculate dt
  - Race: main thread writes, ROS thread reads -> corrupt timestamp value
  - Result: negative or zero dt, unstable steering calculations
  Fix:
  - Wrap timestamp write in run_step() with self._state_lock
  - Now both read and write are properly synchronized
  - Prevents partially-updated timestamp values
  - Ensures stable dt calculations in steering model
  Protected flow:
  1. Main thread: WITH lock, write self.timestamp
  2. ROS thread: WITH lock, read self.timestamp (in first_order_steering)
  3. Both threads use same lock = no race condition
  * fix(autoware_carla_interface): fix None timestamp crash and remove dead code
  Fix two code quality issues:
  1. Fix TypeError crash in first_order_steering when control arrives early
  - Issue: If control command arrives before first run_step(), self.timestamp is None
  - Symptom: TypeError on line 626: unsupported operand type(s) for -: 'NoneType' and 'NoneType'
  - Fix: Guard against None timestamp, return raw steering input until initialized
  - Prevents crash during startup race condition
  2. Remove unused self.channels dead code
  - self.channels was initialized to 0 but never read or written
  - Leftover from previous implementation
  - Removing improves code clarity
  Changes:
  - Add None check at start of first_order_steering()
  - Return unfiltered input when timestamp not yet available
  - Add docstring explaining graceful degradation behavior
  - Remove self.channels from _initialize_instance_variables()
  * fix(autoware_carla_interface): preserve filter state on repeated commands
  Fix critical bug in first_order_steering that caused steering spikes when
  multiple control commands arrived within the same CARLA simulation tick.
  Issue:
  - Planning/control can publish multiple actuation commands per tick
  - Each command with dt=0 would reset steer_output to 0.0
  - We then stored 0.0 in prev_steer_output, losing filter state
  - Result: Artificial steering spike to zero before recovering next tick
  Root cause:
  ```python
  steer_output = 0.0  # Reset to zero
  dt = self.timestamp - self.prev_timestamp
  if dt > 0.0:
  steer_output = computed_value  # Only set if dt > 0
  self.prev_steer_output = steer_output  # Store 0.0 if dt <= 0!
  ```
  Fix:
  - When dt <= 0 (repeated commands same frame): early return prev_steer_output
  - Preserves filter state across intra-frame command bursts
  - Eliminates zero spikes from state loss
  - Only update filter state when time actually advances (dt > 0)
  Also improved first call initialization:
  - Initialize prev_steer_output to first input (not implicit 0.0)
  - Cleaner logic flow with early returns
  * chore(autoware_carla_interface): fix pre-commit linting issues
  - Remove unused imports (datetime, math)
  - Fix docstrings to use imperative mood
  - Remove redundant YAML quotes for yamllint compliance
  * docs(autoware_carla_interface): add multi-camera view setup instructions
  - Add new section explaining how to view combined 6-camera feed in RViz
  - Provide step-by-step instructions to add Image display manually
  - Include note about optionally disabling multi_camera_combiner node
  - Reference screenshot at docs/images/rviz_multi_camera_view.png
  - Avoids maintaining custom RViz config, uses default Autoware config
  * docs(autoware_carla_interface): improve README with updated commands and clearer structure
  - Update launch command to use sensor_model:=carla_sensor_kit
  - Format launch command as multiline for better readability
  - Reorganize Install section with clearer Prerequisites and Map Setup subsections
  - Add emphasis on carla_sensor_kit in Sensor Configuration section
  - Improve Known Issues section formatting
  - Add note about LiDAR concatenation configuration
  * style(pre-commit): autofix
  * refactor(sensor_kit_loader): reduce method complexity to pass CodeScene checks
  Extract helper methods to reduce complexity and improve code health:
  1. load_sensor_mapping (66 → 22 lines):
  - Extract _resolve_mapping_file_path for file path resolution
  - Extract _validate_sensor_mapping_yaml for YAML validation
  - Extract _load_vehicle_config for vehicle config loading
  2. find_sensor_kit_path (65 → 43 lines):
  - Extract _try_find_sensor_kit to eliminate duplicate try-except blocks
  - Extract _create_not_found_error for error message generation
  - Reduce nesting depth from 4 to 2 levels
  3. _create_configs_from_kit (40 → 27 lines):
  - Extract _try_create_sensor_config for single sensor processing
  - Extract _find_sensor_mapping to eliminate nested loop
  - Reduce cyclomatic complexity
  Benefits:
  - All methods now under 30 lines (Large Method fixed)
  - Maximum nesting depth reduced to 2 (Deep Complexity fixed)
  - Each method has single responsibility (Complexity fixed)
  - Eliminates code duplication (DRY principle)
  Addresses CodeScene quality gate failures.
  * style(pre-commit): autofix
  * refactor(autoware_carla_interface): improve code quality and reduce complexity
  - Fix spelling issues: replace 'republishers' with 'Image transport nodes', correct 'ROS2' to 'ROS 2'
  - Consolidate duplicated publisher creation methods into single generic helper function
  - Reduce nested complexity in normalize_sensor_name from 4 to 2 levels using early returns
  - Flatten conditional logic in find_sensor_kit_path to reduce bumpy road complexity
  - Split _validate_sensor_mapping_yaml into focused validation methods
  - Extract wheelbase validation into dedicated method
  These changes reduce cyclomatic complexity, eliminate code duplication, and improve maintainability while preserving functionality.
  * style(pre-commit): autofix
  * refactor(autoware_carla_interface): reduce complexity and improve code health
  Major refactoring to address CodeScene quality gate violations:
  **Bumpy Road Issues Fixed:**
  - carla_wrapper.py: Extract cleanup logic into _cleanup_single_sensor method
  - carla_autoware.py: Split _cleanup into 4 focused methods (_cleanup_sensors,
  _cleanup_ros_interface, _cleanup_ego_actor, _cleanup_carla_provider)
  - Eliminates all nested conditional blocks in cleanup paths
  **Complex Method Improvements:**
  - carla_wrapper.py: Reduce setup_sensors cyclomatic complexity from 13 to 4
  - Extract _setup_single_sensor for per-sensor setup logic
  - Create dedicated attribute configurers per sensor type:
  _configure_camera_attributes, _configure_lidar_attributes,
  _configure_gnss_attributes, _configure_imu_attributes
  - Extract _create_sensor_transform for transform creation
  - carla_ros.py: Extract _create_gnss_covariance_matrix from pose method
  - Reduces pose method from 70 to 40 lines
  **Code Duplication Elimination:**
  - coordinate_transformer.py: Consolidate rotation conversion methods
  - Create _create_carla_rotation helper with angle negation parameter
  - Eliminates 20+ lines of duplicated code between
  carla_rotation_to_carla_rotation and ros_to_carla_rotation
  All refactoring maintains 100% functional equivalence while significantly
  improving maintainability, testability, and code health metrics.
  * style(pre-commit): autofix
  * refactor(autoware_carla_interface): fix remaining CodeScene quality issues
  Address final CodeScene advisory quality gate failures:
  **Code Duplication & Excess Arguments (coordinate_transformer.py):**
  - Inline rotation conversion logic into carla_rotation_to_carla_rotation
  and ros_to_carla_rotation methods
  - Remove _create_carla_rotation helper with 5 arguments (exceeded 4 arg limit)
  - Methods now sufficiently distinct to avoid duplication warnings:
  - carla_rotation_to_carla_rotation: No angle negation
  - ros_to_carla_rotation: Negates pitch and yaw for coordinate system change
  **Complex Method (carla_ros.py):**
  - Reduce camera method cyclomatic complexity from 9 to 3
  - Extract _create_camera_image_message for image conversion
  - Extract _prepare_camera_info for camera info preparation
  - Extract _publish_camera_messages for publishing logic
  All changes maintain functional equivalence while improving code health metrics
  to pass CodeScene quality gates.
  * refactor(carla_interface): fix CodeScene quality gates and CI test import
  Fix CodeScene quality gate failures and CI test collection error:
  Quality Improvements:
  - Move GNSS covariance configuration from hardcoded matrix to YAML config
  - Reduce carla_ros.py complexity by externalizing covariance parameters
  - Eliminate code duplication in coordinate_transformer.py rotation methods
  - Extract common rotation conversion logic into helper method
  Technical Changes:
  - Add covariance field to SensorConfig dataclass
  - Update sensor_mapping.yaml with position_variance and orientation_variance
  - Refactor _create_gnss_covariance_matrix() to read from sensor config
  - Create _convert_rotation_to_carla() helper for shared rotation logic
  - Add try/except import guard for carla module to support test environments
  Impact:
  - Resolves "Lines of Code in a Single File" violation in carla_ros.py
  - Resolves "Code Duplication" violation in coordinate_transformer.py
  - Fixes ModuleNotFoundError during pytest test collection in CI
  - Improves maintainability and configurability
  * style(pre-commit): autofix
  * refactor(autoware_carla_interface): resolve CodeScene code health warnings
  Improves code maintainability by addressing CodeScene quality metrics:
  - Reduce function arguments in coordinate_transformer by grouping angles
  - Simplify conditional logic in GNSS covariance matrix creation
  - Consolidate redundant code and remove unnecessary comments
  - Reduce overall lines of code to improve readability
  This brings the codebase below CodeScene thresholds for complexity and file size.
  * refactor(autoware_carla_interface): consolidate duplicate docstrings in coordinate_transformer
  Reduces code duplication by consolidating verbose docstrings to concise
  one-liners for rotation conversion methods. Both carla_rotation_to_carla_rotation
  and ros_to_carla_rotation now have minimal documentation, as the detailed
  documentation is maintained in the shared _convert_rotation_to_carla helper.
  Also adds 'from __future_\_ import annotations' to enable deferred annotation
  evaluation (PEP 563), which fixes AttributeError during test imports when
  CARLA is not available (carla = None).
  This resolves CodeScene code duplication warning while preserving all
  functional information and fixing test compatibility.
  ---------
  Co-authored-by: pre-commit-ci-lite[bot] <117423508+pre-commit-ci-lite[bot]@users.noreply.github.com>
* chore(carla-interface): fix spell-check (`#11400 <https://github.com/autowarefoundation/autoware_universe/issues/11400>`_)
  chore: fix spell-check
* fix(autoware_carla_interface): improve QoS compatibility and pointcloud handling (`#11372 <https://github.com/autowarefoundation/autoware_universe/issues/11372>`_)
* Contributors: Bingo, Kotaro Uetake, Max-Bin, Ryohsuke Mitsudome

0.47.1 (2025-08-14)
-------------------

0.47.0 (2025-08-11)
-------------------
* style(pre-commit): autofix (`#10982 <https://github.com/autowarefoundation/autoware_universe/issues/10982>`_)
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* Contributors: Ryohsuke Mitsudome

0.46.0 (2025-06-20)
-------------------

0.45.0 (2025-05-22)
-------------------

0.44.2 (2025-06-10)
-------------------

0.44.1 (2025-05-01)
-------------------

0.44.0 (2025-04-18)
-------------------

0.43.0 (2025-03-21)
-------------------
* Merge remote-tracking branch 'origin/main' into chore/bump-version-0.43
* chore: rename from `autoware.universe` to `autoware_universe` (`#10306 <https://github.com/autowarefoundation/autoware_universe/issues/10306>`_)
* Contributors: Hayato Mizushima, Yutaka Kondo

0.42.0 (2025-03-03)
-------------------

0.41.2 (2025-02-19)
-------------------
* chore: bump version to 0.41.1 (`#10088 <https://github.com/autowarefoundation/autoware_universe/issues/10088>`_)
* Contributors: Ryohsuke Mitsudome

0.41.1 (2025-02-10)
-------------------

0.41.0 (2025-01-29)
-------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* fix(autoware_carla_interface): fix lidar topic name (`#9645 <https://github.com/autowarefoundation/autoware_universe/issues/9645>`_)
* Contributors: Fumiya Watanabe, Maxime CLEMENT

0.40.0 (2024-12-12)
-------------------
* Merge branch 'main' into release-0.40.0
* Revert "chore(package.xml): bump version to 0.39.0 (`#9587 <https://github.com/autowarefoundation/autoware_universe/issues/9587>`_)"
  This reverts commit c9f0f2688c57b0f657f5c1f28f036a970682e7f5.
* fix: fix ticket links in CHANGELOG.rst (`#9588 <https://github.com/autowarefoundation/autoware_universe/issues/9588>`_)
* chore(package.xml): bump version to 0.39.0 (`#9587 <https://github.com/autowarefoundation/autoware_universe/issues/9587>`_)
  * chore(package.xml): bump version to 0.39.0
  * fix: fix ticket links in CHANGELOG.rst
  * fix: remove unnecessary diff
  ---------
  Co-authored-by: Yutaka Kondo <yutaka.kondo@youtalk.jp>
* fix: fix ticket links in CHANGELOG.rst (`#9588 <https://github.com/autowarefoundation/autoware_universe/issues/9588>`_)
* fix(autoware_carla_interface): include "modules" submodule in release package and update setup.py (`#9561 <https://github.com/autowarefoundation/autoware_universe/issues/9561>`_)
* feat!: replace tier4_map_msgs with autoware_map_msgs for MapProjectorInfo (`#9392 <https://github.com/autowarefoundation/autoware_universe/issues/9392>`_)
* refactor: correct spelling (`#9528 <https://github.com/autowarefoundation/autoware_universe/issues/9528>`_)
* 0.39.0
* update changelog
* fix: fix ticket links to point to https://github.com/autowarefoundation/autoware_universe (`#9304 <https://github.com/autowarefoundation/autoware_universe/issues/9304>`_)
* fix: fix ticket links to point to https://github.com/autowarefoundation/autoware_universe (`#9304 <https://github.com/autowarefoundation/autoware_universe/issues/9304>`_)
* chore(package.xml): bump version to 0.38.0 (`#9266 <https://github.com/autowarefoundation/autoware_universe/issues/9266>`_) (`#9284 <https://github.com/autowarefoundation/autoware_universe/issues/9284>`_)
  * unify package.xml version to 0.37.0
  * remove system_monitor/CHANGELOG.rst
  * add changelog
  * 0.38.0
  ---------
* Contributors: Esteve Fernandez, Fumiya Watanabe, Jesus Armando Anaya, M. Fatih Cırıt, Ryohsuke Mitsudome, Yutaka Kondo

0.39.0 (2024-11-25)
-------------------
* fix: fix ticket links to point to https://github.com/autowarefoundation/autoware_universe (`#9304 <https://github.com/autowarefoundation/autoware_universe/issues/9304>`_)
* fix: fix ticket links to point to https://github.com/autowarefoundation/autoware_universe (`#9304 <https://github.com/autowarefoundation/autoware_universe/issues/9304>`_)
* chore(package.xml): bump version to 0.38.0 (`#9266 <https://github.com/autowarefoundation/autoware_universe/issues/9266>`_) (`#9284 <https://github.com/autowarefoundation/autoware_universe/issues/9284>`_)
  * unify package.xml version to 0.37.0
  * remove system_monitor/CHANGELOG.rst
  * add changelog
  * 0.38.0
  ---------
* Contributors: Esteve Fernandez, Yutaka Kondo

0.38.0 (2024-11-08)
-------------------
* unify package.xml version to 0.37.0
* fix(autoware_carla_interface): resolve init file error and colcon marker warning (`#9115 <https://github.com/autowarefoundation/autoware_universe/issues/9115>`_)
* fix: removed access to unused ROS_VERSION environment variable. (`#8896 <https://github.com/autowarefoundation/autoware_universe/issues/8896>`_)
* ci(pre-commit): autoupdate (`#7630 <https://github.com/autowarefoundation/autoware_universe/issues/7630>`_)
  * ci(pre-commit): autoupdate
  * style(pre-commit): autofix
  * fix: remove the outer call to dict()
  ---------
  Co-authored-by: github-actions <github-actions@github.com>
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: mitsudome-r <ryohsuke.mitsudome@tier4.jp>
* feat(carla_autoware): add interface to easily use CARLA with Autoware (`#6859 <https://github.com/autowarefoundation/autoware_universe/issues/6859>`_)
  Co-authored-by: Minsu Kim <minsu@korea.ac.kr>
* Contributors: Esteve Fernandez, Giovanni Muhammad Raditya, Jesus Armando Anaya, Yutaka Kondo, awf-autoware-bot[bot]

0.26.0 (2024-04-03)
-------------------
