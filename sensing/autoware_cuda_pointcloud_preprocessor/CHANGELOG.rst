^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_cuda_pointcloud_preprocessor
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.53.0 (2026-09-29)
-------------------
* Merge remote-tracking branch 'origin/main' into prepare-0.53.0-changelog
* fix(cuda_pointcloud_preprocessor): link CUDA utilities in test (`#13435 <https://github.com/autowarefoundation/autoware_universe/issues/13435>`_)
  The test compiles CUDA utility headers with the host compiler. Link the exported target so CUDA 13 CCCL headers are available.
* perf(autoware_cuda_pointcloud_preprocessor): process points in ring order instead of an organized layout (`#13412 <https://github.com/autowarefoundation/autoware_universe/issues/13412>`_)
* test(cuda_pointcloud_preprocessor): cover capacity bounds behavior (`#13137 <https://github.com/autowarefoundation/autoware_universe/issues/13137>`_)
  * perf(cuda_pointcloud_preprocessor): avoid runtime thrust allocations
  Thrust's algorithm calls allocate temporary device memory on every frame.
  Replace them with explicit kernels and CUB calls running on storage that is
  allocated once at startup:
  - thrust_stream::fill / fill_n become a fill kernel launched on the stream
  - thrust::inclusive_scan becomes cub::DeviceScan::InclusiveSum
  - thrust_stream::count becomes cub::DeviceReduce::Sum over a transform
  iterator, accumulating the three diagnostic counters into one device
  buffer that is copied back in a single transfer
  The CUB calls share a single scratch workspace, sized at startup to the
  largest requirement among the sort, the scan and the reductions. The twist
  struct counts become locals, since they are only used within process().
  The counting iterator is thrust::transform_iterator rather than
  cub::TransformInputIterator, since CCCL 3.0 (CUDA 13, used on jazzy) removed
  the latter along with its header.
  * refactor(cuda_pointcloud_preprocessor): name the processing stat indices
  Hoist the device_processing_stats\_ layout into class-scope constants so that
  the buffer sizing in initializeBuffers() and the readback in process() share
  one definition instead of repeating a bare 3.
  Co-Authored-By: Claude Opus 5 (1M context) <noreply@anthropic.com>
  * refactor(cuda_pointcloud_preprocessor): clarify queue update flow
  The twist and IMU queue bookkeeping was spread over the insertion callbacks
  and a pair of bounding helpers, so capacity was enforced after insertion and
  the pruning rules were duplicated per queue. Move the bookkeeping into
  detail:: helpers in a new queue_bounds.hpp and give each queue one update
  path:
  - prepare_queue_update() prunes queue entries older than the current
  pointcloud, drops and counts incoming messages beyond the free capacity,
  and sorts the remainder, so capacity is enforced before insertion
  - twistCallback()/imuCallback() become insertTwistMessage()/
  insertImuMessage(), which now only insert in stamp order
  - boundTwistQueue()/boundImuQueue() are gone, since nothing is inserted
  beyond capacity any more
  Backward time jump handling moves to detail::is_backward_time_jump() and now
  tolerates jumps of up to one second: the queue is cleared only when its
  oldest entry is more than one second ahead of the incoming stamp, rather than
  on any backward step. The old per-entry pop for queues spanning more than a
  second is dropped, since prepare_queue_update() already prunes everything
  older than the current pointcloud.
  * refactor(cuda_pointcloud_preprocessor): address queue_bounds review comments
  - Convert builtin_interfaces/Time to nanoseconds from its own fields, which
  drops the rclcpp dependency from queue_bounds.hpp entirely. A negative
  `sec` throws rather than wrapping, which is what rclcpp::Time did too
  - Document and assert that prune_old_queue_entries() takes a stamp-sorted
  queue and leaves it sorted
  - Document what prepare_queue_update() does and the state it leaves its two
  containers in
  Co-Authored-By: Claude Opus 5 (1M context) <noreply@anthropic.com>
  * test(cuda_pointcloud_preprocessor): cover capacity bounds behavior
  Add a gtest suite for the behavior introduced by the preceding commits in
  this stack:
  - CudaPointcloudPreprocessor accepts positive capacities and rejects
  non-positive ones
  - points exactly at the ring and points-per-ring capacities are processed
  without reporting an overflow, and exceeding either reports one
  - input clouds longer than max_input_point_count are truncated
  - detail::prepare_queue_update() prunes, sorts, bounds and counts drops for
  both the twist and IMU queues, and rejects an already over-capacity queue
  - detail::is_backward_time_jump() honors the one second threshold
  The cases that construct a CudaPointcloudPreprocessor allocate device memory,
  so they use autoware::cuda_utils::CudaTest and self-skip where no CUDA device
  is present; CI containers run without one. The capacity validation and queue
  bound cases are pure host code and always run.
  * fix(cuda_pointcloud_preprocessor): link the test dependencies properly
  `LINKER:--no-as-needed` on the gtest target was papering over two libraries
  that never declared the dependencies they use:
  - `cuda_pointcloud_preprocessor_lib` calls into cuda_blackboard but did not
  link it
  - `concatenate_data` calls the memory utilities that live in
  `pointcloud_preprocessor_filter_base` but did not link it
  Both now link what they use, so `libconcatenate_data.so` and
  `libcuda_pointcloud_preprocessor_lib.so` carry the DT_NEEDED entries they were
  missing. The test target only has to name `cuda_pointcloud_preprocessor_lib`,
  and the linker resolves the rest on its own with `--as-needed` left at its
  default.
  Also split the ring capacity boundary case into one case per capacity that
  `ring_overflow` reports, and pass the ring of every test point explicitly, so
  which of the two limits each case exercises is visible at the call site.
  Co-Authored-By: Claude Opus 5 (1M context) <noreply@anthropic.com>
  * chore: empty commit to nudge the pull request diff
  * refactor(cuda_pointcloud_preprocessor): report ring organization overflow as flags
  `organizeKernel` recorded the maximum *offending* ring index and ring offset
  via `atomicMax`, but the magnitudes never left `process()`: the only sink is
  the `bool ProcessingStats::ring_overflow`, so both were effectively flags
  whose names promised observed maxima. That mismatch made the `>=` comparisons
  against the capacities read like off-by-one errors.
  Raise the two outputs as flags and name them accordingly. The reported
  `ring_overflow` is unchanged for every input.
  Co-Authored-By: Claude Opus 5 (1M context) <noreply@anthropic.com>
  ---------
  Co-authored-by: Claude Opus 5 (1M context) <noreply@anthropic.com>
  Co-authored-by: Manato Hirabayashi <3022416+manato@users.noreply.github.com>
* fix(autoware_cuda_pointcloud_preprocessor): stamp synchronized clouds with oldest_stamp (`#13318 <https://github.com/autowarefoundation/autoware_universe/issues/13318>`_)
  The concatenated cloud already uses oldest_stamp, so downstream nodes pairing the
  two could only match the sensor that carried the oldest stamp. This matches the
  CPU implementation in autoware_pointcloud_preprocessor.
  Co-authored-by: kyo0221 <kyo.yamashita@tier4.jp>
  Co-authored-by: Kento Yabuuchi <kento.yabuuchi.2@tier4.jp>
* fix(autoware_cuda_pointcloud_preprocessor): stop over-allocating the concatenated cloud by the lidar count (`#13287 <https://github.com/autowarefoundation/autoware_universe/issues/13287>`_)
  fix(autoware_cuda_pointcloud_preprocessor): stop over-allocating the concatenated cloud
  max_concat_pointcloud_size\_ is derived from total_data_size, which is already the sum
  over every entry of topic_to_cloud_map, i.e. the size of the whole concatenated cloud.
  Both allocation sites then multiplied it by input_topics\_.size() again, so the buffer
  was allocated N times larger than the data it holds -- 8x on an X2 with eight lidars.
  The guard above the allocation already grows max_concat_pointcloud_size\_ whenever
  total_data_size exceeds it, so the buffer is still always large enough for the frame
  being written; only the surplus goes away.
  The cost is paid every frame, because concatenated_cloud_ptr\_ is std::move()d into the
  result and the '!concatenated_cloud_ptr\_' branch therefore always runs. Measured on an
  X2 replaying recorded lidar at 10 Hz: 110.2 MB allocated per frame to hold 13.7 MB of
  points, from a cuda_blackboard pool that is process-wide and does not shrink. Removing
  the multiplier took the pool's reserved memory from 800 MiB to 416 MiB, and peak live
  usage from 649 MiB to 328 MiB, with no change in throughput.
  Note that max_concat_pointcloud_size\_ is also a running maximum that never decreases,
  so a single unusually dense frame raises the cost of every later frame. That is left
  alone here to keep this change minimal.
  Co-authored-by: Claude Opus 5 (1M context) <noreply@anthropic.com>
* refactor(cuda_pointcloud_preprocessor): clarify queue update flow (`#13136 <https://github.com/autowarefoundation/autoware_universe/issues/13136>`_)
  * perf(cuda_pointcloud_preprocessor): avoid runtime thrust allocations
  Thrust's algorithm calls allocate temporary device memory on every frame.
  Replace them with explicit kernels and CUB calls running on storage that is
  allocated once at startup:
  - thrust_stream::fill / fill_n become a fill kernel launched on the stream
  - thrust::inclusive_scan becomes cub::DeviceScan::InclusiveSum
  - thrust_stream::count becomes cub::DeviceReduce::Sum over a transform
  iterator, accumulating the three diagnostic counters into one device
  buffer that is copied back in a single transfer
  The CUB calls share a single scratch workspace, sized at startup to the
  largest requirement among the sort, the scan and the reductions. The twist
  struct counts become locals, since they are only used within process().
  The counting iterator is thrust::transform_iterator rather than
  cub::TransformInputIterator, since CCCL 3.0 (CUDA 13, used on jazzy) removed
  the latter along with its header.
  * refactor(cuda_pointcloud_preprocessor): name the processing stat indices
  Hoist the device_processing_stats\_ layout into class-scope constants so that
  the buffer sizing in initializeBuffers() and the readback in process() share
  one definition instead of repeating a bare 3.
  Co-Authored-By: Claude Opus 5 (1M context) <noreply@anthropic.com>
  * refactor(cuda_pointcloud_preprocessor): clarify queue update flow
  The twist and IMU queue bookkeeping was spread over the insertion callbacks
  and a pair of bounding helpers, so capacity was enforced after insertion and
  the pruning rules were duplicated per queue. Move the bookkeeping into
  detail:: helpers in a new queue_bounds.hpp and give each queue one update
  path:
  - prepare_queue_update() prunes queue entries older than the current
  pointcloud, drops and counts incoming messages beyond the free capacity,
  and sorts the remainder, so capacity is enforced before insertion
  - twistCallback()/imuCallback() become insertTwistMessage()/
  insertImuMessage(), which now only insert in stamp order
  - boundTwistQueue()/boundImuQueue() are gone, since nothing is inserted
  beyond capacity any more
  Backward time jump handling moves to detail::is_backward_time_jump() and now
  tolerates jumps of up to one second: the queue is cleared only when its
  oldest entry is more than one second ahead of the incoming stamp, rather than
  on any backward step. The old per-entry pop for queues spanning more than a
  second is dropped, since prepare_queue_update() already prunes everything
  older than the current pointcloud.
  * refactor(cuda_pointcloud_preprocessor): address queue_bounds review comments
  - Convert builtin_interfaces/Time to nanoseconds from its own fields, which
  drops the rclcpp dependency from queue_bounds.hpp entirely
  - Document and assert that prune_old_queue_entries() takes a stamp-sorted
  queue and leaves it sorted
  - Document what prepare_queue_update() does and the state it leaves its two
  containers in
  Co-Authored-By: Claude Opus 5 (1M context) <noreply@anthropic.com>
  ---------
  Co-authored-by: Claude Opus 5 (1M context) <noreply@anthropic.com>
* perf(cuda_pointcloud_preprocessor): avoid runtime thrust allocations (`#13135 <https://github.com/autowarefoundation/autoware_universe/issues/13135>`_)
  * perf(cuda_pointcloud_preprocessor): avoid runtime thrust allocations
  Thrust's algorithm calls allocate temporary device memory on every frame.
  Replace them with explicit kernels and CUB calls running on storage that is
  allocated once at startup:
  - thrust_stream::fill / fill_n become a fill kernel launched on the stream
  - thrust::inclusive_scan becomes cub::DeviceScan::InclusiveSum
  - thrust_stream::count becomes cub::DeviceReduce::Sum over a transform
  iterator, accumulating the three diagnostic counters into one device
  buffer that is copied back in a single transfer
  The CUB calls share a single scratch workspace, sized at startup to the
  largest requirement among the sort, the scan and the reductions. The twist
  struct counts become locals, since they are only used within process().
  The counting iterator is thrust::transform_iterator rather than
  cub::TransformInputIterator, since CCCL 3.0 (CUDA 13, used on jazzy) removed
  the latter along with its header.
  * refactor(cuda_pointcloud_preprocessor): name the processing stat indices
  Hoist the device_processing_stats\_ layout into class-scope constants so that
  the buffer sizing in initializeBuffers() and the readback in process() share
  one definition instead of repeating a bare 3.
  Co-Authored-By: Claude Opus 5 (1M context) <noreply@anthropic.com>
  ---------
  Co-authored-by: Claude Opus 5 (1M context) <noreply@anthropic.com>
* perf(cuda_pointcloud_preprocessor): allocate fixed GPU buffers at startup (`#13134 <https://github.com/autowarefoundation/autoware_universe/issues/13134>`_)
  * perf(cuda_pointcloud_preprocessor): allocate fixed GPU buffers at startup
  * fix(cuda_pointcloud_preprocessor): address review comments
  - Report ring_overflow in the diagnostics error message, not just in the
  key-value list, since it can be the sole trigger for the ERROR status.
  - Drop the per-call recomputation of num_organized_points\_ in process();
  it is a constant derived from the capacity and is set in
  initializeBuffers().
  * refactor(cuda_pointcloud_preprocessor): derive buffer extents in the initializer list
  num_rings\_, max_points_per_ring\_ and num_organized_points\_ are constants
  derived from the capacity, so initialize them in the member initializer
  list instead of assigning them in initializeBuffers(), which now only
  allocates. Capacity validation moves ahead of the initializer list so the
  derived members are never computed from unvalidated input, and capacity\_
  is declared before them so the list runs in declaration order.
  ---------
* fix(autoware_cuda_pointcloud_preprocessor): check CUDA launch status so errors do not leak between nodes (`#13157 <https://github.com/autowarefoundation/autoware_universe/issues/13157>`_)
  * fix(autoware_cuda_pointcloud_preprocessor): check CUDA launch status so errors do not leak between nodes
  Launching a kernel for an empty pointcloud produces a zero-sized grid, which
  the launch rejects with cudaErrorInvalidConfiguration. The status was never
  checked, so the error stayed pending on the executor thread.
  thrust's launcher returns cudaPeekAtLastError() as the result of its own
  launch, so the next thrust call on that thread reported the leaked error as
  its own failure ("parallel_for failed: <unrelated error>") and, being
  uncaught, aborted the whole component container -- taking down every other
  node in it. Because the concatenator and the preprocessors share the executor
  threads, the reported error pointed at a node that had done nothing wrong.
  - transform_launch(): treat an empty cloud as valid empty data with nothing to
  launch, instead of launching a zero-sized grid, and check the launch status.
  - common_kernels.cu: check the launch status in the four launch wrappers that
  were missing it; organize/outlier/undistort kernels already did this.
  - Check the status of cub::DeviceSegmentedRadixSort::SortKeys, which was
  discarded and would likewise surface at the next unrelated CUDA call.
  - Check the remaining unchecked CUDA calls in the CUDA concatenate handler.
  Co-Authored-By: Claude Opus 5 (1M context) <noreply@anthropic.com>
  * style(pre-commit): autofix
  * Apply suggestions from code review
  Co-authored-by: Max Schmeller <6088931+mojomex@users.noreply.github.com>
  ---------
  Co-authored-by: Claude Opus 5 (1M context) <noreply@anthropic.com>
  Co-authored-by: pre-commit-ci-lite[bot] <117423508+pre-commit-ci-lite[bot]@users.noreply.github.com>
* feat(cuda_pointcloud_preprocessor): add input bounds diagnostics (`#13106 <https://github.com/autowarefoundation/autoware_universe/issues/13106>`_)
  * feat(cuda_pointcloud_preprocessor): add input bounds diagnostics
  * Update sensing/autoware_cuda_pointcloud_preprocessor/src/cuda_pointcloud_preprocessor/cuda_pointcloud_preprocessor_node.cpp
  ---------
* refactor(sensing): move node design files into each package (`#13105 <https://github.com/autowarefoundation/autoware_universe/issues/13105>`_)
* fix(pre-commit): update pre-commit-hooks-ros to v0.10.3 and adapt include guards (`#13083 <https://github.com/autowarefoundation/autoware_universe/issues/13083>`_)
  * chore: sync files
  * fix(pre-commit): adapt include guards to pre-commit-hooks-ros v0.10.3
  ros-include-guard v0.10.3 only recognises an include guard when #endif is the
  last non-empty line of the file, so that feature test macros are no longer
  mistaken for guards. 29 headers failed that check.
  25 headers wrap the guard in "// clang-format off" / "// clang-format on"
  because the #endif comment plus its // NOLINT exceeds the 100 column limit.
  Drop only the trailing "on" marker; the "off" marker then runs to end of file
  and still protects the line from being wrapped.
  3 CUDA headers ended with "/* *INDENT-ON* */". Move it above the #endif so it
  stays paired with the "/* *INDENT-OFF* */" near the top of the file.
  autoware_behavior_path_planner/test/input.hpp closed its guard immediately after
  opening it, leaving the entire body unguarded. Move the #endif to the end.
  Also hold clang-format at v21.1.8. clang-format 22 migrates
  "AlignAfterOpenBracket: AlwaysBreak" to "BreakAfterOpenBracketIf: true", which
  forces a break after every "if (" whose condition does not fit on one line and
  reformats 88 files.
  ---------
  Co-authored-by: github-actions <github-actions@github.com>
  Co-authored-by: Mete Fatih Cırıt <mfc@autoware.org>
* refactor(autoware_pointcloud_preprocessor): extract matching policy and rename collector matcher (`#12872 <https://github.com/autowarefoundation/autoware_universe/issues/12872>`_)
  * refactor(pointcloud_preprocessor): extract matching policy and rename collector matcher
  Extract the pure cloud-to-collector matching logic (naive / advanced) into
  a ROS-runtime-free MatchingPolicy operating on plain structs
  (IncomingCloudInfo, CandidateCollectorState, CollectorReference).
  Rename CollectorMatchingStrategy -> CollectorMatcher; the matcher classes
  become thin wrappers that adapt the ROS-side collectors to the policy and
  hold the node/logging dependency. Rename MatchingParams -> IncomingCloudInfo
  at the node call sites. The CUDA package's matcher is renamed in lockstep
  (it shares the same header).
  No behavioral change.
  * chore: fix variable naming
  ---------
* Contributors: KyoYamashita, Max Schmeller, Mete Fatih Cırıt, Ryohsuke Mitsudome, Taekjin LEE, Yi-Hsiang Fang (Vivid), awf-autoware-bot[bot]

0.52.0 (2026-06-30)
-------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* feat: polar voxel noise filter (`#12496 <https://github.com/autowarefoundation/autoware_universe/issues/12496>`_)
  * feat: polar voxel noise filter
  * chore: added missing destructor and mutex
  * chore: adressing comments about depercated functions
  * chore: adressing comments about unused items and readme
  * chore: adressing comments about pointcloud format and prefix
  * style(pre-commit): autofix
  * chore: adressing new parameter in bounds checking and suffix renamed variables
  * style(pre-commit): autofix
  * chore: adding unit test for the polar voxel filter
  * style(pre-commit): autofix
  * chore: added test on cmakelist all points primary return default
  * style(pre-commit): autofix
  * chore: handling case when pointcloud without return type information
  * chore: pointcloud msg format validation only once to avoid redundancy
  ---------
  Co-authored-by: Yoshi Ri <yoshiyoshidetteiu@gmail.com>
  Co-authored-by: pre-commit-ci-lite[bot] <117423508+pre-commit-ci-lite[bot]@users.noreply.github.com>
  Co-authored-by: Amadeusz Szymko <amadeusz.szymko.2@tier4.jp>
* Contributors: SergioReyesSan, github-actions

0.51.0 (2026-05-01)
-------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* fix(cuda_pointcloud_preprocessor): add missing initialization (`#12493 <https://github.com/mitsudome-r/autoware_universe/issues/12493>`_)
* perf(pointcloud_preprocessor): use emplace/emplace_back to avoid temporary object creation (`#12227 <https://github.com/mitsudome-r/autoware_universe/issues/12227>`_)
* feat(autoware_cuda_pointcloud_preprocessor): cuda 12.0 build compatibility (`#12194 <https://github.com/mitsudome-r/autoware_universe/issues/12194>`_)
  * feat(autoware_cuda_pointcloud_preprocessor): CUDA 12.0+ build compatibility
  * feat: restore Turing arch
  ---------
* docs(sensing): fix mkdocs macro rendering and links in sensing pages (`#12111 <https://github.com/mitsudome-r/autoware_universe/issues/12111>`_)
  docs(sensing): fix mkdocs macro paths, links, and schema fields
* Contributors: Amadeusz Szymko, Manato Hirabayashi, Max Schmeller, github-actions, nishikawa-masaki

0.50.0 (2026-02-14)
-------------------
* Merge remote-tracking branch 'origin/main' into humble
* fix(cuda_polar_voxel_outlier_filter): replace cub::TransformInputIterator with thrust::transform_iterator (`#12069 <https://github.com/autowarefoundation/autoware_universe/issues/12069>`_)
  * fix(cuda_polar_voxel_outlier_filter): replace cub::TransformInputIterator with thrust::transform_iterator
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci-lite[bot] <117423508+pre-commit-ci-lite[bot]@users.noreply.github.com>
* feat(autoware_cuda_pointcloud_preprocessor): update nvcc flags (`#12059 <https://github.com/autowarefoundation/autoware_universe/issues/12059>`_)
* feat: add negative option for cropbox filtering; aligning with CPU cropbox filter (`#11766 <https://github.com/autowarefoundation/autoware_universe/issues/11766>`_)
  * feat: add negative option for cropbox filtering; aligning with CPU cropbox filter
  * change the internal process of negative to also boolean; simplify the logics
  * change the internal process of negative to also boolean; simplify the logics
  * Apply suggestion from @mojomex
  fix true/false
  Co-authored-by: Max Schmeller <6088931+mojomex@users.noreply.github.com>
  * precommit
  ---------
  Co-authored-by: Max Schmeller <6088931+mojomex@users.noreply.github.com>
* fix(agnocast): build on jazzy, remove from ground_segmentation_cuda (`#11960 <https://github.com/autowarefoundation/autoware_universe/issues/11960>`_)
* Contributors: Amadeusz Szymko, Mete Fatih Cırıt, Ryohsuke Mitsudome, Taekjin LEE, Yuxuan Liu

0.49.0 (2025-12-30)
-------------------
* Merge remote-tracking branch 'origin/main' into prepare-0.49.0-changelog
* fix(cuda_pointcloud_preprocessor): cast timestamp value properly (`#11714 <https://github.com/autowarefoundation/autoware_universe/issues/11714>`_)
  * fix(cuda_pointcloud_preprocessor): cast timestamp value properly
  Since `twist.stamp_nsec` and `last_stamp_nsec` are defined as `std::uint32_t`,
  subtraction may cause wrap-around and unexpected behavior if `twist.stamp_nsec < last_stamp_nsec`
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci-lite[bot] <117423508+pre-commit-ci-lite[bot]@users.noreply.github.com>
* Contributors: Manato Hirabayashi, Ryohsuke Mitsudome

0.48.0 (2025-11-18)
-------------------
* Merge remote-tracking branch 'origin/main' into humble
* fix: tf2 uses hpp headers in rolling (and is backported) (`#11620 <https://github.com/autowarefoundation/autoware_universe/issues/11620>`_)
* fix(cuda_pointcloud_preprocessor): use uint64_t and nanoseconds to prevent potential precision loss (`#11398 <https://github.com/autowarefoundation/autoware_universe/issues/11398>`_)
* feat(autoware_cuda_pointcloud_preprocessor): cuda/polar voxel filter (`#11122 <https://github.com/autowarefoundation/autoware_universe/issues/11122>`_)
  * feat(cuda_utils): support device memory allocation from memory pool
  * feat(cuda_pointcloud_preprocessor): support polar_voxel_outlier_filter
  WIP: use cuda::std::optional. compile passed
  wip: version 1
  wip: update
  wip: update launcher
  * feat(cuda_pointcloud_preprocessor): add a flag to enable/disable ring outlier filter in cuda_pointcloud_preprocessor
  * chore: clean up the code
  * docs: update documents briefly
  * style(pre-commit): autofix
  * style(pre-commit): autofix
  * feat(cuda_polar_voxel_outlier_filter): add xyzirc format support
  * fix(cuda_polar_voxel_outlier_filter): move sync point to avoid unexpected memory release during async copy
  * chore(cuda_polar_voxel_outlier_filter): update parameters
  - add SI unit postfix
  - deprecate `secondary_return_type`
  - and think points with non-primary return value as points with secondary return
  * refactor(cuda_polar_voxel_outlier_filter): explicity specify index integer type
  * refactor(cuda_polar_voxel_outlier_filter): snake case for functions
  * refactor(cuda_polar_voxel_outlier_filter): std::optional for visibility and filter ratio
  And update related task functions for diagnostics
  * fix(cuda_polar_voxel_outlier_filter): register parameters_callback
  * refactor(cuda_polar_voxel_outlier_filter): remove log spam and unneccesary comments
  * refactor(cuda_polar_voxel_outlier_filter): rename `valid_points_mask`
  * feat(cuda_polar_voxel_outlier_filter): make noise pointcloud publishing optional
  Because all parameters are now compatible with
  `autoware_pointcloud_preprocessor::polar_voxel_outlier_filter`,
  this commit also removes the parameter files named
  `cuda_polar_voxel_outlier_filter.param.yaml` to avoid duplicated file copying.
  * refactor(cuda_polar_voxel_outlier_filter): simplify by enforcing use of XYZIRC or XYZIRCAEDT
  * feat(cuda_polar_voxel_outlier_filter): limit range in visibility calculation
  * refactor(cuda_polar_voxel_outlier_filter): use array of int for return_type instead of int64
  * refactor(cuda_polar_voxel_outlier_filter): update parameter callback to align with the CPU implementation
  And small clean up the codes
  * fix(cuda_polar_voxel_outlier_filter): return when invalid index
  fix the error revealed by `compute-sanitizer --tool memcheck`
  * feat(cuda_polar_voxel_outlier_filter): add visibility estimation parameters
  And update visibility calculation to align with the CPU implementation
  * feat(cuda_polar_voxel_outlier_filter): add option to not publish a filtered pointcloud (only estimate visibility)
  * fix(cuda_polar_voxel_outlier_filter): ensure zero started positive values for indices
  * feat(cuda_polar_voxel_outlier_filter): add input validation. align diag format to the CPU implementation
  * perf(cuda_polar_voxel_outlier_filter): skip output generation if visualization_estimation_only==true
  * refactor(cuda_polar_voxel_outlier_filter): clean up the code
  * feat(cuda_polar_voxel_outlier_filter): add intensity parameter for secondary returns
  * fix(cuda_polar_voxel_outlier_filter): update param name to align CPU impl.
  * feat(cuda_polar_voxel_outlier_filter): update codes to align CPU impl.
  * fix(cuda_polar_voxel_outlier_filter): correct unintended comparison
  * fix(cuda_polar_voxel_outlier_filter): correct meaningless cast
  * refactor(cuda_polar_voxel_outlier_filter): unify common calculation
  * chore(cuda_polar_voxel_outlier_filter): use auto for CUDA thread index
  As CUDA grid/block/thread indices, such as threadIdx are defined using unsigned
  int, using size_t is overkill
  * docs(cuda_polar_voxel_outlier_filter): add description for numerical discrepancies
  * fix(cuda_polar_voxel_outlier_filter): guard processing if input size is zero
  * fix(cuda_utils): pass stream and memory bool objects by value to follow CUDA API fashion
  * refactor(cuda_polar_voxel_outlier_filter): restrict variables' scope more precisely
  * chore(cuda_polar_voxel_outlier_filter): clean up the code and add comments
  * docs(cuda_polar_voxel_outlier_filter): apply pre-commit update
  * style(pre-commit): autofix
  * chore(cuda_polar_voxel_outlier_filter): fix typos
  * chore(cuda_polar_voxel_outlier_filter): fix typos
  * chore(cuda_polar_voxel_outlier_filter): remove default params in node construction
  * docs: correct schema path and add missing schema
  * refactor(cuda_polar_voxel_outlier_filter): unmark explicit for the zero-parameter constructor
  * refactor(cuda_polar_voxel_outlier_filter): include what I use
  * style(pre-commit): autofix
  * fix(cuda_polar_voxel_outlier_filter): always count valid points for filter_ratio
  * docs: apply sophisticated suggestions from the reviewer
  Co-authored-by: Max Schmeller <6088931+mojomex@users.noreply.github.com>
  * Apply suggestions from code review
  Co-authored-by: Max Schmeller <6088931+mojomex@users.noreply.github.com>
  * feat(cuda_utils): remove default values related to the  memory pool
  * refactor(cuda_polar_voxel_outlier_filter): rename a subscriber for clarity
  * refactor(cuda_polar_voxel_outlier_filter): use unique_ptr::operator bool for nullptr check
  * refactor(cuda_polar_voxel_outlier_filter): make nested condition in one liner
  `lhs.value() != rhs.value()` will not be evaluated if one of `lhs` or `rhs` is
  std::nullopt due to C++ short-circuit rules
  * fix(cuda_polar_voxel_outlier_filter): returns empty results for empty input
  * refactor(cuda_polar_voxel_outlier_filter): remove redundant comments
  * refactor(cuda_polar_voxel_outlier_filter): separate logic into small functions
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: pre-commit-ci-lite[bot] <117423508+pre-commit-ci-lite[bot]@users.noreply.github.com>
  Co-authored-by: Max Schmeller <6088931+mojomex@users.noreply.github.com>
* fix(fusion node): subscribe from concatenation info (`#11258 <https://github.com/autowarefoundation/autoware_universe/issues/11258>`_)
  * chore: rename concatenate info to manager for clearity
  * feat: add reference min max in the concatenated info
  * chore: replace reading from diagnositc to concatenate info
  * fix: qos settting
  * chore: update for cuda pointcloud preprocessor
  * chore: move info to matching strategy
  * chore: clean code
  * feat: move concat info in launcher
  * chore: fix readme
  * feat: sub to concat info in launcher
  * chore: add concat info in irregular launch
  ---------
* build(autoware_cuda_pointcloud_preprocessor): react to ENABLE_AGNOCAST env var (`#11255 <https://github.com/autowarefoundation/autoware_universe/issues/11255>`_)
* Contributors: Manato Hirabayashi, Max Schmeller, Ryohsuke Mitsudome, Tim Clephas, Yi-Hsiang Fang (Vivid)

0.47.1 (2025-08-14)
-------------------

0.47.0 (2025-08-11)
-------------------
* style(pre-commit): update to clang-format-20 (`#11088 <https://github.com/autowarefoundation/autoware_universe/issues/11088>`_)
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* fix(autoware cuda pointcloud preprocessor): trim IMU/twist queues correctly (`#11055 <https://github.com/autowarefoundation/autoware_universe/issues/11055>`_)
  fix(autoware_cuda_pointcloud_preprocessor): keep IMU/twist messages up to the one before the first point's timestamp
* perf(autoware_cuda_pointcloud_preprocessor): execute thrust operations on the node's own CUDA stream (`#10998 <https://github.com/autowarefoundation/autoware_universe/issues/10998>`_)
  * perf(autoware_cuda_pointcloud_preprocessor): replace default thrust calls with ones with explicit cuda stream
  * chore: remove non-functional mempool allocator
  ---------
* chore(autoware_cuda_pointcloud_preprocessor): add code owners (`#11065 <https://github.com/autowarefoundation/autoware_universe/issues/11065>`_)
* feat(autoware_pointcloud_preprocessor): add publisher for concatenated pointcloud meta info (`#10851 <https://github.com/autowarefoundation/autoware_universe/issues/10851>`_)
  * feat(autoware_pointcloud_preprocessor): add publisher for concatenated pointcloud meta info
  * style(pre-commit): autofix
  * feat(autoware_cuda_pointcloud_preprocessor): handle concatenated pointcloud meta info
  * feat(autoware_pointcloud_preprocessor): serialized config of matching strategy
  * feat(autoware_pointcloud_preprocessor): update msg
  * feat(autoware_pointcloud_preprocessor): update msg (2)
  * docs(autoware_pointcloud_preprocessor): add cloud info topic description
  * feat(autoware_pointcloud_preprocessor): add unit tests for cloud info
  * fix(autoware_pointcloud_preprocessor): pre-commit
  * fix(autoware_pointcloud_preprocessor): remove *_struct headers inclusion
  * fix(autoware_pointcloud_preprocessor): check if the matching strategy cannot be enumerated
  * test(autoware_pointcloud_preprocessor): full cloud repr
  * feat(autoware_pointcloud_preprocessor): auto success set & more unit tests
  * feat(autoware_pointcloud_preprocessor): publish info regardless cloud content
  * style(autoware_pointcloud_preprocessor): typo
  * feat(autoware_pointcloud_preprocessor): make update_concatenated_point_cloud_config static for easier integration
  * docs(autoware_pointcloud_preprocessor): typo
  Co-authored-by: Max Schmeller <6088931+mojomex@users.noreply.github.com>
  * fix(autoware_pointcloud_preprocessor): publish cloud info out of condition block
  * fix(autoware_pointcloud_preprocessor): container access with safe bound checking
  * style(autoware_pointcloud_preprocessor): unify naming convention (part 1 - content)
  * style(autoware_pointcloud_preprocessor): unify naming convention (part 2 - files name)
  * style(autoware_pointcloud_preprocessor): naming convention for main API
  * doc(autoware_pointcloud_preprocessor): add docstring
  * feat(autoware_pointcloud_preprocessor): add remap to launch files
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: Max Schmeller <6088931+mojomex@users.noreply.github.com>
* fix(autoware_cuda_pointcloud_preprocessor): ensure type and API safety (`#10987 <https://github.com/autowarefoundation/autoware_universe/issues/10987>`_)
  * fix: return early on invalid pointcloud format
  * chore: add/remove (un)necessary initializer braces
  * chore: remove useless default destructor
  * fix: make layout check inline to comply with ODR
  * fix: check CUDA error for each API call
  * chore: fix most type-related clang-tidy warnings
  * chore: create point fields with less boilerplate
  * chore: change `num\_` fields back to `size_t`
  * change `thrust::count` result variables to `size_t`
  * chore: static_assert that OutputPointType and InputPointType match Autoware point types
  ---------
* Contributors: Amadeusz Szymko, David Wong, Max Schmeller, Mete Fatih Cırıt

0.46.0 (2025-06-20)
-------------------
* Merge remote-tracking branch 'upstream/main' into tmp/TaikiYamada/bump_version_base
* feat(autoware_cuda_pointcloud_preprocessor): agnocast support (`#10812 <https://github.com/autowarefoundation/autoware_universe/issues/10812>`_)
  * feat(autoware_cuda_pointcloud_preprocessor): add Agnocast support for incoming pointclouds
  * chore: make compilable both with and without agnocast
  * style(pre-commit): autofix
  * ci: statisfy cppcheck and cmake_lint
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* feat(autoware_cuda_pointcloud_preprocessor): diagnostic for cuda pointcloud preprocessor (`#10793 <https://github.com/autowarefoundation/autoware_universe/issues/10793>`_)
  * feat: add ring, crop box diag
  * feat: add distortion correction
  * feat: add concat diagnostics
  * chore: remove if debug publisher
  * chore: clean code
  * chore: clean code
  * chore: utilize mask instead of atomicadd
  * chore: count nan points and numbers of point after crop box filter
  * chore: fix schema
  * chore: move to structure
  * chore: update library
  * chore: prefix output
  * chore: chagne shared pointer to const reference
  * chore: use device vector for thrust count
  * chore: fix output pointcloud name
  * chore: add comment
  * chore: reuse function
  * chore: fix merging issue
  * chore: prefix output
  * chore: fix layout
  * chore: add doc comment
  * chore(autoware_cuda_pointcloud_preprocessor): disable uncrustify
  ---------
  Co-authored-by: Max SCHMELLER <max.schmeller@tier4.jp>
* chore(autoware_cuda_pointcloud_preprocessor): add myself as maintainer (`#10809 <https://github.com/autowarefoundation/autoware_universe/issues/10809>`_)
* fix(cuda_pointcloud_preprocessor): ensure ordered twist/imu queues (`#10748 <https://github.com/autowarefoundation/autoware_universe/issues/10748>`_)
  * fix(cuda_pointcloud_preprocessor): ensure ordered twist/imu queues
  * chore: satisfy uncrustify
  * chore: uncrustify and clang-format conflict, disable uncrustify for statement
  ---------
  Co-authored-by: Max SCHMELLER <msc.schmeller@tier4.jp>
* feat(cuda_pointcloud_preprocessor): update filtering parameter and process (`#10555 <https://github.com/autowarefoundation/autoware_universe/issues/10555>`_)
* fix(cuda_pointcloud_preprocessor): reset data when receiving zero siz… (`#10723 <https://github.com/autowarefoundation/autoware_universe/issues/10723>`_)
  * fix(cuda_pointcloud_preprocessor): reset data when receiving zero size pointcloud
  * fix(cuda_pointcloud_preprocessor): hotfix for ghost output
  - insert memory region reset for every iteration
  - judge if each CUDA thread treat valid input point (or not)
  * fix(cuda_pointcloud_preprocessor): apply valid point mask
  * style(pre-commit): autofix
  ---------
  Co-authored-by: Manato HIRABAYASHI <manato.hirabayashi@tier4.jp>
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: Manato Hirabayashi <3022416+manato@users.noreply.github.com>
* fix(autoware_cuda_pointcloud_preprocessor): fix CMakeLists.txt to install cuda_pointcloud_preprocessor library (`#10740 <https://github.com/autowarefoundation/autoware_universe/issues/10740>`_)
  Co-authored-by: Amadeusz Szymko <amadeusz.szymko.2@tier4.jp>
* feat: accelerate voxel filter (`#10566 <https://github.com/autowarefoundation/autoware_universe/issues/10566>`_)
  * feat: add cuda_voxel_grid_downsample_filter
  * refactor(cuda_voxel_grid_downsample_filter): clean up codes
  * fix(cuda_voxel_grid_downsample_filter): suppress warning for arithmetic on pointer to void
  * chore(cuda_voxel_grid_dowmsample_filter): remove debug code
  * fix(cuda_voxel_grid_downsample_filter): support XYZIRC output format
  Set output format to `Cloud XYZIRC` according to the [design
  document](https://autowarefoundation.github.io/autoware-documentation/main/design/autoware-architecture/sensing/data-types/point-cloud/)
  * style(pre-commit): autofix
  * fix(cuda_boxel_grid_downsamle_filter): rearrange package structure
  * chore(cuda_voxel_grid_dowmsample_filter): reuse OutputPointType in parent namespace
  * chore(cuda_voxel_grid_downsample_filter): use macro defined in autoware_cuda_utils for error checking
  * feat(cuda_voxel_grid_downsample_filter): support multiple data types for input intensity
  * fix(cuda_voxel_grid_downsample_filter): cleanup included header files
  * chore: correct comments
  Co-authored-by: Kenzo Lobos Tsunekawa <kenzo.lobos@gmail.com>
  * style(pre-commit): autofix
  * feat(cuda_voxel_grid_downsample_filter): use cub instead of thrust for better acceleration
  * style(pre-commit): autofix
  * feat(cuda_voxel_grid_downsample_filter): use dedicated memory pool
  * feat(cuda_voxel_grid_downsample_filter): introduce a parameter to control max size for GPU memory pool
  * docs: add/modify schema and documents for cuda_voxel_grid_downsample_filter
  * style(pre-commit): autofix
  * chore: fix spell miss
  * refactor: fix code style divergence error
  * style(pre-commit): autofix
  * fix: re-add INDENT-ON/OFF
  * feat: use most significant bit calculation to make radix sort faster
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: Kenzo Lobos Tsunekawa <kenzo.lobos@gmail.com>
* Contributors: Fumiya Watanabe, Kotaro Uetake, Manato Hirabayashi, Max Schmeller, TaikiYamada4, Yi-Hsiang Fang (Vivid), keita1523

0.45.0 (2025-05-22)
-------------------
* Merge remote-tracking branch 'origin/main' into tmp/notbot/bump_version_base
* feat(autoware_cuda_pointcloud_preprocessor): added target architectures for the cuda pointcloud preprocessor (`#10612 <https://github.com/autowarefoundation/autoware_universe/issues/10612>`_)
  * chore: added target architectures for the cuda pointcloud preprocessor
  * chore: mistook the compute capabilities of edge devices
  * chore: cspell
  ---------
* perf(autoware_tensorrt_common): set cudaSetDeviceFlags explicitly (`#10523 <https://github.com/autowarefoundation/autoware_universe/issues/10523>`_)
  * Synchronize CUDA stream by blocking instead of spin
  * Use blocking-sync in BEVFusion
  * Call cudaSetDeviceFlags in tensorrt_common
* feat(autoware_cuda_pointcloud_preprocessor): replace imu and twist callback with polling subscriber (`#10509 <https://github.com/autowarefoundation/autoware_universe/issues/10509>`_)
  * feat(cuda_pointcloud_preprocessor): replace subscriptions with InterProcessPollingSubscriber for twist and IMU data
  * fix(cuda_pointcloud_preprocessor): remove unused twist_queue\_ variable
  * style(pre-commit): autofix
  ---------
  Co-authored-by: Takahisa.Ishikawa <takahisa.ishikawa@tier4.jp>
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: Kenzo Lobos Tsunekawa <kenzo.lobos@tier4.jp>
* feat(autoware_cuda_pointcloud_preprocessor): pointcloud concatenation (`#10300 <https://github.com/autowarefoundation/autoware_universe/issues/10300>`_)
  * feat: cuda accelerated version of the pointcloud concatenation
  * chore: removed duplicated include
  * chore: changed to header blocks from pragmas :c
  * chore: removed yaml and schema since this node uses the same interface as the non-gpu node
  * chore: fixed rebased induced error
  * fix: used the wrong point type
  * chore: changed pointer to auto
  * chore: rewrote equation for clarity
  * chore: added a comment regarding the reallocation strategy
  * chore: reflected latest changes in the templated version of the concat
  * chore: addressed cppcheck reports
  * chore: fixed dead link
  * chore: solving uncrustify conflicts
  * chore: more uncrustify
  * chore: yet another uncrustify related error
  * chore: hopefully last uncrustify error
  * chore: now fixing uncrustify on source files
  ---------
* Contributors: Kenzo Lobos Tsunekawa, TaikiYamada4, Takahisa Ishikawa, prime number

0.44.2 (2025-06-10)
-------------------

0.44.1 (2025-05-01)
-------------------

0.44.0 (2025-04-18)
-------------------

0.43.0 (2025-03-21)
-------------------
* fix: update tool version
* Merge remote-tracking branch 'origin/main' into chore/bump-version-0.43
* chore(autoware_cuda_pointcloud_preprocessor): add maintainer (`#10297 <https://github.com/autowarefoundation/autoware_universe/issues/10297>`_)
* feat(autoware_cuda_pointcloud_preprocessor): a cuda-accelerated pointcloud preprocessor (`#9454 <https://github.com/autowarefoundation/autoware_universe/issues/9454>`_)
  * feat: moved the cuda pointcloud preprocessor and organized from a personal repository
  * chore: fixed incorrect links
  * chore: fixed dead links pt2
  * chore: fixed spelling errors
  * chore: json schema fixes
  * chore: removed comments and filled the fields
  * fix: fixed the adapter for the case when the number of points in the pointcloud changes after the first iteration
  * feat: used the cuda host allocators for aster host to device copies
  * Update sensing/autoware_cuda_pointcloud_preprocessor/docs/cuda-pointcloud-preprocessor.md
  Co-authored-by: Max Schmeller <6088931+mojomex@users.noreply.github.com>
  * Update sensing/autoware_cuda_pointcloud_preprocessor/src/cuda_pointcloud_preprocessor/cuda_pointcloud_preprocessor.cu
  Co-authored-by: Manato Hirabayashi <3022416+manato@users.noreply.github.com>
  * Update sensing/autoware_cuda_pointcloud_preprocessor/src/cuda_pointcloud_preprocessor/cuda_pointcloud_preprocessor.cu
  Co-authored-by: Manato Hirabayashi <3022416+manato@users.noreply.github.com>
  * style(pre-commit): autofix
  * Update sensing/autoware_cuda_pointcloud_preprocessor/docs/cuda-pointcloud-preprocessor.md
  Co-authored-by: Max Schmeller <6088931+mojomex@users.noreply.github.com>
  * Update sensing/autoware_cuda_pointcloud_preprocessor/README.md
  Co-authored-by: Max Schmeller <6088931+mojomex@users.noreply.github.com>
  * Update sensing/autoware_cuda_pointcloud_preprocessor/README.md
  Co-authored-by: Max Schmeller <6088931+mojomex@users.noreply.github.com>
  * Update sensing/autoware_cuda_pointcloud_preprocessor/src/cuda_pointcloud_preprocessor/cuda_pointcloud_preprocessor.cu
  Co-authored-by: Max Schmeller <6088931+mojomex@users.noreply.github.com>
  * style(pre-commit): autofix
  * Update sensing/autoware_cuda_pointcloud_preprocessor/src/cuda_pointcloud_preprocessor/cuda_pointcloud_preprocessor.cu
  Co-authored-by: Manato Hirabayashi <3022416+manato@users.noreply.github.com>
  * style(pre-commit): autofix
  * Update sensing/autoware_cuda_pointcloud_preprocessor/src/cuda_pointcloud_preprocessor/cuda_pointcloud_preprocessor.cu
  Co-authored-by: Manato Hirabayashi <3022416+manato@users.noreply.github.com>
  * style(pre-commit): autofix
  * chore: fixed code compilation to reflect Hirabayashi-san's  memory pool proposal
  * feat: generalized the number of crop boxes. For two at least, the new approach is actually faster
  * chore: updated config, schema, and handled the null case in a specialized way
  * feat: moving the pointcloud organization into gpu
  * feat: reimplemented the organized pointcloud adapter in cuda. the only bottleneck is the H->D copy
  * chore: removed redundant ternay operator
  * chore: added a temporary memory check. the check will be unified in a later PR
  * chore: refactored the structure to avoid large files
  * chore: updated the copyright year
  * fix: fixed a bug in the undistortion kernel setup. validated it comparing it with the baseline
  * chore: removed unused packages
  * chore: removed mentions of the removed adapter
  * chore: fixed missing autoware prefix
  * fix: missing assignment in else branch
  * chore: added cuda/nvcc debug flags on debug builds
  * chore: refactored parameters for the undistortion settings
  * chore: removed unused headers
  * chore: changed default crop box to no filtering at all
  * feat: added missing restrict keyword
  * chore: spells
  * chore: removed default destructor
  * chore: ocd activated (spelling)
  * chore: fixed the schema
  * chore: improved readibility
  * chore: added dummy crop box
  * chore: added new repositories to ansible
  * chore: CI/CD
  * chore: more CI/CD
  * chore: mode CI/CD. some linters are conflicting
  * style(pre-commit): autofix
  * chore: ignoring uncrustify
  * chore: ignoring more uncrustify
  * chore: missed one more uncrustify exception
  * chore: added meta dep
  ---------
  Co-authored-by: Max Schmeller <6088931+mojomex@users.noreply.github.com>
  Co-authored-by: Manato Hirabayashi <3022416+manato@users.noreply.github.com>
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: Amadeusz Szymko <amadeusz.szymko.2@tier4.jp>
* Contributors: Amadeusz Szymko, Hayato Mizushima, Kenzo Lobos Tsunekawa
