^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_ptv3
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.53.0 (2026-09-29)
-------------------
* Merge remote-tracking branch 'origin/main' into prepare-0.53.0-changelog
* feat(autoware_ptv3): support multisweep densification with time-lag features (`#13300 <https://github.com/autowarefoundation/autoware_universe/issues/13300>`_)
  * feat(autoware_ptv3): support multisweep densification with time-lag features
  * feat(autoware_ptv3): support none source reconstruction mode
  * fix(autoware_ptv3): publish every output cloud in input order
  * perf(autoware_ptv3): drop the per-frame stream sync from sweep aggregation
  * fix(autoware_ptv3): mark the current sweep explicitly instead of inferring it from the lag
  * fix(autoware_ptv3): reject unusable frames when they are cached instead of throwing mid-inference
  * refactor(autoware_ptv3): create the TF buffer and listener only when densification is enabled
  * fix(autoware_ptv3): advertise the filtered cloud only when filter classes are configured
  * refactor(autoware_ptv3): drop the duplicated frame count, config copies and cache walks
  * fix(autoware_ptv3): pass identity for the current frame instead of a pose round trip
  * chore: add voxelizer to the cspell dictionary
  ---------
* feat(autoware_ptv3)!: per-stage voxel-count maximums and input truncation to the stage bounds (`#13390 <https://github.com/autowarefoundation/autoware_universe/issues/13390>`_)
  TensorRT sizes an engine's activation memory for the profile maximum, and until now every
  pooled encoder level was bounded by the input maximum (voxels_num[2]) or, at best, its grid
  capacity, which only binds at the coarsest levels. Since the channel width doubles per level,
  the deepest levels dominated the encoder's and segmentation head's memory even though stride-2
  pooling merges at least half of a lidar sweep's voxels at every level in practice.
  BREAKING CHANGE: the new required parameter encoder.pooled_voxels_num_max gives one voxel-count
  maximum per pooled encoder stage (one entry per pooling stride). encoder.voxels_num keeps its
  [min, opt, max] meaning for the input level. Existing parameter files must add the parameter;
  existing engines are rebuilt because the profiles change.
  PTv3Config::stage_voxel_capacity takes the smaller of a stage's configured maximum and its grid
  bound, and stage_profile_counts (moved from PTv3TRT) derives every stage's [min, opt, max]
  profile from it, so the encoder and head profiles and the per-stage feature buffers all follow
  the parameter. Validation requires one entry per pooling stage, each positive and at most the
  previous stage's maximum. The shipped defaults are measured per-stage peaks with a margin.
  A frame whose level would exceed its maximum is truncated instead of failing. The levels are in
  order-0 serialization order and each parent's children are contiguous, so keeping the first
  `max` voxels of a pooled level keeps a prefix of every finer level down to the input.
  generateSerializedPoolingMetadata builds the levels, finds the longest input prefix whose
  levels all fit (a single-thread device walk over the indptr arrays) and rebuilds from that
  prefix when it is shorter, reading the codes from the caller's buffer at their original stride.
  The walk's result returns with the level counts in the one synchronization the caller already
  needed, so the common path adds no sync and the truncation path adds one. PTv3TRT feeds the
  encoder the truncated count and warns. This drops the voxels with the largest codes (a region
  at the end of the curve); choosing what to prune is left for later.
  Co-authored-by: Claude Fable 5.1 <noreply@anthropic.com>
* feat(autoware_ptv3): declare and bind the encoder IO the artifact exposes (`#13290 <https://github.com/autowarefoundation/autoware_universe/issues/13290>`_)
  * feat(autoware_ptv3): offer the encoder IO the artifact may omit
  LitePT gates convolution and attention off per encoder level, so nothing
  consumes the base serialization order and levels without convolution read no
  head_indices. The ONNX exporter drops inputs the traced graph never consumes,
  so a LitePT encoder declares 20 of the 27 inputs this node hardcoded and
  TrtCommon rejected it on an IO count mismatch: "Failed to setup encoder TRT
  engine."
  Everything except grid_coord, feat and the point features is now offered as
  optional, and TrtCommon drops the optional entries the artifact omits. Address
  binding and per-frame shape setting follow the surviving set. A PTv3 artifact
  declares the full contract and is unaffected.
  Two checks keep this a subset rather than a free-for-all:
  - a tensor the artifact declares that this node cannot supply is rejected by
  name, instead of reaching enqueue with no address ever bound;
  - grid_coord, feat and every point_feat_i must be present, since no encoder
  variant can drop them.
  Pooling metadata is still generated for every stage, so this changes what is
  offered and bound, not what is computed.
  Co-Authored-By: Claude Opus 5 (1M context) <noreply@anthropic.com>
  * style(autoware_ptv3): tighten the optional-IO comment
  Co-Authored-By: Claude Fable 5.1 <noreply@anthropic.com>
  ---------
  Co-authored-by: Claude Opus 5 (1M context) <noreply@anthropic.com>
* feat(autoware_ptv3)!: feed the encoder the level-0 serialization order (`#13155 <https://github.com/autowarefoundation/autoware_universe/issues/13155>`_)
  The encoder engine took the level-0 serialized_code and argsorted it
  in-graph to recover the serialization order, even though every pooled
  level's order and inverse are handed in as inputs. Those two Argsort nodes
  cost ~193us of the encoder's enqueue window on a 122m/0.12m detection
  config, recomputing a ranking the preprocessing pipeline must produce
  anyway to build the pooling metadata.
  PreprocessCuda now publishes the level-0 order and inverse it already
  derives (order 0 is the identity, the rest come from the one remaining
  sort), and the encoder and seg3d-head engines bind those instead of
  serialized_code. serialized_code stays a host-side buffer: it is still
  what chains the pooling stages together.
  Requires the matching autoware-ml change and a model re-export; the
  preceding commit stands on its own without either.
  BREAKING CHANGE: the encoder engine's `serialized_code` input is replaced
  by `serialized_order` and `serialized_inverse`, and the seg3d head's
  stage-0 inputs likewise. Existing engines must be re-exported.
* fix(autoware_cuda_utils): export CCCL for host compilation with CUDA 13 (`#13302 <https://github.com/autowarefoundation/autoware_universe/issues/13302>`_)
  * fix(autoware_cuda_utils): export CCCL for host compilation with CUDA 13
  CUDA 13 moved the Thrust, CUB and libcu++ headers to include/cccl/. nvcc adds
  that directory by itself. The host compiler does not, so g++ fails on any host
  file that reaches a CCCL header.
  The installed thrust_utils.hpp uses Thrust, so the include path belongs in the
  exported interface of autoware_cuda_utils. Export an INTERFACE target that
  carries the package headers and CCCL::CCCL, and add cmake/find_cccl.cmake to
  locate the toolkit CCCL config outside the default CMake search path.
  Consumers that already depend on autoware_cuda_utils need no change. Link the
  target in autoware_ptv3, whose CUDA library is not built through
  ament_target_dependencies.
  * fix(autoware_cuda_utils): mark the CCCL include as system, export CUDA::cudart
  The CCCL config creates non-imported targets, so the cccl include directory
  reached consumers as -I in 79 flag files across 25 packages, and a future
  CCCL header warning would break builds under -Werror. A small imported marker
  target, autoware_cuda_utils::cccl_system, now carries the same directory as a
  system include. CMake unions system include sets across the link closure, and
  the explicit property survives the CMAKE_NO_SYSTEM_FROM_IMPORTED that
  mrt_cmake_modules leaks into consumer scopes. No NVIDIA target is modified.
  The exported interface also carries CUDA::cudart: the installed
  thrust_utils.hpp includes cuda_runtime_api.h, which the package and CCCL
  include directories alone cannot resolve.
  Also state above the autoware_ptv3 link why it is load-bearing: nvcc supplies
  CCCL to the .cu files, and detection3d_postprocess_test host-compiles Thrust
  without ament_target_dependencies.
* feat(ptv3): enable to specify segmentation class remapping from config (`#13201 <https://github.com/autowarefoundation/autoware_universe/issues/13201>`_)
  * feat: enable to specy segmentation class remapping from config
  * refactor: disply all missing remap entries when throwing exception
  * chore: run ros node relevant tests isolatedly
  * test: mix up the order of remap class names
  * refactor: rename class_remap to class_mapping
  ---------
* perf(autoware_ptv3): cut PTv3 preprocessing from 13 radix sorts to 2 (`#13170 <https://github.com/autowarefoundation/autoware_universe/issues/13170>`_)
  * perf(autoware_ptv3): derive serialized pooling metadata with scans
  PTv3 preprocessing ran 13 radix sorts per frame: one to deduplicate
  voxels, then, for each of the 4 pooling stages, one sort to group voxels
  by parent plus one sort per serialization order. On a 122m/0.12m
  detection config those 13 sorts cost ~849us of the ~1274us the whole
  preprocessing stage takes, and their cost is flat: every pooling sort
  runs over max_num_voxels (192k) padded entries regardless of how few
  voxels the stage actually holds, so the deepest stage pays as much as
  the initial deduplication.
  Since the previous change, voxels enter generateSerializedPoolingMetadata
  sorted by their order-0 serialized code. Right-shifting a serialized
  code by 3 bits per pooling level yields the code of the parent cell, and
  that map is monotone, so a level sorted by its serialized code stays
  sorted after pooling. Two consequences:
  - Order 0 never needs sorting. Segments are emitted in increasing parent
  code, so the pooled level's order-0 ranking is the identity at every
  level. The old code spent a full 63-bit sort per stage computing it.
  - The other orders never need sorting either. Every order right-shifts to
  the same parent partition, so walking a level in order-o rank sequence
  visits each parent's children contiguously; compacting the run heads
  yields the pooled level's order-o ranking directly.
  So the hierarchy is now built from prefix scans and run compaction, and
  the only remaining sorts are the per-order sorts at the finest level,
  which are skipped entirely for empty inputs. Net: 13 sorts -> 2, with 4
  fewer int64 scratch buffers of max_num_voxels.
  The sorted-input precondition is documented on the declaration and
  asserted on device in markPoolingRunsKernel, which validates every level
  as it is consumed. The assert compiles out in Release and RelWithDebInfo
  (-DNDEBUG reaches the device front end through FindCUDA's propagated
  host flags); in a Debug build it pinpoints an unsorted input instead of
  silently producing wrong metadata.
  The metadata tests could not detect a swapped serialization order at all,
  because the fixed-input fixture is almost entirely y=0 and so is nearly
  invariant under transposing x and y. Both are fixed:
  - The fixture is replaced with a level whose two orders rank the voxels
  differently at every level and where pooling genuinely merges voxels at
  every stage (10 -> 6 -> 4).
  - expect_orders_diverge asserts that property instead of assuming it, so a
  future fixture edit cannot silently make the test toothless again.
  - A randomized sweep compares 32 occupancy patterns against the CPU
  reference across all eight metadata arrays and every stage.
  Verified by mutating markOrderRunsKernel to walk order 0's rank sequence
  for every order: both reference tests fail, where previously only the
  randomized one did.
  * docs(autoware_ptv3): document pooling buffers, use input-level naming
  Address review feedback on `#13170 <https://github.com/autowarefoundation/autoware_universe/issues/13170>`_: document each pooling-metadata
  buffer, standardize on "input level" over "finest level", and drop a
  stale sentence about the sort-workspace queries.
  * docs(autoware_ptv3): clarify order-sort keys comment
  * docs(autoware_ptv3): avoid rank/ranking jargon in comments
  Describe the per-order permutations as listings in ascending code order
  and rename the kernel-local rank variables to idx to match the sibling
  kernels.
  * docs(autoware_ptv3): add docstrings for remaining pooling kernels
  * refactor(autoware_ptv3): mark pooling kernel params _in/_out/_inout
  * test(autoware_ptv3): replace randomized pooling sweep with hand-crafted edge cases
  Each case pins one boundary of the scan-based derivation with
  hand-verified level sizes: empty input, single voxel, a full parent
  block, no merging, mixed run lengths, and a level at exactly
  max_num_voxels. Mutation-checked: walking order 0's listing for every
  order in markOrderRunsKernel still fails the test (caught by the
  mixed-run-lengths case).
  Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>
  * docs(autoware_ptv3): make pooling buffer member comments doxygen
  Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>
  ---------
  Co-authored-by: Claude Fable 5 <noreply@anthropic.com>
* refactor(autoware_ptv3): replace voxelization hashing with serialized codes (`#13153 <https://github.com/autowarefoundation/autoware_universe/issues/13153>`_)
  * refactor(autoware_ptv3): key voxelization on serialized codes, not hashes
  Voxel deduplication keyed on two hashes of the grid coordinate: an
  FNV-1a hash on grids larger than 2^32 cells and a raster index below
  that. Replace both with the order-0 serialized (Morton) code that the
  serialization step computes anyway. This fixes two dormant defects and
  establishes an ordering the pooling path can later exploit:
  - The FNV-1a hash is not injective, so distinct voxels could silently
  merge. It was dormant (grids under 2^32 cells take the injective
  raster path) but would arm itself on a larger range or finer voxels.
  Serialized codes are injective by construction, so use_64bit_hash\_
  and the whole dual 32/64-bit branch are gone.
  - The deduplication grid and the serialization grid were computed with
  different arithmetic ((p - min_range) / size versus
  floor(p / size) - floor(min_range / size)), so boundary points could
  land in different cells in each. Both now share one gridCoord helper,
  so the deduplication key and the serialized codes always describe the
  same cell.
  Because the deduplication sort orders voxels by their order-0
  serialized code, generateFeatures now emits them in order-0
  serialization order; a new test pins that postcondition (and that the
  emitted codes describe the emitted grid coordinates) at its source.
  Also renames the remaining hash-named identifiers (voxel_hashes,
  sorted_hash_indexes) after their new content, and moves the host-side
  serialize_coord reference implementation into the shared test fixture.
  * fix(autoware_ptv3): size serialization depth for unaligned range boundaries
  The device-side grid mapping floor(p / size) - floor(min_range / size)
  emits one more coordinate than round((max_range - min_range) / size)
  when a range boundary is not voxel-aligned: [0.5, 16.5) with unit voxels
  emits coordinates 0..16. serialization_depth\_ was derived from the
  rounded count, so in the power-of-two case the extra boundary
  coordinate's top Morton bit fell outside the code and its voxels
  silently merged with coordinate 0's during deduplication. Reported by
  Copilot and Codex on review.
  Derive the depth from the coordinates the mapping can actually emit
  instead: the largest one is attained at the largest float below
  max_range (the crop accepts min_range <= p < max_range and float
  division is monotonic), computed on the host with the same float
  arithmetic the kernel uses. Widening the depth is backward-compatible:
  serializeCoord places bit i of each coordinate at a position that does
  not depend on the total depth, so codes are bit-identical for every
  coordinate that already fit, including the entire shipped configuration
  (whose z axis spans a non-integral 66.67 cells and relies on this
  behavior - which is also why rejecting unaligned ranges outright was
  not an option).
  grid\_*_size\_ stays the rounded count: it feeds capacity estimates and
  matches the nominal grid the model was exported against.
  Verified that both new tests fail against the previous derivation
  (depth 4 with two corner voxels merged into one) and pass with it.
  * test(autoware_ptv3): pin float behavior of decimal-aligned range borders
  Ranges like [-102.4, 102.4] with 0.1 voxels are voxel-aligned in
  decimal but involve no exactly-representable binary floats, so whether
  they span 2048 or 2049 coordinates is decided by float rounding. Pin
  the current (clean) outcome at both levels:
  - config: serialization_depth\_ stays 11, i.e. the rounded quotients
  land exactly on the aligned cell count with no spurious boundary
  coordinate;
  - device: a point one ULP below the border lands in the last cell
  (2047) as the host-side nextafter maximization predicts, and a point
  exactly on the border is excluded by the strict upper crop bound -
  the exclusion that maximization models.
  Together these guard the host/device float-arithmetic agreement the
  depth derivation relies on.
  * Update perception/autoware_ptv3/test/ptv3_config_test.cpp
  * fix(autoware_ptv3): bound stage capacities by effective grid cells
  stage_voxel_capacity() still derived its per-stage voxel bound from the
  rounded grid\_*_size\_, while the grid mapping can emit one more
  coordinate per axis on unaligned range boundaries: [0.5, 16.5) with
  unit voxels emits 17 coordinates, so stage 0 can hold up to 17^3
  distinct voxels and a stride-2 stage ceil(17/2)^3. With max_num_voxels\_
  large enough, the rounded bound under-allocates the encoder stage
  buffers and produces TensorRT profiles that reject valid pooled counts.
  Reported by amadeuszsz, Codex and Copilot on review.
  Store the per-axis effective cell counts (already computed for
  serialization_depth\_) and derive the capacity from them. No change for
  voxel-aligned configurations, where effective and rounded counts are
  equal.
  Verified that the new capacity test fails against the rounded bound
  (1024/128 instead of 1445/243) and passes with the effective one.
  * refactor(autoware_ptv3): keep a single grid-size definition
  grid\_*_size\_ (rounded) and effective_grid\_*_size\_ (emittable
  coordinates) duplicated state, and the rounded variant had no consumer
  left that wanted it. Fold them into grid\_*_size\_, defined as the cell
  count the device grid mapping can emit; the voxel-coordinate bounds test
  becomes strictly more precise under this definition.
  Also condense the unaligned-boundary comments to at most three lines
  each, stating each fact once across the config and the tests.
  ---------
* fix(perception): complete node design files for label-based euclidean cluster and PTv3 (`#13253 <https://github.com/autowarefoundation/autoware_universe/issues/13253>`_)
  * refactor(ptv3.node.yaml): update parameter definitions and add new model paths
  * feat(LabelBasedEuclideanCluster): add new node configuration for label-based clustering
  * fix(DetectedObjectFeatureRemover): correct plugin name in node configuration
  ---------
* feat(autoware_ptv3): allow strongly typed engine builds (`#13159 <https://github.com/autowarefoundation/autoware_universe/issues/13159>`_)
  * refactor(autoware_tensorrt_common): rename precision "as-is" to "strongly-typed"
  The value selects a strongly typed build, where tensor precisions come
  from the model. Name it after what it does: "as-is" describes the
  artifact's treatment rather than the build mode, and reads as a
  precision on par with fp32/fp16/int8, which it is not.
  No functional change. The value has no users yet: it was added in
  `#13160 <https://github.com/autowarefoundation/autoware_universe/issues/13160>`_ and its first consumer (autoware_ptv3, `#13159 <https://github.com/autowarefoundation/autoware_universe/issues/13159>`_) is not merged, so
  renaming now costs nothing.
  * feat(autoware_ptv3): allow strongly typed engine builds
  Accept "strongly-typed" as a trt_precision value. The node already
  forwards trt_precision to the engine config of every model, so this only
  widens the parameter's schema: tensor precisions are then taken from the
  ONNX itself instead of being assigned by the TensorRT builder.
  Models exported with reduced precision need this. The weakly typed
  precision-assignment pass would re-process their already reduced
  tensors, and on one of our embedded targets it crashes on PTv3-shaped
  graphs regardless of builder settings; strongly typed builds skip that
  pass entirely.
  Nothing else in the node changes. Engine I/O, node buffers and kernels
  stay fp32 for every model: reduced-precision exports keep fp32 tensors
  at the graph boundary through cast layers inside the model, which the
  exporter handles. Strong typing is independent of the model's precision,
  so an fp32 export can also be built this way for reproducible,
  model-defined numerics.
  * chore: simplify trt_precision description
  ---------
* refactor(autoware_universe): use autoware_ament_auto_package in perception DNN packages (`#12277 <https://github.com/autowarefoundation/autoware_universe/issues/12277>`_)
  Co-authored-by: github-actions <github-actions@github.com>
  Co-authored-by: Taekjin LEE <taekjin.lee@tier4.jp>
* refactor(perception): move node design files into each package (`#13104 <https://github.com/autowarefoundation/autoware_universe/issues/13104>`_)
  Co-authored-by: Claude Opus 5 (1M context) <noreply@anthropic.com>
* test(autoware_ptv3): cover unknown detection label (`#13145 <https://github.com/autowarefoundation/autoware_universe/issues/13145>`_)
* test(autoware_ptv3): cover detection threshold config (`#13144 <https://github.com/autowarefoundation/autoware_universe/issues/13144>`_)
  * test(autoware_ptv3): cover detection ROS conversion
  * test(autoware_ptv3): cover detection threshold config
  ---------
* test(autoware_ptv3): cover detection ROS conversion (`#13143 <https://github.com/autowarefoundation/autoware_universe/issues/13143>`_)
* test(autoware_ptv3): cover detection postprocess decode (`#13142 <https://github.com/autowarefoundation/autoware_universe/issues/13142>`_)
  * test(autoware_ptv3): add detection fixture defaults
  * style(pre-commit): autofix
  * test(autoware_ptv3): exclude max range voxel coords
  * test(autoware_ptv3): assert pooling index invariants
  * test(autoware_ptv3): validate detection grid compatibility
  * test(autoware_ptv3): check detection BEV grid bounds
  * test(autoware_ptv3): cover detection postprocess decode
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci-lite[bot] <117423508+pre-commit-ci-lite[bot]@users.noreply.github.com>
* test(autoware_ptv3): check detection BEV grid bounds (`#13141 <https://github.com/autowarefoundation/autoware_universe/issues/13141>`_)
  * test(autoware_ptv3): add detection fixture defaults
  * style(pre-commit): autofix
  * test(autoware_ptv3): exclude max range voxel coords
  * test(autoware_ptv3): assert pooling index invariants
  * test(autoware_ptv3): validate detection grid compatibility
  * test(autoware_ptv3): check detection BEV grid bounds
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci-lite[bot] <117423508+pre-commit-ci-lite[bot]@users.noreply.github.com>
* test(autoware_ptv3): validate detection grid compatibility (`#13140 <https://github.com/autowarefoundation/autoware_universe/issues/13140>`_)
  * test(autoware_ptv3): add detection fixture defaults
  * style(pre-commit): autofix
  * test(autoware_ptv3): exclude max range voxel coords
  * test(autoware_ptv3): assert pooling index invariants
  * test(autoware_ptv3): validate detection grid compatibility
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci-lite[bot] <117423508+pre-commit-ci-lite[bot]@users.noreply.github.com>
* test(autoware_ptv3): assert pooling index invariants (`#13139 <https://github.com/autowarefoundation/autoware_universe/issues/13139>`_)
  * test(autoware_ptv3): add detection fixture defaults
  * style(pre-commit): autofix
  * test(autoware_ptv3): exclude max range voxel coords
  * test(autoware_ptv3): assert pooling index invariants
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci-lite[bot] <117423508+pre-commit-ci-lite[bot]@users.noreply.github.com>
* test(autoware_ptv3): exclude max range voxel coords (`#13138 <https://github.com/autowarefoundation/autoware_universe/issues/13138>`_)
  * test(autoware_ptv3): add detection fixture defaults
  * style(pre-commit): autofix
  * test(autoware_ptv3): exclude max range voxel coords
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci-lite[bot] <117423508+pre-commit-ci-lite[bot]@users.noreply.github.com>
* test(autoware_ptv3): add detection fixture defaults (`#13133 <https://github.com/autowarefoundation/autoware_universe/issues/13133>`_)
  * test(autoware_ptv3): add detection fixture defaults
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci-lite[bot] <117423508+pre-commit-ci-lite[bot]@users.noreply.github.com>
* feat(ptv3): remove experimental and apply autoware_core features (`#13044 <https://github.com/autowarefoundation/autoware_universe/issues/13044>`_)
* chore(pre-commit): update clang-format to v22.1.5 (`#13126 <https://github.com/autowarefoundation/autoware_universe/issues/13126>`_)
  * chore(pre-commit): update clang-format to v22.1.5
  * style(pre-commit): autofix
  ---------
* feat(ptv3): filter segmented points (`#12821 <https://github.com/autowarefoundation/autoware_universe/issues/12821>`_)
  * feat: add SemanticLabel
  feat: update helper function using autoware msg
  * refactor(ptv3): split semantic label definition and helpers
  * feat: filter segmented points by specified class names
  * test: add test for postprocess
  * style(pre-commit): autofix
  * feat: apply filter segmented point only if filter.apply_to_segmentation:=true
  * feat: remove filter.class_probability_threshold
  ---------
  Co-authored-by: pre-commit-ci-lite[bot] <117423508+pre-commit-ci-lite[bot]@users.noreply.github.com>
* feat(autoware_ptv3): update filtered classes (`#13009 <https://github.com/autowarefoundation/autoware_universe/issues/13009>`_)
* feat(autoware_ptv3): handle segmentation3d decoder (`#12973 <https://github.com/autowarefoundation/autoware_universe/issues/12973>`_)
  * feat(autoware_ptv3): handle segmentation3d decoder
  * style(pre-commit): autofix
  * fix(autoware_ptv3): adjust required schema parameters
  * feat(autoware_ptv3): add confidence calibration and update filtered classes
  * fix(autoware_ptv3): align voxel grid bounds with training voxelization
  * fix(autoware_ptv3): remove voxel count guard handled by decoder graph
  * style(pre-commit): autofix
  * feat(autoware_ptv3): revert unrelated config update
  * refactor(autoware_ptv3): simplify range crop to half-open voxel-grid bounds
  * feat(autoware_ptv3): restore confidence thresholds
  ---------
  Co-authored-by: pre-commit-ci-lite[bot] <117423508+pre-commit-ci-lite[bot]@users.noreply.github.com>
* feat(ptv3): apply SemanticLabel to output points (`#12820 <https://github.com/autowarefoundation/autoware_universe/issues/12820>`_)
  * feat: add SemanticLabel
  feat: update helper function using autoware msg
  * refactor(ptv3): split semantic label definition and helpers
  * feat: add label conversion in postprocess
  * chore: restore original
  * style(pre-commit): autofix
  * fix: resolve build error
  * refactor: move internal linkages to anomymous namespace
  ---------
  Co-authored-by: pre-commit-ci-lite[bot] <117423508+pre-commit-ci-lite[bot]@users.noreply.github.com>
* fix(ptv3): guard extract indices kernel voxel writes against max_num_voxels overflow (`#12914 <https://github.com/autowarefoundation/autoware_universe/issues/12914>`_)
  * fix(preprocess_kernel): limit output to max_num_voxels in extractIndicesKernel
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci-lite[bot] <117423508+pre-commit-ci-lite[bot]@users.noreply.github.com>
* perf(ptv3): update pre process (`#12877 <https://github.com/autowarefoundation/autoware_universe/issues/12877>`_)
  * perf(ptv3): replace thrust API call to cub API call
  * perf(ptv3): utilize host pinned memory to accelerate DeviceToHost copy
  * perf(ptv3): replace cudaStreamSynchronize by cudaEventSynchronize
  This replacement makes it possible that we can postpone synchronize operation
  until the moment when the Device-to-Host copied data is actually used on the
  host side.
  * perf(ptv3): replace std::vector by cuda host pinned memory
  This vector is used to put computation result on the GPU. Using pinned memory
  accelerates Device-to-Host copy.
  * perf(ptv3): use similar delayed sync for hash64 flow
  * perf: set the most significant bit for RadixSort
  * fix(ptv3): check return value of cub::DeviceRadixSort::SortPairs
  * refactor(ptv3): remove unused thrust lines
  * fix(ptv3): move raw cudaEvent_t hendles creation to the very end of the constructor
  These handles may leak if the successive operations in the constructor throw (in
  those cases, the destructor will not be called). Moving the creation to the end
  of the constructor, ensures that all potentially-throwing operations have been
  completed, then creates the raw handlers.
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci-lite[bot] <117423508+pre-commit-ci-lite[bot]@users.noreply.github.com>
* Contributors: Amadeusz Szymko, Kotaro Uetake, Manato Hirabayashi, Max Schmeller, Mete Fatih Cırıt, Ryohsuke Mitsudome, Taekjin LEE, Vishal Chauhan

0.52.0 (2026-06-30)
-------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* feat(ptv3): add SemanticLabel enum (`#12819 <https://github.com/autowarefoundation/autoware_universe/issues/12819>`_)
* test(autoware_ptv3): add pre-process tests (`#12702 <https://github.com/autowarefoundation/autoware_universe/issues/12702>`_)
  * test(ptv3): add unit tests for pre-process
  * refactor(ptv3): use `ament_add_gtest` for the unit test generation of preprocess_kernel
  * refactor(ptv3): simplify redundant CMake condition
  `target_include_directories` can be omitted since global `include_directories`
  does exactly the same
  * test(ptv3): ommit linter tests
  * test(ptv3): handle float equality explicitly
  * test(ptv3): refactor preprocess kernel tests with fixture
  * refactor(ptv3): use cuda_util::CudaUniquePtr instead of local defined one
  * test(ptv3): omit redundant test assertions
  * refactor(ptv3): move common input into the test fixture
  * fix(ptv3): update `PTv3Config` to align the latest definition
  * refactor(ptv3): introduce base test fixture class for reusability
  and apply the base class to `serialized_pooling_metadata_test.cpp` and `test_preprocess_kernel.cpp`
  ---------
* feat(autoware_ptv3): add detection3d head (`#12758 <https://github.com/autowarefoundation/autoware_universe/issues/12758>`_)
  * feat(autoware_ptv3): add detection3d head
  * refactor(autoware_ptv3): use shared utils
  * docs(autoware_ptv3): params description
  * style(autoware_ptv3): naming convention
  * fix(autoware_ptv3): remove redundant param
  * fix(autoware_ptv3): voxel size safe guard
  * fix(autoware_ptv3): log
  * fix(autoware_ptv3): func rename
  * fix(autoware_ptv3): func rename
  * fix(autoware_ptv3): remove redundant call
  * fix(autoware_ptv3): add sigmoid lambda
  * fix(autoware_ptv3): later definition
  * refactor(autoware_ptv3): func name
  ---------
* chore: add myself as PTv3 maintainer (`#12823 <https://github.com/autowarefoundation/autoware_universe/issues/12823>`_)
* feat(ptv3)!: precompute serialized pooling metadata (`#12727 <https://github.com/autowarefoundation/autoware_universe/issues/12727>`_)
  * perf: serialized pooling optimization
  * chore: fix rebase
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci-lite[bot] <117423508+pre-commit-ci-lite[bot]@users.noreply.github.com>
* feat(autoware_ptv3): split the backbone and head (`#12655 <https://github.com/autowarefoundation/autoware_universe/issues/12655>`_)
  * feat(autoware_ptv3): add multi-head support (segmentation3d and detection3d)
  * fix(autoware_ptv3): cspell
  * fix(autoware_ptv3): typo
  * fix(autoware_ptv3): centerhead dims order
  * feat(autoware_ptv3): remove center_head support for now
  * refactor(autoware_ptv3): PR split - drop det3d head
  * feat(autoware_ptv3): cleanup
  * feat(autoware_ptv3): explicit types for all tensors
  * feat(autoware_ptv3): pre-commit
  * feat(autoware_ptv3): restore old sources
  * fix(autoware_ptv3): use of member initializer lists
  * fix(autoware_ptv3): remove redundant initialization
  * fix(autoware_ptv3): wrong type
  * fix(autoware_ptv3): redundant sync
  * fix(autoware_ptv3): guard pointer
  * feat(autoware_ptv3): update model params
  ---------
* feat(autoware_ptv3): source cloud reconstruction & entropy (`#12547 <https://github.com/autowarefoundation/autoware_universe/issues/12547>`_)
  * feat(autoware_ptv3): source cloud reconstruction & entropy
  * refactor(ptv3): clarify reconstruction feature handling
  * refactor(autoware_ptv3): remove unnecessary buffers clear
  * refactor(ptv3): simplify source reconstruction buffers
  * fix(autoware_ptv3): remove redundant memory clear
  * feat(autoware_ptv3): add __restrict\_\_
  Co-authored-by: Manato Hirabayashi <3022416+manato@users.noreply.github.com>
  * style(pre-commit): autofix
  * feat(autoware_ptv3): add restrict
  * feat(autoware_ptv3): parallelizaiton
  ---------
  Co-authored-by: Manato Hirabayashi <3022416+manato@users.noreply.github.com>
  Co-authored-by: pre-commit-ci-lite[bot] <117423508+pre-commit-ci-lite[bot]@users.noreply.github.com>
* Contributors: Amadeusz Szymko, Kotaro Uetake, Manato Hirabayashi, Max Schmeller, github-actions

0.51.0 (2026-05-01)
-------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* fix(autoware_ptv3): bad cmake variable syntax (`#12513 <https://github.com/mitsudome-r/autoware_universe/issues/12513>`_)
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
* feat(autoware_ptv3): add multi-type input and output & update model (`#12362 <https://github.com/mitsudome-r/autoware_universe/issues/12362>`_)
  * feat(autoware_ptv3): add multi-type input and output & update model
  * fix(autoware_ptv3): missing reference
  Co-authored-by: Kyoichi Sugahara <32741405+kyoichi-sugahara@users.noreply.github.com>
  * docs(autoware_ptv3): add troubleshooting
  ---------
  Co-authored-by: Kyoichi Sugahara <32741405+kyoichi-sugahara@users.noreply.github.com>
* feat(autoware_ptv3): cuda 12.0 build compatibility (`#12188 <https://github.com/mitsudome-r/autoware_universe/issues/12188>`_)
  * feat(autoware_ptv3): CUDA 12.0+ build compatibility
  * feat: restore Turing arch
  ---------
* Contributors: Amadeusz Szymko, Mete Fatih Cırıt, Vincent Richard, github-actions

0.50.0 (2026-02-14)
-------------------
* Merge remote-tracking branch 'origin/main' into humble
* feat(autoware_ptv3): update nvcc flags (`#12053 <https://github.com/autowarefoundation/autoware_universe/issues/12053>`_)
* chore(autoware_ptv3): remove cudnn dependency (`#11894 <https://github.com/autowarefoundation/autoware_universe/issues/11894>`_)
* chore: add maintainer of PTv3, FRNet, and CalibrationStatusClassifier (`#11945 <https://github.com/autowarefoundation/autoware_universe/issues/11945>`_)
  * chore: update `autoware_ptv3` maintainer
  * chore: update `autoware_lidar_frnet` maintainer
  * chore: update `autoware_calibration_status_classifier` maintainer
  ---------
* Contributors: Amadeusz Szymko, Manato Hirabayashi, Ryohsuke Mitsudome

0.49.0 (2025-12-30)
-------------------

0.48.0 (2025-11-18)
-------------------
* Merge remote-tracking branch 'origin/main' into humble
* feat(autoware_ptv3): implemented an inference node for ptv3 using tensorrt (`#10600 <https://github.com/autowarefoundation/autoware_universe/issues/10600>`_)
  * feat: implemented an inference node for ptv3 using tensorrt
  * chore: cspells
  * chore: schemas
  * chore: lint (line was too long)
  * chore: more schemas
  * fix: mistook the compute capabilities of edge devices
  * chore: replaced incorrect bevfusion -> ptv3
  * chore: forgot to remove unused schema
  * chore: duplicated variable
  * chore: changed package dep name
  * chore: fixed schema comment
  * chore: removed unused headers in the post process kernels
  * chore: replaced in favor of auto
  * chore: removed unused headers
  * chore: changed initialization order
  * chore: replaced 0 by nullptr
  * chore: replaced type in favor of auto
  * chore: removed redundant message
  * chore: fixed compilation due to review changes
  * fix: replaced int64 by uint64
  * chore: added more descriptive comment in the schema
  * style(autoware_ptv3): cleanup
  ---------
  Co-authored-by: Amadeusz Szymko <amadeusz.szymko.2@tier4.jp>
* Contributors: Kenzo Lobos Tsunekawa, Ryohsuke Mitsudome
