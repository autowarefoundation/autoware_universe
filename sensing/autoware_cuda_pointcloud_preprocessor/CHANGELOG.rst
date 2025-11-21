^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_cuda_pointcloud_preprocessor
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
