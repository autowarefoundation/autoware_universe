^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_cuda_utils
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
* Merge remote-tracking branch 'origin/main' into humble
* feat(autoware_bevfusion): implementation of bevfusion using tensorrt (`#10024 <https://github.com/autowarefoundation/autoware_universe/issues/10024>`_)
  * feat: moved from personal repository https://github.com/knzo25/bevfusion_ros2
  * feat: added fp16 support. it is faster than centerpoint !
  * chore: spells and ci/cd
  * chore: more ci/cd
  * chore: and yet more spells
  * chore: more spells
  * chore: updated the schema
  * chore: reverted unintented change
  * chore: added documentation
  * chore: updated copyrights
  * chore: ci/cd fixes
  * chore: missed on transfusion mention in bevfusion
  * chore: removed another mention of transfusion in the launchers
  * chore: re enabled the standard paths for the ml models
  * chore: replaced stream references to pass by value
  * feat: updated bevfusion to follow https://github.com/autowarefoundation/autoware_universe/pull/9595
  * chore: removed unused headers
  * chore: updated cases
  * chore: replaced the layout check in the package for the one in the autoware_point_types package
  * chore: added meta dep
  * fix: bug in the camera mask
  * chore: avoided tmp objects through proper use of emplace
  * chore: replaced nested namespaces for concatenated ones
  * chore: made the operator a const one
  * chore: replaced the use of boost optionals for std ones
  * chore: added a check for empty pointclouds
  * chore: logging macros did not require stream
  * chore: addressed better a border condition
  * chore: added a check for empty sweep points (can happen when individual sweeps are bigger than the buffer's capacity)
  * chore: removed unused headers
  * chore: replaced the computation of the number of blocks in a kernel since it was quite bad taste
  * chore: removed unused variables from the kernel
  * chore: forgot to apply the changes to the kernel call
  * chore: deleted comments
  * chore: deleted unused variable
  * chore: added autos
  * chore: missing period
  * chore: improved logging
  * Update perception/autoware_lidar_bevfusion/lib/bevfusion_trt.cpp
  Co-authored-by: Amadeusz Szymko <amadeuszszymko@gmail.com>
  * chore: changed the name of the package from autoware_lidar_bevfusion to autoware_bevfusion since the model has both lidar and camera_lidar modalities
  * feat: added final config files and launchers
  * doc: finished documentation
  * chore: updated schemas
  * fix: schemas
  * fix: due to how schemas work, changed the names
  * Update perception/autoware_bevfusion/README.md
  Co-authored-by: Amadeusz Szymko <amadeuszszymko@gmail.com>
  * fix: broken link
  * chore: removed simlinks since the schema implementation matches the base name against a wildcard
  * fix: forgot mkdown
  * feat: added oss links
  ---------
  Co-authored-by: Amadeusz Szymko <amadeusz.szymko.2@tier4.jp>
  Co-authored-by: Amadeusz Szymko <amadeuszszymko@gmail.com>
* Contributors: Kenzo Lobos Tsunekawa, Ryohsuke Mitsudome

0.43.0 (2025-03-21)
-------------------
* Merge remote-tracking branch 'origin/main' into chore/bump-version-0.43
* chore: rename from `autoware.universe` to `autoware_universe` (`#10306 <https://github.com/autowarefoundation/autoware_universe/issues/10306>`_)
* refactor: add autoware_cuda_dependency_meta (`#10073 <https://github.com/autowarefoundation/autoware_universe/issues/10073>`_)
* Contributors: Esteve Fernandez, Hayato Mizushima, Yutaka Kondo

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
* fix: fix package names in changelog files (`#9500 <https://github.com/autowarefoundation/autoware_universe/issues/9500>`_)
* 0.39.0
* update changelog
* Merge commit '6a1ddbd08bd' into release-0.39.0
* Revert "refactor(sensing): move CUDA related packages to `sensing/cuda` directory" (`#9394 <https://github.com/autowarefoundation/autoware_universe/issues/9394>`_)
  Revert "refactor(sensing): move CUDA related packages to `sensing/cuda` direc…"
  This reverts commit be8235d785597c41d01782ec35da862ba0906578.
* fix: fix ticket links to point to https://github.com/autowarefoundation/autoware_universe (`#9304 <https://github.com/autowarefoundation/autoware_universe/issues/9304>`_)
* chore(package.xml): bump version to 0.38.0 (`#9266 <https://github.com/autowarefoundation/autoware_universe/issues/9266>`_) (`#9284 <https://github.com/autowarefoundation/autoware_universe/issues/9284>`_)
  * unify package.xml version to 0.37.0
  * remove system_monitor/CHANGELOG.rst
  * add changelog
  * 0.38.0
  ---------
* refactor(cuda_utils): prefix package and namespace with autoware (`#9171 <https://github.com/autowarefoundation/autoware_universe/issues/9171>`_)
* Contributors: Esteve Fernandez, Fumiya Watanabe, Ryohsuke Mitsudome, Yutaka Kondo

0.39.0 (2024-11-25)
-------------------
* Merge commit '6a1ddbd08bd' into release-0.39.0
* fix: fix ticket links to point to https://github.com/autowarefoundation/autoware_universe (`#9304 <https://github.com/autowarefoundation/autoware_universe/issues/9304>`_)
* chore(package.xml): bump version to 0.38.0 (`#9266 <https://github.com/autowarefoundation/autoware_universe/issues/9266>`_) (`#9284 <https://github.com/autowarefoundation/autoware_universe/issues/9284>`_)
  * unify package.xml version to 0.37.0
  * remove system_monitor/CHANGELOG.rst
  * add changelog
  * 0.38.0
  ---------
* refactor(cuda_utils): prefix package and namespace with autoware (`#9171 <https://github.com/autowarefoundation/autoware_universe/issues/9171>`_)
* Contributors: Esteve Fernandez, Yutaka Kondo

0.38.0 (2024-11-08)
-------------------
* unify package.xml version to 0.37.0
* chore: added maintainers to the cuda_utils package (`#9042 <https://github.com/autowarefoundation/autoware_universe/issues/9042>`_)
* refactor(cuda_utils): move cuda_utils to sensing (`#8729 <https://github.com/autowarefoundation/autoware_universe/issues/8729>`_)
  fix: move cuda_utils to sensing
* Contributors: Kenzo Lobos Tsunekawa, Yoshi Ri, Yutaka Kondo

0.26.0 (2024-04-03)
-------------------
