^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_bevfusion
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.47.1 (2025-08-14)
-------------------

0.47.0 (2025-08-11)
-------------------
* style(pre-commit): update to clang-format-20 (`#11088 <https://github.com/autowarefoundation/autoware_universe/issues/11088>`_)
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* Contributors: Mete Fatih Cırıt

0.46.0 (2025-06-20)
-------------------
* Merge remote-tracking branch 'upstream/main' into tmp/TaikiYamada/bump_version_base
* fix(autoware_bevfusion): fix clang-tidy errors by removing unused fields (`#10850 <https://github.com/autowarefoundation/autoware_universe/issues/10850>`_)
  * fix clang-tidy errors by removing unused fields
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* fix(cmake): update spconv availability messages to use STATUS and WAR… (`#10690 <https://github.com/autowarefoundation/autoware_universe/issues/10690>`_)
  fix(cmake): update spconv availability messages to use STATUS and WARNING
* Contributors: Taiki Yamada, TaikiYamada4, Yukihiro Saito

0.45.0 (2025-05-22)
-------------------
* fix(autoware_bevfusion): fix clang-tidy errors by removing unused fields (`#10850 <https://github.com/autowarefoundation/autoware_universe/issues/10850>`_)
  * fix clang-tidy errors by removing unused fields
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* Merge remote-tracking branch 'origin/main' into tmp/notbot/bump_version_base
* chore: perception code owner update (`#10645 <https://github.com/autowarefoundation/autoware_universe/issues/10645>`_)
  * chore: update maintainers in multiple perception packages
  * Revert "chore: update maintainers in multiple perception packages"
  This reverts commit f2838c33d6cd82bd032039e2a12b9cb8ba6eb584.
  * chore: update maintainers in multiple perception packages
  * chore: add Kok Seang Tan as maintainer in multiple perception packages
  ---------
* feat(autoware_bevfusion): added target architectures for bevfusion (`#10613 <https://github.com/autowarefoundation/autoware_universe/issues/10613>`_)
  * chore: added target architectures for bevfusion
  * chore: mistook the architecture of edge devices
  ---------
* feat(bevfusion.schema): add default values for sensor_fusion and thresholds array (`#10608 <https://github.com/autowarefoundation/autoware_universe/issues/10608>`_)
* fix(autoware_bevfusion): build error when using ninja-build tool (`#10551 <https://github.com/autowarefoundation/autoware_universe/issues/10551>`_)
* feat(autoware_bevfusion): integrated the cuda blackboard to bevfusion (`#10540 <https://github.com/autowarefoundation/autoware_universe/issues/10540>`_)
  * feat: integrated the cuda blackboard to bevfusion
  * chore: typo
  * chore: the wildcard matching of schemas is a pain
  ---------
* Contributors: Kenzo Lobos Tsunekawa, Taekjin LEE, TaikiYamada4, Zulfaqar Azmi

0.44.2 (2025-06-10)
-------------------

0.44.1 (2025-05-01)
-------------------

0.44.0 (2025-04-18)
-------------------
* chore: match all package versions
* Merge remote-tracking branch 'origin/main' into humble
* feat(autoware_tensorrt_plugins): created a package for tensorrt extensions (`#10445 <https://github.com/autowarefoundation/autoware_universe/issues/10445>`_)
  * feat: moved the plugins in bevfusion to a separate package since some of them will be reused
  * doc: doc regarding the plugins and the supported ops
  * chore: wrong upper cases
  * chore: wrong quotes
  * chore: fixed docs
  ---------
* chore(autoware_bevfusion): add maintainer (`#10444 <https://github.com/autowarefoundation/autoware_universe/issues/10444>`_)
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
* Contributors: Amadeusz Szymko, Kenzo Lobos Tsunekawa, Ryohsuke Mitsudome
