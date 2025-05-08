^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_bevfusion
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
