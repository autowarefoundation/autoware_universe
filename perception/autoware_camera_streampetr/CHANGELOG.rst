^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_camera_streampetr
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.48.0 (2025-11-18)
-------------------
* Merge remote-tracking branch 'origin/main' into humble
* fix: tf2 uses hpp headers in rolling (and is backported) (`#11620 <https://github.com/autowarefoundation/autoware_universe/issues/11620>`_)
* feat(autoware_camera_streampetr): cuda based undistortion with rectification   (`#11420 <https://github.com/autowarefoundation/autoware_universe/issues/11420>`_)
  * working inference with distortion
  * removed unnecessary code
  * remove unused parameter
  * style(pre-commit): autofix
  * fixed based on comments
  * style(pre-commit): autofix
  * added unroll
  * style(pre-commit): autofix
  * comment for clarity
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci-lite[bot] <117423508+pre-commit-ci-lite[bot]@users.noreply.github.com>
* feat(autoware_camera_streampetr): implementation of StreamPETR using tensorrt (`#11139 <https://github.com/autowarefoundation/autoware_universe/issues/11139>`_)
  * added streampetr
  * use trt_common for build and forward pass
  * style(pre-commit): autofix
  * use optional parameters
  * remove unused methods
  * style(pre-commit): autofix
  * fix lint errors
  * ament
  * style(pre-commit): autofix
  * refactor complex code
  * simplified functions
  * style(pre-commit): autofix
  * removed uncrustify
  * fix clang errors
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: pre-commit-ci-lite[bot] <117423508+pre-commit-ci-lite[bot]@users.noreply.github.com>
* Contributors: Ryohsuke Mitsudome, Samrat Thapa, Tim Clephas
