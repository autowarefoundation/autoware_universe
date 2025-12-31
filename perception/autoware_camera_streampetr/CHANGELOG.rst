^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_camera_streampetr
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.49.0 (2025-12-30)
-------------------
* Merge remote-tracking branch 'origin/main' into prepare-0.49.0-changelog
* chore(streampetr): removed thrust stream policy (`#11800 <https://github.com/autowarefoundation/autoware_universe/issues/11800>`_)
  * removed thrust stream
  * syncronize stream before thrust
  ---------
* fix: prevent possible dangling pointer from .str().c_str() pattern (`#11609 <https://github.com/autowarefoundation/autoware_universe/issues/11609>`_)
  * Fix dangling pointer caused by the .str().c_str() pattern.
  std::stringstream::str() returns a temporary std::string,
  and taking its c_str() leads to a dangling pointer when the temporary is destroyed.
  This patch replaces such usage with a const reference of std::string variable to ensure pointer validity.
  * Revert the changes made to the functions. They should only be applied to the macros.
  ---------
  Co-authored-by: Shumpei Wakabayashi <42209144+shmpwk@users.noreply.github.com>
  Co-authored-by: Junya Sasaki <junya.sasaki@tier4.jp>
* feat(streampetr): class wise confidence threshold (`#11756 <https://github.com/autowarefoundation/autoware_universe/issues/11756>`_)
  * class wise threshold
  * style(pre-commit): autofix
  * add checks
  * style(pre-commit): autofix
  * prevent cuda reallocatoin
  * updated conf values
  * style(pre-commit): autofix
  * removed unused import
  * add policy for thrust
  * style(pre-commit): autofix
  * use cuda_utils helpers
  ---------
  Co-authored-by: pre-commit-ci-lite[bot] <117423508+pre-commit-ci-lite[bot]@users.noreply.github.com>
* feat(stream_petr): use dynamic triangle filter for image downsampling   (`#11724 <https://github.com/autowarefoundation/autoware_universe/issues/11724>`_)
  * downsample with anti-aliasing
  * synchronize streams
  * remove unused changes
  * style(pre-commit): autofix
  * fixed cuda sync location
  * remove unused code
  * added optimize TODO
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci-lite[bot] <117423508+pre-commit-ci-lite[bot]@users.noreply.github.com>
* Contributors: Ryohsuke Mitsudome, Samrat Thapa, Takatoshi Kondo

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
