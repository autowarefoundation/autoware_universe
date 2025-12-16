^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_calibration_status_classifier
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.48.0 (2025-11-18)
-------------------
* Merge remote-tracking branch 'origin/main' into humble
* fix: tf2 uses hpp headers in rolling (and is backported) (`#11620 <https://github.com/autowarefoundation/autoware_universe/issues/11620>`_)
* feat(autoware_calibration_status_classifier): add ML-based miscalibration detection module (`#11222 <https://github.com/autowarefoundation/autoware_universe/issues/11222>`_)
  * feat(autoware_calibration_status): add ML-based miscalibration detection module
  * feat(autoware_calibration_status): extended configuration and diagnostics
  * fix(autoware_calibration_status): model's input array format
  * test(autoware_calibration_status): inference only test
  * style(pre-commit): autofix
  * refactor(autoware_calibration_status): rename lidar_range to max_depth
  * fix(autoware_calibration_status): add missing header
  * feat(autoware_calibration_status): add naive number of objects filter
  * feat(autoware_calibration_status): add periodic and manual mode
  * refactor(autoware_calibration_status): improve image handling and optimize calibration pipeline
  Refactors the calibration status module to handle both distorted and rectified images,
  reorganizes data structures, and optimizes the processing pipeline. Adds new utility
  classes for better camera/LiDAR information management.
  * style(pre-commit): autofix
  * style(autoware_calibration_status): pre-commit
  * test(autoware_calibration_status): make that CI skip unit tests
  * style(autoware_calibration_status): cspell
  * test(autoware_calibration_status): skip test before loading data
  * test(autoware_calibration_status): another yet attempt to fix CI
  * style(autoware_calibration_status): cspell
  * fix(autoware_calibration_status): correct types
  Co-authored-by: Manato Hirabayashi <3022416+manato@users.noreply.github.com>
  * fix(autoware_calibration_status): correct types
  Co-authored-by: Manato Hirabayashi <3022416+manato@users.noreply.github.com>
  * style(pre-commit): autofix
  * fix(autoware_calibration_status): user desired function for cuda memory allocation
  * style(autoware_calibration_status): early return instead of scoped implementaiton
  * feat(autoware_calibration_status): use of  __restrict_\_ keyword
  * docs(autoware_calibration_status): update future work
  * fix(autoware_calibration_status): include missing directory
  * fix(autoware_calibration_status): use preallocated class member
  * style(pre-commit): autofix
  * style(autoware_calibration_status): use lambda for adding diagnostic
  * style(autoware_calibration_status): split function
  * style(pre-commit): autofix
  * refactor(autoware_calibration_status): change atomic operation logic and extras
  * refactor(autoware_calibration_status): use autoware diagnostic interface
  * fix(autoware_calibration_status): cspell
  * feat(autoware_calibration_status_classifier): rename autoware_calibration_status to autoware_calibration_status_classifier
  * style(pre-commit): autofix
  * fix(autoware_calibration_status_classifier): prevent potential race condition
  * fix(autoware_calibration_status_classifier): add mutex for input msgs data access
  * fix(autoware_calibration_status_classifier): pre-commit
  ---------
  Co-authored-by: pre-commit-ci-lite[bot] <117423508+pre-commit-ci-lite[bot]@users.noreply.github.com>
  Co-authored-by: Manato Hirabayashi <3022416+manato@users.noreply.github.com>
* Contributors: Amadeusz Szymko, Ryohsuke Mitsudome, Tim Clephas
