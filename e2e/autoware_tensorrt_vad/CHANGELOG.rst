^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_tensorrt_vad
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.50.0 (2026-02-14)
-------------------
* Merge remote-tracking branch 'origin/main' into humble
* feat(autoware_tensorrt_vad): update nvcc flags (`#12044 <https://github.com/autowarefoundation/autoware_universe/issues/12044>`_)
  Co-authored-by: Max-Bin <vborisw@gmail.com>
* fix(autoware_tensorrt_vad): remove unnecessary USE_SCOPED_HEADER_INST… (`#11988 <https://github.com/autowarefoundation/autoware_universe/issues/11988>`_)
  fix(autoware_tensorrt_vad): remove unnecessary USE_SCOPED_HEADER_INSTALL_DIR
  The USE_SCOPED_HEADER_INSTALL_DIR flag is unnecessary for this package
  because:
  - The package has no include/ directory (no public headers)
  - All headers are private in src/ directory
  - This flag only affects public header installation paths
  - It can cause build failures in certain configurations
  This flag was the only usage in autoware_universe and provided no benefit.
* chore: remove autoware_launch exec_depend from e2e/autoware_tensorrt_vad (`#11879 <https://github.com/autowarefoundation/autoware_universe/issues/11879>`_)
  Co-authored-by: Max-Bin <vborisw@gmail.com>
* fix(autoware_tensorrt_vad): remove unused function `load_object_configuration` (`#11909 <https://github.com/autowarefoundation/autoware_universe/issues/11909>`_)
* fix(autoware_tensorrt_vad): remove unused function `load_map_configuration` (`#11908 <https://github.com/autowarefoundation/autoware_universe/issues/11908>`_)
* fix(autoware_tensorrt_vad): remove unused function `load_image_normalization` (`#11910 <https://github.com/autowarefoundation/autoware_universe/issues/11910>`_)
* fix: add cv_bridge.hpp support (`#11873 <https://github.com/autowarefoundation/autoware_universe/issues/11873>`_)
* Contributors: Amadeusz Szymko, Max-Bin, Mete Fatih Cırıt, Ryohsuke Mitsudome, Ryuta Kambe, Taeseung Sohn

0.49.0 (2025-12-30)
-------------------
* chore: align version number
* Merge remote-tracking branch 'origin/main' into prepare-0.49.0-changelog
* refactor: migrate autoware_tensorrt_vad to e2e directory (`#11730 <https://github.com/autowarefoundation/autoware_universe/issues/11730>`_)
  Move autoware_tensorrt_vad package from planning/ to e2e/ directory
  to align with AWF's organization of end-to-end learning-based
  components.
  Changes:
  - Created e2e/ directory for end-to-end components
  - Moved all autoware_tensorrt_vad files from planning/ to e2e/
  - Updated documentation references to reflect new location
  - Verified package builds successfully in new location
  This follows the pattern established in the AWF main repository:
  https://github.com/autowarefoundation/autoware_universe/tree/main/e2e
* Contributors: Max-Bin, Ryohsuke Mitsudome
