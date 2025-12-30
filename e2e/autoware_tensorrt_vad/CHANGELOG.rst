^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_tensorrt_vad
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
