^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_tensorrt_bevformer
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.48.0 (2025-11-18)
-------------------
* Merge remote-tracking branch 'origin/main' into humble
* fix: cppcheck for struct Detection (`#11649 <https://github.com/autowarefoundation/autoware_universe/issues/11649>`_)
  * fix: remove unused Detection struct
  * fix: add namespace for bevformer postprocessing
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci-lite[bot] <117423508+pre-commit-ci-lite[bot]@users.noreply.github.com>
* fix: tf2 uses hpp headers in rolling (and is backported) (`#11620 <https://github.com/autowarefoundation/autoware_universe/issues/11620>`_)
* feat(autoware_bevformer): implementation of bevformer using tensorrt (`#11076 <https://github.com/autowarefoundation/autoware_universe/issues/11076>`_)
* Contributors: Naveen, Ryohsuke Mitsudome, Tim Clephas
