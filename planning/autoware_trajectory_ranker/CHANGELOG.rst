^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_trajectory_ranker
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.48.0 (2025-11-18)
-------------------
* Merge remote-tracking branch 'origin/main' into humble
* fix: tf2 uses hpp headers in rolling (and is backported) (`#11620 <https://github.com/autowarefoundation/autoware_universe/issues/11620>`_)
* feat(trajectory_ranker): add trajectory ranker (`#11318 <https://github.com/autowarefoundation/autoware_universe/issues/11318>`_)
  * add basic implementation of trajectory ranker
  * fix repeating call when calculation jerk
  * use range based for loop
  * remove unnecessary static_cast<std::ptrdiff_t>
  * use move semantic
  * change file name and include directory structure
  * change type double to float
  * fix package.xml
  * fix CMakeList
  * add ndoe suffix
  * use early return
  * avoid zero division for metrics calculation
  * use parameter for ttc calcultion in metrics
  * fix metric calculation
  ---------
* Contributors: Go Sakayori, Ryohsuke Mitsudome, Tim Clephas
