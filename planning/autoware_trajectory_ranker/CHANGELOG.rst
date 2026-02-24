^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_trajectory_ranker
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.50.0 (2026-02-14)
-------------------
* Merge remote-tracking branch 'origin/main' into humble
* chore(trajectory_ranker,trajectory_traffic_rule_filter): add maintainers (`#12020 <https://github.com/autowarefoundation/autoware_universe/issues/12020>`_)
  chore(trajectory_ranker,traffic_rule_filter): add maintainers
* fix(autoware_trajectory_ranker): prevent node crash when performing inner product operation (`#12009 <https://github.com/autowarefoundation/autoware_universe/issues/12009>`_)
  * fix: prevent not crash when performing inner product operation
  * fix: missing code
  * fix: wrong variable
  ---------
* Contributors: Maxime CLEMENT, Ryohsuke Mitsudome, Zulfaqar Azmi

0.49.0 (2025-12-30)
-------------------
* Merge remote-tracking branch 'origin/main' into prepare-0.49.0-changelog
* feat(trajectory_ranker): add trajectory consistency score (`#11762 <https://github.com/autowarefoundation/autoware_universe/issues/11762>`_)
  * add trajectory consistency score
  * add test
  * metric with configurable parameters
  * fix pre-commit
  * fix comments
  * fix calculation of total variance
  * extract common ego frame transformation logic
  * updated test
  ---------
* Contributors: Go Sakayori, Ryohsuke Mitsudome

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
