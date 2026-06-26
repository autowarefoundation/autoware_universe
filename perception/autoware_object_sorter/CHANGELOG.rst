^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_object_sorter
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.52.0 (2026-06-26)
-------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* fix(autoware_object_sorter): remove hard coded label configuration (`#12828 <https://github.com/autowarefoundation/autoware_universe/issues/12828>`_)
  * fix label handling
  * add test
  ---------
* fix(object_sorter): update params for new ANIMAL and HAZARD classes (`#12744 <https://github.com/autowarefoundation/autoware_universe/issues/12744>`_)
  * fix(object_sorter): update params for new ANIMAL and HAZARD classes
  * fix: update tracked_objects config
  ---------
* feat(object_sorter): apply `agnocast_wrapper::Node` to object_sorter (`#12705 <https://github.com/autowarefoundation/autoware_universe/issues/12705>`_)
  * apply agnocast
  * style(pre-commit): autofix
  * delete unnecessary comments
  * delete suppress comments
  * add intra_subscription_count: copilot review
  * make Cmakelists.txt simple
  ---------
  Co-authored-by: pre-commit-ci-lite[bot] <117423508+pre-commit-ci-lite[bot]@users.noreply.github.com>
* Contributors: Koichi Imai, Masaki Baba, badai nguyen, github-actions

0.51.0 (2026-05-01)
-------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* chore(perception): move perception node configuration file to each package (`#12440 <https://github.com/mitsudome-r/autoware_universe/issues/12440>`_)
  move perception node configuration file to each package
* feat(tracked_object_sorter): apply autoware_agnocast_wrapper for CIE (`#12333 <https://github.com/mitsudome-r/autoware_universe/issues/12333>`_)
  * feat(autoware_object_sorter): apply autoware_agnocast_wrapper for CIE
  * feat(autoware_object_sorter): apply CIE to DetectedObjectSorterNode as well
  ---------
* Contributors: Taekjin LEE, atsushi yano, github-actions

0.50.0 (2026-02-14)
-------------------
* Merge remote-tracking branch 'origin/main' into humble
* fix(autoware_object_sorter): fix object transformation (`#11957 <https://github.com/autowarefoundation/autoware_universe/issues/11957>`_)
  * fix object transform
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci-lite[bot] <117423508+pre-commit-ci-lite[bot]@users.noreply.github.com>
* Contributors: Masaki Baba, Ryohsuke Mitsudome

0.49.0 (2025-12-30)
-------------------

0.48.0 (2025-11-18)
-------------------
* Merge remote-tracking branch 'origin/main' into humble
* feat(object_sorter): add per-axis min-max range filter for each class label (`#11563 <https://github.com/autowarefoundation/autoware_universe/issues/11563>`_)
  * add min max detection range and class settings
  * fix typo
  * ensure the highest label get selected
  * refactor
  Co-authored-by: Yoshi Ri <yoshiyoshidetteiu@gmail.com>
  * fix transform to process in the target frame
  ---------
  Co-authored-by: Yoshi Ri <yoshiyoshidetteiu@gmail.com>
* chore(perception): add maintainer (`#11458 <https://github.com/autowarefoundation/autoware_universe/issues/11458>`_)
  add maintainer
* Contributors: Masaki Baba, Ryohsuke Mitsudome

0.47.1 (2025-08-14)
-------------------

0.47.0 (2025-08-11)
-------------------
* feat(autoware_object_sorter): add object_sorter package (`#10925 <https://github.com/autowarefoundation/autoware_universe/issues/10925>`_)
  * add object_sorter
  * add transform
  * rename parameters
  * add target frame id paramter
  * change to skipping when transfrom fails
  * fix url and typo
  * add maintainer
  ---------
  Co-authored-by: Taekjin LEE <taekjin.lee@tier4.jp>
* Contributors: Masaki Baba
