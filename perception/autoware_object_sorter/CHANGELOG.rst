^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_object_sorter
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
