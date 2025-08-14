^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_object_merger
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.47.1 (2025-08-14)
-------------------

0.47.0 (2025-08-11)
-------------------
* fix(autoware_object_merger): add merger priority_mode (`#11042 <https://github.com/autowarefoundation/autoware_universe/issues/11042>`_)
  * fix: add merger priority_mode
  fix: add priority mode into launch
  fix: add class based priority matrix
  fix: adjust priority matrix
  * fix: add Confidence mode support
  * docs: schema update
  * fix: launch
  * fix: schema json
  ---------
* Contributors: badai nguyen

0.46.0 (2025-06-20)
-------------------

0.45.0 (2025-05-22)
-------------------
* Merge remote-tracking branch 'origin/main' into tmp/notbot/bump_version_base
* chore: perception code owner update (`#10645 <https://github.com/autowarefoundation/autoware_universe/issues/10645>`_)
  * chore: update maintainers in multiple perception packages
  * Revert "chore: update maintainers in multiple perception packages"
  This reverts commit f2838c33d6cd82bd032039e2a12b9cb8ba6eb584.
  * chore: update maintainers in multiple perception packages
  * chore: add Kok Seang Tan as maintainer in multiple perception packages
  ---------
* Contributors: Taekjin LEE, TaikiYamada4

0.44.2 (2025-06-10)
-------------------

0.44.1 (2025-05-01)
-------------------

0.44.0 (2025-04-18)
-------------------
* Merge remote-tracking branch 'origin/main' into humble
* feat(object_merger): add diagnostics warning for timeout when merging (`#10283 <https://github.com/autowarefoundation/autoware_universe/issues/10283>`_)
  * feat(object_merger): add diagnostics warning for timeout when merge messages are incomplete
  * feat(object_merger): add parameters loading
  * style(pre-commit): autofix
  * feat(object_merger): add schema parameters
  * feat(object_merger):  1. add initialization timeout check 2. handle for now() uninitialized problem in constructor  3. threshold tuned
  * feat(object_merger): threshold modified and schema accordingly
  * feat(object_merger): check both interval and elapsed time
  * Update perception/autoware_object_merger/src/object_association_merger_node.cpp
  Co-authored-by: Taekjin LEE <technolojin@gmail.com>
  * Update perception/autoware_object_merger/src/object_association_merger_node.cpp
  Co-authored-by: Taekjin LEE <technolojin@gmail.com>
  * feat(object_merger): only update the message arrival timing in objectCallback
  * feat(object_merger): checkstatus impl moved to diagcallback
  * Update perception/autoware_object_merger/src/object_association_merger_node.cpp
  Co-authored-by: Taekjin LEE <technolojin@gmail.com>
  * Update perception/autoware_object_merger/src/object_association_merger_node.cpp
  Co-authored-by: Taekjin LEE <technolojin@gmail.com>
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: Taekjin LEE <technolojin@gmail.com>
* Contributors: Ryohsuke Mitsudome, lei.gu

0.43.0 (2025-03-21)
-------------------
* Merge remote-tracking branch 'origin/main' into chore/bump-version-0.43
* chore: rename from `autoware.universe` to `autoware_universe` (`#10306 <https://github.com/autowarefoundation/autoware_universe/issues/10306>`_)
* Contributors: Hayato Mizushima, Yutaka Kondo

0.42.0 (2025-03-03)
-------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* feat(autoware_utils): replace autoware_universe_utils with autoware_utils  (`#10191 <https://github.com/autowarefoundation/autoware_universe/issues/10191>`_)
* Contributors: Fumiya Watanabe, 心刚

0.41.2 (2025-02-19)
-------------------
* chore: bump version to 0.41.1 (`#10088 <https://github.com/autowarefoundation/autoware_universe/issues/10088>`_)
* Contributors: Ryohsuke Mitsudome

0.41.1 (2025-02-10)
-------------------

0.41.0 (2025-01-29)
-------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* feat(autoware_object_merger): tier4_debug_msgs changed to autoware_internal_debug_msgs in fil… (`#9893 <https://github.com/autowarefoundation/autoware_universe/issues/9893>`_)
  feat: tier4_debug_msgs changed to autoware_internal_debug_msgs in files perception/autoware_object_merger
  Co-authored-by: Taekjin LEE <taekjin.lee@tier4.jp>
* fix(perception): fix perception docs (`#9766 <https://github.com/autowarefoundation/autoware_universe/issues/9766>`_)
  * fix: fix perception docs
  * fix: fix missing parameter in schema
  * Update perception/autoware_object_merger/schema/data_association_matrix.schema.json
  Co-authored-by: Taekjin LEE <technolojin@gmail.com>
  * Update perception/autoware_object_merger/schema/data_association_matrix.schema.json
  Co-authored-by: Taekjin LEE <technolojin@gmail.com>
  * Update perception/autoware_object_merger/schema/data_association_matrix.schema.json
  Co-authored-by: Taekjin LEE <technolojin@gmail.com>
  * Update perception/autoware_object_merger/schema/data_association_matrix.schema.json
  Co-authored-by: Taekjin LEE <technolojin@gmail.com>
  * style(pre-commit): autofix
  * chore: seperate paramters for different nodes
  ---------
  Co-authored-by: Taekjin LEE <technolojin@gmail.com>
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* feat(autoware_object_merger, autoware_tracking_object_merger): enable anonymized node names to be configurable (`#9733 <https://github.com/autowarefoundation/autoware_universe/issues/9733>`_)
  feat: enable anonymized node names to be configurable
* Contributors: Fumiya Watanabe, Taekjin LEE, Vishal Chauhan, Yi-Hsiang Fang (Vivid)

0.40.0 (2024-12-12)
-------------------
* Merge branch 'main' into release-0.40.0
* Revert "chore(package.xml): bump version to 0.39.0 (`#9587 <https://github.com/autowarefoundation/autoware_universe/issues/9587>`_)"
  This reverts commit c9f0f2688c57b0f657f5c1f28f036a970682e7f5.
* fix: fix ticket links in CHANGELOG.rst (`#9588 <https://github.com/autowarefoundation/autoware_universe/issues/9588>`_)
* chore(package.xml): bump version to 0.39.0 (`#9587 <https://github.com/autowarefoundation/autoware_universe/issues/9587>`_)
  * chore(package.xml): bump version to 0.39.0
  * fix: fix ticket links in CHANGELOG.rst
  * fix: remove unnecessary diff
  ---------
  Co-authored-by: Yutaka Kondo <yutaka.kondo@youtalk.jp>
* fix: fix ticket links in CHANGELOG.rst (`#9588 <https://github.com/autowarefoundation/autoware_universe/issues/9588>`_)
* fix(cpplint): include what you use - perception (`#9569 <https://github.com/autowarefoundation/autoware_universe/issues/9569>`_)
* 0.39.0
* update changelog
* fix: fix ticket links to point to https://github.com/autowarefoundation/autoware_universe (`#9304 <https://github.com/autowarefoundation/autoware_universe/issues/9304>`_)
* fix: fix ticket links to point to https://github.com/autowarefoundation/autoware_universe (`#9304 <https://github.com/autowarefoundation/autoware_universe/issues/9304>`_)
* chore(package.xml): bump version to 0.38.0 (`#9266 <https://github.com/autowarefoundation/autoware_universe/issues/9266>`_) (`#9284 <https://github.com/autowarefoundation/autoware_universe/issues/9284>`_)
  * unify package.xml version to 0.37.0
  * remove system_monitor/CHANGELOG.rst
  * add changelog
  * 0.38.0
  ---------
* Contributors: Esteve Fernandez, Fumiya Watanabe, M. Fatih Cırıt, Ryohsuke Mitsudome, Yutaka Kondo

0.39.0 (2024-11-25)
-------------------
* fix: fix ticket links to point to https://github.com/autowarefoundation/autoware_universe (`#9304 <https://github.com/autowarefoundation/autoware_universe/issues/9304>`_)
* fix: fix ticket links to point to https://github.com/autowarefoundation/autoware_universe (`#9304 <https://github.com/autowarefoundation/autoware_universe/issues/9304>`_)
* chore(package.xml): bump version to 0.38.0 (`#9266 <https://github.com/autowarefoundation/autoware_universe/issues/9266>`_) (`#9284 <https://github.com/autowarefoundation/autoware_universe/issues/9284>`_)
  * unify package.xml version to 0.37.0
  * remove system_monitor/CHANGELOG.rst
  * add changelog
  * 0.38.0
  ---------
* Contributors: Esteve Fernandez, Yutaka Kondo

0.38.0 (2024-11-08)
-------------------
* unify package.xml version to 0.37.0
* refactor(object_recognition_utils): add autoware prefix to object_recognition_utils (`#8946 <https://github.com/autowarefoundation/autoware_universe/issues/8946>`_)
* fix(autoware_object_merger): default merger priority within enum range (`#8858 <https://github.com/autowarefoundation/autoware_universe/issues/8858>`_)
  fix: default merger priority within enum range
* fix(object_association_merger_node): fix the frame id of output object msg  (`#8674 <https://github.com/autowarefoundation/autoware_universe/issues/8674>`_)
  fix: fix the object msg header
* fix(doc, object_merger): fix object merger document path (`#8292 <https://github.com/autowarefoundation/autoware_universe/issues/8292>`_)
  fix object merger document path
* fix(autoware_object_merger): fix passedByValue (`#8232 <https://github.com/autowarefoundation/autoware_universe/issues/8232>`_)
  fix:passedByValue
* fix(multi_object_tracker, object_merger, radar_object_tracker, tracking_object_merger): fix knownConditionTrueFalse warnings (`#8137 <https://github.com/autowarefoundation/autoware_universe/issues/8137>`_)
  * fix: cppcheck knownConditionTrueFalse
  * fix
  * fix
  ---------
* refactor(autoware_object_merger): move headers to src and rename package (`#7804 <https://github.com/autowarefoundation/autoware_universe/issues/7804>`_)
* Contributors: Esteve Fernandez, Ryuta Kambe, Taekjin LEE, Yi-Hsiang Fang (Vivid), Yutaka Kondo, Yuxuan Liu, kobayu858

0.26.0 (2024-04-03)
-------------------
