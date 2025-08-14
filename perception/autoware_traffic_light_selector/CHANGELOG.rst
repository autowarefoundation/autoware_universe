^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_traffic_light_selector
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.47.1 (2025-08-14)
-------------------

0.47.0 (2025-08-11)
-------------------

0.46.0 (2025-06-20)
-------------------

0.45.0 (2025-05-22)
-------------------
* Merge remote-tracking branch 'origin/main' into tmp/notbot/bump_version_base
* chore: update traffic light packages code owner (`#10644 <https://github.com/autowarefoundation/autoware_universe/issues/10644>`_)
  chore: add Taekjin Lee as maintainer to multiple perception packages
* feat(autoware_traffic_light_selector): new matching algorithm and unit test (`#10352 <https://github.com/autowarefoundation/autoware_universe/issues/10352>`_)
  * refactor and test
  * style(pre-commit): autofix
  * add dependency
  * unnecessary dependency
  * chore
  * fix typo
  * remove change in category_merger
  * chnage var name
  * add validation shiftRoi
  * add new matching algo
  * modify unittest
  * remove unnecessary file
  * change type from uint8_t to int64_t
  * change  variable name in looping
  Co-authored-by: badai nguyen  <94814556+badai-nguyen@users.noreply.github.com>
  * use move instead of copy
  Co-authored-by: badai nguyen  <94814556+badai-nguyen@users.noreply.github.com>
  * change variable name in utils
  * apply  header file
  * to pass cppcheck
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: badai nguyen <94814556+badai-nguyen@users.noreply.github.com>
* Contributors: Masato Saeki, Taekjin LEE, TaikiYamada4

0.44.2 (2025-06-10)
-------------------

0.44.1 (2025-05-01)
-------------------

0.44.0 (2025-04-18)
-------------------

0.43.0 (2025-03-21)
-------------------
* Merge remote-tracking branch 'origin/main' into chore/bump-version-0.43
* chore: rename from `autoware.universe` to `autoware_universe` (`#10306 <https://github.com/autowarefoundation/autoware_universe/issues/10306>`_)
* Contributors: Hayato Mizushima, Yutaka Kondo

0.42.0 (2025-03-03)
-------------------
* fix: fix version
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* feat(autoware_utils): replace autoware_universe_utils with autoware_utils  (`#10191 <https://github.com/autowarefoundation/autoware_universe/issues/10191>`_)
* build(autoware_traffic_light_selector): fix missing sophus dependency (`#10141 <https://github.com/autowarefoundation/autoware_universe/issues/10141>`_)
  * build(autoware_traffic_light_selector): fix missing sophus dependency
  * fix missing cgal dependency
  ---------
* fix(autoware_traffic_light_selector): add camera_info into message_filter (`#10089 <https://github.com/autowarefoundation/autoware_universe/issues/10089>`_)
  * add mutex
  * change message filter
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* feat(traffic_light_selector): add new node for traffic light selection (`#9721 <https://github.com/autowarefoundation/autoware_universe/issues/9721>`_)
  * feat: add traffic light selector node
  feat: add traffic ligth selector node
  * fix: add check expect roi iou
  * fix: tl selector
  * fix: launch file
  * fix: update matching score
  * fix: calc sum IOU for whole shifted image
  * fix: check inside rough roi
  * fix: check inside function
  * feat: add max_iou_threshold
  * chore: pre-commit
  * docs: add readme
  * refactor: launch file
  * docs: pre-commit
  * docs
  * chore: typo
  * refactor
  * fix: add unknown in selector
  * fix: change to GenIOU
  * feat: add debug topic
  * fix: add maintainer
  * chore: pre-commit
  * fix:cmake
  * fix: move param to yaml file
  * fix: typo
  * fix: add schema
  * fix
  * style(pre-commit): autofix
  * fix typo
  ---------
  Co-authored-by: Masato Saeki <78376491+MasatoSaeki@users.noreply.github.com>
  Co-authored-by: MasatoSaeki <masato.saeki@tier4.jp>
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* Contributors: Esteve Fernandez, Fumiya Watanabe, Masato Saeki, badai nguyen, 心刚

* fix: fix version
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* feat(autoware_utils): replace autoware_universe_utils with autoware_utils  (`#10191 <https://github.com/autowarefoundation/autoware_universe/issues/10191>`_)
* build(autoware_traffic_light_selector): fix missing sophus dependency (`#10141 <https://github.com/autowarefoundation/autoware_universe/issues/10141>`_)
  * build(autoware_traffic_light_selector): fix missing sophus dependency
  * fix missing cgal dependency
  ---------
* fix(autoware_traffic_light_selector): add camera_info into message_filter (`#10089 <https://github.com/autowarefoundation/autoware_universe/issues/10089>`_)
  * add mutex
  * change message filter
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* feat(traffic_light_selector): add new node for traffic light selection (`#9721 <https://github.com/autowarefoundation/autoware_universe/issues/9721>`_)
  * feat: add traffic light selector node
  feat: add traffic ligth selector node
  * fix: add check expect roi iou
  * fix: tl selector
  * fix: launch file
  * fix: update matching score
  * fix: calc sum IOU for whole shifted image
  * fix: check inside rough roi
  * fix: check inside function
  * feat: add max_iou_threshold
  * chore: pre-commit
  * docs: add readme
  * refactor: launch file
  * docs: pre-commit
  * docs
  * chore: typo
  * refactor
  * fix: add unknown in selector
  * fix: change to GenIOU
  * feat: add debug topic
  * fix: add maintainer
  * chore: pre-commit
  * fix:cmake
  * fix: move param to yaml file
  * fix: typo
  * fix: add schema
  * fix
  * style(pre-commit): autofix
  * fix typo
  ---------
  Co-authored-by: Masato Saeki <78376491+MasatoSaeki@users.noreply.github.com>
  Co-authored-by: MasatoSaeki <masato.saeki@tier4.jp>
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* Contributors: Esteve Fernandez, Fumiya Watanabe, Masato Saeki, badai nguyen, 心刚
