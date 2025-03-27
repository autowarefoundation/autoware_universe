^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_traffic_light_selector
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
