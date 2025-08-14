^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_traffic_light_visualization
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.47.1 (2025-08-14)
-------------------

0.47.0 (2025-08-11)
-------------------
* style(pre-commit): update to clang-format-20 (`#11088 <https://github.com/autowarefoundation/autoware_universe/issues/11088>`_)
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* Contributors: Mete Fatih Cırıt

0.46.0 (2025-06-20)
-------------------

0.45.0 (2025-05-22)
-------------------
* Merge remote-tracking branch 'origin/main' into tmp/notbot/bump_version_base
* chore(autoware_traffic_light_visualization): modify docs (`#10345 <https://github.com/autowarefoundation/autoware_universe/issues/10345>`_)
  * fix docs
  * style(pre-commit): autofix
  * add images of docs
  * fix path in json
  * fix typo
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* chore: update traffic light packages code owner (`#10644 <https://github.com/autowarefoundation/autoware_universe/issues/10644>`_)
  chore: add Taekjin Lee as maintainer to multiple perception packages
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
* chore(perception): refactor perception launch (`#10186 <https://github.com/autowarefoundation/autoware_universe/issues/10186>`_)
  * fundamental change
  * style(pre-commit): autofix
  * fix typo
  * fix params and modify some packages
  * pre-commit
  * fix
  * fix spell check
  * fix typo
  * integrate model and label path
  * style(pre-commit): autofix
  * for pre-commit
  * run pre-commit
  * for awsim
  * for simulatior
  * style(pre-commit): autofix
  * fix grammer in launcher
  * add schema for yolox_tlr
  * style(pre-commit): autofix
  * fix file name
  * fix
  * rename
  * modify arg name  to
  * fix typo
  * change param name
  * style(pre-commit): autofix
  * chore
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: Shintaro Tomie <58775300+Shin-kyoto@users.noreply.github.com>
  Co-authored-by: Kenzo Lobos Tsunekawa <kenzo.lobos@tier4.jp>
* Contributors: Hayato Mizushima, Masato Saeki, Yutaka Kondo

0.42.0 (2025-03-03)
-------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* feat(tier4_perception_launch): add option for new TL detector model (`#9731 <https://github.com/autowarefoundation/autoware_universe/issues/9731>`_)
  * feat: add traffic_light_detector launch
  fix: tier4 perception launch
  fix: add multi tlr detector launch
  fix: tier4 launch
  fix: tl detector launch
  fix: data director
  fix: precision int8
  chore: revert to fp16
  feat: remove occlusion and add car ped classification merger
  fix: launch for multi camera
  chore: pre-commit
  fix: update matching score
  feat: add max_iou_threshold
  feat: add occlusion unknown classifier
  * fix: tl detector launch
  * refactor: traffic_light_launch.xml
  * fix: remove tl fine detector
  * fix: refactor
  * chore: pre-commit
  * fix: cspelling check
  * fix: error after rename package
  * fix: default tl model name
  * fix: new tlr for multi cameras
  * modify args
  * style(pre-commit): autofix
  * refactor
  * add category_merger to container
  * fix args
  * run pre-commit
  ---------
  Co-authored-by: Masato Saeki <78376491+MasatoSaeki@users.noreply.github.com>
  Co-authored-by: MasatoSaeki <masato.saeki@tier4.jp>
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* Contributors: Fumiya Watanabe, badai nguyen

0.41.2 (2025-02-19)
-------------------
* chore: bump version to 0.41.1 (`#10088 <https://github.com/autowarefoundation/autoware_universe/issues/10088>`_)
* Contributors: Ryohsuke Mitsudome

0.41.1 (2025-02-10)
-------------------

0.41.0 (2025-01-29)
-------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* fix(autoware_traffic_light_visualization): fix bugprone-branch-clone (`#9668 <https://github.com/autowarefoundation/autoware_universe/issues/9668>`_)
  fix: bugprone-error
* Contributors: Fumiya Watanabe, kobayu858

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
* fix(traffic_light_roi_visualizer): show unknown results correctly (`#9467 <https://github.com/autowarefoundation/autoware_universe/issues/9467>`_)
  fix: show unknown results correctly
* 0.39.0
* update changelog
* Merge commit '6a1ddbd08bd' into release-0.39.0
* fix: fix ticket links to point to https://github.com/autowarefoundation/autoware_universe (`#9304 <https://github.com/autowarefoundation/autoware_universe/issues/9304>`_)
* chore(autoware_traffic_light_visualization): include opencv as system (`#9331 <https://github.com/autowarefoundation/autoware_universe/issues/9331>`_)
* fix: fix ticket links to point to https://github.com/autowarefoundation/autoware_universe (`#9304 <https://github.com/autowarefoundation/autoware_universe/issues/9304>`_)
* chore(autoware_traffic_light*): add maintainer (`#9280 <https://github.com/autowarefoundation/autoware_universe/issues/9280>`_)
  * add fundamental commit
  * add forgot package
  ---------
* chore(package.xml): bump version to 0.38.0 (`#9266 <https://github.com/autowarefoundation/autoware_universe/issues/9266>`_) (`#9284 <https://github.com/autowarefoundation/autoware_universe/issues/9284>`_)
  * unify package.xml version to 0.37.0
  * remove system_monitor/CHANGELOG.rst
  * add changelog
  * 0.38.0
  ---------
* Contributors: Esteve Fernandez, Fumiya Watanabe, M. Fatih Cırıt, Masato Saeki, Ryohsuke Mitsudome, Tao Zhong, Yukinari Hisaki, Yutaka Kondo

0.39.0 (2024-11-25)
-------------------
* Merge commit '6a1ddbd08bd' into release-0.39.0
* fix: fix ticket links to point to https://github.com/autowarefoundation/autoware_universe (`#9304 <https://github.com/autowarefoundation/autoware_universe/issues/9304>`_)
* fix: fix ticket links to point to https://github.com/autowarefoundation/autoware_universe (`#9304 <https://github.com/autowarefoundation/autoware_universe/issues/9304>`_)
* chore(autoware_traffic_light*): add maintainer (`#9280 <https://github.com/autowarefoundation/autoware_universe/issues/9280>`_)
  * add fundamental commit
  * add forgot package
  ---------
* chore(package.xml): bump version to 0.38.0 (`#9266 <https://github.com/autowarefoundation/autoware_universe/issues/9266>`_) (`#9284 <https://github.com/autowarefoundation/autoware_universe/issues/9284>`_)
  * unify package.xml version to 0.37.0
  * remove system_monitor/CHANGELOG.rst
  * add changelog
  * 0.38.0
  ---------
* Contributors: Esteve Fernandez, Masato Saeki, Yutaka Kondo

0.38.0 (2024-11-08)
-------------------
* unify package.xml version to 0.37.0
* fix(docs): fix documentation for traffic light visualization (`#8303 <https://github.com/autowarefoundation/autoware_universe/issues/8303>`_)
  fix docs traffic light visualization
* fix(autoware_traffic_light_visualization): fix to visualize correct color and shapes (`#8428 <https://github.com/autowarefoundation/autoware_universe/issues/8428>`_)
  fix(autoware_traffic_light_visualization): fix vialization to draw correct shapes
  Co-authored-by: Yi-Hsiang Fang (Vivid) <146902905+vividf@users.noreply.github.com>
* fix(traffic_light_visualization): fix funcArgNamesDifferent (`#8156 <https://github.com/autowarefoundation/autoware_universe/issues/8156>`_)
  fix:funcArgNamesDifferent
* fix(traffic_light_visualizer): remove cerr temporarily to avoid flooding logs (`#8294 <https://github.com/autowarefoundation/autoware_universe/issues/8294>`_)
  * fix(traffic_light_visualizer): remove cerr temporarily to avoid flooding logs
  * fix precommit
  * fix
  ---------
* fix(autoware_traffic_light_visualization): fix passedByValue (`#8241 <https://github.com/autowarefoundation/autoware_universe/issues/8241>`_)
  fix:passedByValue
* feat(traffic_light_roi_visualizer): add an option to use normal publisher instead of image tranport in traffic light roi visualizer (`#8157 <https://github.com/autowarefoundation/autoware_universe/issues/8157>`_)
  * apply new parameter schemes, set default parameters
  add an option to use normal publisher instead of image tranport in traffic light roi visualizer
  * small fix on default value
  ---------
* refactor(traffic_light\_*)!: add package name prefix of autoware\_ (`#8159 <https://github.com/autowarefoundation/autoware_universe/issues/8159>`_)
  * chore: rename traffic_light_fine_detector to autoware_traffic_light_fine_detector
  * chore: rename traffic_light_multi_camera_fusion to autoware_traffic_light_multi_camera_fusion
  * chore: rename traffic_light_occlusion_predictor to autoware_traffic_light_occlusion_predictor
  * chore: rename traffic_light_classifier to autoware_traffic_light_classifier
  * chore: rename traffic_light_map_based_detector to autoware_traffic_light_map_based_detector
  * chore: rename traffic_light_visualization to autoware_traffic_light_visualization
  ---------
* Contributors: Kotaro Uetake, Taekjin LEE, Yutaka Kondo, Yuxuan Liu, kminoda, kobayu858

0.26.0 (2024-04-03)
-------------------
