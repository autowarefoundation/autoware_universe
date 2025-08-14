^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_raw_vehicle_cmd_converter
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.47.1 (2025-08-14)
-------------------

0.47.0 (2025-08-11)
-------------------
* feat(raw_vehicle_cmd_converter): add control component latency measuremement and publisher (`#11044 <https://github.com/autowarefoundation/autoware_universe/issues/11044>`_)
  feat(raw_vehicle_cmd_converter): add control component latency measurement and publisher
* Contributors: Kyoichi Sugahara

0.46.0 (2025-06-20)
-------------------

0.45.0 (2025-05-22)
-------------------

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
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* feat(autoware_utils): replace autoware_universe_utils with autoware_utils  (`#10191 <https://github.com/autowarefoundation/autoware_universe/issues/10191>`_)
* fix: add missing includes to autoware_universe_utils (`#10091 <https://github.com/autowarefoundation/autoware_universe/issues/10091>`_)
* Contributors: Fumiya Watanabe, Ryohsuke Mitsudome, 心刚

0.41.2 (2025-02-19)
-------------------
* chore: bump version to 0.41.1 (`#10088 <https://github.com/autowarefoundation/autoware_universe/issues/10088>`_)
* Contributors: Ryohsuke Mitsudome

0.41.1 (2025-02-10)
-------------------

0.41.0 (2025-01-29)
-------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* feat(autoware_raw_vehicle_cmd_converter)!: tier4_debug_msgs changed to autoware_internal_debug_msgs in autoware_raw_vehicle_cmd_converter (`#9924 <https://github.com/autowarefoundation/autoware_universe/issues/9924>`_)
  feat: tier4_debug_msgs changed to autoware_internal_debug_msgs in files vehicle/autoware_raw_vehicle_cmd_converter
  Co-authored-by: Ryohsuke Mitsudome <43976834+mitsudome-r@users.noreply.github.com>
* fix(raw_veihicle_converter): fix too long line (`#9716 <https://github.com/autowarefoundation/autoware_universe/issues/9716>`_)
* feat(raw_vehicle_cmd_converter): add vehicle adaptor  (`#8782 <https://github.com/autowarefoundation/autoware_universe/issues/8782>`_)
  * feat(raw_vehicle_cmd_converter): add vehicle adaptor
  sub operation status
  * feat(raw_vehicle_cmd_converter): publish vehicle adaptor output
  * use control horizon
  * revert carla
  * update docs
  ---------
* Contributors: Fumiya Watanabe, Kosuke Takeuchi, Vishal Chauhan

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
* fix(cpplint): include what you use - vehicle (`#9575 <https://github.com/autowarefoundation/autoware_universe/issues/9575>`_)
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
* fix(simple_planning_simulator, raw_vehicle_cmd_converter): swap row index and column index for csv loader  (`#8963 <https://github.com/autowarefoundation/autoware_universe/issues/8963>`_)
  swap row and column
* test(raw_vehicle_cmd_converter): add tests (`#8951 <https://github.com/autowarefoundation/autoware_universe/issues/8951>`_)
  * remove header file according to clangd warning
  * add test
  * fix
  * add test for get function
  * apply clang tidy
  * fix test content
  ---------
* refactor(autoware_interpolation): prefix package and namespace with autoware (`#8088 <https://github.com/autowarefoundation/autoware_universe/issues/8088>`_)
  Co-authored-by: kosuke55 <kosuke.tnp@gmail.com>
* refactor(start_planner,raw_vechile_cmd_converter): align parameter with autoware_launch's parameter (`#8913 <https://github.com/autowarefoundation/autoware_universe/issues/8913>`_)
  * align autoware_raw_vehicle_cmd_converter's parameter
  * align start_planner's parameter
  ---------
* fix(raw_vehicle_cmd_converter): fix convert_steer_cmd_method condition (`#8813 <https://github.com/autowarefoundation/autoware_universe/issues/8813>`_)
* fix(raw_vehicle_cmd_converter): fix null check (`#8677 <https://github.com/autowarefoundation/autoware_universe/issues/8677>`_)
* chore(raw_vehicle_cmd_converter): add maintainer (`#8671 <https://github.com/autowarefoundation/autoware_universe/issues/8671>`_)
* feat(raw_vehicle_cmd_converter): set convert_actuation_to_steering_status false by default (`#8668 <https://github.com/autowarefoundation/autoware_universe/issues/8668>`_)
* feat(raw_vehicle_cmd_converter): disable actuation to steering (`#8588 <https://github.com/autowarefoundation/autoware_universe/issues/8588>`_)
* feat(raw_vehicle_cmd_converter): add steer command conversion with VGR (`#8504 <https://github.com/autowarefoundation/autoware_universe/issues/8504>`_)
  * feat(raw_vehicle_cmd_converter): add steer command conversion with VGR
  * make class and add test
  * remove member vgr_coef from node
  * update readme
  * add svg
  * add plot scripts
  * Update vehicle/autoware_raw_vehicle_cmd_converter/README.md
  Co-authored-by: Takamasa Horibe <horibe.takamasa@gmail.com>
  * not always subscribe actuation_status
  * add comment for using normal sub for steering status
  ---------
  Co-authored-by: Takamasa Horibe <horibe.takamasa@gmail.com>
* refactor(universe_utils/motion_utils)!: add autoware namespace (`#7594 <https://github.com/autowarefoundation/autoware_universe/issues/7594>`_)
* feat(autoware_universe_utils)!: rename from tier4_autoware_utils (`#7538 <https://github.com/autowarefoundation/autoware_universe/issues/7538>`_)
  Co-authored-by: kosuke55 <kosuke.tnp@gmail.com>
* feat(raw_vehicle_cmd_converter): use polling subscriber (`#7319 <https://github.com/autowarefoundation/autoware_universe/issues/7319>`_)
  * replace subscription
  * fix document
  * sum up functions
  * add maintainer
  ---------
  Co-authored-by: Shumpei Wakabayashi <42209144+shmpwk@users.noreply.github.com>
* refactor(accel_brake_map_calibrator)!: add autoware\_ prefix (`#7351 <https://github.com/autowarefoundation/autoware_universe/issues/7351>`_)
  * add prefix to the codes
  change dir name
  update
  update
  * delete debug
  * fix format
  * fix format
  * restore
  * poi
  ---------
* refactor(raw_vehicle_cmd_converter)!: prefix package and namespace with autoware (`#7385 <https://github.com/autowarefoundation/autoware_universe/issues/7385>`_)
  * add prefix
  * fix other packages
  * fix cppcheck
  * pre-commit
  * fix
  ---------
* Contributors: Esteve Fernandez, Go Sakayori, Kosuke Takeuchi, Kyoichi Sugahara, Sho Iwasawa, Takayuki Murooka, Yuki TAKAGI, Yutaka Kondo

0.26.0 (2024-04-03)
-------------------
