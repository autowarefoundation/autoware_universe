^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_map_tf_generator
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.53.0 (2026-09-29)
-------------------
* Merge remote-tracking branch 'origin/main' into prepare-0.53.0-changelog
* feat(autoware_map_tf_generator): add a node design for vector_map_tf_generator (`#13297 <https://github.com/autowarefoundation/autoware_universe/issues/13297>`_)
  * feat(autoware_map_tf_generator): add a node design for vector_map_tf_generator
  Describe the interfaces, parameters and process of the vector map TF
  generator as a system design node file.
  * fix: update global to remap_target for various nodes in YAML configurations
  ---------
* fix: declare the dependencies the simulator, e2e and map packages use (`#13209 <https://github.com/autowarefoundation/autoware_universe/issues/13209>`_)
  * fix: declare the dependencies the simulator, e2e and map packages use
  Three packages use headers or symbols of packages that they never declare. Add the 12 missing <depend> entries.
  * fix: declare tf2 in tier4_dummy_object_rviz_plugin
  The plugin uses tf2::Transform and tf2::Quaternion directly, but only tf2_geometry_msgs and tf2_ros were declared.
  ---------
* refactor(map): move node design files into each package (`#13094 <https://github.com/autowarefoundation/autoware_universe/issues/13094>`_)
  Co-authored-by: Claude Opus 5 (1M context) <noreply@anthropic.com>
* feat(map_tf_generator): apply `agnocast_wrapper::Node` to `map_tf_generator` (`#12832 <https://github.com/autowarefoundation/autoware_universe/issues/12832>`_)
  apply agnocast_wrapper::Node
* Contributors: Koichi Imai, Mete Fatih Cırıt, Ryohsuke Mitsudome, Taekjin LEE

0.52.0 (2026-06-30)
-------------------

0.51.0 (2026-05-01)
-------------------

0.50.0 (2026-02-14)
-------------------

0.49.0 (2025-12-30)
-------------------
* Merge remote-tracking branch 'origin/main' into prepare-0.49.0-changelog
* feat(autoware_lanelet2_utils): replace from/toBinMsg (Map and Localization Component) (`#11787 <https://github.com/autowarefoundation/autoware_universe/issues/11787>`_)
  * localization component toBinMsg replacement
  * map component fromBinMsg replacement
  * localization component fromBinMsg replacement
  * remove unused dep from localization/autoware_pose_estimator_arbiter
  ---------
* Contributors: Ryohsuke Mitsudome, Sarun MUKDAPITAK

0.48.0 (2025-11-18)
-------------------
* Merge remote-tracking branch 'origin/main' into humble
* fix: tf2 uses hpp headers in rolling (and is backported) (`#11620 <https://github.com/autowarefoundation/autoware_universe/issues/11620>`_)
* chore: update maintainer (`#11373 <https://github.com/autowarefoundation/autoware_universe/issues/11373>`_)
  * chore: update maintainer
  remove Ryu Yamamoto
  * chore: update maintainer
  remove Kento Yabuuchi
  * chore: update maintainer
  remove Shintaro Sakoda
  ---------
* Contributors: Motz, Ryohsuke Mitsudome, Tim Clephas

0.47.1 (2025-08-14)
-------------------

0.47.0 (2025-08-11)
-------------------

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

0.41.2 (2025-02-19)
-------------------
* chore: bump version to 0.41.1 (`#10088 <https://github.com/autowarefoundation/autoware_universe/issues/10088>`_)
* Contributors: Ryohsuke Mitsudome

0.41.1 (2025-02-10)
-------------------

0.41.0 (2025-01-29)
-------------------

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
* fix(cpplint): include what you use - map (`#9568 <https://github.com/autowarefoundation/autoware_universe/issues/9568>`_)
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
* refactor(map_tf_generator)!: prefix package and namespace with autoware (`#8422 <https://github.com/autowarefoundation/autoware_universe/issues/8422>`_)
  * add autoware\_ prefix
  * add autoware\_ prefix
  ---------
  Co-authored-by: SakodaShintaro <shintaro.sakoda@tier4.jp>
* Contributors: Masaki Baba, Yutaka Kondo

0.26.0 (2024-04-03)
-------------------
