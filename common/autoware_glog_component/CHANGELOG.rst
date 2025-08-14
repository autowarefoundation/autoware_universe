^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_glog_component
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
* chore(package.xml): bump version to 0.39.0 (`#9587 <https://github.com/autowarefoundation/autoware_universe/issues/9587>`_)
  * chore(package.xml): bump version to 0.39.0
  * fix: fix ticket links in CHANGELOG.rst
  * fix: remove unnecessary diff
  ---------
  Co-authored-by: Yutaka Kondo <yutaka.kondo@youtalk.jp>
* refactor(glog_component): prefix package and namespace with autoware (`#9302 <https://github.com/autowarefoundation/autoware_universe/issues/9302>`_)
  Co-authored-by: Takagi, Isamu <43976882+isamu-takagi@users.noreply.github.com>
* Contributors: Esteve Fernandez, Fumiya Watanabe, Ryohsuke Mitsudome

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
* chore(glog): add initialization check (`#6792 <https://github.com/autowarefoundation/autoware_universe/issues/6792>`_)
* Contributors: Takamasa Horibe, Yutaka Kondo

0.26.0 (2024-04-03)
-------------------
* feat(glog): add glog in planning and control modules (`#4714 <https://github.com/autowarefoundation/autoware_universe/issues/4714>`_)
  * feat(glog): add glog component
  * formatting
  * remove namespace
  * remove license
  * Update launch/tier4_planning_launch/launch/scenario_planning/lane_driving/motion_planning/motion_planning.launch.py
  Co-authored-by: Daisuke Nishimatsu <42202095+wep21@users.noreply.github.com>
  * Update launch/tier4_planning_launch/launch/scenario_planning/lane_driving/behavior_planning/behavior_planning.launch.py
  Co-authored-by: Daisuke Nishimatsu <42202095+wep21@users.noreply.github.com>
  * Update common/glog_component/CMakeLists.txt
  Co-authored-by: Daisuke Nishimatsu <42202095+wep21@users.noreply.github.com>
  * Update launch/tier4_control_launch/launch/control.launch.py
  Co-authored-by: Daisuke Nishimatsu <42202095+wep21@users.noreply.github.com>
  * add copyright
  ---------
  Co-authored-by: Daisuke Nishimatsu <42202095+wep21@users.noreply.github.com>
* Contributors: Takamasa Horibe
