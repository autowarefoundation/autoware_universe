^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_rtc_interface
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.48.0 (2025-11-18)
-------------------
* Merge remote-tracking branch 'origin/main' into humble
* feat(roundabout): add autoware_behavior_velocity_roundabout_module (`#11142 <https://github.com/autowarefoundation/autoware_universe/issues/11142>`_)
  * feat(behavior_velocity_roundabout_module): add roundabout behavior velocity planner module
  * feat(behavior_planning): add roundabout module to behavior planning launch and RTC interface
  * Add functions for behavior velocity planner in roundabout module
  * Refactor roundabout module
  * feat(roundabout_module): integrate Roundabout regulatory element
  * refactor(roundabout_module): remove associative_ids from attention lane
  * move header files to include
  * refactor(roundabout_module): remove util files and update includes for intersection module
  * fix(roundabout_module): update include path for interpolated_path_info
  * refactor(roundabout_module): update include paths for consistency
  * fix(scene_roundabout): update include path for result.hpp
  * refactor(roundabout_module): clean up unused structures and methods
  * refactor(roundabout_module): clean up and optimize code structure and comments
  * refactor(object_manager): remove unused stopline handling from ObjectInfo initialization
  * refactor(roundabout_module): remove unused variables and comments related to second pass judge line
  * refactor(roundabout_module): improve comments and remove unnecessary code in decision result and collision handling
  * feat(ttc_visualizer): add TTC visualizer script for time-to-collision analysis
  * refactor(roundabout_module): remove consider_wrong_direction_vehicle parameter and related code
  * refactor(roundabout_module): simplify object management and improve code clarity
  * style(pre-commit): autofix
  * fix(manager): include <set> header for improved functionality
  * fix(roundabout): update parameter name
  * docs(roundabout): update README to clarify module role and functionality
  * fix(readme): fix README
  * fix(readme): fix README
  * style(pre-commit): autofix
  * refactor(roundabout): fix attention lanelet  logic
  * style(pre-commit): autofix
  * review roundabout
  * update utils
  * refactor(roundabout): fix attention lanelet logic
  * feat(cmake): add installation for ttc.py script
  * refactor(detectOcclusion):  fix cppcheck error
  * style(pre-commit): autofix
  * docs(README): fix unkown word ci error
  * style(pre-commit): autofix
  * fix: update copyright year to 2025 in multiple files
  * docs(README): add flowchart for roundabout behavior velocity module logic
  * style(pre-commit): autofix
  * fix: correct capitalization in copyright notice
  * refactor(roundabout-module): streamline roundabout module initialization and remove unused conflicting area logic
  * refactor(roundabout-module): remove unnecessary comments and streamline lanelet handling
  * refactor(roundabout-module): rename variables for clarity and improve lanelet handling
  * style(pre-commit): autofix
  * refactor(roundabout-module): remove attention_area_length parameter and update related logic
  * style(pre-commit): autofix
  * build(build_depends_humble.repos): bump autoware_lanelet2_extension to 0.9.0
  * fix(package): add dependency on tf2_geometry_msgs
  * feat(planning): add roundabout handling to planning factors and conversion map
  * Revert "feat(planning): add roundabout handling to planning factors and conversion map"
  This reverts commit e79eab23e89969f88bc7485c4ed337e843d33a7f.
  * refactor(roundabout):  Move the parameter definition
  * style(pre-commit): autofix
  * fix(package): add  maintainer
  * refactor(debug): clean up includes and remove unused color definitions
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: Y.Hisaki <yhisaki31@gmail.com>
  Co-authored-by: pre-commit-ci-lite[bot] <117423508+pre-commit-ci-lite[bot]@users.noreply.github.com>
* feat(RTC, behavior_velocity_planner): set manual RTC via the lanelet map (`#11340 <https://github.com/autowarefoundation/autoware_universe/issues/11340>`_)
  * first attempt at a solution to set the auto/manual mode for a module
  * implementation for crosswalk and intersections modules
  * update READMEs
  * fix crosswalk -> intersection
  Co-authored-by: Satoshi OTA <44889564+satoshi-ota@users.noreply.github.com>
  ---------
  Co-authored-by: Satoshi OTA <44889564+satoshi-ota@users.noreply.github.com>
* Contributors: Maxime CLEMENT, Ryohsuke Mitsudome, Sho Iwasawa

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
* fix(autoware_rtc_interface): fix bugprone-branch-clone (`#9698 <https://github.com/autowarefoundation/autoware_universe/issues/9698>`_)
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
* fix(cpplint): include what you use - planning (`#9570 <https://github.com/autowarefoundation/autoware_universe/issues/9570>`_)
* 0.39.0
* update changelog
* Merge commit '6a1ddbd08bd' into release-0.39.0
* fix: fix ticket links to point to https://github.com/autowarefoundation/autoware_universe (`#9304 <https://github.com/autowarefoundation/autoware_universe/issues/9304>`_)
* fix: fix ticket links to point to https://github.com/autowarefoundation/autoware_universe (`#9304 <https://github.com/autowarefoundation/autoware_universe/issues/9304>`_)
* chore(package.xml): bump version to 0.38.0 (`#9266 <https://github.com/autowarefoundation/autoware_universe/issues/9266>`_) (`#9284 <https://github.com/autowarefoundation/autoware_universe/issues/9284>`_)
  * unify package.xml version to 0.37.0
  * remove system_monitor/CHANGELOG.rst
  * add changelog
  * 0.38.0
  ---------
* fix(autoware_rtc_interface): fix dependency (`#9237 <https://github.com/autowarefoundation/autoware_universe/issues/9237>`_)
* fix(rtc_interface): update requested field for every cooperateStatus state (`#9211 <https://github.com/autowarefoundation/autoware_universe/issues/9211>`_)
  * fix rtc_interface
  * fix test condition
  ---------
* feat(rtc_interface): add requested field (`#9202 <https://github.com/autowarefoundation/autoware_universe/issues/9202>`_)
  * add requested feature
  * Update planning/autoware_rtc_interface/test/test_rtc_interface.cpp
  Co-authored-by: Satoshi OTA <44889564+satoshi-ota@users.noreply.github.com>
  ---------
  Co-authored-by: Satoshi OTA <44889564+satoshi-ota@users.noreply.github.com>
* Contributors: Esteve Fernandez, Fumiya Watanabe, Go Sakayori, M. Fatih Cırıt, Ryohsuke Mitsudome, Yutaka Kondo

0.39.0 (2024-11-25)
-------------------
* Merge commit '6a1ddbd08bd' into release-0.39.0
* fix: fix ticket links to point to https://github.com/autowarefoundation/autoware_universe (`#9304 <https://github.com/autowarefoundation/autoware_universe/issues/9304>`_)
* fix: fix ticket links to point to https://github.com/autowarefoundation/autoware_universe (`#9304 <https://github.com/autowarefoundation/autoware_universe/issues/9304>`_)
* chore(package.xml): bump version to 0.38.0 (`#9266 <https://github.com/autowarefoundation/autoware_universe/issues/9266>`_) (`#9284 <https://github.com/autowarefoundation/autoware_universe/issues/9284>`_)
  * unify package.xml version to 0.37.0
  * remove system_monitor/CHANGELOG.rst
  * add changelog
  * 0.38.0
  ---------
* fix(autoware_rtc_interface): fix dependency (`#9237 <https://github.com/autowarefoundation/autoware_universe/issues/9237>`_)
* fix(rtc_interface): update requested field for every cooperateStatus state (`#9211 <https://github.com/autowarefoundation/autoware_universe/issues/9211>`_)
  * fix rtc_interface
  * fix test condition
  ---------
* feat(rtc_interface): add requested field (`#9202 <https://github.com/autowarefoundation/autoware_universe/issues/9202>`_)
  * add requested feature
  * Update planning/autoware_rtc_interface/test/test_rtc_interface.cpp
  Co-authored-by: Satoshi OTA <44889564+satoshi-ota@users.noreply.github.com>
  ---------
  Co-authored-by: Satoshi OTA <44889564+satoshi-ota@users.noreply.github.com>
* Contributors: Esteve Fernandez, Go Sakayori, Yutaka Kondo

0.38.0 (2024-11-08)
-------------------
* unify package.xml version to 0.37.0
* fix(rtc_interface): update cooperateStatus state transition (`#8883 <https://github.com/autowarefoundation/autoware_universe/issues/8883>`_)
  fix state transition for failure/success
* feat(rtc_interface, lane_change): check state transition for cooperate status (`#8855 <https://github.com/autowarefoundation/autoware_universe/issues/8855>`_)
  * update rtc state transition
  * remove transition from failuer and succeeded
  * fix
  * check initial state for cooperate status
  * change rtc cooperate status according to module status
  ---------
* feat(rtc_inteface): add function to check force deactivate (`#8221 <https://github.com/autowarefoundation/autoware_universe/issues/8221>`_)
  add function to check for deactivation
* fix(autoware_rtc_interface): fix constParameterReference (`#8042 <https://github.com/autowarefoundation/autoware_universe/issues/8042>`_)
  * fix:constParameterReference
  * fix: clang format
  * fix:constParameterReference
  ---------
* fix(rtc_interface): fix build error (`#8111 <https://github.com/autowarefoundation/autoware_universe/issues/8111>`_)
  * fix
  * fix format
  ---------
* fix(bpp, rtc_interface): fix state transition (`#7743 <https://github.com/autowarefoundation/autoware_universe/issues/7743>`_)
  * fix(rtc_interface): check rtc state
  * fix(bpp_interface): check rtc state
  * feat(rtc_interface): print
  ---------
* feat(static_obstacle_avoidance): enable force execution under unsafe conditions (`#8094 <https://github.com/autowarefoundation/autoware_universe/issues/8094>`_)
  * add force execution for static obstacle avoidance
  * fix
  * erase unused function in RTC interface
  * refactor with lamda function
  * fix rtc_interface
  * add warn throtthle and move code block
  * fix
  ---------
* docs(planning): fix wrong link (`#7751 <https://github.com/autowarefoundation/autoware_universe/issues/7751>`_)
  * fix page link
  * fix out of lane link
  * fix
  * fix cost map generator link
  ---------
* docs(rtc_replayer): fix wrong link (`#7714 <https://github.com/autowarefoundation/autoware_universe/issues/7714>`_)
  * fix link for rtc_replayer
  * delete RTC replayer header
  * fix
  ---------
* refactor(rtc_interface)!: rename to include/autoware/{package_name} (`#7531 <https://github.com/autowarefoundation/autoware_universe/issues/7531>`_)
  Co-authored-by: Fumiya Watanabe <rej55.g@gmail.com>
* refactor(rtc_interface)!: prefix package and namespace with autoware (`#7321 <https://github.com/autowarefoundation/autoware_universe/issues/7321>`_)
  refactor(rtc_interface): add autoware prefix
* Contributors: Fumiya Watanabe, Go Sakayori, Kosuke Takeuchi, Satoshi OTA, Yutaka Kondo, kobayu858

0.26.0 (2024-04-03)
-------------------
