^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_control_validator
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.46.0 (2025-06-20)
-------------------

0.45.0 (2025-05-22)
-------------------
* Merge remote-tracking branch 'origin/main' into tmp/notbot/bump_version_base
* feat(control_validator): add lateral jerk validation (`#10619 <https://github.com/autowarefoundation/autoware_universe/issues/10619>`_)
  * feat(control_validator): add steering rate validation and update parameters
  * feat(lateral_jerk_validator): rename and implement lateral jerk validation logic
  - Steering rate validation has been renamed to lateral jerk validation for clarity.
  - Updated the comment to assume constant velocity and adjusted related parameters.
  ---------
* refactor(control_validator): visualize stop reason to virtual wall (`#10593 <https://github.com/autowarefoundation/autoware_universe/issues/10593>`_)
  * feat(control_validator): add error message generation based on validation status
  * feat(control_validator): update push_virtual_wall method to include message display
  * refactor(control_validator): update error messages for clarity in generate_error_message method
  ---------
* chore(control_validator): update maintainer (`#10594 <https://github.com/autowarefoundation/autoware_universe/issues/10594>`_)
  update
* fix(control_validator): fix less trajectory point check (`#10508 <https://github.com/autowarefoundation/autoware_universe/issues/10508>`_)
* Contributors: Kyoichi Sugahara, TaikiYamada4, Yuki TAKAGI

0.44.2 (2025-06-10)
-------------------

0.44.1 (2025-05-01)
-------------------

0.44.0 (2025-04-18)
-------------------
* Merge remote-tracking branch 'origin/main' into humble
* feat(control_validator): add over run estimation feature (`#10422 <https://github.com/autowarefoundation/autoware_universe/issues/10422>`_)
* refactor(control validaor): refactor control_validator (`#10363 <https://github.com/autowarefoundation/autoware_universe/issues/10363>`_)
* fix(control_validator): memory-related crash when processing large trajectories (`#10372 <https://github.com/autowarefoundation/autoware_universe/issues/10372>`_)
  * fix(control_validator): momory
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* fix(control validator): combine callback functions to fix error count increment bug (`#10355 <https://github.com/autowarefoundation/autoware_universe/issues/10355>`_)
* fix(control_validator): fix sign miss and add code test (`#10341 <https://github.com/autowarefoundation/autoware_universe/issues/10341>`_)
* feat(control_validator)!: add acceleration check (`#10326 <https://github.com/autowarefoundation/autoware_universe/issues/10326>`_)
* Contributors: Ryohsuke Mitsudome, Shumpei Wakabayashi, Yuki TAKAGI

0.43.0 (2025-03-21)
-------------------
* Merge remote-tracking branch 'origin/main' into chore/bump-version-0.43
* chore: rename from `autoware.universe` to `autoware_universe` (`#10306 <https://github.com/autowarefoundation/autoware_universe/issues/10306>`_)
* feat(control_validator)!: add overrun check (`#10236 <https://github.com/autowarefoundation/autoware_universe/issues/10236>`_)
* feat(control_validator): add diag to check control component latency (`#10240 <https://github.com/autowarefoundation/autoware_universe/issues/10240>`_)
  * feat(control_validator): add diag to check control component latency
  * fix: missing param
  ---------
* Contributors: Hayato Mizushima, Satoshi OTA, Yuki TAKAGI, Yutaka Kondo

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
* feat:  tier4_debug_msgs to autoware_internal_debug_msgs in file contr… (`#9837 <https://github.com/autowarefoundation/autoware_universe/issues/9837>`_)
  feat:  tier4_debug_msgs to autoware_internal_debug_msgs in file control/autoware_control_validator
* Contributors: Fumiya Watanabe, Vishal Chauhan

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
* fix(cpplint): include what you use - control (`#9565 <https://github.com/autowarefoundation/autoware_universe/issues/9565>`_)
* 0.39.0
* update changelog
* Merge commit '6a1ddbd08bd' into release-0.39.0
* fix: fix ticket links to point to https://github.com/autowarefoundation/autoware_universe (`#9304 <https://github.com/autowarefoundation/autoware_universe/issues/9304>`_)
* feat: suppress warning/error of the empty predicted trajectory by MPC (`#9373 <https://github.com/autowarefoundation/autoware_universe/issues/9373>`_)
* fix(autoware_control_validator): fix clang-diagnostic-unused-private-field (`#9381 <https://github.com/autowarefoundation/autoware_universe/issues/9381>`_)
* fix(control): missing dependency in control components (`#9073 <https://github.com/autowarefoundation/autoware_universe/issues/9073>`_)
* fix: fix ticket links to point to https://github.com/autowarefoundation/autoware_universe (`#9304 <https://github.com/autowarefoundation/autoware_universe/issues/9304>`_)
* chore(package.xml): bump version to 0.38.0 (`#9266 <https://github.com/autowarefoundation/autoware_universe/issues/9266>`_) (`#9284 <https://github.com/autowarefoundation/autoware_universe/issues/9284>`_)
  * unify package.xml version to 0.37.0
  * remove system_monitor/CHANGELOG.rst
  * add changelog
  * 0.38.0
  ---------
* Contributors: Esteve Fernandez, Fumiya Watanabe, M. Fatih Cırıt, Ryohsuke Mitsudome, Ryuta Kambe, Takayuki Murooka, Yutaka Kondo, ぐるぐる

0.39.0 (2024-11-25)
-------------------
* Merge commit '6a1ddbd08bd' into release-0.39.0
* fix: fix ticket links to point to https://github.com/autowarefoundation/autoware_universe (`#9304 <https://github.com/autowarefoundation/autoware_universe/issues/9304>`_)
* fix(control): missing dependency in control components (`#9073 <https://github.com/autowarefoundation/autoware_universe/issues/9073>`_)
* fix: fix ticket links to point to https://github.com/autowarefoundation/autoware_universe (`#9304 <https://github.com/autowarefoundation/autoware_universe/issues/9304>`_)
* chore(package.xml): bump version to 0.38.0 (`#9266 <https://github.com/autowarefoundation/autoware_universe/issues/9266>`_) (`#9284 <https://github.com/autowarefoundation/autoware_universe/issues/9284>`_)
  * unify package.xml version to 0.37.0
  * remove system_monitor/CHANGELOG.rst
  * add changelog
  * 0.38.0
  ---------
* Contributors: Esteve Fernandez, Yutaka Kondo, ぐるぐる

0.38.0 (2024-11-08)
-------------------
* unify package.xml version to 0.37.0
* feat(control_validator): add hold and lpf (`#9120 <https://github.com/autowarefoundation/autoware_universe/issues/9120>`_)
* feat(costmap_generator, control_validator, scenario_selector, surround_obstacle_checker, vehicle_cmd_gate): add processing time pub. (`#9065 <https://github.com/autowarefoundation/autoware_universe/issues/9065>`_)
  * feat(costmap_generator, control_validator, scenario_selector, surround_obstacle_checker, vehicle_cmd_gate): Add: processing_time_pub
  * fix: pre-commit
  * feat(costmap_generator): fix: No output when not Active.
  * fix: clang-format
  * Re: fix: clang-format
  ---------
* fix(control): align the parameters with launcher (`#8789 <https://github.com/autowarefoundation/autoware_universe/issues/8789>`_)
  align the control parameters
* feat(autoware_control_validator): refactoring & testing (`#8096 <https://github.com/autowarefoundation/autoware_universe/issues/8096>`_)
  * refactoring
  * updating...
  * update
  * fix
  * fix
  * Update CMakeLists.txt
  * use yaml to load vehicle info
  ---------
* fix(control_validator): fix param names and doc (`#8104 <https://github.com/autowarefoundation/autoware_universe/issues/8104>`_)
  * fix
* feat(control_validator)!: add velocity check (`#7806 <https://github.com/autowarefoundation/autoware_universe/issues/7806>`_)
  * add velocity check
  ---------
* refactor(universe_utils/motion_utils)!: add autoware namespace (`#7594 <https://github.com/autowarefoundation/autoware_universe/issues/7594>`_)
* refactor(motion_utils)!: add autoware prefix and include dir (`#7539 <https://github.com/autowarefoundation/autoware_universe/issues/7539>`_)
  refactor(motion_utils): add autoware prefix and include dir
* feat(autoware_universe_utils)!: rename from tier4_autoware_utils (`#7538 <https://github.com/autowarefoundation/autoware_universe/issues/7538>`_)
  Co-authored-by: kosuke55 <kosuke.tnp@gmail.com>
* refactor(control)!: refactor directory structures of the control checkers (`#7524 <https://github.com/autowarefoundation/autoware_universe/issues/7524>`_)
  * aeb
  * control_validator
  * lane_departure_checker
  * shift_decider
  * fix
  ---------
* feat(autoware_control_validator): add polling subcribers (`#7426 <https://github.com/autowarefoundation/autoware_universe/issues/7426>`_)
  * add polling subs
  * delete extra line
  ---------
* fix(autoware_control_validator): fix vehicle info utils (`#7417 <https://github.com/autowarefoundation/autoware_universe/issues/7417>`_)
* refactor(control_validator)!: prefix package and namespace with autoware (`#7304 <https://github.com/autowarefoundation/autoware_universe/issues/7304>`_)
  * rename folders
  * rename add prefix
  * change param path
  * fix pluggin problem
  * fix extra prefixes
  * change back launchers
  * add namespace to address conflict
  * delete stubborn file
  ---------
* Contributors: Fumiya Watanabe, Kazunori-Nakajima, Kosuke Takeuchi, Takayuki Murooka, Yuki TAKAGI, Yukinari Hisaki, Yutaka Kondo, Zhe Shen, danielsanchezaran

0.26.0 (2024-04-03)
-------------------
