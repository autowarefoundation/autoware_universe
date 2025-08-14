^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_default_adapi
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.47.1 (2025-08-14)
-------------------

0.47.0 (2025-08-11)
-------------------
* style(pre-commit): update to clang-format-20 (`#11088 <https://github.com/autowarefoundation/autoware_universe/issues/11088>`_)
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* feat(default_adapi_universe): check mrm state when reset diag (`#10894 <https://github.com/autowarefoundation/autoware_universe/issues/10894>`_)
* feat(autoware_default_adapi_universe)!: remove interface ported to Autoware Core (`#10918 <https://github.com/autowarefoundation/autoware_universe/issues/10918>`_)
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: Takagi, Isamu <isamu.takagi@tier4.jp>
* docs(default_adapi_universe): add detailed description for autoware state arrived (`#10904 <https://github.com/autowarefoundation/autoware_universe/issues/10904>`_)
  * focs(default_adapi_universe): add time for autoware state arrived
  * add description
  ---------
* feat(default_ad_api): use polling subscription (`#7588 <https://github.com/autowarefoundation/autoware_universe/issues/7588>`_)
* Contributors: Mete Fatih Cırıt, Ryohsuke Mitsudome, Takagi, Isamu

0.46.0 (2025-06-20)
-------------------
* Merge remote-tracking branch 'upstream/main' into tmp/TaikiYamada/bump_version_base
* feat(default_adapi_universe): add mrm description api (`#10838 <https://github.com/autowarefoundation/autoware_universe/issues/10838>`_)
  * feat(default_adapi_universe): add mrm description api
  * update message name
  ---------
* feat(default_adapi_universe): support diag level latch and filter (`#10846 <https://github.com/autowarefoundation/autoware_universe/issues/10846>`_)
  * update aggregator
  * update utils
  * update hazard converter
  * update adapi
  * fix build error
  * ignore spell check for yamls
  * fix copyright year
  * ignore spell check for timeline
  * change link index to fix out of bounds access
  * reflect link index change to utils
  * feat(default_adapi_universe): support diagnostics latch
  * relay reset service
  ---------
* feat(default_adapi): add vehicle door diags (`#10735 <https://github.com/autowarefoundation/autoware_universe/issues/10735>`_)
  * feat(default_adapi): add vehicle door diags
  * modify response
  * fix check order
  * fix check order
  ---------
* feat(default_adapi_universe): rename command api (`#10834 <https://github.com/autowarefoundation/autoware_universe/issues/10834>`_)
* feat!: replace autoware_internal_localization_msgs with autoware_localization_msgs for InitializeLocalization service (`#10844 <https://github.com/autowarefoundation/autoware_universe/issues/10844>`_)
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* feat(diagnostic_graph_aggregator): support latch and dependent error (`#10829 <https://github.com/autowarefoundation/autoware_universe/issues/10829>`_)
  * update aggregator
  * update utils
  * update hazard converter
  * update adapi
  * fix build error
  * ignore spell check for yamls
  * fix copyright year
  * ignore spell check for timeline
  * change link index to fix out of bounds access
  * reflect link index change to utils
  * fix for cppcheck
  * fix for cppcheck
  ---------
* feat!: replace tier4_planning_msgs service with autoware_planning_msgs (`#10827 <https://github.com/autowarefoundation/autoware_universe/issues/10827>`_)
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* feat(default_adapi): add vehicle command api (`#10764 <https://github.com/autowarefoundation/autoware_universe/issues/10764>`_)
* feat(default_adapi): add vehicle metrics api (`#10553 <https://github.com/autowarefoundation/autoware_universe/issues/10553>`_)
* refactor(default_adapi): rename vehicle status file (`#10761 <https://github.com/autowarefoundation/autoware_universe/issues/10761>`_)
* chore(default_adapi): rename package (`#10756 <https://github.com/autowarefoundation/autoware_universe/issues/10756>`_)
* Contributors: Ryohsuke Mitsudome, TaikiYamada4, Takagi, Isamu

0.45.0 (2025-05-22)
-------------------
* Merge remote-tracking branch 'origin/main' into tmp/notbot/bump_version_base
* feat(localization): replace tier4_localization_msgs used by ndt_align_srv with autoware_internal_localization_msgs (`#10567 <https://github.com/autowarefoundation/autoware_universe/issues/10567>`_)
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* feat(default_adapi): add state diagnostics (`#10539 <https://github.com/autowarefoundation/autoware_universe/issues/10539>`_)
* feat(default_adapi): add mrm request api (`#10550 <https://github.com/autowarefoundation/autoware_universe/issues/10550>`_)
* docs(default_adapi): add document of params and diags (`#10557 <https://github.com/autowarefoundation/autoware_universe/issues/10557>`_)
  * doc(default_adapi): add document of params and diags
  * fix cmakelists
  ---------
* Contributors: TaikiYamada4, Takagi, Isamu, 心刚

0.44.2 (2025-06-10)
-------------------

0.44.1 (2025-05-01)
-------------------

0.44.0 (2025-04-18)
-------------------
* Merge remote-tracking branch 'origin/main' into humble
* fix: add missing exec_depend (`#10404 <https://github.com/autowarefoundation/autoware_universe/issues/10404>`_)
  * fix missing exec depend
  * remove fixed depend
  * remove the removed dependency
  ---------
* feat(autoware_default_adapi): release adapi v1.8.0 (`#10380 <https://github.com/autowarefoundation/autoware_universe/issues/10380>`_)
* feat: manual control (`#10354 <https://github.com/autowarefoundation/autoware_universe/issues/10354>`_)
  * feat(default_adapi): add manual control
  * add conversion
  * update selector
  * update selector depends
  * update converter
  * modify heartbeat name
  * update launch
  * update api
  * fix pedal callback
  * done todo
  * apply message rename
  * fix test
  * fix message type and qos
  * fix steering_tire_velocity
  * fix for clang-tidy
  ---------
* feat(autoware_default_adapi): disable sample web server (`#10327 <https://github.com/autowarefoundation/autoware_universe/issues/10327>`_)
  * feat(autoware_default_adapi): disable sample web server
  * fix unused inport
  ---------
* feat(autoware_default_adapi): log autoware state change (`#10364 <https://github.com/autowarefoundation/autoware_universe/issues/10364>`_)
* Contributors: Ryohsuke Mitsudome, Takagi, Isamu

0.43.0 (2025-03-21)
-------------------
* Merge remote-tracking branch 'origin/main' into chore/bump-version-0.43
* chore: rename from `autoware.universe` to `autoware_universe` (`#10306 <https://github.com/autowarefoundation/autoware_universe/issues/10306>`_)
* feat(planning_factor): support new cruise planner's factor (`#10229 <https://github.com/autowarefoundation/autoware_universe/issues/10229>`_)
  * support cruise planner's factor
  * not slowdown but slow_down
  ---------
* feat(Autoware_planning_factor_interface): replace tier4_msgs with autoware_internal_msgs (`#10204 <https://github.com/autowarefoundation/autoware_universe/issues/10204>`_)
* Contributors: Hayato Mizushima, Kento Yabuuchi, Yutaka Kondo, 心刚

0.42.0 (2025-03-03)
-------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* feat(autoware_default_adapi): allow route clear while vehicle is stopped (`#10158 <https://github.com/autowarefoundation/autoware_universe/issues/10158>`_)
  * feat(autoware_default_adapi): allow route clear while vehicle is stopped
  * fix parameter
  ---------
* Contributors: Fumiya Watanabe, Takagi, Isamu

0.41.2 (2025-02-19)
-------------------
* chore: bump version to 0.41.1 (`#10088 <https://github.com/autowarefoundation/autoware_universe/issues/10088>`_)
* Contributors: Ryohsuke Mitsudome

0.41.1 (2025-02-10)
-------------------

0.41.0 (2025-01-29)
-------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* feat: apply `autoware` prefix for `component_state_monitor` and its dependencies (`#9961 <https://github.com/autowarefoundation/autoware_universe/issues/9961>`_)
* feat: apply `autoware\_` prefix for `diagnostic_graph_utils` (`#9968 <https://github.com/autowarefoundation/autoware_universe/issues/9968>`_)
* feat: apply `autoware\_` prefix for `default_ad_api_helpers` (`#9965 <https://github.com/autowarefoundation/autoware_universe/issues/9965>`_)
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: Takagi, Isamu <isamu.takagi@tier4.jp>
* feat(autoware_component_interface_specs_universe!): rename package (`#9753 <https://github.com/autowarefoundation/autoware_universe/issues/9753>`_)
* fix(obstacle_stop_planner): migrate planning factor (`#9939 <https://github.com/autowarefoundation/autoware_universe/issues/9939>`_)
  * fix(obstacle_stop_planner): migrate planning factor
  * fix(autoware_default_adapi): add coversion map
  ---------
* feat(planning_factor)!: remove velocity_factor, steering_factor and introduce planning_factor (`#9927 <https://github.com/autowarefoundation/autoware_universe/issues/9927>`_)
  Co-authored-by: Satoshi OTA <44889564+satoshi-ota@users.noreply.github.com>
  Co-authored-by: Ryohsuke Mitsudome <43976834+mitsudome-r@users.noreply.github.com>
  Co-authored-by: satoshi-ota <satoshi.ota928@gmail.com>
* feat(autoware_default_adapi): release adapi v1.6.0 (`#9704 <https://github.com/autowarefoundation/autoware_universe/issues/9704>`_)
  * feat: reject clearing route during autonomous mode
  * feat: modify check and relay door service
  * fix door condition
  * fix error and add option
  * update v1.6.0
  ---------
* fix(autoware_default_adapi): fix bugprone-branch-clone (`#9726 <https://github.com/autowarefoundation/autoware_universe/issues/9726>`_)
  fix: bugprone-error
* Contributors: Fumiya Watanabe, Junya Sasaki, Mamoru Sobue, Ryohsuke Mitsudome, Satoshi OTA, Takagi, Isamu, kobayu858

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
* fix(cpplint): include what you use - system (`#9573 <https://github.com/autowarefoundation/autoware_universe/issues/9573>`_)
* 0.39.0
* update changelog
* Merge commit '6a1ddbd08bd' into release-0.39.0
* fix: fix ticket links to point to https://github.com/autowarefoundation/autoware_universe (`#9304 <https://github.com/autowarefoundation/autoware_universe/issues/9304>`_)
* feat(bpp): add velocity interface (`#9344 <https://github.com/autowarefoundation/autoware_universe/issues/9344>`_)
  * feat(bpp): add velocity interface
  * fix(adapi): subscribe additional velocity factors
  ---------
* fix(run_out): output velocity factor (`#9319 <https://github.com/autowarefoundation/autoware_universe/issues/9319>`_)
  * fix(run_out): output velocity factor
  * fix(adapi): subscribe run out velocity factor
  ---------
* fix: fix ticket links to point to https://github.com/autowarefoundation/autoware_universe (`#9304 <https://github.com/autowarefoundation/autoware_universe/issues/9304>`_)
* refactor(autoware_ad_api_specs): prefix package and namespace with autoware (`#9250 <https://github.com/autowarefoundation/autoware_universe/issues/9250>`_)
  * refactor(autoware_ad_api_specs): prefix package and namespace with autoware
  * style(pre-commit): autofix
  * chore(autoware_adapi_specs): rename ad_api to adapi
  * style(pre-commit): autofix
  * chore(autoware_adapi_specs): rename ad_api to adapi
  * chore(autoware_adapi_specs): rename ad_api to adapi
  * chore(autoware_adapi_specs): rename ad_api_specs to adapi_specs
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* chore(package.xml): bump version to 0.38.0 (`#9266 <https://github.com/autowarefoundation/autoware_universe/issues/9266>`_) (`#9284 <https://github.com/autowarefoundation/autoware_universe/issues/9284>`_)
  * unify package.xml version to 0.37.0
  * remove system_monitor/CHANGELOG.rst
  * add changelog
  * 0.38.0
  ---------
* fix(autoware_default_adapi): change subscribing steering factor topic name for obstacle avoidance and lane changes (`#9273 <https://github.com/autowarefoundation/autoware_universe/issues/9273>`_)
  feat(planning): add new steering factor topics for obstacle avoidance and lane changes
* refactor(component_interface_utils): prefix package and namespace with autoware (`#9092 <https://github.com/autowarefoundation/autoware_universe/issues/9092>`_)
* Contributors: Esteve Fernandez, Fumiya Watanabe, Kyoichi Sugahara, M. Fatih Cırıt, Ryohsuke Mitsudome, Satoshi OTA, Yutaka Kondo

0.39.0 (2024-11-25)
-------------------
* Merge commit '6a1ddbd08bd' into release-0.39.0
* fix: fix ticket links to point to https://github.com/autowarefoundation/autoware_universe (`#9304 <https://github.com/autowarefoundation/autoware_universe/issues/9304>`_)
* fix: fix ticket links to point to https://github.com/autowarefoundation/autoware_universe (`#9304 <https://github.com/autowarefoundation/autoware_universe/issues/9304>`_)
* refactor(autoware_ad_api_specs): prefix package and namespace with autoware (`#9250 <https://github.com/autowarefoundation/autoware_universe/issues/9250>`_)
  * refactor(autoware_ad_api_specs): prefix package and namespace with autoware
  * style(pre-commit): autofix
  * chore(autoware_adapi_specs): rename ad_api to adapi
  * style(pre-commit): autofix
  * chore(autoware_adapi_specs): rename ad_api to adapi
  * chore(autoware_adapi_specs): rename ad_api to adapi
  * chore(autoware_adapi_specs): rename ad_api_specs to adapi_specs
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* chore(package.xml): bump version to 0.38.0 (`#9266 <https://github.com/autowarefoundation/autoware_universe/issues/9266>`_) (`#9284 <https://github.com/autowarefoundation/autoware_universe/issues/9284>`_)
  * unify package.xml version to 0.37.0
  * remove system_monitor/CHANGELOG.rst
  * add changelog
  * 0.38.0
  ---------
* fix(autoware_default_adapi): change subscribing steering factor topic name for obstacle avoidance and lane changes (`#9273 <https://github.com/autowarefoundation/autoware_universe/issues/9273>`_)
  feat(planning): add new steering factor topics for obstacle avoidance and lane changes
* refactor(component_interface_utils): prefix package and namespace with autoware (`#9092 <https://github.com/autowarefoundation/autoware_universe/issues/9092>`_)
* Contributors: Esteve Fernandez, Kyoichi Sugahara, Yutaka Kondo

0.38.0 (2024-11-08)
-------------------
* unify package.xml version to 0.37.0
* refactor(component_interface_specs): prefix package and namespace with autoware (`#9094 <https://github.com/autowarefoundation/autoware_universe/issues/9094>`_)
* fix(default_ad_api): fix unusedFunction (`#8581 <https://github.com/autowarefoundation/autoware_universe/issues/8581>`_)
  * fix: unusedFunction
  * Revert "fix: unusedFunction"
  This reverts commit c70a36d4d29668f02dae9416f202ccd05abee552.
  * fix: unusedFunction
  ---------
  Co-authored-by: kobayu858 <129580202+kobayu858@users.noreply.github.com>
* chore(autoware_default_adapi)!: prefix autoware to package name (`#8533 <https://github.com/autowarefoundation/autoware_universe/issues/8533>`_)
* Contributors: Esteve Fernandez, Hayate TOBA, Takagi, Isamu, Yutaka Kondo

0.26.0 (2024-04-03)
-------------------
