^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_overlay_rviz_plugin
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.43.0 (2025-03-21)
-------------------
* Merge remote-tracking branch 'origin/main' into chore/bump-version-0.43
* chore: rename from `autoware.universe` to `autoware_universe` (`#10306 <https://github.com/autowarefoundation/autoware_universe/issues/10306>`_)
* feat!: replace VelocityLimit messages with autoware_internal_planning_msgs (`#10273 <https://github.com/autowarefoundation/autoware_universe/issues/10273>`_)
* Contributors: Hayato Mizushima, Ryohsuke Mitsudome, Yutaka Kondo

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
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* fix(autoware_overlay_rviz_plugin): fix clang-tidy errors (`#9627 <https://github.com/autowarefoundation/autoware_universe/issues/9627>`_)
  * fix: clang-tidy errors
  * fix: clang-fmt
  ---------
* Contributors: Fumiya Watanabe, kobayu858

0.40.0 (2024-12-12)
-------------------
* Merge branch 'main' into release-0.40.0
* Revert "chore(package.xml): bump version to 0.39.0 (`#9587 <https://github.com/autowarefoundation/autoware_universe/issues/9587>`_)"
  This reverts commit c9f0f2688c57b0f657f5c1f28f036a970682e7f5.
* chore: move rviz plugins from common to visualization/ folder (`#9417 <https://github.com/autowarefoundation/autoware_universe/issues/9417>`_)
* Contributors: Esteve Fernandez, Fumiya Watanabe

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
* fix: missing dependency in common components (`#9072 <https://github.com/autowarefoundation/autoware_universe/issues/9072>`_)
* Contributors: Esteve Fernandez, Yutaka Kondo, ぐるぐる

0.38.0 (2024-11-08)
-------------------
* unify package.xml version to 0.37.0
* fix(speed_display): always show speed positive and depend on gear for negatives (`#8957 <https://github.com/autowarefoundation/autoware_universe/issues/8957>`_)
* style: update state panel plugin (`#8846 <https://github.com/autowarefoundation/autoware_universe/issues/8846>`_)
* feat: add color customization to gear speed mission speed limit and traffic displays (`#8142 <https://github.com/autowarefoundation/autoware_universe/issues/8142>`_)
  feat: Add color customization to gear, speed, mission,speedlimit and traffic displays
* fix(autoware_overlay_rviz_plugin): topic type of traffic light (`#8098 <https://github.com/autowarefoundation/autoware_universe/issues/8098>`_)
  * fix(autoware_overlay_rviz_plugin): topic type of traffic light
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* feat: add handle angle scale property to signal display (`#7774 <https://github.com/autowarefoundation/autoware_universe/issues/7774>`_)
  * feat: add handle angle scale property to signal display
  * fix: set default steering angle to 0.0° when not received
  * fix: set default steering angle to N/A when not received and check for msg_ptr instead of float value
  * chore: update steering wheel font size and wheel icon center is bigger
  * chore: update steering wheel display to center the steering angle text
  * chore: align steering angle text in both negative and positive cases
  ---------
* feat!: replace autoware_auto_msgs with autoware_msgs for common modules (`#7239 <https://github.com/autowarefoundation/autoware_universe/issues/7239>`_)
  Co-authored-by: Cynthia Liu <cynthia.liu@autocore.ai>
  Co-authored-by: NorahXiong <norah.xiong@autocore.ai>
  Co-authored-by: beginningfan <beginning.fan@autocore.ai>
* fix(autoware_overlay_rviz_plugin): fix subs and cleanup (`#6978 <https://github.com/autowarefoundation/autoware_universe/issues/6978>`_)
* feat: update rviz2 overlay (`#6883 <https://github.com/autowarefoundation/autoware_universe/issues/6883>`_)
* feat(autoware_overlay_rviz_plugin): get the current traffic light (`#6899 <https://github.com/autowarefoundation/autoware_universe/issues/6899>`_)
* Contributors: Khalil Selyan, M. Fatih Cırıt, Maxime CLEMENT, Ryohsuke Mitsudome, Tomohito ANDO, Yutaka Kondo

0.26.0 (2024-04-03)
-------------------
* build(autoware_overlay_rviz_plugin): add missing ament_cmake_auto dependency (`#6519 <https://github.com/autowarefoundation/autoware_universe/issues/6519>`_)
* feat: update vehicle overlay plugin (`#6323 <https://github.com/autowarefoundation/autoware_universe/issues/6323>`_)
* Contributors: Esteve Fernandez, Khalil Selyan
