^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_agnocast_wrapper
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.49.0 (2025-12-30)
-------------------

0.48.0 (2025-11-18)
-------------------
* Merge remote-tracking branch 'origin/main' into humble
* fix(agnocast_wrapper): fix const rvalue reference binding error in AgnocastPollingSubscriber (`#11425 <https://github.com/autowarefoundation/autoware_universe/issues/11425>`_)
  fix AgnocastPollingSubscriber in agnocast_wrapper
* feat(autoware_agnocast_wrapper): support runtime fallback to ROS 2 behavior (`#11060 <https://github.com/autowarefoundation/autoware_universe/issues/11060>`_)
  * Create message_ptr
  * Create use_agnocast
  * Create Subscription
  * Create Publisher
  * Create PollingSubscriber (wip)
  * Allow copying message_ptr with shared ownership
  * style(pre-commit): autofix
  * Fix typo
  * fix
  * fix
  * style(pre-commit): autofix
  * fix for tidy
  * fix tidy
  * style(pre-commit): autofix
  * try avoiding cpplint error
  * fix
  * fix ament_lint_commont to autoware_lint_common
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: Koichi Imai <koichi.imai.2@tier4.jp>
  Co-authored-by: Koichi Imai <45482193+Koichi98@users.noreply.github.com>
  Co-authored-by: Takahiro Ishikawa-Aso <sykwer@gmail.com>
  Co-authored-by: pre-commit-ci-lite[bot] <117423508+pre-commit-ci-lite[bot]@users.noreply.github.com>
* Contributors: Guojun Wu, Koichi Imai, Ryohsuke Mitsudome

0.47.1 (2025-08-14)
-------------------

0.47.0 (2025-08-11)
-------------------

0.46.0 (2025-06-20)
-------------------

0.45.0 (2025-05-22)
-------------------
* Merge remote-tracking branch 'origin/main' into tmp/notbot/bump_version_base
* chore: re-introduce agnocast and fix version to v2.1.0 (`#10526 <https://github.com/autowarefoundation/autoware_universe/issues/10526>`_)
  delete colcon ignore and fix agnocast version to v2.1.0
* Contributors: TaikiYamada4, Takahiro Ishikawa-Aso
