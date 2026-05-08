^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_control_command_gate
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.51.0 (2026-05-01)
-------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* feat(autoware_control_command_gate): adopt cie (`#12321 <https://github.com/mitsudome-r/autoware_universe/issues/12321>`_)
  * feat: adopt cie
  * fix: revert ros2 executor
  * Potential fix for pull request finding
  Co-authored-by: Copilot Autofix powered by AI <175728472+Copilot@users.noreply.github.com>
  * fix
  * fix: remove unnecessary lines
  * fix: remove find pakcage
  ---------
  Co-authored-by: Copilot Autofix powered by AI <175728472+Copilot@users.noreply.github.com>
* feat(control_command_filter, vehicle_cmd_gate): add logging capabilities for command limits (`#12250 <https://github.com/mitsudome-r/autoware_universe/issues/12250>`_)
  * feat(control_command_filter): add logging capabilities for command limits
  * feat(vehicle_cmd_filter): add logging for velocity, acceleration, jerk, and steering limits
  * feat(command_filter): enhance logging by adding clock support for throttling
* Contributors: Kyoichi Sugahara, Tetsuhiro Kawaguchi, github-actions

0.50.0 (2026-02-14)
-------------------
* Merge remote-tracking branch 'origin/main' into humble
* fix(control_command_gate): fix funcArgNamesDifferent (`#11975 <https://github.com/autowarefoundation/autoware_universe/issues/11975>`_)
* Contributors: Ryohsuke Mitsudome, Ryuta Kambe

0.49.0 (2025-12-30)
-------------------

0.48.0 (2025-11-18)
-------------------
* Merge remote-tracking branch 'origin/main' into humble
* feat(control_command_gate): sync command filter  (`#11196 <https://github.com/autowarefoundation/autoware_universe/issues/11196>`_)
  * feat(control_command_gate): sync command filter
  * update params
  * sync
  ---------
* fix(control_command_gate): fix comment style (`#11465 <https://github.com/autowarefoundation/autoware_universe/issues/11465>`_)
* fix(autoware_control_command_gate, autoware_vehicle_cmd_gate): remove unused function  (`#11207 <https://github.com/autowarefoundation/autoware_universe/issues/11207>`_)
  * fix(autoware_control_command_gate): remove unused function
  * remove unused function
  ---------
* fix(autoware_control_command_gate): solve ignoredReturnValue warning (`#11181 <https://github.com/autowarefoundation/autoware_universe/issues/11181>`_)
* Contributors: Ryohsuke Mitsudome, Ryuta Kambe, Takagi, Isamu

0.47.1 (2025-08-14)
-------------------

0.47.0 (2025-08-11)
-------------------
* feat: enable to remap control cmd from autoware launch (`#11019 <https://github.com/autowarefoundation/autoware_universe/issues/11019>`_)
  * feat: add args
  * feat: add another cmd
  * fix
  * feat: add comment
  * style(pre-commit): autofix
  * feat: remove ~/
  * feat: remap vehicle cmd gate as well
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* feat(control_command_gate): support vehicle_cmd_gate interface (`#10917 <https://github.com/autowarefoundation/autoware_universe/issues/10917>`_)
* fix(control_command_gate): fix for pause interface  (`#10877 <https://github.com/autowarefoundation/autoware_universe/issues/10877>`_)
  * fix(control_command_gate): add pause interface
  * fix acceleration
  ---------
* Contributors: Takagi, Isamu, Tetsuhiro Kawaguchi

0.46.0 (2025-06-20)
-------------------
* Merge remote-tracking branch 'upstream/main' into tmp/TaikiYamada/bump_version_base
* feat(control_command_gate): create package (`#10169 <https://github.com/autowarefoundation/autoware_universe/issues/10169>`_)
* Contributors: TaikiYamada4, Takagi, Isamu
