^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_command_mode_decider
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.50.0 (2026-02-14)
-------------------

0.49.0 (2025-12-30)
-------------------
* Merge remote-tracking branch 'origin/main' into prepare-0.49.0-changelog
* feat(command_mode_decider, switcher): use velocity only and steer only conditions (`#11811 <https://github.com/autowarefoundation/autoware_universe/issues/11811>`_)
  * feat(command_mode_decider): fix to use velocity only and steer only
  status
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci-lite[bot] <117423508+pre-commit-ci-lite[bot]@users.noreply.github.com>
* Contributors: Makoto Kurihara, Ryohsuke Mitsudome

0.48.0 (2025-11-18)
-------------------
* Merge remote-tracking branch 'origin/main' into humble
* fix(command_mode_decider): fix driving mode change condition (`#11659 <https://github.com/autowarefoundation/autoware_universe/issues/11659>`_)
* fix(autoware_command_mode_decider): fix operation mode timeout to manual (`#11379 <https://github.com/autowarefoundation/autoware_universe/issues/11379>`_)
  fix operation mode timeout to manual
  Co-authored-by: SHtokuda <165623782+shtokuda@users.noreply.github.com>
* docs: add document of mode transition debug topic (`#11278 <https://github.com/autowarefoundation/autoware_universe/issues/11278>`_)
  * docs: add document of mode transition debug topic
  * fix
  ---------
* fix(autoware_command_mode_decider): remove unused function (`#11183 <https://github.com/autowarefoundation/autoware_universe/issues/11183>`_)
* Contributors: Ryohsuke Mitsudome, Ryuta Kambe, Takagi, Isamu

0.47.1 (2025-08-14)
-------------------

0.47.0 (2025-08-11)
-------------------
* feat(autoware_command_mode_decider): add initialize (`#10935 <https://github.com/autowarefoundation/autoware_universe/issues/10935>`_)
  * feat: add initialize
  * fix: override in plugins
  ---------
* feat(command_mode_decider): optimize update function call rate (`#10869 <https://github.com/autowarefoundation/autoware_universe/issues/10869>`_)
  * feat(command_mode_decider): optimize update function calls
  * fix manual mode bug
  ---------
* feat(command_mode_decider): reflect message porting (`#10858 <https://github.com/autowarefoundation/autoware_universe/issues/10858>`_)
* Contributors: Takagi, Isamu, Tetsuhiro Kawaguchi

0.46.0 (2025-06-20)
-------------------
* Merge remote-tracking branch 'upstream/main' into tmp/TaikiYamada/bump_version_base
* feat(command_mode_decider): create package (`#10171 <https://github.com/autowarefoundation/autoware_universe/issues/10171>`_)
* Contributors: TaikiYamada4, Takagi, Isamu
