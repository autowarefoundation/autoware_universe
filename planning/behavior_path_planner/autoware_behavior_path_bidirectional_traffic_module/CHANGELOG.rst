^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_behavior_path_bidirectional_traffic_module
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.53.0 (2026-09-29)
-------------------
* Merge remote-tracking branch 'origin/main' into prepare-0.53.0-changelog
* chore(pre-commit): update clang-format to v22.1.5 (`#13126 <https://github.com/autowarefoundation/autoware_universe/issues/13126>`_)
  * chore(pre-commit): update clang-format to v22.1.5
  * style(pre-commit): autofix
  ---------
* fix(pre-commit): update pre-commit-hooks-ros to v0.10.3 and adapt include guards (`#13083 <https://github.com/autowarefoundation/autoware_universe/issues/13083>`_)
  * chore: sync files
  * fix(pre-commit): adapt include guards to pre-commit-hooks-ros v0.10.3
  ros-include-guard v0.10.3 only recognises an include guard when #endif is the
  last non-empty line of the file, so that feature test macros are no longer
  mistaken for guards. 29 headers failed that check.
  25 headers wrap the guard in "// clang-format off" / "// clang-format on"
  because the #endif comment plus its // NOLINT exceeds the 100 column limit.
  Drop only the trailing "on" marker; the "off" marker then runs to end of file
  and still protects the line from being wrapped.
  3 CUDA headers ended with "/* *INDENT-ON* */". Move it above the #endif so it
  stays paired with the "/* *INDENT-OFF* */" near the top of the file.
  autoware_behavior_path_planner/test/input.hpp closed its guard immediately after
  opening it, leaving the entire body unguarded. Move the #endif to the end.
  Also hold clang-format at v21.1.8. clang-format 22 migrates
  "AlignAfterOpenBracket: AlwaysBreak" to "BreakAfterOpenBracketIf: true", which
  forces a break after every "if (" whose condition does not fit on one line and
  reformats 88 files.
  ---------
  Co-authored-by: github-actions <github-actions@github.com>
  Co-authored-by: Mete Fatih Cırıt <mfc@autoware.org>
* Contributors: Mete Fatih Cırıt, Ryohsuke Mitsudome, awf-autoware-bot[bot]

0.52.0 (2026-06-30)
-------------------

0.51.0 (2026-05-01)
-------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* feat(planning): replace autoware_universe_utils with specific autoware_utils sub-packagesr (`#12443 <https://github.com/mitsudome-r/autoware_universe/issues/12443>`_)
* Contributors: Vishal Chauhan, github-actions

0.50.0 (2026-02-14)
-------------------

0.49.0 (2025-12-30)
-------------------

0.48.0 (2025-11-18)
-------------------
* Merge remote-tracking branch 'origin/main' into humble
* fix(bidirectional_traffic): suppress warning (`#11221 <https://github.com/autowarefoundation/autoware_universe/issues/11221>`_)
* fix(autoware_behavior_path_bidirectional_traffic_module): remove unused function (`#11185 <https://github.com/autowarefoundation/autoware_universe/issues/11185>`_)
* Contributors: Ryohsuke Mitsudome, Ryuta Kambe, Yukinari Hisaki

0.47.1 (2025-08-14)
-------------------

0.47.0 (2025-08-11)
-------------------
* style(pre-commit): update to clang-format-20 (`#11088 <https://github.com/autowarefoundation/autoware_universe/issues/11088>`_)
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* chore(bidirectional_traffic): modify readme (`#10943 <https://github.com/autowarefoundation/autoware_universe/issues/10943>`_)
  Update README.md
* Contributors: Mete Fatih Cırıt, Yukinari Hisaki

0.46.0 (2025-06-20)
-------------------
* Merge remote-tracking branch 'upstream/main' into tmp/TaikiYamada/bump_version_base
* feat(autoware_behavior_path_bidirectional_traffic_module): add a functionality for bidirectional traffic (`#10394 <https://github.com/autowarefoundation/autoware_universe/issues/10394>`_)
  * add bidirectional traffic module
  * first version
  * add virtual wall
  * WIP
  * apply new message type
  * revert autoware_trajectory change
  * WIP
  * developping...
  * fix bug
  * update module
  * remove stopping in BackToNormalLane mode
  * fix bug maybe
  * tidy
  * add document
  * rename images
  * add test, tidy.
  * catch up with latest branch
  * fix
  * style(pre-commit): autofix
  * update
  * downsizing svg files
  * fix
  * fix
  * tidy
  * fix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* Contributors: TaikiYamada4, Yukinari Hisaki
