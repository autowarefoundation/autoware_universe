^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_predicted_path_postprocessor
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.53.0 (2026-09-29)
-------------------
* Merge remote-tracking branch 'origin/main' into prepare-0.53.0-changelog
* refactor(perception): move node design files into each package (`#13104 <https://github.com/autowarefoundation/autoware_universe/issues/13104>`_)
  Co-authored-by: Claude Opus 5 (1M context) <noreply@anthropic.com>
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
* Contributors: Mete Fatih Cırıt, Ryohsuke Mitsudome, Taekjin LEE, awf-autoware-bot[bot]

0.52.0 (2026-06-30)
-------------------

0.51.0 (2026-05-01)
-------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* feat(predicted_path_postprocessor): add a processor to refine penetration by static objects (`#11672 <https://github.com/mitsudome-r/autoware_universe/issues/11672>`_)
  * feat: add processor to refine penetration for static objects
  * feat: add a utility function to build interpolation function
  * feat: update builder logic, config and schema file
  * fix: try-catch exception while converting obstacle to polygon
  * test: add unit testings for RefinePenetrationByStaticObjects
  * docs: update README
  * feat: update find_collision() to consider OBB collision detection
  * fix: resolve shadow variable
  * chore: clean up includes
  * chore: fix cpplint error
  ---------
* Contributors: Kotaro Uetake, github-actions

0.50.0 (2026-02-14)
-------------------
* Merge remote-tracking branch 'origin/main' into humble
* fix: add missing ament_index_cpp dependency (`#11875 <https://github.com/autowarefoundation/autoware_universe/issues/11875>`_)
* Contributors: Mete Fatih Cırıt, Ryohsuke Mitsudome

0.49.0 (2025-12-30)
-------------------
* Merge remote-tracking branch 'origin/main' into prepare-0.49.0-changelog
* feat(autoware_lanelet2_utils): replace from/toBinMsg (Sensing, Visualization and Perception Component) (`#11785 <https://github.com/autowarefoundation/autoware_universe/issues/11785>`_)
  * perception component toBinMsg replacement
  * visualization component fromBinMsg replacement
  * sensing component fromBinMsg replacement
  * perception component fromBinMsg replacement
  ---------
* Contributors: Ryohsuke Mitsudome, Sarun MUKDAPITAK

0.48.0 (2025-11-18)
-------------------
* Merge remote-tracking branch 'origin/main' into humble
* feat(predicted_path_postprocessor): add a new package to perform post-process for predicted paths (`#11421 <https://github.com/autowarefoundation/autoware_universe/issues/11421>`_)
  * feat: add baseline
  * feat: add support of publishing debug information
  * refactor: modify input to mutable reference and not to return object
  * feat: add new processor to refine objecs paths by their speed
  * chore: remove sample processor
  * chore: rename namespace
  * docs: add processor naming rules to README
  * fix: consider monotonically increasing
  * feat: update to publish processing time and cyclic time in ms
  * refactor: replace IntermediatePublisher into DebugPublisher
  * feat: include processing time in intermediate reports
  * feat: add lanelet data to the context
  * feat: add Report class and apply it to proccesor output
  * refactor: rename launch arguments
  * refactor: aggregate parameter files
  * refactor: store objects message as shared_ptr in context to enable reflecting their processed result
  * chore: update README and add JSON schema
  * chore: fix typo
  * chore: update maintainers and codeowners
  * test: resolve test failure
  * feat: update processor interface
  * feat: enable to specify interpolation method from config
  ---------
  Co-authored-by: Taekjin LEE <taekjin.lee@tier4.jp>
* Contributors: Kotaro Uetake, Ryohsuke Mitsudome
