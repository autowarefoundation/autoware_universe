^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_redundancy_switcher_interface_plugins
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.53.0 (2026-09-29)
-------------------
* Merge remote-tracking branch 'origin/main' into prepare-0.53.0-changelog
* feat(autoware_redundancy_switcher_interface): add packages (`#13350 <https://github.com/autowarefoundation/autoware_universe/issues/13350>`_)
  * feat: add packages
  * style(pre-commit): autofix
  * fix: refacor and pre-commit
  * style(pre-commit): autofix
  * fix: remove command mode feature
  * fix: response to cppcheck
  * fix: response to spellcheck
  * docs(autoware_redundancy_switcher_interface): add missing is_main_ecu arg to non-redundant launch examples
  is_main_ecu has no default in redundancy_switcher_interface.launch.xml, so the
  non-redundant launch examples in both READMEs failed to start as written.
  ---------
  Co-authored-by: pre-commit-ci-lite[bot] <117423508+pre-commit-ci-lite[bot]@users.noreply.github.com>
* Contributors: Ryohsuke Mitsudome, Tetsuhiro Kawaguchi
