^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_redundancy_command_selector
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.53.0 (2026-09-29)
-------------------
* Merge remote-tracking branch 'origin/main' into prepare-0.53.0-changelog
* feat(autoware_redundancy_command_selector): apply agnocast redundancy command selector (`#13414 <https://github.com/autowarefoundation/autoware_universe/issues/13414>`_)
  * refactor(autoware_redundancy_command_selector): migrate to agnocast_wrapper::Node
  * refactor(autoware_redundancy_command_selector): preload the Agnocast heaphook in the launch file
  Include agnocast_env.launch.xml and set LD_PRELOAD on the node so that
  launching this file with ENABLE_AGNOCAST=1 runs the node with the heaphook.
  ---------
* feat(autoware_redundancy_command_selector): add redunduncy command selector node (`#12699 <https://github.com/autowarefoundation/autoware_universe/issues/12699>`_)
  * feat: add node
  * style(pre-commit): autofix
  * fix: refactor
  * fix: package version
  ---------
  Co-authored-by: pre-commit-ci-lite[bot] <117423508+pre-commit-ci-lite[bot]@users.noreply.github.com>
* Contributors: Ryohsuke Mitsudome, Tetsuhiro Kawaguchi
