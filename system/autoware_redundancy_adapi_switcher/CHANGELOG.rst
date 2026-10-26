^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_redundancy_adapi_switcher
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.53.0 (2026-09-29)
-------------------
* Merge remote-tracking branch 'origin/main' into prepare-0.53.0-changelog
* feat(autoware_redundancy_adapi_switcher): apply agnocast redundancy adapi switcher (`#13415 <https://github.com/autowarefoundation/autoware_universe/issues/13415>`_)
  * refactor(autoware_redundancy_adapi_switcher): migrate to agnocast_wrapper::Node
  * refactor(autoware_redundancy_adapi_switcher): preload the Agnocast heaphook in the launch file
  Include agnocast_env.launch.xml and set LD_PRELOAD on the node so that
  launching this file with ENABLE_AGNOCAST=1 runs the node with the heaphook.
  ---------
* feat(autoware_redundancy_adapi_switcher): add redundancy adapi switcher node (`#12700 <https://github.com/autowarefoundation/autoware_universe/issues/12700>`_)
  * feat: add node
  * fix: node exec
  * style(pre-commit): autofix
  * fix: refactor
  * docs(autoware_redundancy_adapi_switcher): clarify Neither ECU ID behavior in README
  The README claimed the other ECU becomes the output master when
  neither ECU ID is active, but this node cannot guarantee that. Its
  own behavior is to block its own output in this case.
  ---------
  Co-authored-by: pre-commit-ci-lite[bot] <117423508+pre-commit-ci-lite[bot]@users.noreply.github.com>
* Contributors: Ryohsuke Mitsudome, Tetsuhiro Kawaguchi
