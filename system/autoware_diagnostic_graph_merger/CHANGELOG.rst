^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_diagnostic_graph_merger
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.53.0 (2026-09-29)
-------------------
* Merge remote-tracking branch 'origin/main' into prepare-0.53.0-changelog
* feat(autoware_diagnostic_graph_merger): apply agnocast diagnostic graph merger (`#13416 <https://github.com/autowarefoundation/autoware_universe/issues/13416>`_)
  * refactor(autoware_diagnostic_graph_merger): migrate to agnocast_wrapper::Node
  * refactor(autoware_diagnostic_graph_merger): preload the Agnocast heaphook in the launch file
  Include agnocast_env.launch.xml and set LD_PRELOAD on the node so that
  launching this file with ENABLE_AGNOCAST=1 runs the node with the heaphook.
  ---------
* feat(autoware_diagnostic_graph_merger): dd diagnostic graph merger (`#12765 <https://github.com/autowarefoundation/autoware_universe/issues/12765>`_)
  * feat: add node
  * feat: add README
  * style(pre-commit): autofix
  * fix: pre-commit
  ---------
  Co-authored-by: pre-commit-ci-lite[bot] <117423508+pre-commit-ci-lite[bot]@users.noreply.github.com>
* Contributors: Ryohsuke Mitsudome, Tetsuhiro Kawaguchi
