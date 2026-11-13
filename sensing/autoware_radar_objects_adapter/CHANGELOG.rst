^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_radar_objects_adapter
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.53.0 (2026-09-29)
-------------------
* Merge remote-tracking branch 'origin/main' into prepare-0.53.0-changelog
* test(autoware_radar_objects_adapter): add characterization test for RadarObjectsAdapter startup and radar info gate (`#13405 <https://github.com/autowarefoundation/autoware_universe/issues/13405>`_)
  * test(autoware_radar_objects_adapter): add a characterization test harness for RadarObjectsAdapter
  Phase 1 of the node/logic separation of radar_objects_adapter: a gtest
  that drives the node over its real topics from a peer node, pumped from
  the test thread. This first part pins how the node starts: the six
  default\_* parameters are required, the classification_remap.* ones are
  not. The topic names, message types and the QoS the node subscribes
  with are exercised by the peer's endpoints in every case that feeds the
  node, so they get no case of their own. The behavior of the conversion
  follows in later commits.
  The test is added as an isolated ROS gtest and is skipped when
  ENABLE_AGNOCAST=1, like the other ROS-based tests in this repository.
  No production code is changed.
  Co-Authored-By: Claude Fable 5.1 <noreply@anthropic.com>
  * test(autoware_radar_objects_adapter): pin the radar info gate of RadarObjectsAdapter
  Radar objects are converted only after a radar info message has
  declared all eight required fields; until then they are dropped and not
  replayed once the radar info arrives. These tests pin that gate as it
  is today.
  The builders for radar info and radar objects messages, and the fixture
  helpers that send them, come in with this commit because these are the
  first tests that feed the node.
  Co-Authored-By: Claude Fable 5.1 <noreply@anthropic.com>
  * test(autoware_radar_objects_adapter): say what the incomplete radar info case tells apart
  The comment on Gate_RadarInfoMissingRequiredField_ObjectsDropped now
  states what the case distinguishes (a gate that reads the radar info
  from one that opens on any) and what it does not observe (whether the
  message was rejected or ignored, and which fields are required), as
  discussed in review. Comment only.
  Co-Authored-By: Claude Fable 5.1 <noreply@anthropic.com>
  ---------
  Co-authored-by: Claude Fable 5.1 <noreply@anthropic.com>
* feat(radar_objects_adapter): apply `agnocast_wrapper::Node` to `radar_objects_adapter` (`#12876 <https://github.com/autowarefoundation/autoware_universe/issues/12876>`_)
  * apply agnocast_wrapper::Node
  * keep const reference subscription callbacks
  * refactor(radar_objects_adapter): read the input topic name from the subscription
  * fix(cuda_utils): guard the CHECK_CUDA_ERROR macro against redefinition
  * Revert "fix(cuda_utils): guard the CHECK_CUDA_ERROR macro against redefinition"
  This reverts commit d469313341c666d0e9e5401696bc86363f5e712c.
  ---------
  Co-authored-by: kobayu858 <yutaro.kobayashi.2@tier4.jp>
* refactor(sensing): move node design files into each package (`#13105 <https://github.com/autowarefoundation/autoware_universe/issues/13105>`_)
* Contributors: Kentaro Nagatomo, Koichi Imai, Ryohsuke Mitsudome, Taekjin LEE

0.52.0 (2026-06-30)
-------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* feat(radar_objects_adapter): apply agnocast for publisher of `radar_objects_adapter` (`#12756 <https://github.com/autowarefoundation/autoware_universe/issues/12756>`_)
  apply agnocast for publisher
* Contributors: Koichi Imai, github-actions

0.51.0 (2026-05-01)
-------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* feat(autoware_radar_objects_adapter): remove dependency on autoware_universe_utils in sensing (`#12407 <https://github.com/mitsudome-r/autoware_universe/issues/12407>`_)
  * feat(autoware_radar_objects_adapter): replace autoware_universe_utils with autoware_utils
  Replace dependency on autoware_universe_utils with autoware_utils
  in sensing/autoware_radar_objects_adapter.
  Related to `#12376 <https://github.com/mitsudome-r/autoware_universe/issues/12376>`_
  * fix: rename createQuaternionFromYaw to create_quaternion_from_yaw
  * fix(autoware_radar_objects_adapter): use autoware_utils_geometry
  ---------
  Co-authored-by: github-actions <github-actions@github.com>
* feat(radar_objects_adapter): apply autoware_agnocast_wrapper for CIE (`#12326 <https://github.com/mitsudome-r/autoware_universe/issues/12326>`_)
  feat(autoware_radar_objects_adapter): apply autoware_agnocast_wrapper for CIE
* docs(sensing): fix mkdocs macro rendering and links in sensing pages (`#12111 <https://github.com/mitsudome-r/autoware_universe/issues/12111>`_)
  docs(sensing): fix mkdocs macro paths, links, and schema fields
* Contributors: Max Schmeller, Vishal Chauhan, atsushi yano, github-actions

0.50.0 (2026-02-14)
-------------------

0.49.0 (2025-12-30)
-------------------

0.48.0 (2025-11-18)
-------------------
* Merge remote-tracking branch 'origin/main' into humble
* feat(radar_objects_adapter): enable to class remap and relocate bike labels into car (`#11570 <https://github.com/autowarefoundation/autoware_universe/issues/11570>`_)
  * feat: add radar class remap function to radar remap
  * chore: fix schema
  * fix: update default classification
  * docs: update readme
  * style(pre-commit): autofix
  * docs: fix readme
  * Update sensing/autoware_radar_objects_adapter/schema/radar_objects_adapter.schema.json
  Co-authored-by: Copilot <175728472+Copilot@users.noreply.github.com>
  * fix: precommit-fix
  * fix: fix radar object adapter param from list to dict style
  * chore: fix schema
  ---------
  Co-authored-by: pre-commit-ci-lite[bot] <117423508+pre-commit-ci-lite[bot]@users.noreply.github.com>
  Co-authored-by: Copilot <175728472+Copilot@users.noreply.github.com>
  Co-authored-by: Taekjin LEE <taekjin.lee@tier4.jp>
* fix(autoware_radar_objects_adapter): object orientation availability (`#11164 <https://github.com/autowarefoundation/autoware_universe/issues/11164>`_)
  * fix(radar_objects_adapter): set orientation availability for detected and tracked objects
  * style(pre-commit): autofix
  * fix(radar_objects_adapter): enhance kinematics data for detected and tracked objects
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* Contributors: Ryohsuke Mitsudome, Taekjin LEE, Yoshi Ri

0.47.1 (2025-08-14)
-------------------

0.47.0 (2025-08-11)
-------------------

0.46.0 (2025-06-20)
-------------------

0.45.0 (2025-05-22)
-------------------
* Merge remote-tracking branch 'origin/main' into tmp/notbot/bump_version_base
* fix(autoware_radar_objects_adapter): update schema path in readme (`#10607 <https://github.com/autowarefoundation/autoware_universe/issues/10607>`_)
  fix(autoware_radar_objects_adapter): update schema path in README.md
* feat(autoware_radar_objects_adapter): add publisher for tracks with uuid (`#10556 <https://github.com/autowarefoundation/autoware_universe/issues/10556>`_)
  * feat: added the option to publish tracks in addition to detections
  * chore: forgot to add the uuids
  * feat: added hashes to avoid conflicts between radars
  * feat(radar_objects_adapter): update QoS settings for detections and tracks publishers
  * refactor: integrate common convert processes
  refactor: simplify function signatures in radar_objects_adapter
  * refactor: rename parameters for clarity in radar covariance functions
  * refactor: rename input parameter for clarity in objects_callback and related functions
  ---------
  Co-authored-by: Taekjin LEE <taekjin.lee@tier4.jp>
* feat(autoware_radar_objects_adapter): adapter from sensing radar objects into perception detections (`#10459 <https://github.com/autowarefoundation/autoware_universe/issues/10459>`_)
  * feat: adapter from sensing radar objects into perception detections
  * chore: bumped the autoware_msgs tag
  * Update sensing/autoware_radar_objects_adapter/package.xml
  Co-authored-by: Taekjin LEE <technolojin@gmail.com>
  * Update sensing/autoware_radar_objects_adapter/package.xml
  ---------
  Co-authored-by: Taekjin LEE <taekjin.lee@tier4.jp>
  Co-authored-by: Taekjin LEE <technolojin@gmail.com>
* Contributors: Kenzo Lobos Tsunekawa, Taekjin LEE, TaikiYamada4
