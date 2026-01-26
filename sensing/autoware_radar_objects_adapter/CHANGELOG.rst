^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_radar_objects_adapter
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
