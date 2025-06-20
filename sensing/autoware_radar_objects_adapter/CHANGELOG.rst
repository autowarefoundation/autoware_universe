^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_radar_objects_adapter
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
