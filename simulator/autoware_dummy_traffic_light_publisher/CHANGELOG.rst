^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_dummy_traffic_light_publisher
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.52.0 (2026-06-26)
-------------------
* chore: align package versions to 0.51.0 and reset changelogs
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* feat(dummy_traffic_light_publisher): add new node to publish traffic light message instead of Rviz (`#12456 <https://github.com/autowarefoundation/autoware_universe/issues/12456>`_)
  * feat(simulator): add dummy traffic light publisher node for Planning simulator
  * chore(dummy_traffic_light_publisher): add new description.
  * feat(dummy_traffic_light_publisher): updated test codes and implementation
  * style(pre-commit): autofix
  * chore: fixed mistype and cpplint
  * style(pre-commit): autofix
  * fixed lint
  * abstract method.
  * Apply suggestions from code review
  I merged the reviewer's suggestion
  Co-authored-by: Junya Sasaki <j2sasaki1990@gmail.com>
  * refactor(autoware_dummy_traffic_light_publisher): move public headers to private headers under src
  * fix(autoware_dummy_traffic_light_publisher): address review comments
  - use rclcpp::Rate(publish_rate).period() for timer period to avoid
  integer-millisecond truncation and a potential 0 ms busy loop
  - validate TrafficLightCycle durations are positive to avoid fmod by zero
  - document running the node directly with a params file
  * fix(autoware_dummy_traffic_light_publisher): make build_empty_message static
  It does not access any member, so cppcheck's functionStatic check (run
  with --inconclusive in CI) flagged it. Declaring it static resolves the
  cppcheck-differential failure.
  * feat(autoware_dummy_traffic_light_publisher): add fixed mode
  Add a dedicated 'fixed' mode that publishes a single constant color
  (fixed_color: green/yellow/red) for all traffic lights in the vector
  map, for tests that need an always-red or always-green state.
  A dedicated mode is preferred over reusing standalone with 0.0 s
  durations to avoid a magic-number configuration. Per-ID signals are
  intentionally left to pass-through, since the IDs are discovered from
  the vector map at runtime.
  ---------
  Co-authored-by: pre-commit-ci-lite[bot] <117423508+pre-commit-ci-lite[bot]@users.noreply.github.com>
  Co-authored-by: Junya Sasaki <junya.sasaki@tier4.jp>
  Co-authored-by: Junya Sasaki <j2sasaki1990@gmail.com>
* Contributors: Takayuki AKAMINE, github-actions
