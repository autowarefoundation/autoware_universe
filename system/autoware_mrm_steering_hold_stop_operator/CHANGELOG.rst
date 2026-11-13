^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_mrm_steering_hold_stop_operator
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.53.0 (2026-09-29)
-------------------
* Merge remote-tracking branch 'origin/main' into prepare-0.53.0-changelog
* feat(autoware_mrm_steering_hold_stop_operator): add package (`#13423 <https://github.com/autowarefoundation/autoware_universe/issues/13423>`_)
  * feat(autoware_mrm_steering_hold_stop_operator): add package
  Add an MRM operator that holds the steering angle measured at the MRM
  start and stops the vehicle with a constant-jerk deceleration, without
  depending on any planner or trajectory follower. It is triggered by
  tier4_system_msgs/InLaneStopTrigger and owns the deceleration
  constraints per profile as parameters.
  Co-Authored-By: Claude Opus 5.5 (1M context) <noreply@anthropic.com>
  * fix(autoware_mrm_steering_hold_stop_operator): fix cppcheck and spell-check findings
  Co-Authored-By: Claude Opus 5.5 (1M context) <noreply@anthropic.com>
  ---------
  Co-authored-by: Claude Opus 5.5 (1M context) <noreply@anthropic.com>
* Contributors: Makoto Kurihara, Ryohsuke Mitsudome
