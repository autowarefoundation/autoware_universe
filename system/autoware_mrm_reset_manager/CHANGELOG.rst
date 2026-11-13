^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_mrm_reset_manager
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.53.0 (2026-09-29)
-------------------
* Merge remote-tracking branch 'origin/main' into prepare-0.53.0-changelog
* feat: add mrm reset manager (`#12746 <https://github.com/autowarefoundation/autoware_universe/issues/12746>`_)
  * feat(autoware_mrm_reset_manager): add MRM reset manager node
  * fix: codescene
  * fix: codescene
  * fix: service compatibility
  * fix(autoware_mrm_reset_manager): guard switcher interface init when non-redundant
  set_redundancy_switcher_interface_initializing() was called unconditionally
  during init and on the ready-state transition, even though its service is
  only expected to exist when is_redundant is true (are_required_services_ready
  already skips waiting for it otherwise). On a non-redundant configuration the
  call would time out every tick, so the init state machine never reached
  DONE and the aggregator initializing flag (diag graph latch) was never
  cleared.
  Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>
  * fix(autoware_mrm_reset_manager): keep periodic reset until actually ready
  on_periodic_reset_check() gated the periodic reset_redundancy_switcher
  retry on is_autoware_ready(), which only checks whether the
  localization/route/operation_mode topics have been received at least
  once, not their actual values. Once all three arrived (even while
  still uninitialized/unset/not-in-autonomous-control), the periodic
  reset stopped firing and never resumed, even though the system was
  not actually ready to leave the initializing phase.
  Gate on is_ready_for_operation() as well, so the reset keeps being
  requested every tick until localization is initialized, the route is
  set, and autonomous control is enabled.
  Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>
  * docs(autoware_mrm_reset_manager): note is_redundant skips switcher-interface calls too
  The is_redundant parameter description and the init/ready-state
  sections only mentioned reset_redundancy_switcher being skipped when
  is_redundant=false, not set_redundancy_switcher_interface_initializing.
  Now that the latter is guarded the same way, document both.
  Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>
  * style(pre-commit): autofix
  ---------
  Co-authored-by: Claude Sonnet 5 <noreply@anthropic.com>
  Co-authored-by: pre-commit-ci-lite[bot] <117423508+pre-commit-ci-lite[bot]@users.noreply.github.com>
* Contributors: Ryohsuke Mitsudome, Tetsuhiro Kawaguchi
