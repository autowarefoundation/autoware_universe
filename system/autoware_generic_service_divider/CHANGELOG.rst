^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_generic_service_divider
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.53.0 (2026-09-29)
-------------------
* Merge remote-tracking branch 'origin/main' into prepare-0.53.0-changelog
* feat: add autoware generic service divider (`#12745 <https://github.com/autowarefoundation/autoware_universe/issues/12745>`_)
  * feat: add node
  * style(pre-commit): autofix
  * feat: add ekf trigger node
  * fix(generic_service_divider): add log and fix trigger_node name
  * style(pre-commit): autofix
  * feat(generic_service_divider): wait all server
  * feat(generic_service_divider): add diag
  * feat(generic_service_divider): add control mode request plugins
  * style(pre-commit): autofix
  * fix: pre-commit
  * fix: refactor
  * feat: add README.md
  * fix(autoware_generic_service_divider): fix response finalization race and document known limits (`#12 <https://github.com/autowarefoundation/autoware_universe/issues/12>`_)
  * fix(autoware_generic_service_divider): fix response finalization race and request registration order
  Four fixes found while reviewing the package:
  * `try_finalize_response()` decided whether the division was complete from
  `awaiting_count` alone. `mark_output_completed()` releases `pending->mutex`
  before its caller re-acquires it in `try_finalize_response()`, so when the
  last two outputs complete on different threads both can observe
  `awaiting_count == 0` and finalize the same request, sending a second
  response for the same `rmw_request_id_t`. Add a `finalized` flag guarded by
  `pending->mutex` so finalization runs exactly once. This also covers the
  `catch` path in `forward_request()`, which called `try_finalize_response()`
  without checking the `mark_output_completed()` return value.
  * `GenericClient::async_send_request()` called `rcl_send_request()` outside
  `pending_requests_mutex\_` and only then inserted the pending entry, so a
  response arriving before the insert was dropped by `handle_response()` and
  the division could only end through the timeout path. Hold the mutex across
  both, matching upstream `rclcpp::GenericClient`.
  * Rename the `trigger_node` configuration block to `ekf_trigger_node`, the
  prefix `EkfTriggerNodeDivider` actually reads. Enabling that plugin with the
  old block name produced a divider with no outputs, which still advertises its
  input service but never sends any response.
  * Align the hardcoded default for `reset_redundancy_switcher.input_service`
  with the shipped configuration and the README.
  Co-Authored-By: Claude Opus 5 (1M context) <noreply@anthropic.com>
  * docs(autoware_generic_service_divider): document configuration pitfalls and future work
  Extend "Assumptions / Known limits":
  * `primaries` is not validated: omitting it, or marking no output primary,
  makes every call return "Primary service did not respond" even when all
  outputs succeeded, with no startup warning.
  * A plugin whose configuration block is missing or misnamed falls back to an
  empty output list, advertises its input service and then never replies.
  * Output service names must be unique within a plugin, otherwise the
  outstanding-response counter never reaches zero and the caller hangs.
  * A total plugin load failure is reported as OK by `service_startup_readiness`,
  so `plugin_count` has to be checked against the configured list.
  Drop the note about the unused `trigger_node` block, which is now renamed.
  Add a "Future work" section covering the stale entries in
  `GenericClient::pending_requests\_` and the fail-open startup diagnostics.
  Co-Authored-By: Claude Opus 5 (1M context) <noreply@anthropic.com>
  ---------
  Co-authored-by: Claude Opus 5 (1M context) <noreply@anthropic.com>
  ---------
  Co-authored-by: pre-commit-ci-lite[bot] <117423508+pre-commit-ci-lite[bot]@users.noreply.github.com>
  Co-authored-by: Makoto Kurihara <mkuri8m@gmail.com>
  Co-authored-by: Claude Opus 5 (1M context) <noreply@anthropic.com>
* Contributors: Ryohsuke Mitsudome, Tetsuhiro Kawaguchi
