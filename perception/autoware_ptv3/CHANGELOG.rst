^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_ptv3
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.48.0 (2025-11-18)
-------------------
* Merge remote-tracking branch 'origin/main' into humble
* feat(autoware_ptv3): implemented an inference node for ptv3 using tensorrt (`#10600 <https://github.com/autowarefoundation/autoware_universe/issues/10600>`_)
  * feat: implemented an inference node for ptv3 using tensorrt
  * chore: cspells
  * chore: schemas
  * chore: lint (line was too long)
  * chore: more schemas
  * fix: mistook the compute capabilities of edge devices
  * chore: replaced incorrect bevfusion -> ptv3
  * chore: forgot to remove unused schema
  * chore: duplicated variable
  * chore: changed package dep name
  * chore: fixed schema comment
  * chore: removed unused headers in the post process kernels
  * chore: replaced in favor of auto
  * chore: removed unused headers
  * chore: changed initialization order
  * chore: replaced 0 by nullptr
  * chore: replaced type in favor of auto
  * chore: removed redundant message
  * chore: fixed compilation due to review changes
  * fix: replaced int64 by uint64
  * chore: added more descriptive comment in the schema
  * style(autoware_ptv3): cleanup
  ---------
  Co-authored-by: Amadeusz Szymko <amadeusz.szymko.2@tier4.jp>
* Contributors: Kenzo Lobos Tsunekawa, Ryohsuke Mitsudome
