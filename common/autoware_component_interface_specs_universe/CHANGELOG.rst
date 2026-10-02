^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_component_interface_specs_universe
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.53.0 (2026-09-29)
-------------------
* Merge remote-tracking branch 'origin/main' into prepare-0.53.0-changelog
* refactor(autoware_component_interface_specs_universe): re-export the core interface specs as the single version authority (`#13115 <https://github.com/autowarefoundation/autoware_universe/issues/13115>`_)
  * refactor(autoware_component_interface_specs_universe): re-export the core interface specs (single version authority)
  Make autoware_component_interface_specs (core) the single definition
  and version authority for the shared component interface specs, and
  convert autoware_component_interface_specs_universe into re-export
  shims (using-declarations) of the core symbols.
  Every core-authoritative struct that this package previously
  duplicated is deleted and replaced with a using-declaration
  that aliases the core type, so the type identity is preserved:
  existing universe consumers keep compiling unchanged and
  now resolve the canonical core type (proven by the per-domain
  alias-identity static_asserts). This also picks up the new core
  symbols per domain (perception TrafficSignals/DetectedObjects;
  control GearCommand/TurnIndicatorsCommand/HazardLightsCommand;
  map VectorMap/PointCloudMap/GetDifferentialPointCloudMap; vehicle
  VelocityStatus; system HazardStatus and the promoted MrmState) plus
  each domain's version and Specs.
  A new sensing.hpp shim re-exports the new core sensing domain
  (VehicleVelocityConverterTwist, version, Specs).
  The tier4-only / vendor structs stay in this package exactly as they
  were, unversioned: control keeps ActuationCommand, SetPause, IsPaused,
  IsStartRequested, SetStop, IsStopped; vehicle keeps EnergyStatus,
  DoorCommand, DoorLayout, DoorStatus. They are not registered in the
  core Specs tuples.
  package.xml gains a single
  <depend>autoware_component_interface_specs</depend> so the
  shims can see the core package. It also drops the seven
  message-package dependencies whose only direct includes this
  change removes (autoware_localization_msgs, autoware_map_msgs,
  autoware_perception_msgs, autoware_planning_msgs, autoware_system_msgs,
  autoware_vehicle_msgs, nav_msgs): nothing in this package includes
  them directly any more, and core already depends on them and re-exports
  that dependency to this package's consumers.
  No behavior change: no QoS/topic/service-name change to any live wire;
  type identities are preserved via using-declarations.
  The existing universe unit tests are kept untouched and act as the
  QoS/name parity oracle against the re-exported core definitions; a
  diff of every deleted universe struct against its core copy showed no
  field divergence. Each domain test additionally gets an alias-identity
  static_assert, and a new test_sensing.cpp is added and registered
  in CMakeLists.txt.
  * refactor(autoware_component_interface_specs_universe): drop the PointCloudMap re-export
  PointCloudMap's spec (/map/point_cloud_map) is a never-published topic,
  it has no in-tree universe consumer, and core already excludes it from
  the versioned Specs tuple, since it is a raw, high-bandwidth topic that
  core deliberately keeps out of the versioned spec set. Re-exporting
  it added a symbol nobody resolves through the universe namespace,
  so remove the `using` and explain the deliberate exclusion in the
  header comment. The other map re-exports (VectorMap, MapProjectorInfo,
  GetDifferentialPointCloudMap, Specs, version) are unchanged.
  * refactor(autoware_component_interface_specs_universe): drop the HazardStatus re-export
  The core system domain no longer defines HazardStatus
  (/system/emergency/hazard_status was removed from the
  autowarefoundation-owned spec set because it has no OSS consumers),
  so remove the alias. No universe code consumes it.
  * docs(autoware_component_interface_specs_universe): state re-export rationale directly in comments
  The header and test comments in this package now state the re-export
  rationale directly, instead of pointing readers to outside documents:
  - core is the sole definition and version authority for these specs
  - MrmState was promoted from universe to core and keeps compiling via
  this re-export
  - PointCloudMap is a raw, high-bandwidth topic that core deliberately
  excludes from its versioned spec set
  - the tier4-only / tier4-adapi-only vendor structs stay unversioned
  until vendor-specific specs get their own versioned registry,
  separate from core's OSS-facing Specs tuple
  No functional change.
  * fix(autoware_component_interface_specs_universe): drop the DetectedObjects re-export
  The core perception domain does not define a DetectedObjects spec; it
  only registers ObjectRecognition, TrafficSignals, and TrackedObjects.
  The using-declaration and its alias-identity static_assert referenced
  a symbol that has never existed in core, so both failed to compile.
  This package's own perception.hpp never declared a DetectedObjects
  spec either, so no consumer can be relying on it. Remove both.
  * fix(autoware_component_interface_specs_universe): match the core sensing QoS depth
  The core spec declares depth 10 for VehicleVelocityConverterTwist
  (the vehicle-velocity-converter twist topic), but this test asserted
  a stale local value of 1. Since core is now the single version
  authority for this spec, the test must assert core's actual value.
  * fix(autoware_component_interface_specs_universe): alias five specs missing from the shim
  control, map, perception, and vehicle each re-export every spec core
  registers in that domain except for ControlModeRequest,
  PredictedTrajectory, GetPartialPointCloudMap, TrackedObjects, and
  ControlModeStatus. Add the missing using-declarations so the shim
  tracks core's registered surface consistently within each header.
  ---------
* Contributors: Ryohsuke Mitsudome, Yutaka Kondo

0.52.0 (2026-06-30)
-------------------

0.51.0 (2026-05-01)
-------------------

0.50.0 (2026-02-14)
-------------------

0.49.0 (2025-12-30)
-------------------

0.48.0 (2025-11-18)
-------------------

0.47.1 (2025-08-14)
-------------------

0.47.0 (2025-08-11)
-------------------
* feat: change planning output topic name to /planning/trajectory (`#11135 <https://github.com/autowarefoundation/autoware_universe/issues/11135>`_)
  * change planning output topic name to /planning/trajectory
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* Contributors: Yukihiro Saito

0.46.0 (2025-06-20)
-------------------
* Merge remote-tracking branch 'upstream/main' into tmp/TaikiYamada/bump_version_base
* feat!: replace tier4_system_msgs with autoware_system_msgs for services (`#10842 <https://github.com/autowarefoundation/autoware_universe/issues/10842>`_)
  * feat!: replace tier4_system_msgs with autoware_system_msgs for services
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* feat!: replace autoware_internal_localization_msgs with autoware_localization_msgs for InitializeLocalization service (`#10844 <https://github.com/autowarefoundation/autoware_universe/issues/10844>`_)
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* feat!: replace tier4_planning_msgs service with autoware_planning_msgs (`#10827 <https://github.com/autowarefoundation/autoware_universe/issues/10827>`_)
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* feat(default_adapi): add vehicle command api (`#10764 <https://github.com/autowarefoundation/autoware_universe/issues/10764>`_)
* Contributors: Ryohsuke Mitsudome, TaikiYamada4, Takagi, Isamu

0.45.0 (2025-05-22)
-------------------
* Merge remote-tracking branch 'origin/main' into tmp/notbot/bump_version_base
* feat(localization): replace tier4_localization_msgs used by ndt_align_srv with autoware_internal_localization_msgs (`#10567 <https://github.com/autowarefoundation/autoware_universe/issues/10567>`_)
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* Contributors: TaikiYamada4, 心刚

0.44.2 (2025-06-10)
-------------------

0.44.1 (2025-05-01)
-------------------

0.44.0 (2025-04-18)
-------------------

0.43.0 (2025-03-21)
-------------------
* Merge remote-tracking branch 'origin/main' into chore/bump-version-0.43
* chore: rename from `autoware.universe` to `autoware_universe` (`#10306 <https://github.com/autowarefoundation/autoware_universe/issues/10306>`_)
* Contributors: Hayato Mizushima, Yutaka Kondo

0.42.0 (2025-03-03)
-------------------

0.41.2 (2025-02-19)
-------------------
* chore: bump version to 0.41.1 (`#10088 <https://github.com/autowarefoundation/autoware_universe/issues/10088>`_)
* Contributors: Ryohsuke Mitsudome

0.41.1 (2025-02-10)
-------------------

0.41.0 (2025-01-29)
-------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* feat(autoware_component_interface_specs_universe!): rename package (`#9753 <https://github.com/autowarefoundation/autoware_universe/issues/9753>`_)
* Contributors: Fumiya Watanabe, Ryohsuke Mitsudome

0.40.0 (2024-12-12)
-------------------
* Merge branch 'main' into release-0.40.0
* Revert "chore(package.xml): bump version to 0.39.0 (`#9587 <https://github.com/autowarefoundation/autoware_universe/issues/9587>`_)"
  This reverts commit c9f0f2688c57b0f657f5c1f28f036a970682e7f5.
* fix: fix ticket links in CHANGELOG.rst (`#9588 <https://github.com/autowarefoundation/autoware_universe/issues/9588>`_)
* chore(package.xml): bump version to 0.39.0 (`#9587 <https://github.com/autowarefoundation/autoware_universe/issues/9587>`_)
  * chore(package.xml): bump version to 0.39.0
  * fix: fix ticket links in CHANGELOG.rst
  * fix: remove unnecessary diff
  ---------
  Co-authored-by: Yutaka Kondo <yutaka.kondo@youtalk.jp>
* fix: fix ticket links in CHANGELOG.rst (`#9588 <https://github.com/autowarefoundation/autoware_universe/issues/9588>`_)
* feat!: replace tier4_map_msgs with autoware_map_msgs for MapProjectorInfo (`#9392 <https://github.com/autowarefoundation/autoware_universe/issues/9392>`_)
* 0.39.0
* update changelog
* fix: fix ticket links to point to https://github.com/autowarefoundation/autoware_universe (`#9304 <https://github.com/autowarefoundation/autoware_universe/issues/9304>`_)
* fix: fix ticket links to point to https://github.com/autowarefoundation/autoware_universe (`#9304 <https://github.com/autowarefoundation/autoware_universe/issues/9304>`_)
* chore(package.xml): bump version to 0.38.0 (`#9266 <https://github.com/autowarefoundation/autoware_universe/issues/9266>`_) (`#9284 <https://github.com/autowarefoundation/autoware_universe/issues/9284>`_)
  * unify package.xml version to 0.37.0
  * remove system_monitor/CHANGELOG.rst
  * add changelog
  * 0.38.0
  ---------
* Contributors: Esteve Fernandez, Fumiya Watanabe, Ryohsuke Mitsudome, Yutaka Kondo

0.39.0 (2024-11-25)
-------------------
* fix: fix ticket links to point to https://github.com/autowarefoundation/autoware_universe (`#9304 <https://github.com/autowarefoundation/autoware_universe/issues/9304>`_)
* fix: fix ticket links to point to https://github.com/autowarefoundation/autoware_universe (`#9304 <https://github.com/autowarefoundation/autoware_universe/issues/9304>`_)
* chore(package.xml): bump version to 0.38.0 (`#9266 <https://github.com/autowarefoundation/autoware_universe/issues/9266>`_) (`#9284 <https://github.com/autowarefoundation/autoware_universe/issues/9284>`_)
  * unify package.xml version to 0.37.0
  * remove system_monitor/CHANGELOG.rst
  * add changelog
  * 0.38.0
  ---------
* Contributors: Esteve Fernandez, Yutaka Kondo

0.38.0 (2024-11-08)
-------------------
* unify package.xml version to 0.37.0
* fix: fix internal door interface qos (`#9144 <https://github.com/autowarefoundation/autoware_universe/issues/9144>`_)
* refactor(component_interface_specs): prefix package and namespace with autoware (`#9094 <https://github.com/autowarefoundation/autoware_universe/issues/9094>`_)
* Contributors: Esteve Fernandez, Takagi, Isamu, Yutaka Kondo

0.26.0 (2024-04-03)
-------------------
