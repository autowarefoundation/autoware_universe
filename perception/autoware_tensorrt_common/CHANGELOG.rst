^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_tensorrt_common
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.53.0 (2026-09-29)
-------------------
* Merge remote-tracking branch 'origin/main' into prepare-0.53.0-changelog
* feat(autoware_tensorrt_common): let a caller offer IO the model may omit (`#13347 <https://github.com/autowarefoundation/autoware_universe/issues/13347>`_)
  * feat(autoware_tensorrt_common): let a caller offer IO the model may omit
  A caller declares the IO its node can serve, and setup() compares that list
  against the model. An architecture variant that gates blocks off declares less
  than the full contract, and the ONNX exporter drops inputs the traced graph
  never consumes, so such a model arrives narrower than the offered list and
  validateNetworkIO rejects it on the count. Every node facing that has to
  enumerate the model itself and pre-filter its own lists before calling setup().
  NetworkIO and ProfileDims entries can now be marked optional, and setup() drops
  the optional ones the model does not declare before profiling or comparing
  them. An all-required list behaves exactly as it does today.
  Reconciliation then requires the offer and the model to account for each other
  in both directions, refusing a mismatch before an engine is built:
  Model does not declare the required IO tensor(s): [...]
  Model declares IO tensor(s) that were not offered: [...]
  The second direction takes over from validateNetworkIO's cardinality check for
  caller error. That check compares counts rather than sets, so one tensor the
  model declares and one surplus offered entry cancelled out and setup() could
  succeed leaving a tensor unbound; and because it runs after the engine is
  loaded, a mismatch discarded a good cached engine and burned a full rebuild
  before failing. The count check stays, now guarding only against the engine's
  IO set diverging from the parsed network's, which reconciliation runs too
  early to see.
  On success setup() says which of the two happened, once:
  Model declares all 27 offered IO tensors
  Model declares 20 of 27 offered IO tensors; skipping the optional tensors it
  does not declare: [serialized_code, ...]
  so a narrower model is visible at load instead of being inferred from a shape
  error later.
  Name resolution for index-addressed entries moves ahead of the profile loop and
  now covers NetworkIO too, since the name is what the comparison and the log
  report; an index that cannot be named is an error rather than a crash assigning
  nullptr to std::string.
  Co-Authored-By: Claude Opus 5 (1M context) <noreply@anthropic.com>
  * test(autoware_tensorrt_common): cover IO reconciliation against a dummy network
  Adds the package's first unit tests. A 256-byte ONNX fixture declares three
  dynamically shaped inputs feeding one output, which is enough to exercise every
  branch of the offered-IO reconciliation: an artifact declaring everything, an
  optional tensor the artifact omits (dropped, and both by-name setters no-op for
  it while a never-offered name still fails), the same when only the profile list
  offers it, a required tensor the artifact omits (refused before an engine is
  built, asserted by the absent engine file), the same via the profile list alone,
  an out-of-range tensor index, a caller offering fewer tensors than the model
  declares, equal counts covering different sets, and an offer of nothing but
  optional tensors the artifact omits. The last two would pass a size comparison
  alone.
  The tensor-metadata cases need no device and always run. The rest build an
  engine, so they use autoware_cuda_utils' SKIP_TEST_IF_CUDA_UNAVAILABLE() to skip
  where there is no GPU, as the CUDA CI job has none.
  Co-Authored-By: Claude Opus 5 (1M context) <noreply@anthropic.com>
  ---------
  Co-authored-by: Claude Opus 5 (1M context) <noreply@anthropic.com>
* feat(autoware_tensorrt_common): add getIOTensorNames (`#13346 <https://github.com/autowarefoundation/autoware_universe/issues/13346>`_)
  Callers that need to know what a model declares had to enumerate
  getNbIOTensors/getIOTensorName themselves, or ask getTensorIOMode one name at
  a time. The latter is a trap: TensorRT logs an error for every name the engine
  does not have, so probing a contract against a narrower model fills the log
  with errors that report nothing wrong.
  getIOTensorNames enumerates once and returns the set, and its documentation
  says why it is the right way to ask. Like the accessors it wraps, it reads the
  parsed network before setup() and the engine afterwards.
  Co-authored-by: Claude Opus 5 (1M context) <noreply@anthropic.com>
* fix(autoware_tensorrt_common): stop warning on the documented network fallback (`#13288 <https://github.com/autowarefoundation/autoware_universe/issues/13288>`_)
  getIOTensorName, getNbIOTensors and getTensorShape(index) fall back to the
  parsed network when the engine is not built yet. That is the documented
  contract - the header already says "with fallback from TensorRT network" - and
  it is how a caller inspects the IO a model declares before setup() builds or
  loads the engine.
  Logging kWARNING on every such call turns intended use into noise: enumerating
  a 27-input encoder emits 27 warnings before the engine exists, which buries the
  warnings that matter. The log is dropped; a missing network stays an error.
  The three fallbacks are also flattened to early returns - network missing is an
  error, an engine short-circuits, otherwise read the network - instead of
  nesting the network path inside `if (!engine\_)`. network\_ is created by
  initialize() from the constructor and never reset, so testing it first is
  safe.
  Every member that reads only state established by the constructor now says so:
  "Valid immediately after construction. Valid to call before `setup()`."
  Co-authored-by: Claude Opus 5 (1M context) <noreply@anthropic.com>
* chore(autoware_tensorrt_common): add mojomex and vividf as maintainers (`#13205 <https://github.com/autowarefoundation/autoware_universe/issues/13205>`_)
  chore: add mojomex and vividf as maintainers
* refactor(autoware_tensorrt_common): rename precision "as-is" to "strongly-typed" (`#13191 <https://github.com/autowarefoundation/autoware_universe/issues/13191>`_)
  The value selects a strongly typed build, where tensor precisions come
  from the model. Name it after what it does: "as-is" describes the
  artifact's treatment rather than the build mode, and reads as a
  precision on par with fp32/fp16/int8, which it is not.
  No functional change. The value has no users yet: it was added in
  `#13160 <https://github.com/autowarefoundation/autoware_universe/issues/13160>`_ and its first consumer (autoware_ptv3, `#13159 <https://github.com/autowarefoundation/autoware_universe/issues/13159>`_) is not merged, so
  renaming now costs nothing.
* feat(autoware_tensorrt_common): support strongly typed network builds (`#13160 <https://github.com/autowarefoundation/autoware_universe/issues/13160>`_)
  * feat(autoware_tensorrt_common): support strongly typed network builds
  Accept "as-is" as a TrtCommonConfig precision value. With it, the
  network is created with the kSTRONGLY_TYPED flag: every tensor
  precision is taken from the model itself instead of being requested
  via builder precision flags, which do not exist for this mode. The
  existing precision values are unchanged, so no current caller changes
  behavior.
  In as-is mode, precision constraints and NetworkIO dtype overrides are
  skipped (TensorRT rejects them for strongly typed networks); dtypes
  requested via NetworkIO are still validated against the built engine.
  An isStronglyTyped() accessor exposes the mode to consumers.
  Strong typing is independent of the model's precision: any export
  (fp32 or reduced precision) can be built as-is, trading the builder's
  freedom to mix precisions for reproducible, model-defined numerics.
  Reduced-precision (e.g. fp16) exports should be built this way, since
  the weakly typed precision-assignment machinery would re-process the
  already-reduced tensors (and is broken on some target platforms).
  Verified with weakly typed fp32 models (behavior unchanged), with fp32
  models built as-is (pure fp32 engines), and with explicit-fp16 PTv3
  models built as-is on x86 and DRIVE Thor. Adoption by autoware_ptv3 is
  a separate stacked change.
  * build(autoware_tensorrt_common): require TensorRT 10
  kSTRONGLY_TYPED does not exist before TensorRT 10, so the declared
  minimum of 8.5 no longer matches what the sources compile against.
  Raise the requirement instead of guarding the flag by version: 8.5 was
  already only nominal (autoware_tensorrt_plugins, and with it every
  consumer that loads plugins, requires 10.0), and no supported
  Autoware target ships a pre-10 TensorRT.
  With the minimum at 10, the kEXPLICIT_BATCH branch in initialize() is
  unreachable — networks are always explicit-batch in TensorRT 10 — so
  drop it. createNetworkV2() is called with the same flags as before.
  * refactor(autoware_tensorrt_common): drop dead pre-TensorRT-10 guard
  validateEngine()'s body was gated on TensorRT >= 8.6, which the package's
  new 10.0 minimum makes unconditionally true. Without the guard the plan
  version check is compiled the same way it already was, and returning
  true unconditionally on pre-8.6 (i.e. accepting any cached engine
  unvalidated) is no longer a reachable behavior.
  This was the last NV_TENSORRT version guard in the package.
  ---------
* Contributors: Max Schmeller, Ryohsuke Mitsudome

0.52.0 (2026-06-30)
-------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* feat(autoware_tensorrt_common): enable tensor io type setting (`#12716 <https://github.com/autowarefoundation/autoware_universe/issues/12716>`_)
  * feat(autoware_tensorrt_common): enable tensort io type setting
  * fix(autoware_tensorrt_common): replace logic
  * fix(autoware_tensorrt_common): remove wrong docstring
  ---------
* fix(autoware_tensorrt_common): fix typo & adjust macro offset (`#12567 <https://github.com/autowarefoundation/autoware_universe/issues/12567>`_)
* Contributors: Amadeusz Szymko, github-actions

0.51.0 (2026-05-01)
-------------------

0.50.0 (2026-02-14)
-------------------
* Merge remote-tracking branch 'origin/main' into humble
* chore(autoware_tensorrt_common): remove cudnn dependency (`#11896 <https://github.com/autowarefoundation/autoware_universe/issues/11896>`_)
* Contributors: Amadeusz Szymko, Ryohsuke Mitsudome

0.49.0 (2025-12-30)
-------------------

0.48.0 (2025-11-18)
-------------------
* Merge remote-tracking branch 'origin/main' into humble
* fix(tensorrt_common): resolve error message of clang (`#11434 <https://github.com/autowarefoundation/autoware_universe/issues/11434>`_)
  fix: resolve error message by clang
* feat(autoware_camera_streampetr): implementation of StreamPETR using tensorrt (`#11139 <https://github.com/autowarefoundation/autoware_universe/issues/11139>`_)
  * added streampetr
  * use trt_common for build and forward pass
  * style(pre-commit): autofix
  * use optional parameters
  * remove unused methods
  * style(pre-commit): autofix
  * fix lint errors
  * ament
  * style(pre-commit): autofix
  * refactor complex code
  * simplified functions
  * style(pre-commit): autofix
  * removed uncrustify
  * fix clang errors
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: pre-commit-ci-lite[bot] <117423508+pre-commit-ci-lite[bot]@users.noreply.github.com>
* Contributors: Kotaro Uetake, Ryohsuke Mitsudome, Samrat Thapa

0.47.1 (2025-08-14)
-------------------

0.47.0 (2025-08-11)
-------------------
* chore(autoware_tensorrt_common): improved logging when loading plugins (`#10605 <https://github.com/autowarefoundation/autoware_universe/issues/10605>`_)
  chore: added a print with the cause of the error in case loading the plugins fails
* Contributors: Kenzo Lobos Tsunekawa

0.46.0 (2025-06-20)
-------------------

0.45.0 (2025-05-22)
-------------------
* Merge remote-tracking branch 'origin/main' into tmp/notbot/bump_version_base
* chore: perception code owner update (`#10645 <https://github.com/autowarefoundation/autoware_universe/issues/10645>`_)
  * chore: update maintainers in multiple perception packages
  * Revert "chore: update maintainers in multiple perception packages"
  This reverts commit f2838c33d6cd82bd032039e2a12b9cb8ba6eb584.
  * chore: update maintainers in multiple perception packages
  * chore: add Kok Seang Tan as maintainer in multiple perception packages
  ---------
* perf(autoware_tensorrt_common): set cudaSetDeviceFlags explicitly (`#10523 <https://github.com/autowarefoundation/autoware_universe/issues/10523>`_)
  * Synchronize CUDA stream by blocking instead of spin
  * Use blocking-sync in BEVFusion
  * Call cudaSetDeviceFlags in tensorrt_common
* Contributors: Taekjin LEE, TaikiYamada4, prime number

0.44.2 (2025-06-10)
-------------------

0.44.1 (2025-05-01)
-------------------

0.44.0 (2025-04-18)
-------------------
* Merge remote-tracking branch 'origin/main' into humble
* feat: should be using NvInferRuntime.h (`#10399 <https://github.com/autowarefoundation/autoware_universe/issues/10399>`_)
* feat(autoware_tenssort_common): validate TensorRT engine version for cached engine (`#10320 <https://github.com/autowarefoundation/autoware_universe/issues/10320>`_)
  * autoware_tenssort_common): validate TensorRT engine version for cached engine
  * style(autoware_tensorrt_common): typo
  Co-authored-by: Kenzo Lobos Tsunekawa <kenzo.lobos@tier4.jp>
  * style(autoware_tensorrt_common): typo
  Co-authored-by: Kenzo Lobos Tsunekawa <kenzo.lobos@tier4.jp>
  * style(autoware_tensorrt_common): typo
  Co-authored-by: Kenzo Lobos Tsunekawa <kenzo.lobos@tier4.jp>
  * docs(autoware_tensorrt_common): add source
  ---------
  Co-authored-by: Kenzo Lobos Tsunekawa <kenzo.lobos@tier4.jp>
* Contributors: Amadeusz Szymko, Ryohsuke Mitsudome, Yuxuan Liu

0.43.0 (2025-03-21)
-------------------
* Merge remote-tracking branch 'origin/main' into chore/bump-version-0.43
* chore: rename from `autoware.universe` to `autoware_universe` (`#10306 <https://github.com/autowarefoundation/autoware_universe/issues/10306>`_)
* refactor: add autoware_cuda_dependency_meta (`#10073 <https://github.com/autowarefoundation/autoware_universe/issues/10073>`_)
* Contributors: Esteve Fernandez, Hayato Mizushima, Yutaka Kondo

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
* refactor(autoware_tensorrt_common): multi-TensorRT compatibility & tensorrt_common as unified lib for all perception components (`#9762 <https://github.com/autowarefoundation/autoware_universe/issues/9762>`_)
  * refactor(autoware_tensorrt_common): multi-TensorRT compatibility & tensorrt_common as unified lib for all perception components
  * style(pre-commit): autofix
  * style(autoware_tensorrt_common): linting
  * style(autoware_lidar_centerpoint): typo
  Co-authored-by: Kenzo Lobos Tsunekawa <kenzo.lobos@tier4.jp>
  * docs(autoware_tensorrt_common): grammar
  Co-authored-by: Kenzo Lobos Tsunekawa <kenzo.lobos@tier4.jp>
  * fix(autoware_lidar_transfusion): reuse cast variable
  * fix(autoware_tensorrt_common): remove deprecated inference API
  * style(autoware_tensorrt_common): grammar
  Co-authored-by: Kenzo Lobos Tsunekawa <kenzo.lobos@tier4.jp>
  * style(autoware_tensorrt_common): grammar
  Co-authored-by: Kenzo Lobos Tsunekawa <kenzo.lobos@tier4.jp>
  * fix(autoware_tensorrt_common): const pointer
  * fix(autoware_tensorrt_common): remove unused method declaration
  * style(pre-commit): autofix
  * refactor(autoware_tensorrt_common): readability
  Co-authored-by: Kotaro Uetake <60615504+ktro2828@users.noreply.github.com>
  * fix(autoware_tensorrt_common): return if layer not registered
  * refactor(autoware_tensorrt_common): readability
  Co-authored-by: Kotaro Uetake <60615504+ktro2828@users.noreply.github.com>
  * fix(autoware_tensorrt_common): rename struct
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: Kenzo Lobos Tsunekawa <kenzo.lobos@tier4.jp>
  Co-authored-by: Kotaro Uetake <60615504+ktro2828@users.noreply.github.com>
* fix(autoware_tensorrt_common): fix bugprone-integer-division (`#9660 <https://github.com/autowarefoundation/autoware_universe/issues/9660>`_)
  fix: bugprone-error
* Contributors: Amadeusz Szymko, Fumiya Watanabe, kobayu858

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
* fix(cpplint): include what you use - perception (`#9569 <https://github.com/autowarefoundation/autoware_universe/issues/9569>`_)
* fix(autoware_tensorrt_common): fix clang-diagnostic-unused-private-field (`#9493 <https://github.com/autowarefoundation/autoware_universe/issues/9493>`_)
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
* Contributors: Esteve Fernandez, Fumiya Watanabe, M. Fatih Cırıt, Ryohsuke Mitsudome, Yutaka Kondo, kobayu858

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
* refactor(tensorrt_common)!: fix namespace, directory structure & move to perception namespace (`#9099 <https://github.com/autowarefoundation/autoware_universe/issues/9099>`_)
  * refactor(tensorrt_common)!: fix namespace, directory structure & move to perception namespace
  * refactor(tensorrt_common): directory structure
  * style(pre-commit): autofix
  * fix(tensorrt_common): correct package name for logging
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: Kenzo Lobos Tsunekawa <kenzo.lobos@tier4.jp>
* Contributors: Amadeusz Szymko, Yutaka Kondo

0.26.0 (2024-04-03)
-------------------
