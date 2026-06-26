^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_tensorrt_plugins
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.52.0 (2026-06-26)
-------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* perf(autoware_tensorrt_plugins): keep SegmentCSR allocation-free (`#12555 <https://github.com/autowarefoundation/autoware_universe/issues/12555>`_)
  Initialize the SegmentCSR output buffer directly instead of allocating, filling, copying, and freeing a scratch base buffer on every launch.
  Co-authored-by: Copilot <223556219+Copilot@users.noreply.github.com>
* feat(autoware_tensorrt_plugins): support fused bias/activation in ImplicitGemmPlugin (`#12658 <https://github.com/autowarefoundation/autoware_universe/issues/12658>`_)
  * feat(autoware_tensorrt_plugins): add fused activation and optional bias support to ImplicitGemmPlugin
  - Add act_type field to ImplicitGemmParameters for fused activation
  (kNone/kReLU/kSigmoid/kLeakyReLU)
  - Support optional 6th input for per-channel fused bias (BN-folded);
  the plugin now accepts 5 (no bias) or 6 (with bias) inputs
  - Pass act_alpha, act_beta, act_type, and bias tensor to
  ConvGemmOps::implicit_gemm instead of hardcoded kNone/empty
  - Track num_plugin_inputs\_ across clone/configurePlugin/onShapeChange
  - Extend parse_fields lambda to accept act_type as optional 7th ONNX
  attribute (backward-compatible with 6-field ONNX graphs)
  * chore: fix naming
  * chore: fix implicit gemm
  * chore: clean code
  * chore: fix naming
  ---------
* refactor(autoware_tensorrt_plugins): remove SegmentCSR nD indptr path (`#12739 <https://github.com/autowarefoundation/autoware_universe/issues/12739>`_)
  Remove the unused n-dimensional indptr launcher path and offset helper now that the plugin contract is explicitly 1D.
  Co-authored-by: Copilot <223556219+Copilot@users.noreply.github.com>
* fix(autoware_tensorrt_plugins): fix implicitgemm build and runtime error (`#12734 <https://github.com/autowarefoundation/autoware_universe/issues/12734>`_)
  * fix(autoware_tensorrt_plugins): fix kBUILD/kRUNTIME deserialization and type issues
  * refactor(autoware_tensorrt_plugins): extract field parsing into lambda in createPlugin
  * chore: fix cspell
  ---------
* refactor(autoware_tensorrt_plugins): clarify SegmentCSR contract (`#12741 <https://github.com/autowarefoundation/autoware_universe/issues/12741>`_)
  * refactor(autoware_tensorrt_plugins): clarify SegmentCSR contract
  Document the plugin's existing 2D source and 1D indptr contract, and replace the SegmentCSR launcher's output tuple with named output pointers.
  Co-authored-by: Copilot <223556219+Copilot@users.noreply.github.com>
  * fix: fix memory leak in kernel error path
  Co-authored-by: Copilot Autofix powered by AI <175728472+Copilot@users.noreply.github.com>
  ---------
  Co-authored-by: Copilot <223556219+Copilot@users.noreply.github.com>
  Co-authored-by: Copilot Autofix powered by AI <175728472+Copilot@users.noreply.github.com>
* chore(autoware_tensorrt_plugins): add manato, mojomex as maintainers (`#12755 <https://github.com/autowarefoundation/autoware_universe/issues/12755>`_)
* test(autoware_tensorrt_plugins): add SegmentCSR kernel tests (`#12740 <https://github.com/autowarefoundation/autoware_universe/issues/12740>`_)
  * test(autoware_tensorrt_plugins): add SegmentCSR kernel tests
  Add parameterized reference-kernel coverage for SegmentCSR mean and max reductions, including empty segments, no-output cases, empty source input, and zero-column output. Guard the existing launcher against zero-output work so these edge cases complete without zero-size kernel launches or reduction-axis division by zero.
  Co-authored-by: Copilot <223556219+Copilot@users.noreply.github.com>
  * style(pre-commit): autofix
  ---------
  Co-authored-by: Copilot <223556219+Copilot@users.noreply.github.com>
  Co-authored-by: pre-commit-ci-lite[bot] <117423508+pre-commit-ci-lite[bot]@users.noreply.github.com>
* perf(autoware_tensorrt_plugins): remove Thrust from sort kernels (`#12554 <https://github.com/autowarefoundation/autoware_universe/issues/12554>`_)
  * perf(autoware_tensorrt_plugins): remove Thrust from sort kernels
  * docs(autoware_tensorrt_plugins): clarify unique workspace flow
  * Clarify unique offset sentinel workspace
  * Document unique workspace layout fields
  * perf(autoware_tensorrt_plugins): avoid plugin stream syncs
  * style(pre-commit): autofix
  * refactor(autoware_tensorrt_plugins): share kernel helpers
  * docs(autoware_tensorrt_plugins): explain inverse index offset
  * style(autoware_tensorrt_plugins): clarify unique count branch
  * style(autoware_tensorrt_plugins): mark workspace size maybe unused
  * style(autoware_tensorrt_plugins): use byte workspace arithmetic
  * style(pre-commit): autofix
  * perf(autoware_tensorrt_plugins): use run length encode for unique counts
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci-lite[bot] <117423508+pre-commit-ci-lite[bot]@users.noreply.github.com>
* feat(autoware_tensorrt_plugins): add do sort attribute in GetIndicePairsImplicitGemmPlugin for faster inference (`#12631 <https://github.com/autowarefoundation/autoware_universe/issues/12631>`_)
  * feat: add do sort attribute for faster inference
  * style(pre-commit): autofix
  * chore: clean up code and add better warning msg
  ---------
  Co-authored-by: pre-commit-ci-lite[bot] <117423508+pre-commit-ci-lite[bot]@users.noreply.github.com>
* test(autoware_tensorrt_plugins): add reference kernel tests (`#12561 <https://github.com/autowarefoundation/autoware_universe/issues/12561>`_)
  * test(autoware_tensorrt_plugins): add reference kernel tests
  * style(pre-commit): autofix
  * refactor(autoware_tensorrt_plugins): split tests into logical units
  Co-authored-by: Copilot <copilot@github.com>
  * chore: add include guard
  Co-authored-by: Copilot <copilot@github.com>
  * style(pre-commit): autofix
  * chore: remove unused includes
  * test: hopefully made tests GPU-less CI friendly
  Co-authored-by: Copilot <copilot@github.com>
  ---------
  Co-authored-by: pre-commit-ci-lite[bot] <117423508+pre-commit-ci-lite[bot]@users.noreply.github.com>
  Co-authored-by: Copilot <copilot@github.com>
* fix(autoware_tensorrt_plugins): correct CustomUnique count offset (`#12502 <https://github.com/autowarefoundation/autoware_universe/issues/12502>`_)
  * fix(autoware_tensorrt_plugins): correct CustomUnique count offset
  Co-authored-by: Copilot <223556219+Copilot@users.noreply.github.com>
  * style(pre-commit): autofix
  ---------
  Co-authored-by: Copilot <223556219+Copilot@users.noreply.github.com>
  Co-authored-by: pre-commit-ci-lite[bot] <117423508+pre-commit-ci-lite[bot]@users.noreply.github.com>
* Contributors: Max Schmeller, Yi-Hsiang Fang (Vivid), github-actions

0.51.0 (2026-05-01)
-------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* fix(autoware_tensorrt_plugins): avoid tv::zeros and tv::empty (`#12378 <https://github.com/mitsudome-r/autoware_universe/issues/12378>`_)
  Co-authored-by: Amadeusz Szymko <amadeusz.szymko.2@tier4.jp>
* perf(perception): use emplace_back and emplace to avoid temporary object creation (`#12201 <https://github.com/mitsudome-r/autoware_universe/issues/12201>`_)
  * perf(perception): use emplace_back to avoid temporary object creation
  * style(pre-commit): autofix
  * perf(perception): use emplace/emplace_back for most containers
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci-lite[bot] <117423508+pre-commit-ci-lite[bot]@users.noreply.github.com>
  Co-authored-by: Taekjin LEE <taekjin.lee@tier4.jp>
* feat(autoware_tensorrt_plugins): restore Turing arch compatibility (`#12211 <https://github.com/mitsudome-r/autoware_universe/issues/12211>`_)
* feat(autoware_tensorrt_plugins): cuda 12.0 build compatibility (`#12191 <https://github.com/mitsudome-r/autoware_universe/issues/12191>`_)
  feat(autoware_tensorrt_plugins): CUDA 12.0+ build compatibility
* Contributors: Amadeusz Szymko, Ryuta Kambe, github-actions, nishikawa-masaki

0.50.0 (2026-02-14)
-------------------
* Merge remote-tracking branch 'origin/main' into humble
* feat(autoware_tensorrt_plugins): update nvcc flags (`#12056 <https://github.com/autowarefoundation/autoware_universe/issues/12056>`_)
  Co-authored-by: Kotaro Uetake <60615504+ktro2828@users.noreply.github.com>
* chore(autoware_tensorrt_plugins): adjust flags for build system (`#11956 <https://github.com/autowarefoundation/autoware_universe/issues/11956>`_)
* chore(autoware_tensorrt_plugins): remove cudnn dependency (`#11897 <https://github.com/autowarefoundation/autoware_universe/issues/11897>`_)
* Contributors: Amadeusz Szymko, Ryohsuke Mitsudome

0.49.0 (2025-12-30)
-------------------

0.48.0 (2025-11-18)
-------------------
* Merge remote-tracking branch 'origin/main' into humble
* fix(autoware_tensorrt_plugins): install cuda_ops to be available when autoware_tensorrt_plugins installs (`#11354 <https://github.com/autowarefoundation/autoware_universe/issues/11354>`_)
  * fix(cmake): install cuda_ops to be available when autoware_tensorrt_plugins installs
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci-lite[bot] <117423508+pre-commit-ci-lite[bot]@users.noreply.github.com>
  Co-authored-by: Kotaro Uetake <60615504+ktro2828@users.noreply.github.com>
* Contributors: Ryohsuke Mitsudome, oabdelgawad

0.47.1 (2025-08-14)
-------------------

0.47.0 (2025-08-11)
-------------------
* feat(autoware_tensorrt_plugins): add vad trt plugins suppport (`#11092 <https://github.com/autowarefoundation/autoware_universe/issues/11092>`_)
  * feat(tensorrt_plugins): add multi_scale_deformable_attention, rotate, and
  select_and_pad plugins
  Add three new TensorRT plugins to support advanced vision model
  operations:
  - MultiScaleDeformableAttentionPlugin: Implements multi-scale deformable
  attention mechanism for vision transformers with CUDA kernels for
  efficient GPU execution
  - RotatePlugin: Provides image rotation functionality with support for
  both bilinear and nearest neighbor interpolation modes
  - SelectAndPadPlugin: Enables conditional selection and padding of
  tensor
  elements based on input flags, useful for dynamic batching scenarios
  Key changes:
  - Migrate plugins from IPluginV2DynamicExt to IPluginV3 interface
  - Add CUDA kernel implementations in separate ops subdirectories
  - Update plugin registration to include new creators (count: 8 -> 11)
  - Fix build issues by using SHARED libraries for CUDA ops
  - Add proper namespace organization (autoware::tensorrt_plugins)
  The plugins are designed to integrate seamlessly with the existing
  Autoware TensorRT framework and support both FP32 and FP16 precision.
  * refactor(tensorrt_plugins): reorganize ops directories and fix naming conventions
  - Move *_ops directories from include/autoware/tensorrt_plugins to include/autoware
  - Rename rotateKernel.{h,cu} to rotate_kernel.{h,cu} following snake_case convention
  - Update all include paths to reflect new directory structure
  - Add missing copyright header to ms_deform_attn_kernel.hpp
  - Update CMakeLists.txt to reference renamed source files
  - Update header guards to match new directory structure
  ---------
* build: fix missing tensorrt_cmake_module dependency (`#10984 <https://github.com/autowarefoundation/autoware_universe/issues/10984>`_)
* Contributors: Bingo, Esteve Fernandez

0.46.0 (2025-06-20)
-------------------
* Merge remote-tracking branch 'upstream/main' into tmp/TaikiYamada/bump_version_base
* fix(cmake): update spconv availability messages to use STATUS and WAR… (`#10690 <https://github.com/autowarefoundation/autoware_universe/issues/10690>`_)
  fix(cmake): update spconv availability messages to use STATUS and WARNING
* Contributors: TaikiYamada4, Yukihiro Saito

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
* chore(autoware_tensorrt_plugins): update maintainer (`#10627 <https://github.com/autowarefoundation/autoware_universe/issues/10627>`_)
  * chore(autoware_tensorrt_plugins): update maintainer
  * chore(autoware_tensorrt_plugins): update maintainer
  ---------
* Contributors: Amadeusz Szymko, Taekjin LEE, TaikiYamada4

0.44.2 (2025-06-10)
-------------------

0.44.1 (2025-05-01)
-------------------

0.44.0 (2025-04-18)
-------------------
* chore: match all package versions
* Merge remote-tracking branch 'origin/main' into humble
* feat(autoware_tensorrt_plugins): created a package for tensorrt extensions (`#10445 <https://github.com/autowarefoundation/autoware_universe/issues/10445>`_)
  * feat: moved the plugins in bevfusion to a separate package since some of them will be reused
  * doc: doc regarding the plugins and the supported ops
  * chore: wrong upper cases
  * chore: wrong quotes
  * chore: fixed docs
  ---------
* Contributors: Kenzo Lobos Tsunekawa, Ryohsuke Mitsudome
