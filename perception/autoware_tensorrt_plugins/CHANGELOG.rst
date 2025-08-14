^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_tensorrt_plugins
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
