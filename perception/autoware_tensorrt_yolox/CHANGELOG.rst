^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_tensorrt_yolox
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
* feat(autoware_tensorrt_yolox): added target architectures for yolox (`#10611 <https://github.com/autowarefoundation/autoware_universe/issues/10611>`_)
  * chore: added target architectures for yolox
  * chore: mistook the compute capabilities of edge devices
  * chore: cspell
  ---------
* Contributors: Kenzo Lobos Tsunekawa, Taekjin LEE, TaikiYamada4

0.44.2 (2025-06-10)
-------------------

0.44.1 (2025-05-01)
-------------------

0.44.0 (2025-04-18)
-------------------
* Merge remote-tracking branch 'origin/main' into humble
* feat(tensorrt_yolox): add autoware_utils packages (`#10460 <https://github.com/autowarefoundation/autoware_universe/issues/10460>`_)
  Co-authored-by: t4-adc <grp-rd-1-adc-admin@tier4.jp>
* fix(autoware_tensorrt_yolox): explicitly install shared library (`#10454 <https://github.com/autowarefoundation/autoware_universe/issues/10454>`_)
* Contributors: Kazunori-Nakajima, Manato Hirabayashi, Ryohsuke Mitsudome

0.43.0 (2025-03-21)
-------------------
* Merge remote-tracking branch 'origin/main' into chore/bump-version-0.43
* chore: rename from `autoware.universe` to `autoware_universe` (`#10306 <https://github.com/autowarefoundation/autoware_universe/issues/10306>`_)
* chore(perception): refactor perception launch (`#10186 <https://github.com/autowarefoundation/autoware_universe/issues/10186>`_)
  * fundamental change
  * style(pre-commit): autofix
  * fix typo
  * fix params and modify some packages
  * pre-commit
  * fix
  * fix spell check
  * fix typo
  * integrate model and label path
  * style(pre-commit): autofix
  * for pre-commit
  * run pre-commit
  * for awsim
  * for simulatior
  * style(pre-commit): autofix
  * fix grammer in launcher
  * add schema for yolox_tlr
  * style(pre-commit): autofix
  * fix file name
  * fix
  * rename
  * modify arg name  to
  * fix typo
  * change param name
  * style(pre-commit): autofix
  * chore
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: Shintaro Tomie <58775300+Shin-kyoto@users.noreply.github.com>
  Co-authored-by: Kenzo Lobos Tsunekawa <kenzo.lobos@tier4.jp>
* refactor: add autoware_cuda_dependency_meta (`#10073 <https://github.com/autowarefoundation/autoware_universe/issues/10073>`_)
* Contributors: Esteve Fernandez, Hayato Mizushima, Masato Saeki, Yutaka Kondo

0.42.0 (2025-03-03)
-------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* feat(autoware_utils): replace autoware_universe_utils with autoware_utils  (`#10191 <https://github.com/autowarefoundation/autoware_universe/issues/10191>`_)
* Contributors: Fumiya Watanabe, 心刚

0.41.2 (2025-02-19)
-------------------
* chore: bump version to 0.41.1 (`#10088 <https://github.com/autowarefoundation/autoware_universe/issues/10088>`_)
* Contributors: Ryohsuke Mitsudome

0.41.1 (2025-02-10)
-------------------

0.41.0 (2025-01-29)
-------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* feat(autoware_tensorrt_yolox)!: tier4_debug_msgs changed to autoware_internal_debug_msgs in autoware_tensorrt_yolox (`#9898 <https://github.com/autowarefoundation/autoware_universe/issues/9898>`_)
* feat(tensorrt_yolox): add launch for tlr model (`#9828 <https://github.com/autowarefoundation/autoware_universe/issues/9828>`_)
  * feat(tensorrt_yolox): add launch for tlr model
  * chore: typo
  * docs: update readme and description
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* fix(autoware_tensorrt_yolox): modify tensorrt_yolox_node name (`#9156 <https://github.com/autowarefoundation/autoware_universe/issues/9156>`_)
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
* fix(autoware_tensorrt_yolox): fix bugprone-exception-escape (`#9734 <https://github.com/autowarefoundation/autoware_universe/issues/9734>`_)
  * fix: bugprone-error
  * fix: fmt
  * fix: fmt
  ---------
* Contributors: Amadeusz Szymko, Fumiya Watanabe, Vishal Chauhan, badai nguyen, cyn-liu, kobayu858

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
* fix(autoware_tensorrt_yolox): fix clang-diagnostic-inconsistent-missing-override (`#9512 <https://github.com/autowarefoundation/autoware_universe/issues/9512>`_)
  fix: clang-diagnostic-inconsistent-missing-override
* 0.39.0
* update changelog
* Merge commit '6a1ddbd08bd' into release-0.39.0
* fix: fix ticket links to point to https://github.com/autowarefoundation/autoware_universe (`#9304 <https://github.com/autowarefoundation/autoware_universe/issues/9304>`_)
* fix: fix ticket links to point to https://github.com/autowarefoundation/autoware_universe (`#9304 <https://github.com/autowarefoundation/autoware_universe/issues/9304>`_)
* chore(package.xml): bump version to 0.38.0 (`#9266 <https://github.com/autowarefoundation/autoware_universe/issues/9266>`_) (`#9284 <https://github.com/autowarefoundation/autoware_universe/issues/9284>`_)
  * unify package.xml version to 0.37.0
  * remove system_monitor/CHANGELOG.rst
  * add changelog
  * 0.38.0
  ---------
* refactor(cuda_utils): prefix package and namespace with autoware (`#9171 <https://github.com/autowarefoundation/autoware_universe/issues/9171>`_)
* Contributors: Esteve Fernandez, Fumiya Watanabe, M. Fatih Cırıt, Ryohsuke Mitsudome, Yutaka Kondo, kobayu858

0.39.0 (2024-11-25)
-------------------
* Merge commit '6a1ddbd08bd' into release-0.39.0
* fix: fix ticket links to point to https://github.com/autowarefoundation/autoware_universe (`#9304 <https://github.com/autowarefoundation/autoware_universe/issues/9304>`_)
* fix: fix ticket links to point to https://github.com/autowarefoundation/autoware_universe (`#9304 <https://github.com/autowarefoundation/autoware_universe/issues/9304>`_)
* chore(package.xml): bump version to 0.38.0 (`#9266 <https://github.com/autowarefoundation/autoware_universe/issues/9266>`_) (`#9284 <https://github.com/autowarefoundation/autoware_universe/issues/9284>`_)
  * unify package.xml version to 0.37.0
  * remove system_monitor/CHANGELOG.rst
  * add changelog
  * 0.38.0
  ---------
* refactor(cuda_utils): prefix package and namespace with autoware (`#9171 <https://github.com/autowarefoundation/autoware_universe/issues/9171>`_)
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
* refactor(object_recognition_utils): add autoware prefix to object_recognition_utils (`#8946 <https://github.com/autowarefoundation/autoware_universe/issues/8946>`_)
* feat(autoware_tensorrt_yolox): add GPU - CUDA device option (`#8245 <https://github.com/autowarefoundation/autoware_universe/issues/8245>`_)
  * init CUDA device option
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* chore(autoware_tensorrt_yolox): add Kotaro Uetake as maintainer (`#8595 <https://github.com/autowarefoundation/autoware_universe/issues/8595>`_)
  chore: add Kotaro Uetake as maintainer
* fix: cpp17 namespaces (`#8526 <https://github.com/autowarefoundation/autoware_universe/issues/8526>`_)
  Use traditional-style nameplace nesting for nvcc
  Co-authored-by: Yuri Guimaraes <yuri.kgpps@gmail.com>
* fix(docs): fix docs for tensorrt yolox (`#8304 <https://github.com/autowarefoundation/autoware_universe/issues/8304>`_)
  fix docs for tensorrt yolox
* refactor(tensorrt_yolox): move utils into perception_utils (`#8435 <https://github.com/autowarefoundation/autoware_universe/issues/8435>`_)
  * chore(tensorrt_yolo): refactor utils
  * style(pre-commit): autofix
  * fix: tensorrt_yolox
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* fix(autoware_tensorrt_yolox): fix variableScope (`#8430 <https://github.com/autowarefoundation/autoware_universe/issues/8430>`_)
  fix: variableScope
  Co-authored-by: kobayu858 <129580202+kobayu858@users.noreply.github.com>
* fix(tensorrt_yolox): add run length encoding for sematic segmentation mask (`#7905 <https://github.com/autowarefoundation/autoware_universe/issues/7905>`_)
  * fix: add rle compress
  * fix: rle compress
  * fix: move rle into utils
  * chore: pre-commit
  * Update perception/autoware_tensorrt_yolox/src/utils.cpp
  Co-authored-by: Yukihiro Saito <yukky.saito@gmail.com>
  * fix: remove unused variable
  * Update perception/autoware_tensorrt_yolox/src/utils.cpp
  Co-authored-by: Manato Hirabayashi <3022416+manato@users.noreply.github.com>
  * style(pre-commit): autofix
  * feat: add unit test for utils
  * style(pre-commit): autofix
  * fix: unit test
  * chore: change to explicit index
  * style(pre-commit): autofix
  * fix: cuda cmake
  * fix: separate unit test into different PR
  ---------
  Co-authored-by: Yukihiro Saito <yukky.saito@gmail.com>
  Co-authored-by: Manato Hirabayashi <3022416+manato@users.noreply.github.com>
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* fix(autoware_tensorrt_yolox): fix unreadVariable (`#8356 <https://github.com/autowarefoundation/autoware_universe/issues/8356>`_)
  * fix:unreadVariable
  * fix:unreadVariable
  ---------
* refactor: image transport decompressor/autoware prefix (`#8197 <https://github.com/autowarefoundation/autoware_universe/issues/8197>`_)
  * refactor: add `autoware` namespace prefix to image_transport_decompressor
  * refactor(image_transport_decompressor): add `autoware` prefix to the package code
  * refactor: update package name in CODEOWNER
  * fix: merge main into the branch
  * refactor: update packages which depend on image_transport_decompressor
  * refactor(image_transport_decompressor): update README
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: Taekjin LEE <taekjin.lee@tier4.jp>
* refactor(tensorrt_yolox)!: fix namespace and directory structure (`#7992 <https://github.com/autowarefoundation/autoware_universe/issues/7992>`_)
  * refactor: add autoware namespace prefix to `tensorrt_yolox`
  * refactor: apply `autoware` namespace to tensorrt_yolox
  * chore: update CODEOWNERS
  * fix: resolve `yolox_tiny` to work
  ---------
* Contributors: Abraham Monrroy Cano, Amadeusz Szymko, Esteve Fernandez, Ismet Atabay, Kotaro Uetake, Manato Hirabayashi, Nagi70, Yutaka Kondo, Yuxuan Liu, badai nguyen, kobayu858

0.26.0 (2024-04-03)
-------------------
