^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_ground_segmentation_cuda
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.50.0 (2026-02-14)
-------------------
* chore: match all package versions
* Merge remote-tracking branch 'origin/main' into humble
* feat(autoware_ground_segmentation_cuda): update nvcc flags (`#12047 <https://github.com/autowarefoundation/autoware_universe/issues/12047>`_)
* fix(agnocast): build on jazzy, remove from ground_segmentation_cuda (`#11960 <https://github.com/autowarefoundation/autoware_universe/issues/11960>`_)
* fix(cuda_scan_ground_segmentation): add missing include (`#11850 <https://github.com/autowarefoundation/autoware_universe/issues/11850>`_)
* feat: gpu ground segmentation (`#11371 <https://github.com/autowarefoundation/autoware_universe/issues/11371>`_)
  * init cuda ground segmentation node
  * feat: add cuda_scan_ground_segmentation node
  * fix: launch
  * refactor
  * refactor
  * fix: atan error
  * fix: num output point host reference
  * pre-commit
  * feat: add ground pointcloud publish for debug
  * refactor
  * refactor
  * refactor
  * fix: param path
  * feat: add debug processing time
  * delete pointcloud preprocesor ground segmentation
  * typo
  * refactoring
  * refactoring
  * fix: variable type
  * refactoring
  * typo
  * style(pre-commit): autofix
  * revert launch change
  * docs
  * style(pre-commit): autofix
  * revise variable type
  * fix: avoid very small d_radius
  * docs: add README
  * style(pre-commit): autofix
  * docs
  * refactor
  * style(pre-commit): autofix
  * docs: add schema
  * fix(recheck): classifiied local pointcloud index bug
  * fix: launch
  * remove vehicle info depend
  * Adding GPU ground segmentation
  * Finished coding
  * Fixed
  * Fixed
  * Fixed all. Debug code remains.
  * Remove debug code.
  * style(pre-commit): autofix
  * Remove wrong code
  * Fixed pre-commit-lite
  * style(pre-commit): autofix
  * Fixed pre-commit-lite
  * Fix typos
  * Ignore CUDAH spell check
  * Add license info
  * style(pre-commit): autofix
  * Replace sensor_msgs PointCloud2 by cuda_blackboard PointCloud2
  * style(pre-commit): autofix
  * Use cuda_blackboard pointcloud 2
  * style(pre-commit): autofix
  * add launch option
  * style(pre-commit): autofix
  * replace radius_max by xzy max min
  * style(pre-commit): autofix
  * Remove point number limit of each thread
  * style(pre-commit): autofix
  * Fixed build error
  * fix: input topics
  * fix schema check
  * style(pre-commit): autofix
  * docs: update readme
  * refactor: remove unused param
  * typo
  * fix: center shift
  * Fixed errors
  * style(pre-commit): autofix
  * Replace CUDAH by CUDA_HOSTDEV
  * style(pre-commit): autofix
  * fix: add zero-clearing
  * use flt_max instead of
  * chore: separate tier4_perception_launch
  * chore: separate tier4_perception_launch
  * chore: add maintainer
  * chore: fix author
  * chore: copyright
  * chore: ci -check
  * fix: ci check
  * shorten line
  * fix: ci
  * style(pre-commit): autofix
  * fix CI check E501
  * fix CI double Q
  * style(pre-commit): autofix
  * change lint depend
  * skip test
  ---------
  Co-authored-by: badai-nguyen <dai.nguyen@tier4.jp>
  Co-authored-by: pre-commit-ci-lite[bot] <117423508+pre-commit-ci-lite[bot]@users.noreply.github.com>
  Co-authored-by: Anh Nguyen <anh.nguyen.2@tier4.jp>
  Co-authored-by: Anh Nguyen <anhnv.s@hblab.vn>
  Co-authored-by: badai nguyen <94814556+badai-nguyen@users.noreply.github.com>
* Contributors: Amadeusz Szymko, Anh Nguyen, Mete Fatih Cırıt, Ryohsuke Mitsudome
