^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_image_object_locator
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.50.0 (2026-02-14)
-------------------
* Merge remote-tracking branch 'origin/main' into humble
* feat!: remove ROS 2 Galactic codes (`#11905 <https://github.com/autowarefoundation/autoware_universe/issues/11905>`_)
* Contributors: Ryohsuke Mitsudome

0.49.0 (2025-12-30)
-------------------

0.48.0 (2025-11-18)
-------------------
* Merge remote-tracking branch 'origin/main' into humble
* feat(image_object_locator): add covariance calculation (`#11522 <https://github.com/autowarefoundation/autoware_universe/issues/11522>`_)
  * add covariance calculation
  * fix cppcheck
  * fix typo
  * add default destructor
  * remove unused
  * remove unused
  * change covariance modeling to more lighter one
  * improve comment
  * refactor truncation check
  * fix comment
  * fix params
  * remove sampler
  * fix typo
  * remove unused variable
  * fix param name
  * fix comment
  ---------
* feat(autoware_image_object_locator): add autoware_image_object_locator package (`#11411 <https://github.com/autowarefoundation/autoware_universe/issues/11411>`_)
  * feat: initialize roi_based_detector node
  * fix: add camera_info sub
  * fix: add transform to base link (wip)
  * fix: transform
  * chore: add debug distance topic
  * fix: transform
  * fix: tensorrt_yolox for new 2class box model
  * chore: refactor
  * refactor
  * typo
  * chore: typo
  * fix: dimensions
  * fix: replace camera_lidar_fusion/objects with roi_based_detection/objects to test prediction
  * refactor
  * fix: add target frame
  * fix: add class selection
  * fix depend
  * fix: change camera
  * style(pre-commit): autofix
  * fix format
  * use opencv to calc projection and take multiple cam inputs
  * refactor roi calculations
  * cleanup
  * update
  * add readme
  * add schema
  * change default topic name and fix typo
  * add autoware prefix
  * fix default param
  * fix typo and add ignore setting for Matx
  * add static
  * remove const
  * add pedestrian width limits
  * change to autoware_lint_common
  * remove roi truncation check
  * change pedestrian config
  * add position adjustment for pedestrian
  * remove unsed function
  * set rois_ids from launch file
  * add rois_ids to schema
  * fix schema
  * change param name
  * change default detection range parameter to more restricted one
  * add roi confidence threshold
  * add Taekjin-san as a maintainer
  Co-authored-by: badai nguyen  <94814556+badai-nguyen@users.noreply.github.com>
  * change default value to debug friendly
  Co-authored-by: badai nguyen  <94814556+badai-nguyen@users.noreply.github.com>
  * change package name
  * fix doc
  * change package to image_object_locator
  * fix param
  ---------
  Co-authored-by: badai-nguyen <dai.nguyen@tier4.jp>
  Co-authored-by: pre-commit-ci-lite[bot] <117423508+pre-commit-ci-lite[bot]@users.noreply.github.com>
  Co-authored-by: badai nguyen <94814556+badai-nguyen@users.noreply.github.com>
* Contributors: Masaki Baba, Ryohsuke Mitsudome
