^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_tensorrt_bevdet
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.50.0 (2026-02-14)
-------------------
* Merge remote-tracking branch 'origin/main' into humble
* chore(autoware_tensorrt_bevdet): remove cudnn dependency (`#11904 <https://github.com/autowarefoundation/autoware_universe/issues/11904>`_)
* fix: add cv_bridge.hpp support (`#11873 <https://github.com/autowarefoundation/autoware_universe/issues/11873>`_)
* Contributors: Amadeusz Szymko, Mete Fatih Cırıt, Ryohsuke Mitsudome

0.49.0 (2025-12-30)
-------------------

0.48.0 (2025-11-18)
-------------------
* Merge remote-tracking branch 'origin/main' into humble
* fix: tf2 uses hpp headers in rolling (and is backported) (`#11620 <https://github.com/autowarefoundation/autoware_universe/issues/11620>`_)
* feat(planning, perception): replace wall_timer with generic timer (`#11005 <https://github.com/autowarefoundation/autoware_universe/issues/11005>`_)
  * feat(planning, perception): replace wall_timer with generic timer
  * use rclcpp::create_timer
  * remove period_ns
  ---------
* Contributors: Mamoru Sobue, Ryohsuke Mitsudome, Tim Clephas

0.47.1 (2025-08-14)
-------------------

0.47.0 (2025-08-11)
-------------------
* feat(autoware_bevdet): implementation of bevdet using tensorrt (`#10441 <https://github.com/autowarefoundation/autoware_universe/issues/10441>`_)
* Contributors: rahulsundar-mcw
