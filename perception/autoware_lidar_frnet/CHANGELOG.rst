^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_lidar_frnet
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.49.0 (2025-12-30)
-------------------
* Merge remote-tracking branch 'origin/main' into prepare-0.49.0-changelog
* feat(autoware_lidar_frnet): integrate cuda blackboard into point clooud process (`#11677 <https://github.com/autowarefoundation/autoware_universe/issues/11677>`_)
  * feat(lidar_frnet): integrate cuda_blackboard for enhanced point cloud processing
  * feat(lidar_frnet): integrate CudaBlackboard for point cloud publishing
  * feat(lidar_frnet): add CUDA remappings for point cloud outputs in launch configuration
  Co-authored-by: Amadeusz Szymko <amadeuszszymko@gmail.com>
  * fix(lidar_frnet): make point cloud layout members const for safety
  ---------
  Co-authored-by: Amadeusz Szymko <amadeusz.szymko.2@tier4.jp>
* Contributors: Kyoichi Sugahara, Ryohsuke Mitsudome

0.48.0 (2025-11-18)
-------------------
* Merge remote-tracking branch 'origin/main' into humble
* feat(autoware_lidar_frnet): add FRNet for LiDAR semantic segmentation (`#10503 <https://github.com/autowarefoundation/autoware_universe/issues/10503>`_)
  * feat(autoware_lidar_frnet): add FRNet for LiDAR semantic segmentation
  * docs(autoware_lidar_frnet): style
  Co-authored-by: Kenzo Lobos Tsunekawa <kenzo.lobos@gmail.com>
  * feat(autoware_lidar_frnet): store cloud layouts in minimal type
  * feat(autoware_lidar_frnet): use clear async
  * fix(autoware_lidar_frnet): correct value for cuda mem set & use clear async
  * fix(autoware_lidar_frnet): remove redundant stream sync
  * fix(autoware_lidar_frnet): avoid cuda memory allocation
  * fix(autoware_lidar_frnet): avoid cuda memory allocation (2)
  * fix(autoware_lidar_frnet): precess only output clouds with active subscribers
  * fix(autoware_lidar_frnet): atomic operation for fp precision point (x, y, z, intensity)
  * fix(autoware_lidar_frnet): explicit device stream sync for thrust
  * feat(autoware_lidar_frnet): use cub::DeviceRadixSort
  * feat(autoware_lidar_frnet): avoid host vectors
  * feat(autoware_lidar_frnet): update cuda flags
  * fix(autoware_lidar_frnet): final adjustment
  ---------
  Co-authored-by: Kenzo Lobos Tsunekawa <kenzo.lobos@gmail.com>
* Contributors: Amadeusz Szymko, Ryohsuke Mitsudome
