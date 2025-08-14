^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_pipeline_latency_monitor
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.47.1 (2025-08-14)
-------------------

0.47.0 (2025-08-11)
-------------------
* feat(autoware_pipeline_latency_monitor): add autoware_pipeline_latency_monitor package (`#10798 <https://github.com/autowarefoundation/autoware_universe/issues/10798>`_)
  * feat(sensor_to_control_latency_checker): add node to monitor sensor data latency and publish diagnostics
  * feat(sensor_to_control_latency_checker): add offset parameters for processing times in latency calculations
  * use end-time based comparison for latency calculation
  * move include to src
  * refactor to be more generatic (input topics as params)
  * add README
  * rename package to autoware_pipeline_latency_monitor
  * refactor(pipeline_latency_monitor): improve variable naming for clarity in latency calculation
  * fix(pipeline_latency_monitor_node): handle negative latency values by treating them as 0.0
  * fix(pipeline_latency_monitor_node): improve total latency calculation by skipping steps with no valid data
  * fix(pipeline_latency_monitor_node): update debug topic names for latency publishing
  * fix(system.launch.xml): update comment for pipeline latency monitor inclusion
  * fix(README.md): update output topic names for clarity in latency monitoring
  * fix(system.launch.xml): add config_file argument to pipeline latency monitor launch
  * feat(package.xml): add dependencies for pipeline latency monitor and other missing packages
  * fix(package.xml): reorder license declaration
  ---------
  Co-authored-by: Maxime CLEMENT <maxime.clement@tier4.jp>
* Contributors: Kyoichi Sugahara
