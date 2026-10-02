^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_traffic_light_pipeline
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.53.0 (2026-09-29)
-------------------
* Merge remote-tracking branch 'origin/main' into prepare-0.53.0-changelog
* feat(autoware_traffic_light_pipeline): add traffic_light_recognition node (`#13367 <https://github.com/autowarefoundation/autoware_universe/issues/13367>`_)
  * feat(autoware_traffic_light_pipeline): add traffic_light_recognition node
  Compose, for one camera, the ROS-free core logic already extracted into
  autoware_tensorrt_yolox, autoware_traffic_light_map_based_detector,
  autoware_traffic_light_selector, autoware_traffic_light_classifier and
  autoware_traffic_light_category_merger into a single rclcpp::Node, in place of the
  5-Node graph production currently launches per camera:
  map_based_detector -> whole_image_detector(yolox) -> selector
  -> car_classifier / pedestrian_classifier -> category_merger
  The fine_detection path (traffic_light_fine_detector +
  traffic_light_occlusion_predictor) is out of scope.
  The package ships a ROS-free core (TrafficLightRecognition) plus a Node adapter
  (TrafficLightRecognitionNode, registered as an rclcpp_components plugin). The core's
  constructor needs the vector map, so the Node builds it on ~/input/vector_map, which
  is also where the three TensorRT engines are built; build_only builds those engines
  without a map and exits.
  The ML artifacts are named in config/traffic_light_recognition.param.yaml relative to
  a single ml_model_path parameter -- the same split autoware_lidar_centerpoint's
  ml_package.param.yaml uses -- so the config file names no user-specific path and needs
  no launch substitution.
  * docs(autoware_traffic_light_pipeline): add launch and test instructions to README
  * fix(autoware_traffic_light_pipeline): clean up unused includes and stale test comment
  * fix(autoware_traffic_light_pipeline): skip build when autoware_tensorrt_yolox is unavailable
  autoware_tensorrt_yolox returns early and installs no headers when
  autoware_tensorrt_common is missing, so this package fails to compile on
  environments without CUDA / TensorRT such as the non-CUDA CI runners.
  Guard the package with the same early return instead.
  * feat(autoware_traffic_light_pipeline): sync image and camera_info with bounded ApproximateTime
  Image and camera_info come from the same camera driver and are expected to
  share a stamp, so ExactTime worked. ApproximateTime tolerates drivers that
  stamp the two slightly differently; with equal stamps it behaves identically,
  publishing as soon as the second message arrives.
  Its default max interval is effectively unbounded, which would silently pair
  the current image with a camera_info from up to a full queue ago whenever one
  is dropped -- and both are subscribed with best-effort SensorDataQoS, so drops
  are expected under load. A stale camera_info makes the map-based detector
  project its ROIs from the ego pose of a different instant than the image was
  captured at, misplacing every ROI. Bound the interval to 50 ms, well below one
  frame period, so such pairs are rejected.
  * fix(autoware_traffic_light_map_based_detector): validate timestamp offset range
  An inverted min/max timestamp offset silently degraded detection: the tf
  sample loop never ran, leaving only the single stamp sample, which shrank
  the rough ROI vibration margin without any error.
  * fix(autoware_traffic_light_pipeline): wait for tf before running recognition
  TrafficLightRecognition::run() looks up transforms through tf2::BufferCore,
  which cannot wait. Without the wait the map based detector returned no ROIs,
  so every detection was discarded and empty signals were published.
  * fix(autoware_traffic_light_pipeline): align min_timestamp_offset with original package
  * fix(autoware_traffic_light_pipeline): rebuild only the map based detector on vector map update
  TrafficLightRecognition was rebuilt from scratch in the vector map callback,
  which reloaded the YOLOX and the two classifier TensorRT engines even though
  none of them depends on the map.
  Construct TrafficLightRecognition once in the node constructor and feed the map
  through set_map(), which recreates only the map based detector. The construction
  failure is now reported instead of escaping the subscription callback, and the
  missing map is reported through the return value of set_route() and run().
  * refactor(autoware_traffic_light_pipeline): return tl::expected from set_route
  * feat(autoware_traffic_light_pipeline): publish exposure diagnostics
  Report the classifiers' over/under exposure flags on /diagnostics. The
  core builds the DiagnosticArray so the node adapter publishes it the same
  way as the other outputs, and a status is emitted on every processed
  frame -- including frames with no selected ROI, which is the normal case
  whenever no traffic light is in view.
  * perf(autoware_traffic_light_pipeline): skip whole image detection when no ROI is expected
  * feat(autoware_traffic_light_pipeline): make model precision and normalization configurable
  The classifiers' mean / std must match the preprocessing their model was
  trained with, and model_path is already a parameter, so these were exposed
  as parameters too. precision is a property of the deployed TensorRT engine
  rather than of the ONNX file, so it is exposed for the whole image detector
  and both classifiers as well.
  car_classifier and pedestrian_classifier now share a ClassifierModelConfig,
  which lets make_car_classifier / make_pedestrian_classifier collapse into a
  single make_classifier.
  * refactor(autoware_traffic_light_map_based_detector): remove duplicated timestamp offset validation from node
  The same min/max_timestamp_offset validation is already performed in the
  TrafficLightMapBasedDetector constructor, so the node-side check is
  redundant.
  * fix(autoware_traffic_light_pipeline): disable intra-process comm for transient_local subscriptions
  The vector map and route subscriptions need transient_local durability, which rclcpp rejects
  when intra-process communication is enabled. Loading the node into a component container with
  use_intra_process_comms:=true therefore threw from the constructor and the component failed to
  load. Disable intra-process communication for these two subscriptions explicitly.
  * chore(autoware_traffic_light_pipeline): add maintainers
  ---------
  Co-authored-by: Masaki Baba <masaki.baba.2@tier4.jp>
* Contributors: Ryohsuke Mitsudome, Takahisa Ishikawa
