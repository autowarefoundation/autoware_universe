^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_multi_object_tracker
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.46.0 (2025-06-20)
-------------------
* Merge remote-tracking branch 'upstream/main' into tmp/TaikiYamada/bump_version_base
* feat: add adaptive covariance threshold for tracker lifecycle management (`#10743 <https://github.com/autowarefoundation/autoware_universe/issues/10743>`_)
  * feat: add adaptive covariance threshold for tracker lifecycle management
  * fix: exclude equal condition when distance is 0 (potential bug)
  fix: remove unnecessary else
  chore: add variable name for adaptive covariance calculation formula
  refactor: store ego pose info in TrackerProcessor
  * style(pre-commit): autofix
  * fix: error in variable name
  * feat: use cache to store pre-calculated adaptive threshold components
  perf: replace divide and exp function in formula with alternatives to reduce computational cost
  fix: correct wrong modification on if condition
  fix: remove unused function definition
  * fix: rebase conflict
  * perf: use distance_sq to remove runtime root square for faster computation
  fix: add missed library inclusion
  * fix: add missed source file to CMakeList
  ---------
  Co-authored-by: Jian Kang <jian.kang@tier4.jp>
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* fix(multi_object_tracker): correct area calculation for cylinder shape in getArea function (`#10790 <https://github.com/autowarefoundation/autoware_universe/issues/10790>`_)
  fix(multi_object_tracker): correct area calculation for CYLINDER shape in getArea function
  Updated the area calculation for the CYLINDER shape to use a quarter of the cylinder's base area, ensuring accurate area representation in the multi-object tracker.
* perf(autoware_multi_object_tracker): tracker association and merge process efficiency improvement (`#10744 <https://github.com/autowarefoundation/autoware_universe/issues/10744>`_)
  * feat(multi_object_tracker): implement caching for tracked objects
  - Added caching mechanism in Tracker class to store and retrieve DynamicObject instances based on time.
  - Introduced methods to update, retrieve, and remove cached objects.
  - Updated PedestrianTracker and VehicleTracker to utilize the caching functionality during object tracking.
  This enhancement improves the efficiency of object retrieval in the tracking process.
  * refactor(multi_object_tracker): remove unnecessary ScopedTimeTrack instantiation
  - Eliminated the ScopedTimeTrack pointer from the canMergeOverlappedTarget method to streamline the code.
  - This change simplifies the function without affecting its logic or performance.
  * style(pre-commit): autofix
  * refactor(multi_object_tracker): optimize cache handling and streamline sorting logic
  - Changed cached_time\_ from rclcpp::Time to int for improved efficiency in the Tracker class.
  - Updated methods to handle cached time as nanoseconds directly, simplifying cache management.
  - Removed unnecessary ScopedTimeTrack instantiations in the mergeOverlappedTracker method to enhance code clarity and performance.
  - Streamlined the sorting logic for list_tracker\_ to eliminate redundant scopes, improving readability.
  * refactor(multi_object_tracker): change cached_time\_ type to rclcpp::Time for improved cache management
  - Updated cached_time\_ from int to rclcpp::Time to enhance type safety and clarity in cache handling.
  - Modified related methods to accommodate the new type, ensuring consistent time comparisons and cache updates.
  - This change simplifies the cache management logic in the Tracker class.
  * refactor(multi_object_tracker): remove unused boost_polygon_utils includes
  - Eliminated unnecessary inclusion of <autoware_utils/geometry/boost_polygon_utils.hpp> from PedestrianTracker, UnknownTracker, and VehicleTracker files.
  - This cleanup reduces dependencies and improves code clarity without affecting functionality.
  * feat(multi_object_tracker): integrate TimeKeeper for performance tracking
  - Added a shared pointer for TimeKeeper in the DataAssociation class to enable performance tracking of association methods.
  - Implemented ScopedTimeTrack in the assign and calcScoreMatrix methods to measure execution time.
  - Updated TrackerProcessor to set the TimeKeeper for the association, enhancing performance monitoring capabilities.
  - Adjusted marker lifetime in TrackerObjectDebugger for improved visualization timing.
  * feat(multi_object_tracker): add area attribute to DynamicObject and refine distance calculation
  - Introduced a new 'area' attribute in the DynamicObject structure to enhance object representation.
  - Updated the Mahalanobis distance calculation to return the squared distance directly, improving performance and clarity.
  - Refined angle gate logic to ensure proper threshold checks for angle comparisons in the DataAssociation class.
  * style(pre-commit): autofix
  * feat(multi_object_tracker): enhance area calculations and update distance metrics
  - Added a new function to calculate the area of different shape types, improving object representation.
  - Updated the distance calculation in the DataAssociation class to use squared distance for performance optimization.
  - Refined area gate logic to utilize the new area attribute in DynamicObject, ensuring accurate object scoring during tracking.
  * fix(multi_object_tracker): optimize yaw angle calculation for object tracking
  - Refactored the yaw angle calculation in the getFormedYawAngle function to improve accuracy and performance.
  - Replaced the previous fixed measurement logic with a more efficient raw difference calculation and fast modulo operation.
  - Enhanced front/back and side distinction handling for angle comparisons, ensuring correct angle thresholds are applied.
  * refactor(multi_object_tracker): optimize distance calculations and improve configuration handling
  - Refactored the Mahalanobis distance calculation to eliminate intermediate vector creation, enhancing performance.
  - Updated distance checks in DataAssociation and TrackerProcessor to use squared distances for efficiency.
  - Added pre-processing of configuration matrices in MultiObjectTracker to ensure proper initialization of distance and angle thresholds.
  * refactor(multi_object_tracker): simplify id management in TrackerObjectDebugger
  - Removed the handling of previous and current IDs in the TrackerObjectDebugger class to streamline the marker management process.
  - Eliminated unnecessary clearing and updating of ID sets, improving code clarity and reducing complexity in the reset and process methods.
  * feat(multi_object_tracker): implement R-tree for efficient spatial indexing in DataAssociation
  - Introduced an R-tree structure for spatial indexing of trackers, enhancing the efficiency of distance calculations during object association.
  - Added a method to update maximum search distances based on configuration, optimizing the association process.
  - Refactored the score matrix calculation to utilize the R-tree for querying nearby trackers, improving performance in the assignment of measurements to tracked objects.
  * feat(multi_object_tracker): enhance tracker merging with R-tree spatial indexing
  - Implemented a two-pass merging process for overlapping trackers, utilizing an R-tree for efficient spatial queries.
  - Introduced a TrackerData structure to pre-filter and store valid tracker information, improving data handling.
  - Optimized the merging logic by calculating IoU only when necessary and marking merged trackers for removal.
  - Updated distance calculations to leverage squared distances for performance improvements.
  * chore: avoid override
  * feat(multi_object_tracker): add time attribute to tracked objects in trackers
  - Updated the PedestrianTracker and VehicleTracker classes to include a time attribute in the tracked object structure.
  - Ensured that the time is set when retrieving tracked objects, enhancing the temporal accuracy of tracking data.
  * feat(multi_object_tracker): optimize tracker removal process in mergeOverlappedTracker
  - Introduced an unordered_set for efficient batch removal of merged trackers, improving performance during the final pass of tracker merging.
  - Removed commented-out code for clarity and streamlined the merging logic.
  * feat(multi_object_tracker): optimize R-tree insertion for tracker data
  - Refactored the insertion of tracker data into the R-tree by using a vector to batch insert points, improving performance during spatial indexing.
  - Updated both DataAssociation and TrackerProcessor classes to implement this optimization, enhancing overall efficiency in tracker management.
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* chore(multi_object_tracker): multi channel multi-object-tracker, set topic by launcher (`#10577 <https://github.com/autowarefoundation/autoware_universe/issues/10577>`_)
  * fix(multi_object_tracker): update input channel configuration and reduce max channel size
  * fix(tracking): update input channels and correct radar detection topic names
  * fix(tracking): update radar detection channel and remove deprecated parameters
  * fix(tracking): update input arguments for detection channels and objects in tracking.launch.xml
  * fix(tracking): simplify conditionals for radar and camera lidar fusion in tracking.launch.xml
  * fix(multi_object_tracker): remove deprecated input channel topics from schema
  * fix(multi_object_tracker): update output argument naming for consistency in launch files and publisher
  * docs(multi_object_tracker): update README input channel configuration to reflect type changes
  * Update README.md
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* fix(autoware_multi_object_tracker): bug fix of anchor point (`#10722 <https://github.com/autowarefoundation/autoware_universe/issues/10722>`_)
  * fix(shapes): correct anchor point calculation logic and improve precision check for anchor vector
  * fix(shapes): enhance anchor point offset logic to improve precision and handle edge cases
  * fix(shapes): revert wrong fix with readability improvement
  ---------
* fix(autoware_multi_object_tracker): update Mahalanobis distance threshold for data association (`#10648 <https://github.com/autowarefoundation/autoware_universe/issues/10648>`_)
  * refactor(autoware_multi_object_tracker): update Mahalanobis distance threshold for data association
  Changed the Mahalanobis distance threshold from 3.035 to a new critical value of 3.717, corresponding to a 99.99% confidence level for improved accuracy in object tracking.
  * style(pre-commit): autofix
  * refactor(autoware_multi_object_tracker): rename Mahalanobis distance threshold for clarity
  Updated the Mahalanobis distance threshold variable name to better reflect its purpose in the data association process, enhancing code readability.
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* Contributors: Kang, Taekjin LEE, TaikiYamada4

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
* feat(autoware_multi_object_tracker): tracker confidence check by its uncertainty (`#10378 <https://github.com/autowarefoundation/autoware_universe/issues/10378>`_)
  * preparation
  * feat(autoware_multi_object_tracker): enhance matrix initialization and add debug logging for tracker removal
  * feat(autoware_multi_object_tracker): integrate max distance matrix for tracker removal logic
  * refactor(autoware_multi_object_tracker): remove debug logging for tracker removal process
  style(pre-commit): autofix
  * refactor(autoware_multi_object_tracker): remove unused string include in association.cpp
  * refactor(autoware_multi_object_tracker): remove commented debug logging in association.cpp
  * refactor(autoware_multi_object_tracker): remove distance_threshold parameter and update related configurations
  * refactor(tracker): implement isConfidentTracker method for object confidence assessment
  * refactor(tracker): rename isConfidentTracker to isConfident and add isExpired method for object status management
  * refactor(tracker): enhance updateWithoutMeasurement and isExpired methods for improved object status management
  * refactor(tracker): adjust confidence and expiration thresholds for improved tracking accuracy
  * refactor(tracker): add position covariance methods and improve confidence assessment logic
  * refactor(tracker): add total existence probability calculation and improve debug output
  * refactor(tracker): enhance debug output for expiration checks and adjust minor axis threshold
  * refactor(tracker): simplify overlap removal logic by introducing canRemoveOverlappedTarget method
  refactor(tracker): improve code readability by formatting and enhancing overlap removal logic
  refactor(tracker): enhance UUID handling and improve confidence checks in tracker logic
  refactor(tracker): improve debug output for confidence and expiration checks
  * refactor(tracker): optimize overlap removal by using list iteration instead of sorting
  * refactor(tracker): simplify existence probability retrieval and enhance code clarity
  * refactor(debugger): streamline existence probability retrieval in TrackerObjectDebugger
  * feat(tracker): add time parameter to position covariance and confidence checks
  * refactor(tracker): enhance confidence checks by refining covariance thresholds and improving debug output
  style(pre-commit): autofix
  refactor(tracker): comment out debug messages in confidence and expiration checks for cleaner output
  refactor(processor): simplify object retrieval in removeOverlappedTracker method
  * refactor(tracker): remove redundant debug messages and enhance expiration checks with constants
  * fix: set default value if the given existence probability is almost zero
  * feat: merge overlapped trackers probability
  * refactor(tracker): improve existence probability updates and add debug output for probability vectors
  * style(pre-commit): autofix
  * refactor(tracker): rename updateExistenceProbabilities to mergeExistenceProbabilities and simplify logic
  * refactor(tracker): remove debug output from mergeOverlappedTracker function
  * refactor(tracker): remove debug output for existence probabilities in mergeOverlappedTracker function
  * refactor(tracker): include string header and simplify UUID string retrieval
  * refactor(tracker): rename normalize parameter to clamp and update related logic
  * refactor(tracker): rename EXPIRED_CONFIDENCE_THRESHOLD to EXPIRED_PROBABILITY_THRESHOLD for clarity
  * refactor(tracker): add comment to clarify target removal condition in canMergeOverlappedTarget function
  * style(pre-commit): autofix
  * refactor(tracker): add validation checks for covariance matrix in getPositionCovarianceEigenSq and getPositionCovarianceSizeSq functions
  * refactor(tracker): improve covariance validation logging in getPositionCovarianceEigenSq and getPositionCovarianceSizeSq functions
  * refactor(tracker): optimize iterator handling in mergeOverlappedTracker function
  * refactor(types): change default_existence_probability type from double to float
  * refactor(tracker): rename getPositionCovarianceSizeSq to getPositionCovarianceDeterminant for clarity
  * refactor(tracker): update covariance thresholds to mitigate drawbacks
  * refactor(tracker): adjust covariance thresholds for confidence and expiration checks
  Updated the covariance thresholds in the Tracker class to improve confidence and expiration logic, enhancing the accuracy of object tracking.
  * fix: adjust existence probability threshold of expiration
  * refactor(tracker): improve UUID formatting in tracker_base.hpp
  Updated the UUID formatting logic in the Tracker class to use a constant for the UUID size and ensure proper type casting, enhancing code clarity and maintainability.
  * fix(types): cap existence probability to a maximum of 0.999
  Added a check to ensure that the existence probability does not exceed 0.999, addressing potential issues with overly high values that may not be set correctly.
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* fix(multi_object_tracker): remove unused function getMeasurementYaw (`#10527 <https://github.com/autowarefoundation/autoware_universe/issues/10527>`_)
* fix(multi_object_tracker): remove unused function isChannelSpawnEnabled (`#10528 <https://github.com/autowarefoundation/autoware_universe/issues/10528>`_)
* Contributors: Ryuta Kambe, Taekjin LEE, TaikiYamada4

0.44.2 (2025-06-10)
-------------------

0.44.1 (2025-05-01)
-------------------

0.44.0 (2025-04-18)
-------------------
* Merge remote-tracking branch 'origin/main' into humble
* feat(multi object tracker): tracker overlap threshold (`#10456 <https://github.com/autowarefoundation/autoware_universe/issues/10456>`_)
  * preparation
  * feat(autoware_multi_object_tracker): enhance matrix initialization and add debug logging for tracker removal
  * refactor(autoware_multi_object_tracker): replace vector matrices with Eigen matrices in AssociatorConfig
  * feat(autoware_multi_object_tracker): integrate max distance matrix for tracker removal logic
  * refactor(autoware_multi_object_tracker): remove debug logging for tracker removal process
  * style(pre-commit): autofix
  * refactor(autoware_multi_object_tracker): remove unused string include in association.cpp
  * refactor(autoware_multi_object_tracker): remove commented debug logging in association.cpp
  * refactor(autoware_multi_object_tracker): remove distance_threshold parameter and update related configurations
  * refactor(multi_object_tracker_node): change Eigen::Map to use const for matrix initialization
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* fix(multi_object_tracker): add required headers (`#10461 <https://github.com/autowarefoundation/autoware_universe/issues/10461>`_)
* fix(autoware_multi_object_tracker): fix bicycle renovation vector dimension (`#10449 <https://github.com/autowarefoundation/autoware_universe/issues/10449>`_)
  fix a bug in updateStatePoseVel
* feat(autoware_multi_object_tracker): implement time keeper (`#10431 <https://github.com/autowarefoundation/autoware_universe/issues/10431>`_)
  * feat(multi_object_tracker): integrate ScopedTimeTrack for detailed processing time tracking
  * feat(multi_object_tracker): add parameter for detailed processing time publishing
  ---------
* feat(autoware_multi_object_tracker): vehicle's ego frame as a parameter (`#10428 <https://github.com/autowarefoundation/autoware_universe/issues/10428>`_)
* feat(multi_object_tracker): add diagnostics warning when extrapolation time exceeds limit with latency guarantee enabled (`#10301 <https://github.com/autowarefoundation/autoware_universe/issues/10301>`_)
  * feat(multi_object_tracker): add diagnostics warning when extrapolation time exceeds limit with latency guarantee enabled
  * feat(multi_object_tracker): handled  the case last_updated_time\_ initialized as 0
  * feat(multi_object_tracker): refactored to give better structure
  diagnostic force updated when published
  * style(pre-commit): autofix
  * feat(multi_object_tracker): add published tracker count check
  * style(pre-commit): autofix
  * feat(multi_object_tracker): fix checkAllTiming  complexity
  * style(pre-commit): autofix
  * feat(multi_object_tracker): check consecutive warning duration
  * style(pre-commit): autofix
  * feat(multi_object_tracker): diag messages updated
  * feat(multi_object_tracker): diag messages updated
  * style(pre-commit): autofix
  * style(pre-commit): autofix
  * feat(multi_object_tracker): messages fix
  ---------
  Co-authored-by: lei.gu <lei.gu@tier4.jp>
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* feat(autoware_multi_object_tracker): update overlapped tracker removing process in multi obj tracker  (`#10347 <https://github.com/autowarefoundation/autoware_universe/issues/10347>`_)
  Update overlapped tracker removing process
  Co-authored-by: Taekjin LEE <taekjin.lee@tier4.jp>
* feat(autoware_multi_object_tracker): selective update per channel (`#10277 <https://github.com/autowarefoundation/autoware_universe/issues/10277>`_)
  * refactor(bicycle_motion_model): implement exponential decay for slip angle in state prediction
  * Revert "refactor(multi_object_tracker): simplify input channel configuration by removing trust flags and consolidating parameters"
  This reverts commit c5155ef2e978b411955ace35f412bbf76c96f354.
  * refactor(multi_object_tracker): update measure function signatures to include InputChannel parameter
  * refactor(multi_object_tracker): add updateStatePoseVel method to BicycleMotionModel and update measurement logic in VehicleTracker
  * refactor(multi_object_tracker): update measureWithPose method to include InputChannel parameter and adjust related logic
  * refactor(multi_object_tracker): remove BicycleTracker and update references to use VehicleTracker
  * refactor(bicycle_tracker): add tracking_offset to adjust object position based on motion model
  * refactor(multi_object_tracker): remove BicycleTracker and replace with VehicleTracker in relevant classes
  * refactor(input_channels): disable trust flags for extension and orientation in radar configurations
  * refactor(input_channels): restructure flags for input channel properties
  * refactor(input_channels): remove 'flags' from required properties in schema
  ---------
* Contributors: Amadeusz Szymko, Ryohsuke Mitsudome, TadaKazuto, Taekjin LEE, Takagi, Isamu, lei.gu

0.43.0 (2025-03-21)
-------------------
* Merge remote-tracking branch 'origin/main' into chore/bump-version-0.43
* chore: rename from `autoware.universe` to `autoware_universe` (`#10306 <https://github.com/autowarefoundation/autoware_universe/issues/10306>`_)
* fix(autoware_multi_object_tracker): unknown object orientation (`#10286 <https://github.com/autowarefoundation/autoware_universe/issues/10286>`_)
  * fix(unknown_tracker): update object pose orientation and streamline uncertainty modeling in input manager
  * fix(object_model): correct bounding box calculation by initializing limits and including min_z
  ---------
* refactor(multi_object_tracker): internal message driven process (`#10203 <https://github.com/autowarefoundation/autoware_universe/issues/10203>`_)
  * refactor(multi_object_tracker): streamline input channel configuration handling
  feat(multi_object_tracker): introduce InputChannel struct for input channel configuration
  refactor(multi_object_tracker): improve marker handling and initialization in TrackerObjectDebugger
  feat(multi_object_tracker): enhance InputChannel with trust flags for object properties
  refactor(multi_object_tracker): remove unused channel_size parameter from tracker constructors
  feat(multi_object_tracker): update InputChannel flags to trust object extension and classification
  fix(multi_object_tracker): replace channel.index with channel_index for consistency
  feat(multi_object_tracker): update TrackerObjectDebugger and TrackerProcessor to accept channels_config parameter
  refactor(multi_object_tracker): remove redundant existence probability initialization from tracker constructors
  feat(multi_object_tracker): integrate data association into TrackerProcessor and add associate method
  feat(multi_object_tracker): enhance updateWithMeasurement to include channel_info for improved classification handling
  refactor(multi_object_tracker): replace object_id with uuid in DynamicObject and related classes
  fix(multi_object_tracker): update UUID handling in Tracker to use uuid_msg for consistency
  refactor(multi_object_tracker): simplify pose and covariance handling in tracker classes
  refactor(multi_object_tracker): replace pose_with_covariance with separate pose and covariance attributes in DynamicObject
  refactor: remove z state from tracker. it will uses object state
  refactor(multi_object_tracker): streamline object handling in trackers and remove unnecessary shape processing
  refactor(multi_object_tracker): remove z position handling from trackers and update object kinematics structure
  refactor(multi_object_tracker): remove BoundingBox structure from trackers and implement object extension limits
  refactor(multi_object_tracker): remove unnecessary blank lines in tracker getTrackedObject methods
  refactor(multi_object_tracker): simplify input channel configuration by removing trust flags and consolidating parameters
  * refactor(multi_object_tracker): use const reference in loop and simplify tracker update logic
  * refactor(multi_object_tracker): update shape handling and streamline object tracking logic
  * refactor(multi_object_tracker): update shape handling to use geometry_msgs::msg::Point for anchor vectors
  * style(pre-commit): autofix
  * refactor(multi_object_tracker): modify getNearestCornerOrSurface function signature and update related logic
  refactor(multi_object_tracker): remove self_transform parameter from measure and update methods
  refactor(multi_object_tracker): update calcAnchorPointOffset function signature and streamline object handling
  refactor(multi_object_tracker): set shape type to BOUNDING_BOX for object trackers
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* Contributors: Hayato Mizushima, Taekjin LEE, Yutaka Kondo

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
* feat(multi_object_tracker): integrate odometry and transform processes (`#9912 <https://github.com/autowarefoundation/autoware_universe/issues/9912>`_)
  * feat: Add odometry processor to multi-object tracker
  * refactor: Refactor Odometry class for improved code organization and readability
  * feat: Refactor Odometry class for improved code organization and readability
  * refactor: Transform objects to world coordinate in Odometry class
  refactor: Transform objects to world coordinate in Odometry class
  refactor: Update Odometry class to get transform from tf with source frame ID
  feat: Update Odometry class to get transform from tf with source frame ID
  fix: move necessare tr2 header
  * Revert "refactor: Transform objects to world coordinate in Odometry class"
  This reverts commit efca28a40105f80deb09d57b55cb6f9d83ffda2c.
  * refactor: Remove unnecessary tf2 headers from tracker models
  * fix: move transform obtainer to odometry class
  * refactor: Update Odometry class to get transform from tf with source frame ID
  * refactor: Transform objects to world coordinate in Odometry class
  * refactor: remove transformObjects from shapes
  * refactor: Update Odometry class to use 'updateFromTf' instead of 'setOdometryFromTf'
  * refactor: Update Odometry class to use 'updateFromTf' instead of 'setOdometryFromTf'
  * refactor: Update InputManager to include Odometry in constructor
  * refactor: Move odometry.cpp to lib folder
  * move object transform to input stream
  * refactor: Add enable_odometry_uncertainty parameter to Odometry constructor
  * refactor: Update Odometry class to return optional Odometry from getOdometryFromTf
  * refactor: Update Odometry class to use tf_cache\_ for storing and retrieving transforms
  * refactor: Update Odometry class to use tf_cache\_ for storing and retrieving transforms
  * refactor: bring odometry covariance modeler into odometry class
  * refactor: Remove redundant code for updating tf cache in Odometry::updateTfCache
  * refactor: Update runProcess parameter name to detected_objects
  ---------
* feat: tier4_debug_msgs to autoware_internal_debug_msgs in files  perc… (`#9879 <https://github.com/autowarefoundation/autoware_universe/issues/9879>`_)
  feat: tier4_debug_msgs to autoware_internal_debug_msgs in files  perception/autoware_multi_object_tracker
* chore(autoware_multi_object_tracker): fix autoware univserse documentation page (`#9772 <https://github.com/autowarefoundation/autoware_universe/issues/9772>`_)
  * feat: Add descriptions for confidence thresholds in multi_object_tracker_node schema
  * feat: Update multi_object_tracker_node schema with confidence threshold descriptions
  ---------
* refactor(autoware_multi_object_tracker): define a new internal object class (`#9706 <https://github.com/autowarefoundation/autoware_universe/issues/9706>`_)
  * feat: Add dynamic_object.hpp to object_model directory
  * chore: Update autoware_perception_msgs include statements in association.hpp and dynamic_object.hpp
  * fix: replace object message type to the DynamicObject type
  * chore: Update autoware_perception_msgs include statements in association.hpp and dynamic_object.hpp
  * chore: add channel index to the DynamicObjects
  * Revert "chore: add channel index to the DynamicObjects"
  This reverts commit c7e73f08a8d17b5b085dd330dbf187aabbec6879.
  * fix: replace trackedobject in the process
  * fix: Replace transformObjects with shapes::transformObjects for object transformation
  * chore: add channel index to the DynamicObjects
  * feat: separate shape related functions
  * chore: clean up utils.hpp
  * chore: Update function signatures to use DynamicObjectList instead of DynamicObjects
  * chore: Add channel index to DynamicObject and DynamicObjectList
  * chore: Refactor processor and debugger classes to remove channel_index parameter
  * chore: Refactor multiple_vehicle_tracker.cpp and debugger.cpp
  * Refactor object tracker classes to remove self_transform parameter
  * Refactor object tracker classes to use shapes namespace for shape-related functions
  * Refactor object tracker classes to use types.hpp for object model types
  * Refactor object tracker classes to remove unused utils.hpp
  * Refactor object tracker classes to use types.hpp for object model types
  * chore: rename to types.cpp
  * rename getDynamicObject to toDynamicObject
  * Update perception/autoware_multi_object_tracker/lib/object_model/shapes.cpp
  Co-authored-by: Yukihiro Saito <yukky.saito@gmail.com>
  ---------
  Co-authored-by: Yukihiro Saito <yukky.saito@gmail.com>
* fix(autoware_multi_object_tracker): fix bugprone-errors (`#9651 <https://github.com/autowarefoundation/autoware_universe/issues/9651>`_)
  fix: bugprone-errors
* refactor(autoware_multi_object_tracker): add configurable tracker parameters (`#9621 <https://github.com/autowarefoundation/autoware_universe/issues/9621>`_)
  * refactor(autoware_multi_object_tracker): add configurable tracker parameters
  * style(pre-commit): autofix
  * refactor(autoware_multi_object_tracker): remove default values from parameter declarations
  * refactor(autoware_multi_object_tracker): update schema file
  * style(pre-commit): autofix
  * Update perception/autoware_multi_object_tracker/src/processor/processor.cpp
  * Update perception/autoware_multi_object_tracker/src/processor/processor.cpp
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: Taekjin LEE <technolojin@gmail.com>
* Contributors: Fumiya Watanabe, Taekjin LEE, Vishal Chauhan, jakor97, kobayu858

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
* fix(autoware_multi_object_tracker): measure latency with latest detection update time (`#9533 <https://github.com/autowarefoundation/autoware_universe/issues/9533>`_)
  * fix: measure latency with latest detection update time
  * fix: remove duplicated current_time
  ---------
* fix(cpplint): include what you use - perception (`#9569 <https://github.com/autowarefoundation/autoware_universe/issues/9569>`_)
* ci(pre-commit): autoupdate (`#8949 <https://github.com/autowarefoundation/autoware_universe/issues/8949>`_)
  Co-authored-by: M. Fatih Cırıt <mfc@autoware.org>
* fix(autoware_multi_object_tracker): fix clang-diagnostic-unused-private-field (`#9491 <https://github.com/autowarefoundation/autoware_universe/issues/9491>`_)
* 0.39.0
* update changelog
* fix: fix ticket links to point to https://github.com/autowarefoundation/autoware_universe (`#9304 <https://github.com/autowarefoundation/autoware_universe/issues/9304>`_)
* feat(autoware_multi_object_tracker): new function to add odometry uncertainty (`#9139 <https://github.com/autowarefoundation/autoware_universe/issues/9139>`_)
  * feat: add Odometry uncertainty to object tracking
  * feat: Add odometry heading uncertainty to object pose covariance
  feat: Rotate object pose covariance matrix to account for yaw uncertainty
  Rotate the object pose covariance matrix in the uncertainty_processor.cpp file to account for the yaw uncertainty. This ensures that the covariance matrix accurately represents the position uncertainty of the object.
  Refactor the code to rotate the covariance matrix using Eigen's Rotation2D class. The yaw uncertainty is added to the y-y element of the rotated covariance matrix. Finally, update the object_pose_cov array with the updated covariance values.
  Closes `#123 <https://github.com/autowarefoundation/autoware_universe/issues/123>`_
  * feat: Add odometry motion uncertainty to object pose covariance
  refactoring
  * feat: Update ego twist uncertainty to the object velocity uncertainty
  * feat: update object twist covariance by odometry yaw rate uncertainty
  * feat: move uncertainty modeling to input side
  * feat: add option to select odometry uncertainty
  * refactor: rename consider_odometry_uncertainty to enable_odometry_uncertainty
  * fix: transform to world first, add odometry covariance later
  style(pre-commit): autofix
  * feat: Add odometry heading uncertainty to object pose covariance
  ---------
* fix: fix ticket links to point to https://github.com/autowarefoundation/autoware_universe (`#9304 <https://github.com/autowarefoundation/autoware_universe/issues/9304>`_)
* chore(package.xml): bump version to 0.38.0 (`#9266 <https://github.com/autowarefoundation/autoware_universe/issues/9266>`_) (`#9284 <https://github.com/autowarefoundation/autoware_universe/issues/9284>`_)
  * unify package.xml version to 0.37.0
  * remove system_monitor/CHANGELOG.rst
  * add changelog
  * 0.38.0
  ---------
* Contributors: Esteve Fernandez, Fumiya Watanabe, M. Fatih Cırıt, Ryohsuke Mitsudome, Taekjin LEE, Yutaka Kondo, awf-autoware-bot[bot], kobayu858

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
* refactor(object_recognition_utils): add autoware prefix to object_recognition_utils (`#8946 <https://github.com/autowarefoundation/autoware_universe/issues/8946>`_)
* feat(autoware_multi_object_tracker): Set maximum reverse velocity to bicycle and crtv motion models (`#9019 <https://github.com/autowarefoundation/autoware_universe/issues/9019>`_)
  * feat: Add maximum reverse velocity to bicycle and CTRV motion models
  revert the tracker orientation when the velocity exceed the maximum reverse velocity
  refactor: Update motion model parameters for bicycle and CTRV motion models
  * refactor:  check the max_reverse_vel configuration is correct
  max_reverse_vel is expected to be  negative
  * refactor: remove config checker in the initializer
  ---------
* refactor(autoware_multi_object_tracker): separate detected object covariance modeling (`#9001 <https://github.com/autowarefoundation/autoware_universe/issues/9001>`_)
  * refactor: update object model includes in tracker models
  * feat: add uncertainty processor for object tracking
  feat: refactor uncertainty processing for object tracking
  feat: impl obj class model
  feat: Update object model measurement covariances
  Refactor the object model measurement covariances in the `object_model.hpp` file. Update the velocity long and velocity lat measurement covariances for different object model types.
  refactor: Model object uncertainty in multi_object_tracker_node.cpp
  feat: Update object model measurement covariances in object_model.hpp
  feat: Update uncertainty processing for object tracking
  fix: remove uncertainty modelling in trackers
  refactor: Remove unused function isLargeVehicleLabel
  The function isLargeVehicleLabel in utils.hpp is no longer used and can be safely removed.
  Revert "refactor: Remove unused function isLargeVehicleLabel"
  This reverts commit 23e3eff511b21ef8ceeacb7db47c74f747009a32.
  feat: Normalize uncertainty in object tracking
  This commit adds a new function `normalizeUncertainty` to the `uncertainty_processor.hpp` and `uncertainty_processor.cpp` files. The function normalizes the position and twist covariance matrices of detected objects to ensure minimum values for distance, radius, and velocity. This helps improve the accuracy and reliability of object tracking.
  * refactor: update motion model parameters for object tracking
  * refactor: update yaw rate limit in object model
  * Revert "refactor: update yaw rate limit in object model"
  This reverts commit 6e8b201582cb65673678029dc3a781f2b7126f81.
  * refactor: update object model measurement covariances
  Refactor the object model measurement covariances in the `object_model.hpp` file. Update the velocity long and velocity lat measurement covariances for different object model types.
  * refactor: update motion model parameters comments
  * refactor: remove comment
  * style(pre-commit): autofix
  * feat: Update copyright notice in uncertainty_processor.hpp
  Update the copyright notice in the uncertainty_processor.hpp file to reflect the correct company name.
  * refactor: update runProcess function parameters in multi_object_tracker_node.hpp
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* fix(autoware_multi_object_tracker): update yaw with range-limited innovation (`#8976 <https://github.com/autowarefoundation/autoware_universe/issues/8976>`_)
  fix: update yaw with range-limited innovation
* feat(autoware_multi_object_tracker): reduce trigger latency (`#8657 <https://github.com/autowarefoundation/autoware_universe/issues/8657>`_)
  * feat: timer-based trigger with phase compensation
  * chore: update comments, name of variable
  * chore: declare min and max publish interval ratios
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* fix(autoware_multi_object_tracker): output from screen to both (`#8407 <https://github.com/autowarefoundation/autoware_universe/issues/8407>`_)
* fix(autoware_multi_object_tracker): fix unusedFunction (`#8573 <https://github.com/autowarefoundation/autoware_universe/issues/8573>`_)
  fix:unusedFunction
* chore(autoware_multi_object_tracker): fix typo in input_channels.schema.json (`#8515 <https://github.com/autowarefoundation/autoware_universe/issues/8515>`_)
  * fix(schema): fix typo in input_channels.schema.json
  Fixed a typo in the "lidar_pointpainting" key in the input_channels.schema.json file.
  * fix: fix typo in lidar_pointpainting key
  * chore: fix typo of lidar_pointpainitng channel
  ---------
  Co-authored-by: Shintaro Tomie <58775300+Shin-kyoto@users.noreply.github.com>
* refactor(kalman_filter): prefix package and namespace with autoware (`#7787 <https://github.com/autowarefoundation/autoware_universe/issues/7787>`_)
  * refactor(kalman_filter): prefix package and namespace with autoware
  * move headers to include/autoware/
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* docs(autoware_multi_object_tracker): update input_channels schema with default values (`#8473 <https://github.com/autowarefoundation/autoware_universe/issues/8473>`_)
  chore(perception): update input_channels schema with default values
* fix(autoware_multi_object_tracker): enable trigger publish when delay_compensation is false (`#8484 <https://github.com/autowarefoundation/autoware_universe/issues/8484>`_)
  fix: enable trigger publish when delay_compensation is false
* fix(autoware_multi_object_tracker): fix functionConst (`#8424 <https://github.com/autowarefoundation/autoware_universe/issues/8424>`_)
  fix:functionConst
* docs(autoware_multi_object_tracker): add default values on the schema json (`#8179 <https://github.com/autowarefoundation/autoware_universe/issues/8179>`_)
  * Refractored the parameters, build the schema file, updated the readme file.
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* fix(autoware_multi_object_tracker): fix functionConst (`#8290 <https://github.com/autowarefoundation/autoware_universe/issues/8290>`_)
  * fix:functionConst
  * fix:functionConst
  * fix:clang format
  ---------
* fix(autoware_multi_object_tracker): revert latency reduction logic and bring back to timer trigger (`#8277 <https://github.com/autowarefoundation/autoware_universe/issues/8277>`_)
  * fix: revert latency reduction logic and bring back to timer trigger
  * style(pre-commit): autofix
  * chore: remove unused variables
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* fix(autoware_multi_object_tracker): fix uninitMemberVar (`#8335 <https://github.com/autowarefoundation/autoware_universe/issues/8335>`_)
  fix:uninitMemberVar
* fix(autoware_multi_object_tracker): fix passedByValue (`#8231 <https://github.com/autowarefoundation/autoware_universe/issues/8231>`_)
  fix:passedByValue
* fix(multi_object_tracker, object_merger, radar_object_tracker, tracking_object_merger): fix knownConditionTrueFalse warnings (`#8137 <https://github.com/autowarefoundation/autoware_universe/issues/8137>`_)
  * fix: cppcheck knownConditionTrueFalse
  * fix
  * fix
  ---------
* fix(autoware_multi_object_tracker): missing parameter schema path fix (`#8120 <https://github.com/autowarefoundation/autoware_universe/issues/8120>`_)
  fix: missing parameter schema path fix
* fix(multi_object_tracker): fix funcArgNamesDifferent (`#8079 <https://github.com/autowarefoundation/autoware_universe/issues/8079>`_)
  fix:funcArgNamesDifferent
* refactor(multi_object_tracker): bring parameter schema to new package folder (`#8105 <https://github.com/autowarefoundation/autoware_universe/issues/8105>`_)
  refactor: bring parameter schema to new package folder
* refactor(multi_object_tracker)!: add package name prefix of autoware\_ (`#8083 <https://github.com/autowarefoundation/autoware_universe/issues/8083>`_)
  * refactor: rename multi_object_tracker package to autoware_multi_object_tracker
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* Contributors: Boyang, Esteve Fernandez, Ryuta Kambe, Taekjin LEE, Yutaka Kondo, kminoda, kobayu858

0.26.0 (2024-04-03)
-------------------
