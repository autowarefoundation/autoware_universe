cmake_minimum_required(VERSION 3.5)
project(autoware_bluetooth_monitor)

### Dependencies
find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/bluetooth_monitor.cpp
)

### Target executable
ament_auto_add_executable(l2ping_service
  service/main.cpp
  service/l2ping_service.cpp
  service/l2ping.cpp
)

find_package(Boost REQUIRED COMPONENTS
  serialization
)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})
target_link_libraries(l2ping_service ${Boost_LIBRARIES})

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::bluetooth_monitor::BluetoothMonitor"
  EXECUTABLE ${PROJECT_NAME}_node
)

ament_auto_package(INSTALL_TO_SHARE
  config
  launch
)
