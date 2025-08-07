cmake_minimum_required(VERSION 3.8)
project(robocon2025_a)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# set pigpio directory
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} /home/mechatech/pigpio-master/util)

# pigpio ライブラリとヘッダーファイルの指定
include_directories(/usr/local/include)
link_directories(/usr/local/lib)

###############
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(pigpio REQUIRED)
find_package(Eigen3 REQUIRED)  # 行列計算
find_package(nlohmann_json REQUIRED)  # jsonファイルの取り扱い
find_package(ament_index_cpp REQUIRED)


find_library(PIGPIO_LIB pigpio)

###############
# Add Serialbridge
# set(SERIAL_BRIDGE_DIR "${CMAKE_SOURCE_DIR}/SerialBridge")
# find_library(SERIAL_BRIDGE_LIB NAMES SerialBridge PATHS "${SERIAL_BRIDGE_DIR}/bin")

###############
# include custom header files
include_directories(
  include
  include/${PROJECT_NAME}
)

###############
# Add ROS nodes
set(node_list pan_tilt_node)

foreach(node ${node_list})
  add_executable(${node} 
  src/${node}.cpp
  src/SignalControl.cpp
  )
  ament_target_dependencies(${node} rclcpp std_msgs sensor_msgs geometry_msgs ament_index_cpp)
  target_include_directories(${node} PRIVATE
    # "${SERIAL_BRIDGE_DIR}/src"
    "${rclcpp_INCLUDE_DIRS}"
    "${std_msgs_INCLUDE_DIRS}"
    ${PROJECT_SOURCE_DIR}/include
    $<TARGET_PROPERTY:Eigen3::Eigen,INTERFACE_INCLUDE_DIRECTORIES>
    nlohmann_json::nlohmann_json
  )
  target_include_directories(${node} PUBLIC
  ${PROJECT_SOURCE_DIR}/include
  )
  target_link_libraries(${node}
    # "${SERIAL_BRIDGE_LIB}"
    "${rclcpp_LIBRARIES}"
    "${std_msgs_LIBRARIES}"
    pigpio
  )
endforeach()


###############
# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# config フォルダ内のファイルをshare/your_package/config/にコピー
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)


###############

install(TARGETS
  ${node_list}
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
