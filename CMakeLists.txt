cmake_minimum_required(VERSION 3.0.2)
project(ouster_obstacle_avoidance)

## Use C++17 standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-O2 -Wall -Wextra)

# ── ROS dependencies ──────────────────────────────────────────────────────────
find_package(catkin REQUIRED COMPONENTS
    roscpp
    sensor_msgs
    geometry_msgs
    visualization_msgs
    std_msgs
    message_filters
    tf2
    tf2_ros
    tf2_geometry_msgs
)

# ── Eigen ─────────────────────────────────────────────────────────────────────
find_package(Eigen3 REQUIRED)

# ── jsoncpp (for parsing Ouster metadata) ────────────────────────────────────
find_package(PkgConfig REQUIRED)
pkg_check_modules(JSONCPP REQUIRED jsoncpp)

# ─────────────────────────────────────────────────────────────────────────────
catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ouster_obstacle_avoidance
    CATKIN_DEPENDS
        roscpp
        sensor_msgs
        geometry_msgs
        visualization_msgs
        std_msgs
        message_filters
    tf2
    tf2_ros
    tf2_geometry_msgs
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${JSONCPP_INCLUDE_DIRS}
)

# ── Library ───────────────────────────────────────────────────────────────────
add_library(ouster_obstacle_avoidance
    src/ObstacleDetector.cpp
    src/PotentialField.cpp
    src/VisualizationManager.cpp
    src/AvoidanceNode.cpp
)

target_link_libraries(ouster_obstacle_avoidance
    ${catkin_LIBRARIES}
    ${JSONCPP_LIBRARIES}
    Eigen3::Eigen
)

# ── Node executable ───────────────────────────────────────────────────────────
add_executable(ouster_obstacle_avoidance_node src/main.cpp)

target_link_libraries(ouster_obstacle_avoidance_node
    ouster_obstacle_avoidance
    ${catkin_LIBRARIES}
    ${JSONCPP_LIBRARIES}
    Eigen3::Eigen
)

# ── Install ───────────────────────────────────────────────────────────────────
install(TARGETS ouster_obstacle_avoidance ouster_obstacle_avoidance_node
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch config
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
