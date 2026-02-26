cmake_minimum_required (VERSION 3.14.4)
project(rovtio)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native")
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wnon-virtual-dtor -Woverloaded-virtual)
endif()


# ROVTIO PARAMETERS
set(ROVTIO_NMAXFEATURE 25 CACHE STRING "Number of feature for ROVTIO")
set(ROVTIO_NCAM 1 CACHE STRING "Number of enabled cameras")
set(ROVTIO_NLEVELS 4 CACHE STRING "Number of image leavels for the features")
set(ROVTIO_PATCHSIZE 6 CACHE STRING "Size of patch (edge length in pixel)")
set(ROVTIO_NPOSE 0 CACHE STRING "Additional estimated poses for external pose measurements")
add_definitions(-DROVTIO_NMAXFEATURE=${ROVTIO_NMAXFEATURE})
add_definitions(-DROVTIO_NCAM=${ROVTIO_NCAM})
add_definitions(-DROVTIO_NLEVELS=${ROVTIO_NLEVELS})
add_definitions(-DROVTIO_PATCHSIZE=${ROVTIO_PATCHSIZE})
add_definitions(-DROVTIO_NPOSE=${ROVTIO_NPOSE})

add_subdirectory(lightweight_filtering)

include_directories(${PROJECT_SOURCE_DIR}/lightweight_filtering/include)

# --- Dependencies ---
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core highgui imgproc features2d)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(image_transport REQUIRED)

# YAML
find_package(PkgConfig REQUIRED)
pkg_check_modules(YamlCpp REQUIRED yaml-cpp>=0.5)

find_package(rovtio_interfaces_ros2 REQUIRED)
ament_export_dependencies(rosidl_default_runtime)

# --- Includes ---
include_directories(
		include
		${EIGEN3_INCLUDE_DIR}
		${YamlCpp_INCLUDE_DIRS}
)

# --- Library ---
set(ROVTIO_SRC
		src/Camera.cpp
		src/FeatureCoordinates.cpp
		src/FeatureDistance.cpp
)

if(MAKE_SCENE)
	list(APPEND ROVTIO_SRC src/Scene.cpp)
endif()


add_library(${PROJECT_NAME}_lib ${ROVTIO_SRC})
ament_target_dependencies(${PROJECT_NAME}_lib
		rclcpp
		geometry_msgs
		nav_msgs
		sensor_msgs
		std_msgs
		cv_bridge
		tf2
		tf2_ros
		std_srvs
		visualization_msgs
		rosbag2_cpp
		rosbag2_storage
		rovtio_interfaces_ros2
		image_transport
)
target_link_libraries(${PROJECT_NAME}_lib
		${YamlCpp_LIBRARIES}
		${OpenCV_LIBRARIES}
)


# --- Executables ---
add_executable(rovtio_node src/rovtio_node.cpp)
ament_target_dependencies(rovtio_node rclcpp rovtio_interfaces_ros2)
target_link_libraries(rovtio_node ${PROJECT_NAME}_lib)

add_executable(rovtio_rosbag_loader src/rovtio_rosbag_loader.cpp)
ament_target_dependencies(rovtio_rosbag_loader rclcpp rovtio_interfaces_ros2)
target_link_libraries(rovtio_rosbag_loader ${PROJECT_NAME}_lib)

add_executable(feature_tracker_node src/feature_tracker_node.cpp)
ament_target_dependencies(feature_tracker_node rclcpp rovtio_interfaces_ros2)
target_link_libraries(feature_tracker_node ${PROJECT_NAME}_lib)

# --- Install ---
install(TARGETS
		${PROJECT_NAME}_lib
		rovtio_node
		rovtio_rosbag_loader
		feature_tracker_node
		DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
		DESTINATION include/
)

install(DIRECTORY launch/
		DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY cfg/
		DESTINATION share/${PROJECT_NAME}/cfg)

# --- Export for downstream packages ---
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_lib)
ament_export_dependencies(
		rclcpp
		std_msgs
		geometry_msgs
		nav_msgs
		sensor_msgs
		cv_bridge
		tf2
		tf2_ros
		rosidl_default_runtime
		image_transport
)

ament_package()