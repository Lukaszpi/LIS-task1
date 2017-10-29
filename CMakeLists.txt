cmake_minimum_required(VERSION 2.8.0)
project(tethered_drone)

add_definitions(-std=c++11)

find_package(catkin_simple REQUIRED tf Eigen)

catkin_simple(ALL_DEPS_REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})



cs_add_library(image_processing
  src/library/image_processing.cpp
)

cs_add_executable(image_processing_node
  src/image_processing_node.cpp
)


cs_add_library(trans_rot_estimator
  src/library/trans_rot_estimator.cpp
)

cs_add_library(waypoint_flyer
  src/library/waypoint_flyer.cpp
)

cs_add_executable(attitude_estimator_node
  src/attitude_estimator_node.cpp
)

cs_add_executable(waypoint_flyer_node
  src/waypoint_flyer_node.cpp
)

target_link_libraries(image_processing_node image_processing ${catkin_LIBRARIES} ${OpenCV_LIBS}) 

target_link_libraries(attitude_estimator_node trans_rot_estimator ${catkin_LIBRARIES})

target_link_libraries(waypoint_flyer_node waypoint_flyer ${catkin_LIBRARIES})

cs_install()
cs_export()
