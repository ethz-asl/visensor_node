cmake_minimum_required(VERSION 2.8.3)
project(visensor_node)

find_package(catkin REQUIRED COMPONENTS
  roscpp 
  message_generation
  geometry_msgs
  sensor_msgs 
  cv_bridge 
  std_msgs 
  image_transport 
  camera_info_manager 
  dynamic_reconfigure
  cmake_modules
)

# check libvisensor version, flags not used later
find_package(libvisensor 1.1.0 REQUIRED)

add_message_files(
  DIRECTORY msg
  FILES visensor_imu.msg 
  visensor_time_host.msg
  visensor_calibration.msg
)

add_service_files(
	FILES
	visensor_calibration_service.srv
)

generate_messages(DEPENDENCIES geometry_msgs)

include_directories(include ${catkin_INCLUDE_DIRS} ${libvisensor_INCLUDE_DIRS})

find_package(Eigen REQUIRED)
include_directories(${EIGEN_INCLUDE_DIR})
add_definitions(${EIGEN_DEFINITIONS})

find_package(OpenCV REQUIRED COMPONENTS core highgui imgproc)

generate_dynamic_reconfigure_options(cfg/visensor_node.cfg)

if(NOT DEFINED CMAKE_BUILD_TYPE)
	set(CMAKE_BUILD_TYPE Release)
endif(NOT DEFINED CMAKE_BUILD_TYPE)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native -Wall -std=c++0x -D__STRICT_ANSI__")

catkin_package(
  INCLUDE_DIRS include ${catkin_INCLUDE_DIRS}
  CATKIN_DEPENDS
  roscpp 
  sensor_msgs 
  cv_bridge 
  std_msgs 
  image_transport 
  camera_info_manager 
)

#build and add libvisensor system library dependency
add_executable(visensor_node src/visensor_node.cpp src/visensor.cpp )

add_dependencies(visensor_node ${${PROJECT_NAME}_EXPORTED_TARGETS}})
target_link_libraries(visensor_node ${libvisensor_LIBRARIES} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
