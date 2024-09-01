cmake_minimum_required (VERSION 3.10.2)  # CMake version in Ubuntu 18.04 LTS
project(oak_ffc_4p VERSION 2.9.0 LANGUAGES CXX C)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
add_compile_options(-g)

## is used, also find other catkin packages
if(POLICY CMP0057)
    cmake_policy(SET CMP0057 NEW)
endif()

set(_opencv_version 4)
find_package(OpenCV 4 QUIET COMPONENTS imgproc highgui)
if(NOT OpenCV_FOUND)
  set(_opencv_version 3)
  find_package(OpenCV 3 REQUIRED COMPONENTS imgproc highgui)
endif()

find_package(catkin REQUIRED COMPONENTS
  depthai_ros_msgs
  roscpp
  sensor_msgs
  std_msgs
  cv_bridge
  depthai_bridge
  message_filters
  nodelet
)
find_package(depthai CONFIG REQUIRED)

catkin_package(
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS depthai_ros_msgs roscpp sensor_msgs std_msgs depthai_bridge cv_bridge message_filters
  DEPENDS
  OpenCV
)

macro(dai_add_node node_name node_src)
    add_executable("${node_name}" "${node_src}")

    add_dependencies("${node_name}"
      ${catkin_EXPORTED_TARGETS}
      ${${PROJECT_NAME}_EXPORTED_TARGETS}
    )

    target_link_libraries("${node_name}"
      ${catkin_LIBRARIES}
      depthai::core
      opencv_imgproc
      opencv_highgui
    )
endmacro()

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_library(nodelet_quadcam src/quadcam_nodelet.cpp)

add_dependencies(nodelet_quadcam ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(nodelet_quadcam
  ${catkin_LIBRARIES}
  depthai::opencv
  opencv_imgproc
  opencv_highgui
)

## Install these nodes
install(TARGETS
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
PATTERN ".svn" EXCLUDE
)
install(FILES nodelet_plugins.xml
      DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
