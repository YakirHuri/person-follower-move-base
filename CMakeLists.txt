cmake_minimum_required(VERSION 3.0.2)
project(person_3d_locator)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)


option(COMPLITE_WITH_JETSON_UTILS"COMPLITE_WITH_JETSON_UTILS" OFF) 

if(COMPLITE_WITH_JETSON_UTILS)
  set(OpenCV_DIR /usr/local/share/OpenCV/)
  message("COMPLITE_WITH_JETSON_UTILS")
endif(COMPLITE_WITH_JETSON_UTILS)  


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  image_transport
  vision_msgs
  cv_bridge
  tf
  tf2_geometry_msgs
  tf2_ros
  depth_image_proc
  geometry_msgs
  std_msgs  
  image_geometry      
  visualization_msgs  
  

)
find_package(realsense2 REQUIRED)
# find_package(OpenCV COMPONENTS tracking REQUIRED PATHS "/usr/local/include/opencv2" "/usr/local/lib")
find_package(OpenCV REQUIRED)

if(COMPLITE_WITH_JETSON_UTILS)
  message("COMPLITE_WITH_JETSON_UTILS")
  find_package(jetson-utils REQUIRED)
  find_package(jetson-inference REQUIRED)
  find_package(CUDA REQUIRED)
endif(COMPLITE_WITH_JETSON_UTILS)  

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

# message(STATUS "OpenCV library status:")
# message(STATUS "    config: ${OpenCV_DIR}")
# message(STATUS "    version: ${OpenCV_VERSION}")
# message(STATUS "    libraries: ${OpenCV_LIBS}")
# message(STATUS "    libraries: ${OpenCV_LIBRARIES}")
# message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")

catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES person_3d_locator
 CATKIN_DEPENDS roscpp rospy
 tf2_geometry_msgs
 tf2_ros
 DEPENDS OpenCV
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  
  if(COMPLITE_WITH_JETSON_UTILS)
    ${CUDA_INCLUDE_DIRS}
  endif(COMPLITE_WITH_JETSON_UTILS)  
  
)

message(STATUS "${OpenCV_INCLUDE_DIRS},
${catkin_INCLUDE_DIRS}, 
${CUDA_INCLUDE_DIRS}")

if(COMPLITE_WITH_JETSON_UTILS)

  add_executable(person_3d_locator_node
      src/image_converter.cpp
      src/detectnet.cpp
      src/realsense2_camera.cpp
      src/person_3d_locator.cpp
      src/person_3d_locator_node.cpp
  )
else()
  add_executable(person_3d_locator_node
    src/realsense2_camera.cpp
    src/person_3d_locator.cpp
    src/person_3d_locator_node.cpp
    )
  endif(COMPLITE_WITH_JETSON_UTILS)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


if(COMPLITE_WITH_JETSON_UTILS)
    message("COMPLITE_WITH_JETSON_UTILS")
target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
   ${OpenCV_LIBS}
   ${realsense2_LIBRARY}
   jetson-inference    
)
else()
target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
   ${OpenCV_LIBS}
   ${realsense2_LIBRARY}
)
endif(COMPLITE_WITH_JETSON_UTILS)

unset(COMPLITE_WITH_JETSON_UTILS CACHE) # <---- this is the important!!