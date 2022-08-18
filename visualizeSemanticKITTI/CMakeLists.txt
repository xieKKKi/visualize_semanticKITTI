cmake_minimum_required(VERSION 3.0.2)
project(visualizeSemanticKITTI)

set(CMAKE_CXX_FLAGS "-std=c++17")

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED QUIET)
find_package(Boost REQUIRED COMPONENTS)
find_package(OpenMP REQUIRED)


catkin_package(
        CATKIN_DEPENDS cv_bridge image_transport roscpp sensor_msgs std_msgs
)


include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
        ${Boost_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
)


add_executable(visualize src/visualize.cpp
        src/utility.cpp)
target_compile_options(visualize PRIVATE ${OpenMP_CXX_FLAGS})
target_link_libraries(visualize ${catkin_LIBRARIES}
        ${OpenCV_LIBS}
        ${Boost_LIBRARIES}
        ${PCL_LIBRARIES}
        ${OpenMP_CXX_FLAGS})
