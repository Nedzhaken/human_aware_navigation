cmake_minimum_required(VERSION 2.8.3)
project(object3d_detector_gpu)

find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs visualization_msgs people_msgs pcl_conversions pcl_ros)

find_package(PCL REQUIRED)
find_package(CUDA REQUIRED)

include_directories(include ${catkin_INCLUDE_DIRS} ${CUDA_INCLUDE_DIRS})

catkin_package(LIBRARIES svm)

add_executable(object3d_detector_gpu src/object3d_detector_gpu.cpp)
target_link_libraries(object3d_detector_gpu ${catkin_LIBRARIES} ${CUDA_LIBRARIES} ${PROJECT_SOURCE_DIR}/lib/libcudacluster.so)
if(catkin_EXPORTED_TARGETS)
  add_dependencies(object3d_detector_gpu ${catkin_EXPORTED_TARGETS})
endif()
