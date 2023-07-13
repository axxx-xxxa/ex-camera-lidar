cmake_minimum_required(VERSION 2.8.3)
project(lidar_camera_calib)

set( CMAKE_BUILD_TYPE "Release" )
set(CMAKE_CXX_FLAGS "-O2 -Wall ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
        pcl_ros
        roscpp
        geometry_msgs
        std_msgs
        sensor_msgs
        tf
        cv_bridge
        image_transport
        )


#list(REMOVE_ITEM catkin_INCLUDE_DIRS "/usr/include/opencv4")
#set(OpenCV_INCLUDE_DIRS /home/ubuntu/opencv/opencv-3.4.7/include/opencv2)
# set(OpenCV_DIR "/usr/local/lib/cmake/opencv4")
set(OpenCV_DIR /usr/share/OpenCV/)

find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)
find_package(Eigen3 QUIET)

catkin_package(
        CATKIN_DEPENDS
        pcl_ros
        roscpp
        geometry_msgs
        std_msgs
        sensor_msgs
        tf
        INCLUDE_DIRS include
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
)
link_directories(${OpenCV_LIBRARY_DIRS})

#Euclidean Cluster
add_executable(lidar_camera_calib
        src/lidar_camera_calib.cpp
        # src/ccalib.cpp
        # src/multicalib.cpp
        # src/omnidir.cpp
        # src/randpattern.cpp
        )

target_link_libraries(lidar_camera_calib
        ${OpenCV_LIBRARIES}
        ${catkin_LIBRARIES}
        ${CERES_LIBRARIES}
)

add_dependencies(lidar_camera_calib
        ${catkin_EXPORTED_TARGETS}
        )


install(DIRECTORY include/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(TARGETS
        lidar_camera_calib
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
        PATTERN ".svn" EXCLUDE)
