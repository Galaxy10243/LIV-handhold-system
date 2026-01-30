cmake_minimum_required(VERSION 3.0.2)
project(take_picture_ros)
add_compile_options(-std=c++17)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  cv_bridge
)
find_package(OpenCV 4 REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES take_picture_ros
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(camera_ros_driver_three src/camera_ros_driver_three.cpp)
add_executable(camera_ros_driver_one src/camera_ros_driver_one.cpp)
add_executable(camera_ros_driver_one_with_sync src/camera_ros_driver_one_with_sync.cpp)
add_executable(camera_spacebar_capture src/camera_spacebar_capture.cpp)
add_executable(take_picture_JL src/take_picture_JL.cpp)
#add_executable(Subscriber src/Subscriber.cpp)

target_link_libraries(camera_ros_driver_three
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)
target_link_libraries(camera_ros_driver_one
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)
target_link_libraries(camera_ros_driver_one_with_sync
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)
target_link_libraries(camera_spacebar_capture
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)
target_link_libraries(take_picture_JL
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)
#target_link_libraries(Subscriber
#  ${catkin_LIBRARIES}
#  ${OpenCV_LIBS}
#)

