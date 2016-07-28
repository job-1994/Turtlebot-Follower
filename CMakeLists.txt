cmake_minimum_required(VERSION 2.8.3)
project(followpkg)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  cv_bridge
  image_transport
  sensor_msgs 
)

set(CURSES_LIBRARY "/usr/lib/x86_64-linux-gnu/libncurses.so")
set(CURSES_INCLUDE_PATH "/usr/include")

find_package(Curses REQUIRED)

catkin_package(

)

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)


add_executable(Follow src/Follow.cpp)
	target_link_libraries(Follow ${catkin_LIBRARIES})
	add_dependencies(Follow rviz_generate_messages_cpp)

add_executable(FollowVideo src/FollowVideo.cpp)
	target_link_libraries(FollowVideo ${catkin_LIBRARIES})
	add_dependencies(FollowVideo rviz_generate_messages_cpp)


find_package(OpenCV)
	include_directories(${OpenCV_INCLUDE_DIRS})
	target_link_libraries(Follow ${OpenCV_LIBRARIES})
	target_link_libraries(FollowVideo ${OpenCV_LIBRARIES})
 