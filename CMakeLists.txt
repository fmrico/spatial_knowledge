cmake_minimum_required(VERSION 2.8.3)
project(spatial_knowledge)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  rosplan_knowledge_msgs
  octomap_msgs
  octomap_ros
  octomap_rviz_plugins
	mongodb_store
  roscpp
  rospy
	erl_planning
)
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES sk_client
  CATKIN_DEPENDS rosplan_knowledge_msgs octomap_msgs octomap_ros octomap_rviz_plugins roscpp rospy erl_planning
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(sk_client
   src/sk_client.cpp
 )

add_dependencies(sk_client ${sk_client_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(sk_map_node src/sk_map_node.cpp)
add_dependencies(sk_map_node ${sk_client_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(sk_map_node
   ${catkin_LIBRARIES}
	 sk_client
 )

add_executable(sk_people_node src/sk_people_node.cpp)
add_dependencies(sk_people_node ${sk_client_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(sk_people_node
   ${catkin_LIBRARIES}
	 sk_client
 )

add_executable(sk_object_node src/sk_object_node.cpp)
add_dependencies(sk_object_node ${sk_client_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(sk_object_node
   ${catkin_LIBRARIES}
	 sk_client
 )

add_executable(person_at_node src/person_at_node.cpp)
target_link_libraries(person_at_node
   ${catkin_LIBRARIES}

 )
