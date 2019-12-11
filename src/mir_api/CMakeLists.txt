cmake_minimum_required(VERSION 3.10.2)
project(mir_api)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  mir_api_action.srv
)
## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
 )
catkin_package(
   CATKIN_DEPENDS message_runtime
)
