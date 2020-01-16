cmake_minimum_required(VERSION 2.8.3)
project(wtfActs)

## Find catkin dependencies
find_package(catkin REQUIRED COMPONENTS 
  actionlib 
  message_generation
  std_msgs
)

catkin_python_setup()

## Add actions
add_action_files(DIRECTORY action FILES WTF.action)

## Generate messages
generate_messages(DEPENDENCIES std_msgs actionlib_msgs)

## Define catkin exports
catkin_package(
  CATKIN_DEPENDS message_runtime actionlib
  # DEPENDS
)

## Install scripts and executables
install(PROGRAMS
  scripts/wtf_client.py
  scripts/wtf_server.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY share
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)