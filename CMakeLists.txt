cmake_minimum_required(VERSION 2.8)
project(mofs)

set(CMAKE_CXX_FLAGS "-std=gnu++0x ${CMAKE_CXX_FLAGS}")

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

set( MESSAGE_DEPENDENCIES
  std_msgs
)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  rosbag
  std_srvs
  message_generation
  ${MESSAGE_DEPENDENCIES}
)


add_message_files(DIRECTORY msg
  FILES
  input.msg
  output.msg
)

add_service_files(DIRECTORY srv
  FILES
  FuzzyControl_IO.srv
)

generate_messages(
  DEPENDENCIES 
  ${MESSAGE_DEPENDENCIES}
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS message_runtime roscpp rospy std_msgs std_srvs rosbag
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
)

set(MOFS_SOURCE_FILES
	src/MOFSFuncTri.cpp 
	src/MOFSMCDefuzz.cpp 
	src/MOFSModel.cpp
	src/MOFSRule.cpp 
	src/MOFSVar.cpp
	src/mofs.cpp 
)

#Create a Fuzzy controller instance
add_executable(mofs ${MOFS_SOURCE_FILES})

target_link_libraries(mofs
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

add_dependencies(mofs ${catkin_EXPORTED_TARGETS})
add_dependencies(mofs ${PROJECT_NAME}_generate_messages_cpp)

install(TARGETS mofs
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

