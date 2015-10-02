##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.8.3)
project(eyetracker_talker)

##############################################################################
# Packages
##############################################################################

find_package(catkin
  REQUIRED COMPONENTS roslaunch roscpp std_msgs geometry_msgs message_generation
)

##############################################################################
# ROS Messages
##############################################################################

add_message_files(
  FILES
  GazeData.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

##############################################################################
# Catkin
##############################################################################

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES eyetracker_talker
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs message_runtime
)

include_directories(include ${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

##############################################################################
# ROS Launch check
##############################################################################

roslaunch_add_file_check(launch)

##############################################################################
# Tobii EyeX
##############################################################################

# set the tobii root include dir
set(TOBII_EYEX_INCLUDE_DIR ${CMAKE_CURRENT_LIST_DIR}/include/tobii/ CACHE STRING "Build mode type.")

# searching for library files
find_library(TOBII_EYEX_LIB Tobii.EyeX.Client
  HINTS ${CMAKE_CURRENT_LIST_DIR}/lib/x86/)

find_path(TOBII_EYEX_DLL_DIR Tobii.EyeX.Client.dll
  HINTS ${CMAKE_CURRENT_LIST_DIR}/lib/x86/)

if (TOBII_EYEX_INCLUDE_DIR AND TOBII_EYEX_LIB AND TOBII_EYEX_DLL_DIR)
    # set the appropriate flag
    set(TOBII_EYEX_FOUND TRUE)

    # add definition for conditional compilation
    add_definitions(-DHAVE_LIBEYEX=1)

    # include the lib directory
    include_directories(${TOBII_EYEX_INCLUDE_DIR})

    # append the lib directory to a variable
    set(ALL_LIBS ${ALL_LIBS} ${TOBII_EYEX_LIB})
endif ()

##############################################################################
# Sources
##############################################################################

add_library(eyetracker_talker src/lib/eyetracker_talker.cpp)
target_link_libraries(eyetracker_talker ${ALL_LIBS} ${catkin_LIBRARIES})

add_dependencies(eyetracker_talker eyetracker_talker_generate_messages_cpp)

set_target_properties(eyetracker_talker
  PROPERTIES OUTPUT_NAME libeyetrackertalker
)

install(TARGETS eyetracker_talker
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(DIRECTORY include/eyetracker_talker/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h")

if (TOBII_EYEX_DLL_DIR)
    install(FILES ${TOBII_EYEX_DLL_DIR}/Tobii.EyeX.Client.dll
      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )
endif ()

##############################################################################
# Example
##############################################################################

add_executable(demo_eyetracker_talker src/main.cpp)
target_link_libraries(demo_eyetracker_talker eyetracker_talker ${catkin_LIBRARIES})

set_target_properties(demo_eyetracker_talker
  PROPERTIES OUTPUT_NAME eyetracker_talker
)

install(TARGETS demo_eyetracker_talker 
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

##############################################################################
# ROS Launch files
##############################################################################

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE)

##############################################################################
# Test
##############################################################################

#if(CATKIN_ENABLE_TESTING)
#  find_package(catkin COMPONENTS rostest rosunit)
#  include_directories(${GTEST_INCLUDE_DIRS})
#  catkin_add_gtest(${PROJECT_NAME}-test test/test_${PROJECT_NAME}.cpp)
#  if(TARGET ${PROJECT_NAME}-test)
#    target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME} ${GTEST_LIBRARIES})
#  endif()
#endif()
