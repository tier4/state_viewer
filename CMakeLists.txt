cmake_minimum_required(VERSION 2.8.3)
project(state_viewer)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

#Qt5 configulation
set(CMAKE_AUTOMOC ON)
if(DEFINED ${rviz_QT_VERSION})
set(RVIZ_VERSION ${rviz_QT_VERSION} EXACT)
else()
set(RVIZ_VERSION)
endif()

find_package(Qt5 ${RVIZ_VERSION} REQUIRED
  Core
  Widgets
)
set(QT_LIBRARIES Qt5::Widgets)
add_definitions(-DQT_NO_KEYWORDS)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(state_viewer_node
  src/state_viewer.cpp
  src/state_viewer_node.cpp
)

target_link_libraries(state_viewer_node
  ${catkin_LIBRARIES}
  ${QT_LIBRARIES}
)

install(TARGETS state_viewer_node
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
        PATTERN ".svn" EXCLUDE
)
