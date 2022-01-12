cmake_minimum_required(VERSION 3.0.2)
project(force_visualization)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
    COMPONENTS
        roscpp
        rospy
        rviz
)

catkin_package(
    INCLUDE_DIRS
        include
    LIBRARIES
        ${PROJECT_NAME}
    CATKIN_DEPENDS
        rviz
)

include_directories(
    include
    SYSTEM
        ${catkin_INCLUDE_DIRS}
)
link_directories(${catkin_LIBRARY_DIRS})

set(CMAKE_AUTOMOC ON)

set(INCLUDE_FILES_QT include/${PROJECT_NAME}/imu_display.h
                     include/${PROJECT_NAME}/imu_visual.h
)

set(SRC_FILES src/imu_display.cpp
              src/imu_visual.cpp)
## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
if(rviz_QT_VERSION VERSION_LESS "5")
  message(STATUS "Using Qt4 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt4 ${rviz_QT_VERSION} EXACT REQUIRED QtCore QtGui)
  ## pull in all required include dirs, define QT_LIBRARIES, etc.
  include(${QT_USE_FILE})
  qt4_wrap_cpp(${INCLUDE_FILES_QT})
else()
  message(STATUS "Using Qt5 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Core Widgets)
  ## make target_link_libraries(${QT_LIBRARIES}) pull in all required dependencies
  set(QT_LIBRARIES Qt5::Widgets)
  QT5_WRAP_CPP(MOC_FILES ${INCLUDE_FILES_QT})
endif()

add_definitions(-DQT_NO_KEYWORDS)
add_definitions(-std=c++11)

add_library(${PROJECT_NAME} ${SRC_FILES}
                            ${MOC_FILES}
)
target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES}
                                      ${catkin_LIBRARIES})

catkin_python_setup()

install(
    DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


install(FILES
  plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

catkin_install_python(PROGRAMS
  src/wrench_publisher.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
