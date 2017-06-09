cmake_minimum_required(VERSION 2.8.3)
project(helelani_client)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS rqt_gui rqt_gui_cpp rqt_plot)
find_package(Boost REQUIRED COMPONENTS system)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS src
  LIBRARIES helelani_client
  CATKIN_DEPENDS roscpp std_msgs rqt rqt_plot rviz helelani_common
)

###########
## Build ##
###########

set(CMAKE_AUTOMOC ON)
find_package(Qt5 REQUIRED COMPONENTS Core Widgets)
set(QT_LIBRARIES Qt5::Widgets)

include_directories(src
                    ${CMAKE_BINARY_DIR}
                    ${Boost_INCLUDE_DIRS}
                    ${catkin_INCLUDE_DIRS}
                    ${CATKIN_WORKSPACES}/include)

QT5_WRAP_UI(UI_GENERATED_HEADERS src/HelelaniClient.ui)
add_library(${PROJECT_NAME} ${UI_GENERATED_HEADERS} src/HelelaniClient.cpp src/HelelaniClient.h)

## Add cmake target dependencies of the library
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${QT_LIBRARIES})

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
