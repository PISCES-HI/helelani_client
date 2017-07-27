cmake_minimum_required(VERSION 2.8.3)
project(helelani_client)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS rqt_gui rqt_gui_cpp)
find_package(Boost REQUIRED COMPONENTS system)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS src
  LIBRARIES helelani_client
  CATKIN_DEPENDS roscpp std_msgs rqt_gui_cpp helelani_common
)

###########
## Build ##
###########

set(CMAKE_AUTOMOC ON)
find_package(Qt4 COMPONENTS QtCore QtGui QtSvg REQUIRED)
include(cmake/FindQtAV.cmake)
if(NOT ${QtAV_FOUND})
  message(FATAL_ERROR "QtAV must be installed")
endif()
include(${QT_USE_FILE})
set(QT_LIBRARIES ${QT_QTCORE_LIBRARY} ${QT_QTGUI_LIBRARY} ${QT_QTSVG_LIBRARY})

set(WORKSPACE_INCLUDES "")
foreach(f ${CATKIN_WORKSPACES})
  list(APPEND WORKSPACE_INCLUDES "${f}/include")
endforeach(f)
include_directories(src
                    ${CMAKE_CURRENT_BINARY_DIR}
                    ${Boost_INCLUDE_DIRS}
                    ${catkin_INCLUDE_DIRS}
                    ${WORKSPACE_INCLUDES}
                    ${QTAV_INCLUDE_DIRS}
                    ${QTAVWIDGETS_INCLUDE_DIRS})

QT4_WRAP_UI(UI_GENERATED_HEADERS src/HelelaniIMU.ui src/HelelaniCommand.ui
            src/HelelaniIPCamViewer.ui src/HelelaniCameraControl.ui)
QT4_ADD_RESOURCES(RES_SOURCES resources/resources.qrc)
add_library(${PROJECT_NAME} ${UI_GENERATED_HEADERS} ${RES_SOURCES}
            src/HelelaniIMU.cpp src/HelelaniIMU.h
            src/AttitudeWidget.cpp src/AttitudeWidget.h
            src/HeadingWidget.cpp src/HeadingWidget.h
            src/HelelaniCommand.cpp src/HelelaniCommand.h
            src/HelelaniIPCamViewer.cpp src/HelelaniIPCamViewer.h
            src/HelelaniCameraControl.cpp src/HelelaniCameraControl.h
            src/CameraControlWidget.cpp src/CameraControlWidget.h
            src/SVGHelper.h src/SubVideoRendererWidget.h src/CommandLineEdit.h)

## Add cmake target dependencies of the library
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${QT_LIBRARIES}
                      ${QTAV_LIBRARIES} ${QTAVWIDGETS_LIBRARIES}
)

#############
## Install ##
#############

catkin_python_setup()

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS scripts/helelani_imu scripts/helelani_command
        scripts/helelani_ip_cam scripts/helelani_camera_control
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
