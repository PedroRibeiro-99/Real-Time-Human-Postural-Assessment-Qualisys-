cmake_minimum_required(VERSION 3.0.2)
project(qualysis_data)

## Compile as C++14
add_compile_options(-std=c++14)

## Paths and options
set(SIMULATION 1)
set(RULA_ASSESSMENT 1)
set(COPPELIA_PATH /home/$ENV{USER}/CoppeliaSim_Edu_V4_2_0_Ubuntu18_04)
set(COPPELIA_SCENARIO_PATH ${PROJECT_SOURCE_DIR}/Operator_qualisys.ttt)
set(FILES_PATH ${PROJECT_SOURCE_DIR}/files/tsv_files)

# Configuration header
configure_file(
  "${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/config.hpp.in"
  "${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/config.hpp"
)

## Find catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  tf
  vrep_common
)

## Find Qt5 packages
find_package(Qt5 REQUIRED COMPONENTS Core Gui Widgets Xml)

## Find qualisys_cpp_sdk
set(qualisys_cpp_sdk_DIR "/home/pedro/qualisys_cpp_sdk/build/CMakeFiles/Export/lib/qualisys_cpp_sdk")
find_package(qualisys_cpp_sdk REQUIRED)

## Catkin package
catkin_package(
  CATKIN_DEPENDS vrep_common geometry_msgs roscpp tf
  INCLUDE_DIRS include
  LIBRARIES qualisys_cpp_sdk
)

## Include directories
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Qt5Core_INCLUDE_DIRS}
  ${Qt5Gui_INCLUDE_DIRS}
  ${Qt5Widgets_INCLUDE_DIRS}
  ${Qt5Xml_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}  # <--- Needed for generated ui_*.h files
)

# -- QCustomPlot
add_definitions(-DQCUSTOMPLOT_USE_LIBRARY)
set(QCUSTOMPLOT_HOME /home/$ENV{USER}/qcustomplot)
include_directories(
    ${QCUSTOMPLOT_HOME}/include
)
link_directories(
    ${QCUSTOMPLOT_HOME}/build
)

## Qt5 processing
file(GLOB QT_FORMS ui/*.ui)
file(GLOB QT_RESOURCES resources/*.qrc)
file(GLOB_RECURSE QT_MOC include/${PROJECT_NAME}/*.h)

qt5_wrap_ui(QT_FORMS_HPP ${QT_FORMS})
qt5_add_resources(QT_RESOURCES_CPP ${QT_RESOURCES})
qt5_wrap_cpp(QT_MOC_HPP ${QT_MOC})

## Source files
file(GLOB_RECURSE QT_SOURCES src/*.cpp)

## Executable
add_executable(${PROJECT_NAME} ${QT_SOURCES} ${QT_FORMS_HPP} ${QT_RESOURCES_CPP} ${QT_MOC_HPP})

## Link libraries
target_link_libraries(${PROJECT_NAME}
  Qt5::Core Qt5::Gui Qt5::Widgets Qt5::Xml
  ${catkin_LIBRARIES}
  qualisys_cpp_sdk
)

## QCustomPlot linking
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
  target_link_libraries(${PROJECT_NAME} qcustomplotd)
else()
  target_link_libraries(${PROJECT_NAME} qcustomplot)
endif()

