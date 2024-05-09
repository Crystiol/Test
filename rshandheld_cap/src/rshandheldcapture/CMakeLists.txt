# MIT License

# Copyright (c) 2022 Kristopher Krasnosky

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.


cmake_minimum_required(VERSION 2.8.3)

#get_filename_component(PACKAGE_NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)
#message("Creating Package: ${PACKAGE_NAME}")

project(rshandheld_capture)  ## this package name is the name of the directory this cmake file is in

MESSAGE(STATUS "This is BINARY dir " ${CMAKE_PREFIX_PATH})
################################
##  CONFIGURE THIS FILE HERE  ##
################################
#set(CMAKE_PREFIX_PATH "/home/tuxu/Qt/5.15.2/gcc_64" ${CMAKE_PREFIX_PATH})

#SET(CMAKE_BUILD_TYPE "Debug”)
#list(APPEND CMAKE_PREFIX_PATH "/home/tuxu/Qt/5.15.2/gcc_64")

set(ROS_DEPENDS
  roscpp
  std_msgs
  image_transport
  cv_bridge
  std_srvs
  )

set(EXTERNAL_DEPENDS
  Qt5Widgets
  Qt5Network
  Qt5Qml
  Qt5Gui
  Qt5Quick
  Qt5Core
  Qt5OpenGL
  )

set(EXTERNAL_DEPENDS_LIB
  Qt5::Widgets
  Qt5::Network
  Qt5::Qml
  Qt5::Gui
  Qt5::Quick
  Qt5::Core
  Qt5::OpenGL
  vtkGUISupportQt-7.1
vtkRenderingContextOpenGL2-7.1
vtkTestingRendering-7.1
vtkInteractionWidgets-7.1
vtkRenderingOpenGL2-7.1
vtkRenderingContext2D-7.1
vtkIOImage-7.1
vtkRenderingFreeType-7.1
vtkRenderingCore-7.1
vtkFiltersSources-7.1
vtkFiltersGeneral-7.1
vtkFiltersCore-7.1
vtkCommonExecutionModel-7.1
vtkCommonDataModel-7.1
vtkCommonTransforms-7.1
vtkCommonMisc-7.1
vtkCommonMath-7.1
vtkCommonCore-7.1
vtksys-7.1
vtkInteractionStyle-7.1
vtkRenderingAnnotation-7.1
vtkCommonComputationalGeometry-7.1
vtkCommonColor-7.1
RSSceneViewer
)

set(INCLUDE_DIRECTORIES
)

add_compile_options(-std=c++14)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

foreach(DEPEND ${EXTERNAL_DEPENDS})
  find_package(${DEPEND} REQUIRED)
endforeach(DEPEND)

#find_package(Qt${QT_VERSION_MAJOR} 5.15.2 EXACT REQUIRED COMPONENTS Core Quick Gui Qml Widgets)

find_package(catkin REQUIRED COMPONENTS ${ROS_DEPENDS})

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    ${ROS_DEPENDS}
  DEPENDS
    ${EXTERNAL_DEPENDS}
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${INCLUDE_DIRECTORIES}
  ${CATKIN_DEVEL_PREFIX}/include
  ${CATKIN_DEVEL_PREFIX}/../3rdParty/RSSceneViewer/include
  /usr/include/vtk-7.1
)

link_directories(
${CATKIN_DEVEL_PREFIX}/../3rdParty/RSSceneViewer/lib
)

###########
## Build ##
###########

file(GLOB HDRS
  "include/${PROJECT_NAME}/*.h"
)
file(GLOB SRCS
  "src/*.cpp"
  "src/*.hpp"
)
file(GLOB QMLS
  "src/*.qml"
)
file(GLOB QRC
  "src/*.qrc"
)
file(GLOB_RECURSE UI
  "src/*.ui"
)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

#message("Adding ${PROJECT_NAME} UI files: " )
#foreach(source ${UI})
#    message("  - " ${source})
#endforeach(source)

#make them into headers
#qt5_wrap_ui (UI_HDRS  ${UI})

#message("Generating ${PROJECT_NAME} UI Headers: " )
#foreach(source ${UI_HDRS})
#    message("  - " ${source})
#endforeach(source)

message("buliding ${PROJECT_NAME} node: ")

set(NODE_NAME viewer)

add_executable(${NODE_NAME} ${HDRS} ${SRCS} ${UI} ${QRC} ${QML})
add_dependencies(${NODE_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries (${NODE_NAME} ${catkin_LIBRARIES}  ${EXTERNAL_DEPENDS_LIB} )
target_include_directories(${NODE_NAME} PRIVATE "include/${PROJECT_NAME}/")

list(APPEND NODE_LIST ${NODE_NAME})

install(TARGETS ${NODE_LIST}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  )

