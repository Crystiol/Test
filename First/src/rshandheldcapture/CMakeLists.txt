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

MESSAGE(STATUS "This is BINARY dir " ${RSHandheldCapture_BINARY_DIR})
################################
##  CONFIGURE THIS FILE HERE  ##
################################
#set(CMAKE_PREFIX_PATH "/home/tuxu/Qt/5.15.2/gcc_64")

#SET(CMAKE_BUILD_TYPE "Debug”)
list(APPEND CMAKE_PREFIX_PATH "/home/tuxu/Qt/5.15.2/gcc_64/lib/cmake")

set(ROS_DEPENDS
  roscpp
  std_msgs
  image_transport
  cv_bridge
  )

set(EXTERNAL_DEPENDS
  Qt5Widgets
  Qt5Network
  Qt5Qml
  Qt5Gui
  Qt5Quick
  Qt5Core
  )

set(EXTERNAL_DEPENDS_LIB
  Qt5::Widgets
  Qt5::Network
  Qt5::Qml
  Qt5::Gui
  Qt5::Quick
  Qt5::Core
vtkGUISupportQtQuick-9.3
vtkRenderingContextOpenGL2-9.3
vtkTestingRendering-9.3
vtkInteractionWidgets-9.3
vtkRenderingOpenGL2-9.3
vtkRenderingHyperTreeGrid-9.3
vtkRenderingContext2D-9.3
vtkIOImage-9.3
vtkRenderingFreeType-9.3
vtkfreetype-9.3
vtkzlib-9.3
vtkRenderingUI-9.3
vtkRenderingCore-9.3
vtkFiltersSources-9.3
vtkFiltersGeneral-9.3
vtkFiltersCore-9.3
vtkCommonExecutionModel-9.3
vtkCommonDataModel-9.3
vtkCommonTransforms-9.3
vtkCommonMisc-9.3
vtkCommonMath-9.3
vtkkissfft-9.3
vtkCommonCore-9.3
vtktoken-9.3
vtksys-9.3
vtkInteractionStyle-9.3
vtkRenderingAnnotation-9.3
vtkCommonComputationalGeometry-9.3
vtkCommonColor-9.3
RSSceneViewer
)

set(INCLUDE_DIRECTORIES
)

add_compile_options(-std=c++14)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

foreach(DEPEND ${EXTERNAL_DEPENDS})
  find_package(${DEPEND} REQUIRED PATHS "/home/tuxu/Qt/5.15.2/gcc_64")
endforeach(DEPEND)

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
  /usr/local/include/vtk-9.3
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

set(NODE_NAME Viewer)

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

