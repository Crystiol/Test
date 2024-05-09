cmake_minimum_required(VERSION 3.14)

project(RSSceneViewer LANGUAGES CXX)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
#set(CMAKE_PREFIX_PATH "/home/tuxu/Qt/5.15.2/gcc_64")

find_package(QT NAMES Qt6 Qt5 REQUIRED COMPONENTS Core OpenGL)
find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Core OpenGL)

include_directories(/usr/include/vtk-7.1)

add_library(RSSceneViewer SHARED
  RSSceneViewer_global.h
  RSSceneViewer.cpp
  RSSceneViewer.h
  #RSMultiTouchInteractorStyle.h
  #RSMultiTouchInteractorStyle.cpp
)

target_link_libraries(RSSceneViewer PRIVATE 
Qt${QT_VERSION_MAJOR}::Core
Qt${QT_VERSION_MAJOR}::OpenGL
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
)

target_compile_definitions(RSSceneViewer PRIVATE RSSCENEVIEWER_LIBRARY)

