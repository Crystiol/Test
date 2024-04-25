cmake_minimum_required(VERSION 3.14)

project(RSSceneViewer LANGUAGES CXX)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_PREFIX_PATH "/home/tuxu/Qt/5.15.2/gcc_64")

find_package(QT NAMES Qt6 Qt5 REQUIRED COMPONENTS Core)
find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Core)

include_directories(/usr/local/include/vtk-9.3)

add_library(RSSceneViewer SHARED
  RSSceneViewer_global.h
  RSSceneViewer.cpp
  RSSceneViewer.h
  RSMultiTouchInteractorStyle.h
  RSMultiTouchInteractorStyle.cpp
)

target_link_libraries(RSSceneViewer PRIVATE 
Qt${QT_VERSION_MAJOR}::Core
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
)

target_compile_definitions(RSSceneViewer PRIVATE RSSCENEVIEWER_LIBRARY)

