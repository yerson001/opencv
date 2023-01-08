cmake_minimum_required(VERSION 3.5)

set(CMAKE_CXX_STANDARD 17)

PROJECT(VP)

find_package( OpenCV REQUIRED ) #find_package( OpenCV 4 REQUIRED )

include_directories( ${OpenCV_INCLUDE_DIRS})

MACRO(add_example name)
  ADD_EXECUTABLE(${name} src/${name}.cpp)
  TARGET_LINK_LIBRARIES(${name} ${OpenCV_LIBS})
ENDMACRO()

add_example(main)
#add_example(PuntoFuga)
#add_example(vp)
