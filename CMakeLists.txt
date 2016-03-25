cmake_minimum_required(VERSION 2.6)

project(PuzzleSolver)
find_package( OpenCV REQUIRED  )

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall")
set(RESULT_DIR results)
file(GLOB SRC . GeneratedResults.cpp PuzzleController.cpp PuzzleGenerator.cpp View.cpp)
file(GLOB HDR . constheader.h  generatedresults.h  generator.h  puzzlecontroller.h  view.h)

add_executable(PuzzleSolver ${SRC} ${HDR})
target_link_libraries(PuzzleSolver ${OpenCV_LIBS})

file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/${RESULT_DIR})
