cmake_minimum_required(VERSION 3.22)
project(SungearEngineDemos)
set(CMAKE_CXX_STANDARD 23)

set(SungearEngineEditor_INCLUDE_EXTERNALS ON)
add_definitions(-DSUNGEAR_DEMOS_ROOT=${CMAKE_SOURCE_DIR})

# because of atlr4-runtime macro ERROR
add_definitions(-DNOGDI)

include($ENV{SUNGEAR_SOURCES_ROOT}/cmake/SungearEngineInclude.cmake)

add_subdirectory(Tests/AITest)
add_subdirectory(Tests/ModelDraw)
add_subdirectory(Tests/Physics)
add_subdirectory(Tests/Terrain)
add_subdirectory(Tests/UI)