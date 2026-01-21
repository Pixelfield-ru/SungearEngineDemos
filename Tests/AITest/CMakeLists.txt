include($ENV{SUNGEAR_SOURCES_ROOT}/cmake/utils.cmake)

project(SGAITest)

set(SGPROJECT_SOURCES Main.cpp Main.h Sources/Components/Cart.h Sources/Actions/FindCart.cpp Sources/Actions/PickupCart.cpp Sources/States.cpp)

add_executable(${PROJECT_NAME} ${SGPROJECT_SOURCES})

target_include_directories(${PROJECT_NAME} PRIVATE Sources ${SungearEngine_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} PRIVATE ${SungearEngine_LIBS})

copy_sgcore_dlls(${PROJECT_NAME})