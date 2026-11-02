cmake_minimum_required(VERSION 3.16)

project(foray_navigation VERSION 0.1.0 LANGUAGES CXX)

# 本仓只依赖标准库：不依赖 ROS 2、不依赖任何业务仓（见 .foray-layer）。

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_library(foray_navigation
  src/grid_map.cpp
  src/costmap.cpp
  src/astar_planner.cpp
  src/pursuit_controller.cpp
)
add_library(Foray::Navigation ALIAS foray_navigation)

target_include_directories(foray_navigation
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_compile_features(foray_navigation PUBLIC cxx_std_17)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(foray_navigation PRIVATE -Wall -Wextra -Wpedantic)
endif()

# ---- 测试：每个模块一个独立可执行文件（不依赖外部测试框架）----
enable_testing()

add_executable(grid_map_test test/grid_map_test.cpp)
target_link_libraries(grid_map_test PRIVATE foray_navigation)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(grid_map_test PRIVATE -Wall -Wextra -Wpedantic)
endif()
add_test(NAME grid_map_test COMMAND grid_map_test)

add_executable(costmap_test test/costmap_test.cpp)
target_link_libraries(costmap_test PRIVATE foray_navigation)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(costmap_test PRIVATE -Wall -Wextra -Wpedantic)
endif()
add_test(NAME costmap_test COMMAND costmap_test)

add_executable(astar_planner_test test/astar_planner_test.cpp)
target_link_libraries(astar_planner_test PRIVATE foray_navigation)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(astar_planner_test PRIVATE -Wall -Wextra -Wpedantic)
endif()
add_test(NAME astar_planner_test COMMAND astar_planner_test)

add_executable(pursuit_controller_test test/pursuit_controller_test.cpp)
target_link_libraries(pursuit_controller_test PRIVATE foray_navigation)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(pursuit_controller_test PRIVATE -Wall -Wextra -Wpedantic)
endif()
add_test(NAME pursuit_controller_test COMMAND pursuit_controller_test)

add_executable(types_test test/types_test.cpp)
target_link_libraries(types_test PRIVATE foray_navigation)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(types_test PRIVATE -Wall -Wextra -Wpedantic)
endif()
add_test(NAME types_test COMMAND types_test)

include(GNUInstallDirs)
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(TARGETS foray_navigation EXPORT foray_navigationTargets)
install(EXPORT foray_navigationTargets NAMESPACE Foray:: DESTINATION lib/cmake/foray_navigation)
