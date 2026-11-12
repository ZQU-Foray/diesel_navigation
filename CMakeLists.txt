cmake_minimum_required(VERSION 3.16)

project(diesel_navigation VERSION 0.1.0 LANGUAGES CXX)

# 本仓只依赖标准库：不依赖 ROS 2、不依赖任何业务仓（见 .diesel-layer）。

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_library(diesel_navigation
  src/grid_map.cpp
  src/costmap.cpp
  src/astar_planner.cpp
  src/pursuit_controller.cpp
  src/nav_engine.cpp
  src/params.cpp
  src/regions.cpp
)
add_library(diesel::Navigation ALIAS diesel_navigation)

target_include_directories(diesel_navigation
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_compile_features(diesel_navigation PUBLIC cxx_std_17)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(diesel_navigation PRIVATE -Wall -Wextra -Wpedantic -Werror)
endif()

# ---- 测试：每个模块一个独立可执行文件（不依赖外部测试框架）----
enable_testing()

add_executable(grid_map_test test/grid_map_test.cpp)
target_link_libraries(grid_map_test PRIVATE diesel_navigation)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(grid_map_test PRIVATE -Wall -Wextra -Wpedantic -Werror)
endif()
add_test(NAME grid_map_test COMMAND grid_map_test)

add_executable(costmap_test test/costmap_test.cpp)
target_link_libraries(costmap_test PRIVATE diesel_navigation)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(costmap_test PRIVATE -Wall -Wextra -Wpedantic -Werror)
endif()
add_test(NAME costmap_test COMMAND costmap_test)

add_executable(astar_planner_test test/astar_planner_test.cpp)
target_link_libraries(astar_planner_test PRIVATE diesel_navigation)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(astar_planner_test PRIVATE -Wall -Wextra -Wpedantic -Werror)
endif()
add_test(NAME astar_planner_test COMMAND astar_planner_test)

add_executable(pursuit_controller_test test/pursuit_controller_test.cpp)
target_link_libraries(pursuit_controller_test PRIVATE diesel_navigation)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(pursuit_controller_test PRIVATE -Wall -Wextra -Wpedantic -Werror)
endif()
add_test(NAME pursuit_controller_test COMMAND pursuit_controller_test)

add_executable(nav_engine_test test/nav_engine_test.cpp)
target_link_libraries(nav_engine_test PRIVATE diesel_navigation)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(nav_engine_test PRIVATE -Wall -Wextra -Wpedantic -Werror)
endif()
add_test(NAME nav_engine_test COMMAND nav_engine_test)

# ---- 演示程序（不依赖 ROS）：把规划与跟踪串起来跑一遍 ----
add_executable(nav_demo apps/nav_demo.cpp)
target_link_libraries(nav_demo PRIVATE diesel_navigation)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(nav_demo PRIVATE -Wall -Wextra -Wpedantic -Werror)
endif()

add_executable(nav_trace apps/nav_trace.cpp)
target_link_libraries(nav_trace PRIVATE diesel_navigation)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(nav_trace PRIVATE -Wall -Wextra -Wpedantic -Werror)
endif()

add_executable(nav_plot apps/nav_plot.cpp)
target_link_libraries(nav_plot PRIVATE diesel_navigation)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(nav_plot PRIVATE -Wall -Wextra -Wpedantic -Werror)
endif()

add_executable(nav_fuzz apps/nav_fuzz.cpp)
target_link_libraries(nav_fuzz PRIVATE diesel_navigation)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(nav_fuzz PRIVATE -Wall -Wextra -Wpedantic -Werror)
endif()

add_executable(types_test test/types_test.cpp)
target_link_libraries(types_test PRIVATE diesel_navigation)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(types_test PRIVATE -Wall -Wextra -Wpedantic -Werror)
endif()
add_test(NAME types_test COMMAND types_test)

add_executable(params_test test/params_test.cpp)
target_link_libraries(params_test PRIVATE diesel_navigation)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(params_test PRIVATE -Wall -Wextra -Wpedantic -Werror)
endif()
add_test(NAME params_test COMMAND params_test)

add_executable(regions_test test/regions_test.cpp)
target_link_libraries(regions_test PRIVATE diesel_navigation)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(regions_test PRIVATE -Wall -Wextra -Wpedantic -Werror)
endif()
add_test(NAME regions_test COMMAND regions_test)

# ---- ROS 2 适配层（可选）----
# 核心库**不依赖 ROS**：找不到 ROS 2 时整个目标跳过，纯 CMake 构建照常可用。
find_package(rclcpp QUIET)
find_package(geometry_msgs QUIET)
find_package(nav_msgs QUIET)
if(rclcpp_FOUND AND geometry_msgs_FOUND AND nav_msgs_FOUND)
  add_executable(nav_node ros/nav_node.cpp)
  target_link_libraries(nav_node PRIVATE
    diesel_navigation
    rclcpp::rclcpp
    geometry_msgs::geometry_msgs__rosidl_typesupport_cpp
    nav_msgs::nav_msgs__rosidl_typesupport_cpp)
  if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(nav_node PRIVATE -Wall -Wextra -Wpedantic -Werror)
  endif()
  install(TARGETS nav_node DESTINATION lib/${PROJECT_NAME})
  message(STATUS "ROS 2 适配层：开启（nav_node）")
else()
  message(STATUS "ROS 2 适配层：跳过（未找到 rclcpp / geometry_msgs / nav_msgs）")
endif()

include(GNUInstallDirs)
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(TARGETS diesel_navigation EXPORT diesel_navigationTargets)
install(EXPORT diesel_navigationTargets NAMESPACE diesel:: DESTINATION lib/cmake/diesel_navigation)
