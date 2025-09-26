cmake_minimum_required(VERSION 3.10)
project(TrajectoryFrenet LANGUAGES CXX)

# 1) C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 2) Find dependencies
find_package(Eigen3    REQUIRED)  # header-only
find_package(OpenCV    REQUIRED)  # for visualization


# ===============================================
# Lanelet2
# ===============================================
set(LANELET2_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/Rosless-Lanelet2")
list(APPEND CMAKE_MODULE_PATH "${LANELET2_ROOT}/cmake")
add_subdirectory("${LANELET2_ROOT}" "${CMAKE_BINARY_DIR}/lanelet2_build")

# 3) Core library
add_library(trajectory_frenet
  src/polynomial.cpp
  src/frenet_planner.cpp
  src/GlobalPlanner.cpp
)

# 3a) Expose headers
target_include_directories(trajectory_frenet
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include/
)

# 3b) Link in header-only & external libs
target_link_libraries(trajectory_frenet
  PUBLIC Eigen3::Eigen
  PUBLIC ${OpenCV_LIBS}
  PUBLIC lanelet2_core
  PUBLIC lanelet2_io
  PUBLIC lanelet2_projection
  PUBLIC lanelet2_routing
  PUBLIC lanelet2_traffic_rules
  PUBLIC lanelet2_validation
)

# 4) Main executable
add_executable(main_app
  src/main.cpp
)

# 4a) Tell it where to find your headers
target_include_directories(main_app
  PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include/
)

# 4b) Link against your library + OpenCV
target_link_libraries(main_app
  PRIVATE trajectory_frenet
  PRIVATE ${OpenCV_LIBS}
)
