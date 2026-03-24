cmake_minimum_required(VERSION 3.14)
project(dynamic_map_node CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

# ---------------------------------------------------------------------------
# dimos-lcm (LCM message headers — fetched from GitHub)
# ---------------------------------------------------------------------------
include(FetchContent)
FetchContent_Declare(dimos_lcm
  GIT_REPOSITORY https://github.com/dimensionalOS/dimos-lcm.git
  GIT_TAG        main
  GIT_SHALLOW    TRUE
)
FetchContent_MakeAvailable(dimos_lcm)

# ---------------------------------------------------------------------------
# LCM runtime
# ---------------------------------------------------------------------------
find_package(PkgConfig REQUIRED)
pkg_check_modules(LCM REQUIRED lcm)

# ---------------------------------------------------------------------------
# OctoMap
# ---------------------------------------------------------------------------
find_package(octomap REQUIRED)
if(NOT octomap_FOUND)
  message(FATAL_ERROR
    "octomap not found.  Install it with:\n"
    "  sudo apt install liboctomap-dev\n"
  )
endif()

# ---------------------------------------------------------------------------
# Common headers directory (dimos_native_module.hpp, point_cloud_utils.hpp)
# ---------------------------------------------------------------------------
set(DYNAMIC_MAP_COMMON_DIR "${CMAKE_CURRENT_SOURCE_DIR}/common")

# ---------------------------------------------------------------------------
# Executable
# ---------------------------------------------------------------------------
add_executable(dynamic_map_node dynamic_map_node.cpp)

target_include_directories(dynamic_map_node PRIVATE
  ${DYNAMIC_MAP_COMMON_DIR}
  ${dimos_lcm_SOURCE_DIR}/generated/cpp_lcm_msgs
  ${LCM_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
)

target_link_libraries(dynamic_map_node PRIVATE
  ${LCM_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
)

target_link_directories(dynamic_map_node PRIVATE
  ${LCM_LIBRARY_DIRS}
)

install(TARGETS dynamic_map_node DESTINATION bin)
