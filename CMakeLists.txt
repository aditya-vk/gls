cmake_minimum_required(VERSION 2.8.3)
project(gls)
list(INSERT CMAKE_MODULE_PATH 0 "${PROJECT_SOURCE_DIR}/cmake")

#================================================================================
# Compiler settings
#================================================================================
# Enforce the use of C++11
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra -Wuninitialized -Wpedantic")

add_compile_options(-Wall -Wextra -Wpedantic)
if(GENERALIZED_LAZY_SEARCH_TREAT_WARNINGS_AS_ERRORS)
  add_compile_options(-Werror)
endif()

#================================================================================
# Link Libraries
#================================================================================

find_package(catkin REQUIRED COMPONENTS roscpp)
find_package(ompl REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread filesystem graph program_options system)
find_package(OpenCV REQUIRED)

find_package(DART REQUIRED
  COMPONENTS utils-urdf optimizer-nlopt utils)
find_package(aikido 0.0.2 REQUIRED
  COMPONENTS
    common
    constraint
    control
    control_ros
    distance
    perception
    planner
    planner_kunzretimer
    planner_ompl
    planner_parabolic
    planner_vectorfield
    robot
    rviz
    statespace
    io
)
find_package(libherb REQUIRED)
find_package(pr_assets REQUIRED)
find_package(pr_tsr REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  DEPENDS Boost
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OMPL_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
  ${DART_INCLUDE_DIRS}
  ${libherb_INCLUDE_DIRS}
  ${pr_assets_INCLUDE_DIRS}
  ${pr_tsr_INCLUDE_DIRS}
)

link_directories(
  ${catkin_LIBRARY_DIRS}
)

link_libraries(
  ${catkin_LIBRARIES}
  ${OMPL_LIBRARIES}
  ${Boost_LIBRARIES}
)

add_library(${PROJECT_NAME}
  src/datastructures/Graph.cpp
  src/datastructures/SearchQueue.cpp
  src/datastructures/State.cpp
  src/GLS.cpp
  src/event/Event.cpp
  src/event/ShortestPathEvent.cpp
  src/event/ConstantDepthEvent.cpp
  src/selector/Selector.cpp
  src/selector/AlternateSelector.cpp
  src/selector/BackwardSelector.cpp
  src/selector/FailFastSelector.cpp
  src/selector/ForwardSelector.cpp
)

#================================================================================
# Example Scripts
#================================================================================

# Correctness Test Script
add_executable(test_ShortestPath_Forward examples/test_ShortestPath_Forward.cpp)
target_link_libraries(test_ShortestPath_Forward ${PROJECT_NAME} ${OpenCV_LIBS})
install(TARGETS test_ShortestPath_Forward RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(test_ConstantDepth_Forward examples/test_ConstantDepth_Forward.cpp)
target_link_libraries(test_ConstantDepth_Forward ${PROJECT_NAME} ${OpenCV_LIBS})
install(TARGETS test_ConstantDepth_Forward RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# Reaching for the shelf in a vanilla setting.
add_executable(reach_for_shelf_vanilla examples/reach_for_shelf_vanilla.cpp)
target_link_libraries(reach_for_shelf_vanilla
  ${PROJECT_NAME}
  ${DART_LIBRARIES}
  ${aikido_LIBRARIES}
  ${BOOST_LIBRARIES}
  libherb)
install(TARGETS reach_for_shelf_vanilla RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# Reaching for the shelf in a constrained setting.
add_executable(reach_for_shelf_constrained examples/reach_for_shelf_constrained.cpp)
target_link_libraries(reach_for_shelf_constrained
  ${PROJECT_NAME}
  ${DART_LIBRARIES}
  ${aikido_LIBRARIES}
  ${BOOST_LIBRARIES}
  libherb)
install(TARGETS reach_for_shelf_constrained RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# Reaching into the shelf.
add_executable(reach_into_shelf examples/reach_into_shelf.cpp)
target_link_libraries(reach_into_shelf
  ${PROJECT_NAME}
  ${DART_LIBRARIES}
  ${aikido_LIBRARIES}
  ${BOOST_LIBRARIES}
  libherb)
install(TARGETS reach_into_shelf RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

#================================================================================
# Installation
#================================================================================

install(TARGETS gls EXPORT glsConfig
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  )

install(DIRECTORY include/ DESTINATION include)

# This makes the project importable from the install directory
install(EXPORT glsConfig DESTINATION share/gls/cmake)

# This makes the project importable from the build directory
export(TARGETS gls FILE glsConfig.cmake)

# Install the package.xml file (to satisfy REP-136).
install(FILES "package.xml"
  DESTINATION "share/gls")

#================================================================================
# Formatting
#================================================================================
include(ClangFormat)

clang_format_setup(VERSION 3.8)

if (CLANG_FORMAT_EXECUTABLE)
  FILE(GLOB_RECURSE gls_headers "include/*.hpp")
  FILE(GLOB_RECURSE gls_srcs "include/*.cpp")
  clang_format_add_sources(${gls_headers})
  clang_format_add_sources(${gls_srcs})

  clang_format_add_targets()
endif()

#================================================================================
# IDE Comfort
#================================================================================

# This custom target doesn't affect building the project but is only for
# displaying the header files in IDEs.
FILE(GLOB_RECURSE LibFiles "include/*.hpp")
add_custom_target(headers SOURCES ${LibFiles})
