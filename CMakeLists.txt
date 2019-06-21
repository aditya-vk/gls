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

find_package(catkin REQUIRED)
find_package(ompl REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread filesystem graph program_options system)
find_package(OpenCV REQUIRED)


find_package(DART REQUIRED
  COMPONENTS utils utils-urdf optimizer-nlopt)
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
#find_package(libherb REQUIRED)
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
  #${libherb_INCLUDE_DIRS}
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

add_executable(test_cycle examples/test_cycle.cpp)
target_link_libraries(test_cycle ${PROJECT_NAME} ${OpenCV_LIBS})
install(TARGETS test_cycle RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# add_executable(exampleHERB examples/exampleHERB.cpp)
# target_link_libraries(exampleHERB
#   ${PROJECT_NAME}
#   ${DART_LIBRARIES}
#   ${aikido_LIBRARIES}
#   ${BOOST_LIBRARIES}
#   libherb)
# install(TARGETS exampleHERB RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(test_SE2_OMPLApp examples/test_SE2_OMPLApp.cpp)
target_link_libraries(test_SE2_OMPLApp
  ${PROJECT_NAME}
  ${DART_LIBRARIES}
  ${aikido_LIBRARIES}
  ${BOOST_LIBRARIES})
install(TARGETS test_SE2_OMPLApp RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(createSE2Graph scripts/createSE2Graph.cpp)
target_link_libraries(createSE2Graph
  ${PROJECT_NAME}
  ${DART_LIBRARIES}
  ${aikido_LIBRARIES}
  ${BOOST_LIBRARIES})
install(TARGETS createSE2Graph RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(createSE3Graph scripts/createSE3Graph.cpp)
target_link_libraries(createSE3Graph
  ${PROJECT_NAME}
  ${DART_LIBRARIES}
  ${aikido_LIBRARIES}
  ${BOOST_LIBRARIES})
install(TARGETS createSE3Graph RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(visualizeFinalPath scripts/visualizeFinalPath.cpp)
target_link_libraries(visualizeFinalPath
  ${PROJECT_NAME}
  ${DART_LIBRARIES}
  ${aikido_LIBRARIES}
  ${BOOST_LIBRARIES})
install(TARGETS visualizeFinalPath RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

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
FILE(GLOB_RECURSE LibFiles "include/*")
add_custom_target(headers SOURCES ${LibFiles})
