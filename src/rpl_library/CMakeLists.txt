cmake_minimum_required(VERSION 3.8)

set(PROJECT_NAME rpl_library)
set(LIBRARY_NAME rpl)

set(LIBRARY_HEADERS_DIR include/${LIBRARY_NAME})

#[=[ This is actually unneccessary
set(LIBRARY_HEADERS
    # /
    ${LIBRARY_HEADERS_DIR}/common.hpp
    ${LIBRARY_HEADERS_DIR}/types.hpp
    ${LIBRARY_HEADERS_DIR}/interface.hpp
    
    # clipper
    ${LIBRARY_HEADERS_DIR}/clipper/clipper.hpp
    
    # internal
    ${LIBRARY_HEADERS_DIR}/internal/geometry.hpp
    ${LIBRARY_HEADERS_DIR}/internal/parallel.hpp
    ${LIBRARY_HEADERS_DIR}/internal/rplintrin.hpp
    ${LIBRARY_HEADERS_DIR}/internal/utils.hpp

    # io
    ${LIBRARY_HEADERS_DIR}/io/Minkowski.hpp
    ${LIBRARY_HEADERS_DIR}/io/WorldDescriptor.hpp

    # map
    ${LIBRARY_HEADERS_DIR}/map/Graph.hpp
    ${LIBRARY_HEADERS_DIR}/map/RoadMap.hpp

    # data_structures
    ${LIBRARY_HEADERS_DIR}/data_structures/AVLTree.hpp
    ${LIBRARY_HEADERS_DIR}/data_structures/PseudoAVLTree.hpp
    
    # planning
    ${LIBRARY_HEADERS_DIR}/planning/Dubins.hpp
    ${LIBRARY_HEADERS_DIR}/planning/ParallelDubins.hpp

)
#]=]
set(LIBRARY_SOURCE_DIR src)
set(LIBRARY_SOURCE

  # clipper
  ${LIBRARY_SOURCE_DIR}/clipper/clipper.cpp

  # io
  ${LIBRARY_SOURCE_DIR}/io/Minkowski.cpp
  ${LIBRARY_SOURCE_DIR}/io/WorldDescriptor.cpp

  # map
  ${LIBRARY_SOURCE_DIR}/map/Graph.cpp
  ${LIBRARY_SOURCE_DIR}/map/RoadMap.cpp

  # data_structures
  ${LIBRARY_SOURCE_DIR}/data_structures/AVLTree.cpp
  ${LIBRARY_SOURCE_DIR}/data_structures/PseudoAVLTree.cpp

  # planning
  ${LIBRARY_SOURCE_DIR}/planning/Dubins.cpp
  ${LIBRARY_SOURCE_DIR}/planning/ParallelDubins.cpp
)

project(${PROJECT_NAME})

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # added O2 and march=native to use simd intrinsics
  add_compile_options(-Wall -Wextra -Wpedantic -O2 -march=native)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)

  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_library(${LIBRARY_NAME}
  SHARED
  ${LIBRARY_SOURCE})

# that's how I roll get over it
# I believe the first argument should be project_name and not library_name
target_include_directories(${LIBRARY_NAME}
  PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${LIBRARY_NAME}>
  $<INSTALL_INTERFACE:include/${LIBRARY_NAME}>)

target_include_directories(${LIBRARY_NAME}
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_export_targets(${LIBRARY_NAME} HAS_LIBRARY_TARGET)

install(
  DIRECTORY ${LIBRARY_HEADERS_DIR}
  DESTINATION include)

install(
  TARGETS ${LIBRARY_NAME}
  EXPORT ${LIBRARY_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include)

ament_package()
