# @author Munzir Zafar
# @date Nov 14, 2018

cmake_minimum_required(VERSION 2.6)
project(krach)

# Set the top level directory for use in the code
add_definitions(-DTOP_LEVEL_PATH="${CMAKE_SOURCE_DIR}")

# Set the C99 standard for the C files
set(CMAKE_INSTALL_PREFIX /usr)
#set(CMAKE_C_FLAGS --std=gnu99 -g)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -std=gnu++0x")

# Guard against in-source builds
if(${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_BINARY_DIR})
  message(FATAL_ERROR "In-source builds are not allowed. You may need to remove CMakeCache.txt.")
endif()

# ===================================================================================
# Include and link to libraries

# Find necessary packages
find_package( PythonLibs 2.7 REQUIRED )
include_directories( ${PYTHON_INCLUDE_DIRS} )

find_package( Boost COMPONENTS python REQUIRED )
include_directories( ${Boost_INCLUDE_DIR} )

# Link to somatic, amino and ach
# NOTE: Ideally we would like to 'find' these packages but for now, we assume they are either
# in /usr/lib or /usr/local/lib
#link_libraries(amino ntcan protobuf-c ach somatic stdc++ filter pcio pthread ncurses dart fcl tinyxml tinyxml2 kore assimp GL)
link_libraries(stdc++ amino protobuf-c ach somatic)

# ====================================================================================
# Build the codebase library and the executables

# Include the project files
include_directories(include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

# Collect the source, script and fortran files
file(GLOB main_source "src/krach.cpp")

# Build the library and set the executables that use it (have to hardcode for now)
add_library(krach SHARED ${main_source})
target_link_libraries(krach ${Boost_LIBRARIES})

# Install
install(TARGETS krach DESTINATION /usr/local/lib)
FILE(GLOB headers "include/krach/krach.h" "include/krach/*.hpp")
install(FILES ${headers} DESTINATION /usr/local/include/krach)

message(STATUS " ")

# uninstall target
if(NOT TARGET uninstall)
    configure_file(
        "${CMAKE_CURRENT_SOURCE_DIR}/cmake/cmake_uninstall.cmake.in"
        "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake"
        IMMEDIATE @ONLY)

    add_custom_target(uninstall
        COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake)
endif()
