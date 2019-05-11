cmake_minimum_required (VERSION 2.8.12)
project (DUALARM_MOBILE_SIM)
SET(CMAKE_CXX_FLAGS "-std=c++11 -I/usr/include -I/usr/local/include")
#SET(CMAKE_BUILD_TYPE "Release")
SET(CMAKE_BUILD_TYPE "Debug")


# set common dependencies
# - eigen3
set(EIGEN3_INCLUDE_DIR $ENV{EIGEN3_INCLUDE_DIR})
if(NOT EIGEN3_INCLUDE_DIR)
  find_package(Eigen3 QUIET)
  # TODO: Find automatic way to find for Ubuntu 14.04
  if(NOT EIGEN3_INCLUDE_DIR)
    set(EIGEN3_INCLUDE_DIR /usr/include/eigen3)
  endif()
endif()
include_directories(${EIGEN3_INCLUDE_DIR})


# # - CHAI3D
find_package(CHAI3D REQUIRED)
include_directories(${CHAI3D_INCLUDE_DIRS})
add_definitions(${CHAI3D_DEFINITIONS})
if (CMAKE_SYSTEM_NAME MATCHES Darwin)
  add_definitions(-D_MACOSX -DMACOSX)
elseif (CMAKE_SYSTEM_NAME MATCHES Linux)
  add_definitions(-D_LINUX -DLINUX)
endif () #NOTE: Windows is currently unsupported
link_directories(${CHAI3D_LIBRARY_DIRS})

# - SAI2-SIMULATION
find_package(SAI2-SIMULATION REQUIRED)
include_directories(${SAI2-SIMULATION_INCLUDE_DIRS})
add_definitions(${SAI2-SIMULATION_DEFINITIONS})

# - SAI2-MODEL
FIND_PACKAGE(SAI2-MODEL REQUIRED)
include_directories(${SAI2-MODEL_INCLUDE_DIRS})
add_definitions(${SAI2-MODEL_DEFINITIONS})

# - SAI2-GRAPHICS
FIND_PACKAGE(SAI2-GRAPHICS REQUIRED)
include_directories(${SAI2-GRAPHICS_INCLUDE_DIRS})
add_definitions(${SAI2-GRAPHICS_DEFINITIONS})

# - SAI2-COMMON
find_package(SAI2-COMMON REQUIRED)
include_directories(${SAI2-COMMON_INCLUDE_DIRS})
add_definitions(${SAI2-COMMON_DEFINITIONS})

# - SAI2-PRIMITIVES
find_package(SAI2-PRIMITIVES REQUIRED)
include_directories(${SAI2-PRIMITIVES_INCLUDE_DIRS})
add_definitions(${SAI2-PRIMITIVES_DEFINITIONS})

# - SAI2-URDF
find_package(SAI2-URDF REQUIRED)
include_directories(${SAI2-URDF_INCLUDE_DIRS})
add_definitions(${SAI2-URDF_DEFINITIONS})

# - glfw3
find_package(glfw3 QUIET)
find_library(GLFW_LIBRARY glfw)

# - hiredis
find_library(HIREDIS_LIBRARY hiredis)

# - rbdl
find_package(RBDL QUIET)
include_directories(${RBDL_INCLUDE_DIR})


set(COMMON_LIBRARIES
	${CHAI3D_LIBRARIES}
    ${SAI2-URDF_LIBRARIES}
    ${SAI2-SIMULATION_LIBRARIES}
    ${SAI2-MODEL_LIBRARIES}
    ${SAI2-GRAPHICS_LIBRARIES}
    ${SAI2-PRIMITIVES_LIBRARIES}
    ${SAI2-COMMON_LIBRARIES}
    ${HIREDIS_LIBRARY}
	${GLFW_LIBRARY}
	)

# add apps
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY                ${PROJECT_SOURCE_DIR}/bin)
# set (CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG          ${PROJECT_SOURCE_DIR}/bin)
# set (CMAKE_RUNTIME_OUTPUT_DIRECTORY_MINSIZEREL     ${PROJECT_SOURCE_DIR}/bin)
# set (CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE        ${PROJECT_SOURCE_DIR}/bin)
# set (CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELWITHDEBINFO ${PROJECT_SOURCE_DIR}/bin)
set (BINARY_DIR ${PROJECT_SOURCE_DIR}/bin)

ADD_SUBDIRECTORY(01-kinematic_sim)
ADD_SUBDIRECTORY(02-dual_arm_sim)