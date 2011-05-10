CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

PROJECT(ssa)

# The library prefix
SET(LIB_PREFIX ssa_)

SET(ssa_C_FLAGS)
SET(ssa_CXX_FLAGS)

# default built type
IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release CACHE STRING
      "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
      FORCE)
ENDIF(NOT CMAKE_BUILD_TYPE)

# postfix, based on type
SET(CMAKE_DEBUG_POSTFIX "_d" CACHE STRING "postfix applied to debug build of libraries")
SET(CMAKE_RELEASE_POSTFIX "" CACHE STRING "postfix applied to release build of libraries")
SET(CMAKE_RELWITHDEBINFO_POSTFIX "_rd" CACHE STRING "postfix applied to release-with-debug-information libraries")
SET(CMAKE_MINSIZEREL_POSTFIX "_s" CACHE STRING "postfix applied to minimium-size-build libraries")

# On the Mac platform, configure the RPATH as per the install, to
# avoid the problem of loading both the built and installed versions
# of the shared targets
IF(APPLE)
  SET(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE) 
  SET(CMAKE_INSTALL_RPATH "")
ENDIF(APPLE)

# Set the output directory for the build executables and libraries
SET(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${ssa_SOURCE_DIR}/lib)
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${ssa_SOURCE_DIR}/bin)
IF(WIN32)
  SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${ssa_SOURCE_DIR}/bin)
ELSE(WIN32)
  SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${ssa_SOURCE_DIR}/lib)
ENDIF(WIN32)

# Set search directory for looking for our custom CMake scripts to
# look for SuiteSparse and QGLViewer
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${ssa_SOURCE_DIR}/cmake_modules)

# Detect OS and define macros appropriately
IF(WIN32)
  IF(CYGWIN)
    MESSAGE(STATUS "Compiling on Cygwin")
    ADD_DEFINITIONS(-DCYGWIN)
  ELSE(CYGWIN)
    ADD_DEFINITIONS(-DWINDOWS)
    MESSAGE(STATUS "Compiling on Windows")
  ENDIF(CYGWIN)
ELSEIF(UNIX)
  ADD_DEFINITIONS(-DUNIX)
  MESSAGE(STATUS "Compiling on Unix")
ENDIF(WIN32)

# For building the CHOLMOD / CSPARSE solvers
FIND_PACKAGE(SuiteSparse)
#IF(CSPARSE_FOUND)
#  SET( BUILD_CSPARSE OFF CACHE BOOL "Build local CSparse library")
#ELSE(CSPARSE_FOUND)
#  SET( BUILD_CSPARSE ON CACHE BOOL "Build local CSparse library")
#  SET(CSPARSE_FOUND TRUE)
#ENDIF(CSPARSE_FOUND)

# # Eigen library parallelise itself, though, presumably due to performance issues
# # OPENMP is experimental. We experienced some slowdown with it
# FIND_PACKAGE(OpenMP)
# SET( G2O_USE_OPENMP OFF CACHE BOOL "Build g2o with OpenMP support (EXPERIMENTAL)")
# IF(OPENMP_FOUND AND G2O_USE_OPENMP)
#   SET(g2o_C_FLAGS "${g2o_C_FLAGS} ${OpenMP_C_FLAGS}")
#   SET(g2o_CXX_FLAGS "${g2o_CXX_FLAGS} -DEIGEN_DONT_PARALLELIZE ${OpenMP_CXX_FLAGS}")
#   MESSAGE(STATUS "Compiling with OpenMP support")
# ENDIF(OPENMP_FOUND AND G2O_USE_OPENMP)

# OpenGL is used in the draw actions for the different types, as well
# as for creating the GUI itself
# FIND_PACKAGE(OpenGL)
# SET(G2O_USE_OPENGL ON CACHE BOOL "Build g2o with OpenGL support for visualization")
# IF (OPENGL_FOUND AND G2O_USE_OPENGL)
#   SET (G2O_HAVE_OPENGL 1)
#   INCLUDE_DIRECTORIES(${OPENGL_INCLUDE})
#   MESSAGE(STATUS "Compiling with OpenGL support")
# ENDIF(OPENGL_FOUND AND G2O_USE_OPENGL)

# For building the GUI
FIND_PACKAGE(QGLViewer)

# shall we build the core apps using the library
SET(SSA_BUILD_APPS ON CACHE BOOL "Build ssa apps")
IF(SSA_BUILD_APPS)
  MESSAGE(STATUS "Compiling ssa apps")
ENDIF(SSA_BUILD_APPS)

# # shall we build the examples
# SET(G2O_BUILD_EXAMPLES ON CACHE BOOL "Build g2o examples")
# IF(G2O_BUILD_EXAMPLES)
#   MESSAGE(STATUS "Compiling g2o examples")
# ENDIF(G2O_BUILD_EXAMPLES)

# Compiler specific options for gcc
IF(CMAKE_COMPILER_IS_GNUCXX)
  # Generic settings for optimisation
  SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -msse4") 
  SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -O3 -msse4") 

  # OS X
  #IF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    #SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE}") 
    #SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE}") 
  #ENDIF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
  # Linux
  IF(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
    SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -march=native") 
    SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -march=native") 
  ENDIF(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
  # activate warnings !!!
  SET(ssa_C_FLAGS "${ssa_C_FLAGS} -Wall -W")
  SET(ssa_CXX_FLAGS "${ssa_CXX_FLAGS} -Wall -W")
ENDIF(CMAKE_COMPILER_IS_GNUCXX)

# specifying compiler flags
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${ssa_CXX_FLAGS}")
SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${ssa_C_FLAGS}")
#MESSAGE("CMAKE_C_FLAGS = ${CMAKE_C_FLAGS}")
#MESSAGE("CMAKE_CXX_FLAGS = ${CMAKE_CXX_FLAGS}")

# Set up the top-level include directories
SET( SSA_EIGEN3_INCLUDE ${ssa_SOURCE_DIR}/EXTERNAL/g2o/EXTERNAL/eigen3 CACHE PATH "Directory of Eigen3")
SET( SSA_G2O_INCLUDE ${ssa_SOURCE_DIR}/EXTERNAL/g2o CACHE PATH "Directory of G2O")
INCLUDE_DIRECTORIES(${ssa_SOURCE_DIR} ${SSA_G2O_INCLUDE} ${SSA_EIGEN3_INCLUDE})

# Add g2o lib dir
LINK_DIRECTORIES( ${LINK_DIRECTORIES} "${SSA_G2O_INCLUDE}/lib" )

#Generate config.h 
configure_file(config.h.in ${PROJECT_BINARY_DIR}/g2o/config.h)
include_directories(${PROJECT_BINARY_DIR})
INSTALL(FILES ${PROJECT_BINARY_DIR}/g2o/config.h DESTINATION ${CMAKE_INSTALL_PREFIX}/include/g2o)

# Include the subdirectories
ADD_SUBDIRECTORY(EXTERNAL)
ADD_SUBDIRECTORY(ssa)
