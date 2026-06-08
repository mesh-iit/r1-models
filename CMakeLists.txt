# Copyright (C) 2006-2023 Istituto Italiano di Tecnologia (IIT)
# All rights reserved.
#
# This software may be modified and distributed under the terms of the
# BSD-3-Clause license. See the accompanying LICENSE file for details.

cmake_minimum_required(VERSION 3.22.1)

project(r1-models
        VERSION 1.0.1
        LANGUAGES NONE)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

include(AddUninstallTarget)
set(BUILD_PREFIX "R1Mk3")
option(BUILD_TESTING "Run tests for the generated models" OFF)
option(COMPILE_R1Mk3Models "Configure and install the URDF models of the R1Mk3 robot" ON)

if(COMPILE_R1Mk3Models)
   add_subdirectory(urdf)
endif()

if (BUILD_TESTING)
  include(CTest)
  enable_testing()
  add_subdirectory(tests)
endif()

