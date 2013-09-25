# Copyright 2013, Florent Lamiraux, CNRS
#
# This file is part of kwsPlus.
# kwsPlus is free software: you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public License
# as published by the Free Software Foundation, either version 3 of
# the License, or (at your option) any later version.
#
# kwsPlus is distributed in the hope that it will be useful, but
# WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# General Lesser Public License for more details.  You should have
# received a copy of the GNU Lesser General Public License along with
# kwsPlus. If not, see <http://www.gnu.org/licenses/>.

# Requires at least CMake 2.6 to configure the package.
CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

INCLUDE(cmake/base.cmake)
INCLUDE(cmake/boost.cmake)
INCLUDE(cmake/cpack.cmake)

SET(PROJECT_NAME kwsPlus)
SET(PROJECT_DESCRIPTION "Extension to Kineoworks")
SET(PROJECT_URL "")

SET(${PROJECT_NAME}_HEADERS
  ${CMAKE_SOURCE_DIR}/include/kwsPlus/roadmap/hppActiveDofSetter.h
  ${CMAKE_SOURCE_DIR}/include/kwsPlus/roadmap/hppConfigurationShooter.h
  ${CMAKE_SOURCE_DIR}/include/kwsPlus/roadmap/hppShooterActiveDof.h
  ${CMAKE_SOURCE_DIR}/include/kwsPlus/roadmap/hppVisRdmBuilder.h
  ${CMAKE_SOURCE_DIR}/include/kwsPlus/roadmap/kwsPlusDiffusionNodePickerFactory.h
  ${CMAKE_SOURCE_DIR}/include/kwsPlus/roadmap/kwsPlusDiffusionShooterFactory.h
  ${CMAKE_SOURCE_DIR}/include/kwsPlus/roadmap/kwsPlusEdge.h
  ${CMAKE_SOURCE_DIR}/include/kwsPlus/roadmap/kwsPlusEdgeFactory.h
  ${CMAKE_SOURCE_DIR}/include/kwsPlus/roadmap/kwsPlusKPCARdmBuilder.h
  ${CMAKE_SOURCE_DIR}/include/kwsPlus/roadmap/kwsPlusLTRdmBuilder.h
  ${CMAKE_SOURCE_DIR}/include/kwsPlus/roadmap/kwsPlusNode.h
  ${CMAKE_SOURCE_DIR}/include/kwsPlus/roadmap/kwsPlusNodeFactory.h
  ${CMAKE_SOURCE_DIR}/include/kwsPlus/roadmap/kwsPlusPCARdmBuilder.h
  ${CMAKE_SOURCE_DIR}/include/kwsPlus/roadmap/kwsPlusStopRdmBuilderDelegate.h
  ${CMAKE_SOURCE_DIR}/include/kwsPlus/roadmap/kwsPlusTalkingBuilder.h
  ${CMAKE_SOURCE_DIR}/include/kwsPlus/roadmap/kwsPlusTalkingShooter.h
  ${CMAKE_SOURCE_DIR}/include/kwsPlus/directPath/directPathVector.h
  ${CMAKE_SOURCE_DIR}/include/kwsPlus/directPath/flicDirectPath.h
  ${CMAKE_SOURCE_DIR}/include/kwsPlus/directPath/flicDistance.h
  ${CMAKE_SOURCE_DIR}/include/kwsPlus/directPath/flicSteeringMethod.h
  ${CMAKE_SOURCE_DIR}/include/kwsPlus/directPath/kwsPlusDPLinear.h
  ${CMAKE_SOURCE_DIR}/include/kwsPlus/directPath/kwsPlusDirectPath.h
  ${CMAKE_SOURCE_DIR}/include/kwsPlus/directPath/kwsPlusDistance.h
  ${CMAKE_SOURCE_DIR}/include/kwsPlus/directPath/kwsPlusDistanceFactory.h
  ${CMAKE_SOURCE_DIR}/include/kwsPlus/directPath/kwsPlusSMLinear.h
  ${CMAKE_SOURCE_DIR}/include/kwsPlus/directPath/kwsPlusSteeringMethodFactory.h
)

SET(CXX_DISABLE_WERROR 1)

SETUP_PROJECT()

SEARCH_FOR_BOOST()
ADD_REQUIRED_DEPENDENCY ("hpp-kwsio >= 1.2")

PKG_CONFIG_APPEND_LIBS(${PROJECT_NAME})

ADD_SUBDIRECTORY(src)

SETUP_PROJECT_FINALIZE()
SETUP_PROJECT_CPACK()
