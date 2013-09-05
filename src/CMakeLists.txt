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

INCLUDE_DIRECTORIES("${CMAKE_CURRENT_SOURCE_DIR}")
INCLUDE_DIRECTORIES("${CMAKE_CURRENT_BINARY_DIR}")

SET(${PROJECT_NAME}_SOURCES
  flicDirectPath.cpp
  flicSteeringMethod.cpp
  kwsPlusDirectPath.cpp
  directPathVector.cpp
  hppConfigurationShooter.cpp
  hppShooterActiveDof.cpp
  hppVisRdmBuilder.cpp
  kwsPlusDPLinear.cpp
  kwsPlusSMLinear.cpp
  kwsPlusDistance.cpp
  kwsPlusNode.cpp
  kwsPlusNodeFactory.cpp
  kwsPlusEdge.cpp
  kwsPlusEdgeFactory.cpp
  flicDistance.cpp
  flicManager.h
  flicManager.cpp
  kwsPlusStopRdmBuilderDelegate.cpp
  )

ADD_LIBRARY(${PROJECT_NAME}
  SHARED
  ${${PROJECT_NAME}_SOURCES}
  )

INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIRS})
LINK_DIRECTORIES(${Boost_LIBRARY_DIRS})

PKG_CONFIG_USE_DEPENDENCY(${PROJECT_NAME} kwsIO)

INSTALL(TARGETS ${PROJECT_NAME} DESTINATION ${CMAKE_INSTALL_LIBDIR})
