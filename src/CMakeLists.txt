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

SET(LIBRARY_NAME ${PROJECT_NAME})

SET(${PROJECT_NAME}_SOURCES
  flic-direct-path.cc
  flic-steering-method.cc
  direct-path.cc
  direct-path-vector.cc
  configuration-shooter.cc
  shooter-active-dof.cc
  vis-rdm-builder.cc
  dp-linear.cc
  sm-linear.cc
  distance.cc
  node.cc
  node-factory.cc
  edge.cc
  edge-factory.cc
  roadmap.cc
  flic-distance.cc
  flic-manager.hh
  flic-manager.cc
  stop-rdm-builder-delegate.cc
  )

ADD_LIBRARY(${LIBRARY_NAME}
  SHARED
  ${${PROJECT_NAME}_SOURCES}
  )

INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIRS})
LINK_DIRECTORIES(${Boost_LIBRARY_DIRS})

SET_TARGET_PROPERTIES(${LIBRARY_NAME} PROPERTIES SOVERSION ${PROJECT_VERSION})

PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} hpp-util)
PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} hpp-kwsio)

INSTALL(TARGETS ${LIBRARY_NAME} DESTINATION ${CMAKE_INSTALL_LIBDIR})
