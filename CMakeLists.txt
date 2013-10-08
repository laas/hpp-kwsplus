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

SET(PROJECT_NAME hpp-kwsplus)
SET(PROJECT_DESCRIPTION "Extension to Kineoworks")
SET(PROJECT_URL "")

SET(${PROJECT_NAME}_HEADERS
  ${CMAKE_SOURCE_DIR}/include/hpp/kwsplus/roadmap/active-dof-setter.hh
  ${CMAKE_SOURCE_DIR}/include/hpp/kwsplus/roadmap/configuration-shooter.hh
  ${CMAKE_SOURCE_DIR}/include/hpp/kwsplus/roadmap/shooter-active-dof.hh
  ${CMAKE_SOURCE_DIR}/include/hpp/kwsplus/roadmap/vis-rdm-builder.hh
  ${CMAKE_SOURCE_DIR}/include/hpp/kwsplus/roadmap/diffusion-node-picker-factory.hh
  ${CMAKE_SOURCE_DIR}/include/hpp/kwsplus/roadmap/diffusion-shooter-factory.hh
  ${CMAKE_SOURCE_DIR}/include/hpp/kwsplus/roadmap/edge.hh
  ${CMAKE_SOURCE_DIR}/include/hpp/kwsplus/roadmap/edge-factory.hh
  ${CMAKE_SOURCE_DIR}/include/hpp/kwsplus/roadmap/kpca-rdm-builder.hh
  ${CMAKE_SOURCE_DIR}/include/hpp/kwsplus/roadmap/lt-rdm-builder.hh
  ${CMAKE_SOURCE_DIR}/include/hpp/kwsplus/roadmap/node.hh
  ${CMAKE_SOURCE_DIR}/include/hpp/kwsplus/roadmap/node-factory.hh
  ${CMAKE_SOURCE_DIR}/include/hpp/kwsplus/roadmap/pca-rdm-builder.hh
  ${CMAKE_SOURCE_DIR}/include/hpp/kwsplus/roadmap/roadmap.hh
  ${CMAKE_SOURCE_DIR}/include/hpp/kwsplus/roadmap/stop-rdm-builder-delegate.hh
  ${CMAKE_SOURCE_DIR}/include/hpp/kwsplus/roadmap/talking-builder.hh
  ${CMAKE_SOURCE_DIR}/include/hpp/kwsplus/roadmap/talking-shooter.hh
  ${CMAKE_SOURCE_DIR}/include/hpp/kwsplus/direct-path/direct-path-vector.hh
  ${CMAKE_SOURCE_DIR}/include/hpp/kwsplus/direct-path/flic-direct-path.hh
  ${CMAKE_SOURCE_DIR}/include/hpp/kwsplus/direct-path/flic-distance.hh
  ${CMAKE_SOURCE_DIR}/include/hpp/kwsplus/direct-path/flic-steering-method.hh
  ${CMAKE_SOURCE_DIR}/include/hpp/kwsplus/direct-path/dp-linear.hh
  ${CMAKE_SOURCE_DIR}/include/hpp/kwsplus/direct-path/direct-path.hh
  ${CMAKE_SOURCE_DIR}/include/hpp/kwsplus/direct-path/distance.hh
  ${CMAKE_SOURCE_DIR}/include/hpp/kwsplus/direct-path/distance-factory.hh
  ${CMAKE_SOURCE_DIR}/include/hpp/kwsplus/direct-path/sm-linear.hh
  ${CMAKE_SOURCE_DIR}/include/hpp/kwsplus/direct-path/steering-method-factory.hh
)

SET(CXX_DISABLE_WERROR 1)

SETUP_PROJECT()

SEARCH_FOR_BOOST()
ADD_REQUIRED_DEPENDENCY ("hpp-util >= 0.8")
ADD_REQUIRED_DEPENDENCY ("hpp-kwsio >= 1.2")

PKG_CONFIG_APPEND_LIBS(${PROJECT_NAME})

ADD_SUBDIRECTORY(src)

SETUP_PROJECT_FINALIZE()
SETUP_PROJECT_CPACK()
