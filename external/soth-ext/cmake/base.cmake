# Copyright (C) 2008-2019 LAAS-CNRS, JRL AIST-CNRS, INRIA.
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

 # ------ #
 # README #
 # ------ #


#.rst:
# .. ifmode:: user
#
# This section lists the variables and macros that must be defined and
# invoked in the right place to enable the features of this cmake modules.
#
# For a minimal working example see :ref:`minimal-working-example`.
#
# Variables
# ---------
#
# .. variable:: PROJECT_NAME
#
#   **REQUIRED**
#
#   Please keep respect our coding style and choose a name
#   which respects the following regexp: ``[a-z][a-z0-9-]*``
#   I.e. a lower-case letter then one or more lower-case
#   letter, number or hyphen ``-``.
#
# .. variable:: PROJECT_URL
#
#   **REQUIRED**
#
#   Project's website.
#
# .. variable:: PROJECT_DESCRIPTION
#
#   **REQUIRED**
#
#   One line summary of the package goal.
#
# .. variable:: PROJECT_VERSION
#
#   Project version (X.Y.Z where X, Y, Z are unsigned
#   integers). If not defined, it will automatically
#   be computed through `git describe`.
#   See :cmake:command:`VERSION_COMPUTE` for more information.
#
# .. variable:: PROJECT_DEBUG_POSTFIX
#
#   If set, ``${PROJECT_DEBUG_POSTFIX}`` will be appended to the libraries
#   generated by the project (as the builtin `CMAKE_DEBUG_POSTFIX
#   <https://cmake.org/cmake/help/v3.0/variable/CMAKE_DEBUG_POSTFIX.html>`_)
#   but this will also trigger the generation of an appropriate debug
#   pkg-config file.
#
# .. variable:: PROJECT_USE_KEYWORD_LINK_LIBRARIES
#
#   If set to true, the jrl-cmakemodules will use the PUBLIC keyword in
#   ``target_link_libraries``. Defaults to false.
#
# .. variable: PROJECT_CUSTOM_HEADER_EXTENSION
#   Allows to define a custome extension for C/C++ header files (e.g. .h, .hh, .hpp).
#   The default value is set to .hh.
#

# Please note that functions starting with an underscore are internal
# functions and should not be used directly.

# Include base features.
INCLUDE(cmake/logging.cmake)
INCLUDE(cmake/portability.cmake)
INCLUDE(cmake/compiler.cmake)
INCLUDE(cmake/debian.cmake)
INCLUDE(cmake/dist.cmake)
INCLUDE(cmake/distcheck.cmake)
INCLUDE(cmake/doxygen.cmake)
INCLUDE(cmake/header.cmake)
INCLUDE(cmake/pkg-config.cmake)
INCLUDE(cmake/uninstall.cmake)
INCLUDE(cmake/install-data.cmake)
INCLUDE(cmake/release.cmake)
INCLUDE(cmake/version.cmake)
INCLUDE(cmake/package-config.cmake)
INCLUDE(cmake/version-script.cmake)
INCLUDE(cmake/test.cmake)

 # --------- #
 # Constants #
 # --------- #

# Variables requires by SETUP_PROJECT.
SET(REQUIRED_VARIABLES PROJECT_NAME PROJECT_DESCRIPTION PROJECT_URL)

 # --------------------- #
 # Project configuration #
 # --------------------- #

# _ADD_TO_LIST LIST VALUE
# -----------------------
#
# Add a value to a comma-separated list.
#
# LIST		: the list.
# VALUE		: the value to be appended.
# SEPARATOR	: the separation symol.
#
MACRO(_ADD_TO_LIST LIST VALUE SEPARATOR)
  IF("${${LIST}}" STREQUAL "")
    SET(${LIST} "${VALUE}")
  ELSE("${${LIST}}" STREQUAL "")
    IF(NOT "${VALUE}" STREQUAL "")
      SET(${LIST} "${${LIST}}${SEPARATOR} ${VALUE}")
    ENDIF(NOT "${VALUE}" STREQUAL "")
  ENDIF("${${LIST}}" STREQUAL "")
ENDMACRO(_ADD_TO_LIST LIST VALUE)

# _CONCATENATE_ARGUMENTS
# ----------------------
#
# Concatenate all arguments into the output variable.
#
# OUTPUT	: the output variable.
# SEPARTOR	: the list separation symbol.
# ARG1...ARGN	: the values to be concatenated.
#
MACRO(_CONCATENATE_ARGUMENTS OUTPUT SEPARATOR)
  FOREACH(I RANGE 2 ${ARGC})
    _ADD_TO_LIST("${OUTPUT}" "${ARGV${I}}" "${SEPARATOR}")
  ENDFOREACH(I RANGE 2 ${ARGC})
  MESSAGE(${${OUTPUT}})
ENDMACRO(_CONCATENATE_ARGUMENTS OUTPUT)

#.rst:
# .. ifmode:: user
#
# .. command:: SETUP_PROJECT
#
#   Initialize the project. Should be called first in the root
#   CMakeList.txt.
#
#   This function does not take any argument but check that some
#   variables are defined (see documentation at the beginning of this
#   file).
#
MACRO(SETUP_PROJECT)
  INCLUDE(cmake/GNUInstallDirs.cmake)
  SET(CMAKE_INSTALL_FULL_PKGLIBDIR ${CMAKE_INSTALL_FULL_LIBDIR}/${PROJECT_NAME})
  SET(CMAKE_INSTALL_PKGLIBDIR ${CMAKE_INSTALL_LIBDIR}/${PROJECT_NAME})

  # Check that required variables are defined.
  FOREACH(VARIABLE ${REQUIRED_VARIABLES})
    IF (NOT DEFINED ${VARIABLE})
      MESSAGE(FATAL_ERROR
	"Required variable ``${VARIABLE}'' has not been defined.")
    ENDIF(NOT DEFINED ${VARIABLE})
  ENDFOREACH(VARIABLE)

  # If the project version number is not set, compute it automatically.
  IF(NOT DEFINED PROJECT_VERSION)
    VERSION_COMPUTE()
  ELSE()
    IF(NOT DEFINED PROJECT_VERSION_MAJOR AND
        NOT DEFINED PROJECT_VERSION_MINOR AND
        NOT DEFINED PROJECT_VERSION_PATCH)
      SPLIT_VERSION_NUMBER(${PROJECT_VERSION}
        PROJECT_VERSION_MAJOR
        PROJECT_VERSION_MINOR
        PROJECT_VERSION_PATCH)
    ENDIF()
  ENDIF()

  IF(PROJECT_VERSION MATCHES UNKNOWN)
    SET(PROJECT_VERSION_FULL "")
  ELSE(PROJECT_VERSION MATCHES UNKNOWN)
    SET(PROJECT_VERSION_FULL "${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}.${PROJECT_VERSION_PATCH}")
  ENDIF(PROJECT_VERSION MATCHES UNKNOWN)

  # Define project
  IF(${CMAKE_VERSION} VERSION_GREATER 3.0.0)
    # Remove warnings or errors
    IF(POLICY CMP0048)
      CMAKE_POLICY(SET CMP0048 NEW)
    ENDIF(POLICY CMP0048)

    IF(${CMAKE_VERSION} VERSION_GREATER 3.9.0)
      PROJECT(${PROJECT_NAME} LANGUAGES CXX DESCRIPTION ${PROJECT_DESCRIPTION} VERSION ${PROJECT_VERSION_FULL})
    ELSE(NOT ${CMAKE_VERSION} VERSION_GREATER 3.9.0)
      PROJECT(${PROJECT_NAME} LANGUAGES CXX VERSION ${PROJECT_VERSION_FULL})
    ENDIF(${CMAKE_VERSION} VERSION_GREATER 3.9.0)
  ELSE(${CMAKE_VERSION} VERSION_GREATER 3.0.0)
    PROJECT(${PROJECT_NAME} CXX)
  ENDIF(${CMAKE_VERSION} VERSION_GREATER 3.0.0)

  IF(DEFINED PROJECT_DEBUG_POSTFIX)
    SET(CMAKE_DEBUG_POSTFIX ${PROJECT_DEBUG_POSTFIX})
    STRING(TOLOWER "${CMAKE_BUILD_TYPE}" cmake_build_type)
    IF(${cmake_build_type} MATCHES debug)
      SET(PKGCONFIG_POSTFIX ${PROJECT_DEBUG_POSTFIX})
    ELSE()
      SET(PKGCONFIG_POSTFIX "")
    ENDIF()
    IF(DEFINED CMAKE_CONFIGURATION_TYPES)
      SET(PKGCONFIG_POSTFIX ${PROJECT_DEBUG_POSTFIX})
    ENDIF()
  ENDIF()

  IF(NOT DEFINED PROJECT_USE_KEYWORD_LINK_LIBRARIES)
    SET(PROJECT_USE_KEYWORD_LINK_LIBRARIES FALSE)
  ENDIF()
  IF(${PROJECT_USE_KEYWORD_LINK_LIBRARIES})
    SET(PUBLIC_KEYWORD PUBLIC)
  ELSE()
    SET(PUBLIC_KEYWORD "")
  ENDIF()

  IF(${ARGC})
    SET(CMAKE_VERBOSE_MAKEFILE ${ARGV0})
  ELSE(${ARGC})
    # Be verbose by default.
    SET(CMAKE_VERBOSE_MAKEFILE TRUE)
  ENDIF(${ARGC})

  OPTION(INSTALL_DOCUMENTATION "Generate and install the documentation" ON)
  OPTION(INSTALL_GENERATED_HEADERS "Generate and install standard headers" ON)
  OPTION(INSTALL_PKG_CONFIG_FILE "Generate and install standard .pc file" ON)

  INCLUDE(CTest)
  ENABLE_TESTING()

  LOGGING_INITIALIZE()

  #FIXME: normalize naming to <MODULE>_SETUP()
  _SETUP_PROJECT_WARNINGS()
  _SETUP_PROJECT_HEADER()
  _SETUP_PROJECT_DIST()
  DISTCHECK_SETUP()
  RELEASE_SETUP()
  _SETUP_PROJECT_DEB()
  _SETUP_PROJECT_UNINSTALL()
  _SETUP_PROJECT_PKG_CONFIG()
  _SETUP_PROJECT_DOCUMENTATION()
  _SETUP_PROJECT_PACKAGE_INIT()
ENDMACRO(SETUP_PROJECT)

#.rst:
# .. ifmode:: user
#
# .. command:: SETUP_PROJECT_FINALIZE
#
#   To be called at the end of the CMakeLists.txt to
#   finalize the project setup.
#
MACRO(SETUP_PROJECT_FINALIZE)
  IF(INSTALL_PKG_CONFIG_FILE)
    _SETUP_PROJECT_PKG_CONFIG_FINALIZE()
  ENDIF(INSTALL_PKG_CONFIG_FILE)
  _SETUP_PROJECT_DOCUMENTATION_FINALIZE()
  _SETUP_PROJECT_HEADER_FINAlIZE()
  _SETUP_DEBIAN()
  # Install data if needed
  _INSTALL_PROJECT_DATA()

  LOGGING_FINALIZE()
ENDMACRO(SETUP_PROJECT_FINALIZE)

#.rst:
# .. ifmode:: user
#
# .. command:: CHECK_DEBIAN
#
#   Checks is the current system is Debian based
#   You can then use DEBIAN_FOUND
#
MACRO(CHECK_DEBIAN)
  FIND_FILE(DEBIAN_FOUND debian_version debconf.conf
    PATHS /etc)
ENDMACRO(CHECK_DEBIAN)


#.rst:
# .. ifmode:: user
#
# .. command:: CHECK_NETBSD
#
#   Checks is the current system is NetBSD
#   You can then use NETBSD_FOUND
#
MACRO(CHECK_NETBSD)
  FIND_FILE(NETBSD_FOUND netbsd
    PATHS /)
ENDMACRO(CHECK_NETBSD)


#.rst:
# .. ifmode:: user
#
# .. command:: CHECK_ARCHLINUX
#
#   Checks is the current system is ArchLinux
#   You can then use ARCHLINUX_FOUND
#
MACRO(CHECK_ARCHLINUX)
  FIND_FILE(ARCHLINUX_FOUND arch-release
    PATHS /etc)
ENDMACRO(CHECK_ARCHLINUX)