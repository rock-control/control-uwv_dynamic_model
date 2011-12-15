# CMakeLists.txt has to be located in the project folder and cmake has to be
# executed from 'project/build' with 'cmake ../'.

cmake_minimum_required(VERSION 2.6)

project(uwv_dynamic_model)
set(PROJECT_VERSION 1.0)
set(PROJECT_DESCRIPTION "Dynamic model for an underwater vehicle")
INCLUDE(FindPkgConfig)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake")
#include(CommonSetup)

# Include headers within source
# If you create subdirectories within source include headers with subdirectory, i.e. 
# #include "subdirectory/myheader.h"
include_directories(${PROJECT_SOURCE_DIR}/external/include)
include_directories(${PROJECT_SOURCE_DIR}/src)


#INCLUDE_DIRECTORIES(BEFORE ${PROJECT_SOURCE_DIR})

# command line output on, you can also use make VERBOSE=1
# set(CMAKE_VERBOSE_MAKEFILE on)

### Uncomment if you want to use Qt
# set(QT_SEARCH_PATH /opt)
# find_package(Qt4 4.5 REQUIRED QtTest QtThread QtXml QtNetwork QtGui)

# Definitions to use when compiling code that uses Qt
#add_definitions(${QT_DEFINITIONS})

# Set your desired Qt-Version here
#include_directories(${QT_INCLUDE_DIR})

# Path to a Cmake file that can be included to compile Qt4 applications and libraries
#include(${QT_USE_FILE})


### Uncomment if you want to use Boost. Use the first find_package stanza to
### search for headers only, and the second one if you need binary libraries
#set(Boost_ADDITIONAL_VERSION 1.40 1.41)
#find_package(Boost REQUIRED)
#find_package(Boost REQUIRED COMPONENTS graph filesystem)
#include_directories(${Boost_INCLUDE_DIRS})

### Update external libraries that are available
#set(EXTERNAL_LIBS ${Boost_LIBRARIES} ${QT_LIBRARIES})

#INCLUDE_DIRECTORIES(/usr/include/eigen2)
#find_package(Eigen2 REQUIRED)
#include_directories(${EIGEN2_INCLUDE_DIR})

CONFIGURE_FILE(configuration/uwv_dynamic_model.pc.in uwv_dynamic_model.pc @ONLY)
# Process CMakeLists.txt in the following subdirectory
add_subdirectory(src)

