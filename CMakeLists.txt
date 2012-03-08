# LJM

cmake_minimum_required( VERSION 2.8 )
project ( LJM )

set (CMAKE_INSTALL_PREFIX /usr/local )
set (CMAKE_CXX_FLAGS "-g -O0 -Wall -Wextra -Wpointer-arith -fno-default-inline -fno-implicit-inline-templates -DwxUSE_THREADS")

include (${CMAKE_INSTALL_PREFIX}/share/dart/DART_config.cmake)
include (${CMAKE_INSTALL_PREFIX}/share/grip/GRIP_config.cmake)

find_package( PCL 1.1 REQUIRED )

include_directories( /usr/include/eigen3
					 ${PCL_INCLUDE_DIRS}
					. )

link_directories( ${PCL_LIBRARY_DIRS} )
add_definitions( ${PCL_DEFINITIONS} ) 

message ( STATUS "Creating Makefile for LJM2" )
set( CMAKE_CXX_FLAGS "-L /usr/local/lib" )

set( SRC_MY_FUNCTIONS myFunctions/CheckObject.cpp
					  myFunctions/CheckProcess.cpp )

set( HDR_MY_FUNCTIONS myFunctions/CheckObject.h
					  myFunctions/CheckProcess.h )

set( SRC_JT JTFollower/JTFollower.cpp )
set( HDR_JT JTFollower/JTFollower.h )

set( SRC_LJM2 LJM2/LJM2.cpp 
	 		  LJM2/searchFunctionsFree.cpp
			  LJM2/searchFunctionsSurface.cpp
			  kdtree/kdtree.c )

set( HDR_LJM2 LJM2/LJM2.h )

set( SRC_FILES   LJM2TabApp.cpp
 		   		 LJM2Tab.cpp )

set ( HDR_FILES  LJM2Tab.h )

set (wxWidgets_USE_LIBS base core gl)
find_package (wxWidgets)
if (wxWidgets_FOUND)
  include (${wxWidgets_USE_FILE})
  include_directories(${DART_HEADERS} ${GRIP_HEADERS} ${WXWIDGETS_SHARED_INCLUDE_DIRS})
  link_directories(${CMAKE_SOURCE_DIR}/lib ${GRIP_LIBDIR} ${WXWIDGETS_SHARED_LIBRARY_DIRS} )

  add_executable ( LJM2Tab ${SRC_FILES} ${SRC_MY_FUNCTIONS} ${SRC_LJM2} ${HDR_FILES} ${HDR_MY_FUNCTIONS} ${HDR_LJM2} ${SRC_JT} ${HDR_JT})
  target_link_libraries( LJM2Tab ${PCL_LIBRARIES}
								  ${GRIP_LIBRARIES} 
								  ${DART_LIBRARIES} 
								  ${wxWidgets_LIBRARIES} )	

   
else (wxWidgets_FOUND)
  message ("wxWidgets not found!")
endif (wxWidgets_FOUND)
