# - Try to find libOURAPI
# Once done this will define
#
# libOURAPI_FOUND - system has libOURAPI
# libOURAPI_INCLUDE_DIRS - the libOURAPI include directories
# libOURAPI_LIBS - link these to use libOURAPI

find_path(libOURAPI_CORE_INCLUDE_DIR
	NAMES roadpoint.h
	PATHS /usr/include/OUR_API/core
	      /usr/local/include/OUR_API/core
	      ${PROJECT_SOURCE_DIR}/include/OUR_API/core
	      $ENV{INCLUDE}
)

find_path(libOURAPI_COMM_INCLUDE_DIR
	NAMES jointcmd.h
	PATHS /usr/include/OUR_API/comm
	      /usr/local/include/OUR_API/comm
	      ${PROJECT_SOURCE_DIR}/include/OUR_API/comm
	      $ENV{INCLUDE}
)

find_path(libOURAPI_KINEMATICS_INCLUDE_DIR
	NAMES ikfunc.h
	PATHS /usr/include/OUR_API/kinematics
	      /usr/local/include/OUR_API/kinematics
	      ${PROJECT_SOURCE_DIR}/include/OUR_API/kinematics
	      $ENV{INCLUDE}
)


set(libOURAPI_INCLUDE_DIRS ${libOURAPI_CORE_INCLUDE_DIR} ${libOURAPI_COMM_INCLUDE_DIR} ${libOURAPI_KINEMATICS_INCLUDE_DIR})

find_library(libOUR_CORE_LIB
	NAMES OUR_core
	PATHS /usr/lib/OUR_API
	      /usr/local/lib/OUR_API
 	      ${PROJECT_SOURCE_DIR}/lib/OUR_API
)

find_library(libOUR_COMM_LIB
	NAMES OUR_comm
	PATHS /usr/lib/OUR_API
	      /usr/local/lib/OUR_API
 	      ${PROJECT_SOURCE_DIR}/lib/OUR_API
)

find_library(libOUR_ik_LIB
	NAMES ik
	PATHS /usr/lib/OUR_API
	      /usr/local/lib/OUR_API
 	      ${PROJECT_SOURCE_DIR}/lib/OUR_API
)

find_library(libOUR_KINEMATICS_LIB
	NAMES OUR_kinematics
	PATHS /usr/lib/OUR_API
	      /usr/local/lib/OUR_API
 	      ${PROJECT_SOURCE_DIR}/lib/OUR_API
)

set(libOURAPI_LIBS ${libOUR_CORE_LIB} ${libOUR_COMM_LIB} ${libOUR_ik_LIB} ${libOUR_KINEMATICS_LIB})

if(libOURAPI_INCLUDE_DIRS)
	message(STATUS "Found OUR_API include dir: ${libOURAPI_INCLUDE_DIRS}")
else(libOURAPI_INCLUDE_DIRS)
	message(STATUS "Could NOT find OUR_API headers.")
endif(libOURAPI_INCLUDE_DIRS)

if(libOURAPI_LIBS)
	message(STATUS "Found OUR_API library: ${libOURAPI_LIBS}")
else(libOURAPI_LIBS)
	message(STATUS "Could NOT find libOUR_API library.")
endif(libOURAPI_LIBS)

if(libOURAPI_INCLUDE_DIRS AND libOURAPI_LIBS)
	set(libOURAPI_FOUND TRUE)
else(libOURAPI_INCLUDE_DIRS AND libOURAPI_LIBS)
	set(libOURAPI_FOUND FALSE)
	if(libOURAPI_FIND_REQUIRED)
		message(FATAL_ERROR "Could not find OUR_API.")
	endif(libOURAPI_FIND_REQUIRED)
endif(libOURAPI_INCLUDE_DIRS AND libOURAPI_LIBS)
