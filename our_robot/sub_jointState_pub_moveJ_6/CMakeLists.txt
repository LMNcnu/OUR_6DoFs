# SET(CMAKE_INSTALL_PREFIX ${PROJECT_BINARY_DIR})

# find libpcan
# FIND_PACKAGE(pcan REQUIRED)

# find OUR_API
#FIND_PACKAGE(OUR_API REQUIRED)

#SET(LIBJOINTCONTROL_SRC jointcmd.h
#			jointcontrol.h jointcontrol.cpp
#			joint.h joint.cpp)

#INCLUDE_DIRECTORIES(${LIBPCAN_INCLUDE_DIR})

INCLUDE_DIRECTORIES(${libOURAPI_INCLUDE_DIRS})
ADD_EXECUTABLE(sub_jointState_pub_moveJ_6 src/sub_jointState_pub_moveJ_6.cpp)
TARGET_LINK_LIBRARIES(sub_jointState_pub_moveJ_6 ${catkin_LIBRARIES} ${libOURAPI_LIBS})

#ADD_EXECUTABLE(OUR_jointcontrol ${LIBJOINTCONTROL_SRC})
#TARGET_LINK_LIBRARIES(OUR_jointcontrol ${LIBPCAN_LIB} -DNO_RT)
