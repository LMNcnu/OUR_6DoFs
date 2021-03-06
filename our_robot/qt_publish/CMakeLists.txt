# find pthread
find_package(Threads)

# find OUR_API
#FIND_PACKAGE(OUR_API REQUIRED)

# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)
# Instruct CMake to run moc automatically when needed  
set(CMAKE_AUTOMOC ON)

set(CMAKE_PREFIX_PATH "/home/lmn/qt/Qt5.3.2/5.3/gcc/lib/cmake")

#include_directories(${Boost_INCLUDE_DIRS})

# Find the QtWidgets library
find_package(Qt5Widgets)

# Add the include directories for the Qt 5 Widgets module to
# the compile lines.
include_directories(${Qt5Widgets_INCLUDE_DIRS} ${libOURAPI_INCLUDE_DIRS})

# Use the compile definitions defined in the Qt 5 Widgets module
add_definitions(${Qt5Widgets_DEFINITIONS})

# Add compiler flags for building executables (-fPIE)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${Qt5Widgets_EXECUTABLE_COMPILE_FLAGS}")

qt5_wrap_ui(ui_FILES src/mainwindow.ui)

add_executable(qt_publish 
	       src/main.cpp
               src/mainwindow.h src/mainwindow.cpp src/config.h
	       ${ui_FILES})

#Link the helloworld executable to the Qt 5 widgets library.
target_link_libraries(qt_publish Qt5::Widgets ${catkin_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT} ${libOURAPI_LIBS})
