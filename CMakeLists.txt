#We use version 3.1 so we can use the set(CMAKE_CXX_STANDARD 14) call.  If you have earlier versions of CMAKE you can find workarounds here: http://stackoverflow.com/questions/10851247/how-to-activate-c-11-in-cmake
cmake_minimum_required (VERSION 3.1) 

#This allows cmaketools to integrate with Visual Studio Code.  Not required for build system.
include(CMakeToolsHelpers OPTIONAL)

project (KalmanFilter)

#Can probably use C++11 as well.
set (CMAKE_CXX_STANDARD 14)

#Set build directory to /build
set(CMAKE_BUILD_DIRECTORY build/)

#Set local include directories
include_directories(src/include)

#Set Eigen include directory
INCLUDE_DIRECTORIES ( "$ENV{EIGEN3_INCLUDE_DIR}" )

#Setup Boost
set(BOOST_MIN_VERSION 1.4.5)
set(Boost_USE_STATIC_LIBS ON) 
set(Boost_USE_MULTITHREADED OFF)  
set(Boost_USE_STATIC_RUNTIME OFF) 
find_package(Boost ${BOOST_MIN_VERSION} REQUIRED iostreams system filesystem)


if(Boost_FOUND)
	include_directories(${Boost_INCLUDE_DIRS}) 
	add_executable(voltage_kalman examples/voltage_kalman.cpp src/Plotter.cpp src/WhiteNoiseSystem.cpp src/kalman.cpp)
	add_executable(kinematics_kalman examples/kinematics_kalman.cpp src/Plotter.cpp src/WhiteNoiseSystem.cpp src/kalman.cpp)
	target_link_libraries(voltage_kalman ${Boost_LIBRARIES})
	target_link_libraries(kinematics_kalman ${Boost_LIBRARIES})
	#Set where different executables are put
	set_target_properties( voltage_kalman PROPERTIES RUNTIME_OUTPUT_DIRECTORY examples/ )
	set_target_properties( kinematics_kalman PROPERTIES RUNTIME_OUTPUT_DIRECTORY examples/ )
endif()
