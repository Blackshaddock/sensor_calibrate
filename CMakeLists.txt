cmake_minimum_required(VERSION 2.8.7)

project(sensor_calibration)

set(CMAKE_CXX_FLAGS "-std=c++11")

add_definitions(-DUNICODE)
add_definitions(-D_UNICODE)
add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

set(3RDPART_ROOT ${CMAKE_CURRENT_LIST_DIR}/3rdParty)
find_package(3rdparty REQUIRED HINTS ${3RDPART_ROOT})

set(CMAKE_CXX_FLAGS "/EHsc ${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS_MINSIZEREL "${CMAKE_CXX_FLAGS_MINSIZEREL} /Od /Zi")
set(CMAKE_EXE_LINKER_FLAGS_MINSIZEREL "${CMAKE_SHARED_LINKER_FLAGS_MINSIZEREL} /DEBUG")

#ADD_SUBDIRECTORY(plane_detection)

include_directories(${3RDPARTY_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR})
message(${3RDPARTY_INCLUDE_DIRS})


file(GLOB INTERNAL_SRC "internal/*.h" "internal/*.cpp" "internal/*.hpp")
file(GLOB EXAMPLE_SRC "example/*.h" "example/*.cpp" "example/*.hpp")
file(GLOB MODULE_SRC "module/*.h" "module/*.cpp" "module/*.hpp")
#file(GLOB PLANE_DETECT_FILE "plane_detection/*.h" "plane_detection/*.cpp" "plane_detection/*.hpp")
file(GLOB IO_SRC "io/*.h"  "io/*.cpp")
file(GLOB SCAN_SRC "scan/*.h" "scan/*.cpp")
file(GLOB CONFIG_FILE "config/*.yaml")
file(GLOB NDTOMP_SRC "ndt_omp/*.h" "ndt_omp/*.cpp" "ndt_omp/*.hpp")
file(GLOB ODOMETRY_SRC "odometry/*.h"  "odometry/*.cpp")
file(GLOB KDTREE_SRC "kdtree/*.h" "kdtree/*.cpp")
file(GLOB R3LIVE_SRC "r3live/*.h" "r3live/*.cpp" )
file(GLOB ColorPoint_SRC "color_point/*.h" "color_point/*.cpp")
file(GLOB General_Client "general_client/*.h" "general_client/*.cpp" )

source_group(internal FILES ${INTERNAL_SRC})
source_group(example FILES ${EXAMPLE_SRC})
source_group(module FILES ${MODULE_SRC})
#source_group(plane_detection FILES ${PLANE_DETECT_FILE})
source_group(io FILES ${IO_SRC})
source_group(scan FILES ${SCAN_SRC})
source_group(config FILES ${CONFIG_FILE})
source_group(ndtomp FILES ${NDTOMP_SRC})
source_group(odometry FILES ${ODOMETRY_SRC})
source_group(kdtree FILES ${KDTREE_SRC})
source_group(r3live FILES ${R3LIVE_SRC})
source_group(colorpoint FILES ${ColorPoint_SRC})
source_group(general_client FILES ${General_Client})

# ADD_SUBDIRECTORY(PlaneDetection)


add_executable(2000RWebsocket example/GeoSunR2000Websocket.cpp ${General_Client})
set_property(TARGET 2000RWebsocket PROPERTY VS_DEBUGGER_WORKING_DIRECTORY ${3rdparty_DIR}/VS2015_x64/bin/Release)
target_link_libraries(2000RWebsocket  ${3RDPARTY_LIBRARIES} )
target_compile_options(2000RWebsocket 
	PRIVATE 
	$<$<CXX_COMPILER_ID:MSVC>:/bigobj> 
	)

add_executable(lidar_motor_calib example/LidarMotorCalibExample.cpp ${INTERNAL_SRC} ${MODULE_SRC} ${IO_SRC} ${SCAN_SRC} ${CONFIG_FILE} ${NDTOMP_SRC} ${KDTREE_SRC} ${R3LIVE_SRC} ${ColorPoint_SRC})
set_property(TARGET lidar_motor_calib PROPERTY VS_DEBUGGER_WORKING_DIRECTORY ${3rdparty_DIR}/VS2015_x64/bin/Release)
target_link_libraries(lidar_motor_calib  ${3RDPARTY_LIBRARIES} )
target_compile_options(lidar_motor_calib 
	PRIVATE 
	$<$<CXX_COMPILER_ID:MSVC>:/bigobj> 
	)
#add_executable(camera_lidar_tracker example/CameraLidarTrackExample.cpp ${INTERNAL_SRC} ${MODULE_SRC} ${IO_SRC})
#set_property(TARGET camera_lidar_tracker PROPERTY VS_DEBUGGER_WORKING_DIRECTORY ${3rdparty_DIR}/VS2015_x64/bin/Release)
#target_link_libraries(camera_lidar_tracker  ${3RDPARTY_LIBRARIES} planeDetect)
#target_compile_options(camera_lidar_tracker 
#	PRIVATE 
#	$<$<CXX_COMPILER_ID:MSVC>:/bigobj> 
#	)
#add_executable(lidar_camera_calib example/LidarCameraCalibExample.cpp ${INTERNAL_SRC} ${MODULE_SRC})
#set_property(TARGET lidar_camera_calib PROPERTY VS_DEBUGGER_WORKING_DIRECTORY ${3rdparty_DIR}/VS2015_x64/bin/Release)
#target_link_libraries(lidar_camera_calib  ${3RDPARTY_LIBRARIES} planeDetect)

