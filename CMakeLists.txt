cmake_minimum_required(VERSION 2.8.7)

project(sensor_calibration)

set(CMAKE_CXX_FLAGS "-std=c++11")

add_definitions(-DUNICODE)
add_definitions(-D_UNICODE)
#add_definitions(-DUSEPLANEDETECT)
set(3RDPART_ROOT ${CMAKE_CURRENT_LIST_DIR}/3rdParty)
find_package(3rdparty REQUIRED HINTS ${3RDPART_ROOT})

set(CMAKE_CXX_FLAGS "/EHsc ${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS_MINSIZEREL "${CMAKE_CXX_FLAGS_MINSIZEREL} /Od /Zi")
set(CMAKE_EXE_LINKER_FLAGS_MINSIZEREL "${CMAKE_SHARED_LINKER_FLAGS_MINSIZEREL} /DEBUG")

ADD_SUBDIRECTORY(plane_detection)

include_directories(${3RDPARTY_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR})

file(GLOB INTERNAL_SRC "internal/*.h" "internal/*.cpp" "internal/*.hpp")
file(GLOB EXAMPLE_SRC "example/*.h" "example/*.cpp" "example/*.hpp")
file(GLOB MODULE_SRC "module/*.h" "module/*.cpp" "module/*.hpp")
file(GLOB PLANE_DETECT_FILE "plane_detection/*.h" "plane_detection/*.cpp" "plane_detection/*.hpp")

message("src1: " ${INTERNAL_SRC})
message("src2: " ${EXAMPLE_SRC})
message("src3: " ${MODULE_SRC})
message("src4: " ${CONFIG_FILE})
message("src5: " ${PROJECT_SOURCE_DIR})
message("src6: " ${3RDPARTY_INCLUDE_DIRS})
message("src7: " ${planeDetect_INCLUDE_DIRS})

source_group(internal FILES ${INTERNAL_SRC})
source_group(example FILES ${EXAMPLE_SRC})
source_group(module FILES ${MODULE_SRC})
source_group(plane_detection FILES ${PLANE_DETECT_FILE})

# ADD_SUBDIRECTORY(PlaneDetection)

add_executable(lidar_motor_calib example/LidarMotorCalibExample.cpp ${INTERNAL_SRC} ${MODULE_SRC})
set_property(TARGET lidar_motor_calib PROPERTY VS_DEBUGGER_WORKING_DIRECTORY ${3rdparty_DIR}/VS2015_x64/bin/Release)
target_link_libraries(lidar_motor_calib  ${3RDPARTY_LIBRARIES} planeDetect)

add_executable(camera_lidar_tracker example/CameraLidarTrackExample.cpp ${INTERNAL_SRC} ${MODULE_SRC})
set_property(TARGET camera_lidar_tracker PROPERTY VS_DEBUGGER_WORKING_DIRECTORY ${3rdparty_DIR}/VS2015_x64/bin/Release)
target_link_libraries(camera_lidar_tracker  ${3RDPARTY_LIBRARIES} planeDetect)

#add_executable(lidar_camera_calib example/LidarCameraCalibExample.cpp ${INTERNAL_SRC} ${MODULE_SRC})
#set_property(TARGET lidar_camera_calib PROPERTY VS_DEBUGGER_WORKING_DIRECTORY ${3rdparty_DIR}/VS2015_x64/bin/Release)
#target_link_libraries(lidar_camera_calib  ${3RDPARTY_LIBRARIES} planeDetect)

