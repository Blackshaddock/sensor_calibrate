PROJECT(planeDetect)

STRING(TOUPPER ${PROJECT_NAME} PROJECT_NAME_CAPITALIZED)
CMAKE_MINIMUM_REQUIRED(VERSION 2.8.3)
ADD_DEFINITIONS(-DUNICODE)

INCLUDE_DIRECTORIES(${CMAKE_CURRENT_BINARY_DIR})

#FIND_PACKAGE(OpenGL REQUIRED)
FILE(GLOB_RECURSE HDRS_FILES *.h *.hpp)
FILE(GLOB_RECURSE SRCS_FILES *.cpp *.c *.cxx)
#FILE(GLOB_RECURSE UI_FILES *.ui)
#FILE(GLOB CD_FILES *.cd)
#FILE(GLOB_RECURSE RSCS *.qrc)

#QT5_WRAP_CPP(MOC_SRCS ${HDRS_FILES})
#QT5_ADD_RESOURCES(RSC_SRCS ${RSCS})
#QT5_WRAP_UI(UI_HDRS ${UI_FILES})

#opencv
#FIND_PACKAGE(OpenCV REQUIRED)
#include_directories(${OpenCV_ROOT}/include)


INCLUDE_DIRECTORIES(${CMAKE_BINARY_DIR} ${PROJECT_SOURCE_DIR})
INCLUDE_DIRECTORIES(${CMAKE_CURRENT_BINARY_DIR})
SET(FILES_TO_TRANSLATE ${FILES_TO_TRANSLATE} ${SRCS_FILES} ${HDRS_FILES} PARENT_SCOPE)

#SOURCE_GROUP("UI Files" FILES ${UI_FILES})
#SOURCE_GROUP("Generated Files" FILES ${MOC_SRCS} ${UI_HDRS} ${RSC_SRCS})
#SOURCE_GROUP("Class Diagrams" FILES ${CD_FILES} )
#SOURCE_GROUP("Resources" FILES ${RSCS} )

INCLUDE_DIRECTORIES(${CMAKE_CURRENT_BINARY_DIR})

#ADD_EXECUTABLE(${PROJECT_NAME} )
set_source_files_properties(${CD_FILES} PROPERTIES HEADER_FILE_ONLY TRUE) #DO NOT COMPILE CS FILES

ADD_LIBRARY(${PROJECT_NAME} STATIC ${SRCS_FILES} ${HDRS_FILES})
TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${3RDPARTY_LIBRARIES})
SET_TARGET_PROPERTIES(${PROJECT_NAME} PROPERTIES DEFINE_SYMBOL ${PROJECT_NAME_CAPITALIZED}_LIB)#
SET_TARGET_PROPERTIES(${PROJECT_NAME} PROPERTIES FOLDER SRC)
