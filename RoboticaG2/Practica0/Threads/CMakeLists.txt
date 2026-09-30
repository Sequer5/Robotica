cmake_minimum_required(VERSION 3.16)
project(ejemplo1)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

MESSAGE( STATUS "COMPILING WITH QT6" )
find_package(Qt6 REQUIRED COMPONENTS Gui Widgets Sql Core OpenGL Xml)
set(QT_LIBRARIES Qt6::Gui Qt6::Core Qt6::Widgets Qt6::Sql Qt6::OpenGL)

find_package(Threads REQUIRED)

include_directories(${QT_INCLUDES} ${CMAKE_CURRENT_BINARY_DIR})

# Sources set
SET ( SOURCES ${SOURCES} ejemplo1.cpp main.cpp
        ejemplo1.cpp)
# Headers set
SET ( HEADERS ${HEADERS} ejemplo1.h timer.h
        ejemplo1.cpp)

SET( CMAKE_AUTOMOC ON )
SET( CMAKE_AUTOUIC ON )

ADD_EXECUTABLE( ejemplo1 ${SOURCES} ${HEADERS} counterDlg.ui
        ejemplo1.cpp)
TARGET_LINK_LIBRARIES( ejemplo1 ${QT_LIBRARIES} Threads::Threads )
INSTALL(TARGETS ejemplo1 PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE )
