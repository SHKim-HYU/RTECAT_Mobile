###
# Set minimum version of CMake. Since command 'project' use
# VERSION sub-option we need at least 3.0.
# Note: If you use 2.6 or 2.4, God kills a kitten. Seriously.
cmake_minimum_required(VERSION 3.8)

# THIS HAS TO COME BEFORE THE PROJECT LINE
#set(CMAKE_C_COMPILER "gcc")
#set(CMAKE_CXX_COMPILER "gcc")

####
# Set variables:
#   * PROJECT_NAME
#   * PROJECT_VERSION
project(RTECAT_Mobile VERSION 2.0.0 LANGUAGES C CXX)

# Use MACOSX_RPATH by default on OS X. This was added in CMake 2.8.12 and
# became default in CMake 3.0. Explicitly setting this policy is necessary to
# suppress a warning in CMake 3.0 and above.
if(POLICY CMP0042)
    cmake_policy(SET CMP0042 NEW)
endif()

# Simplify variable reference and escape sequence evaluation. This was added in
# CMake 3.1. Explicitly setting this policy is necessary to suppress a warning
# in CMake 3.1 and above.
if(POLICY CMP0053)
    cmake_policy(SET CMP0053 NEW)
endif()

cmake_policy(SET CMP0022 NEW)
if (POLICY CMP0048)
    cmake_policy(SET CMP0048 NEW)
endif()

if (POLICY CMP0025)
    cmake_policy(SET CMP0025 NEW)
endif()

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)
include(FindPkgConfig)
include(modules/CMakeFunctions.cmake)

set(CMAKE_INSTALL_PREFIX ${PROJECT_SOURCE_DIR})
set(PROJ_INSTALL_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/bin/inc)
set(PROJ_INSTALL_BIN_DIR ${PROJECT_SOURCE_DIR}/bin)
set(PROJ_INSTALL_LIB_DIR ${PROJECT_SOURCE_DIR}/lib)

set(CMAKE_DEBUG_POSTFIX "d")
set(CMAKE_MODULE_PATH  ${CMAKE_CURRENT_SOURCE_DIR}/modules)
set(BUILD_SHARED_LIBS TRUE)

#===============================================================================
# Build options
#===============================================================================
#option(BUILD_DOC "Build Documentation" off)
option(RTECAT_Mobile_VERBOSE "Whether print detailed information in CMake process" ON)
option(RTECAT_Mobile_SHARED_LIBS "Build shared libraries" ON)
#===============================================================================
# Build type settings
#===============================================================================
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build, options are: Debug | Release | RelWithDebInfo | MinSizeRel" FORCE)
endif()
string(TOUPPER ${CMAKE_BUILD_TYPE} CMAKE_BUILD_TYPE_UPPERCASE)

set(BUILD_TYPE_DEBUG TRUE)
set(BUILD_TYPE_RELEASE TRUE)
set(BUILD_TYPE_RELWITHDEBINFO FALSE)
set(BUILD_TYPE_MINSIZEREL FALSE)

if("${CMAKE_BUILD_TYPE_UPPERCASE}" STREQUAL "DEBUG")
    set(BUILD_TYPE_DEBUG TRUE)
elseif("${CMAKE_BUILD_TYPE_UPPERCASE}" STREQUAL "RELEASE")
    set(BUILD_TYPE_RELEASE TRUE)
elseif("${CMAKE_BUILD_TYPE_UPPERCASE}" STREQUAL "RELWITHDEBINFO")
    set(BUILD_TYPE_RELWITHDEBINFO TRUE)
elseif("${CMAKE_BUILD_TYPE_UPPERCASE}" STREQUAL "MINSIZEREL")
    set(BUILD_TYPE_MINSIZEREL TRUE)
else()
    message(WARNING "CMAKE_BUILD_TYPE ${CMAKE_BUILD_TYPE} unknown. Valid options are: Debug | Release | RelWithDebInfo | MinSizeRel")
endif()

#===============================================================================
# Compiler flags
#===============================================================================
include(CheckCXXCompilerFlag)
check_cxx_compiler_flag(-std=c++17 CXX17_SUPPORT)
if(CXX17_SUPPORT)
    set(CMAKE_CXX_STANDARD 17)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
    set(CMAKE_CXX_EXTENSIONS OFF)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
else()
    check_cxx_compiler_flag(-std=c++14 CXX14_SUPPORT)
    set(CMAKE_CXX_STANDARD 14)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
    set(CMAKE_CXX_EXTENSIONS OFF)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14" )
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -fPIC" )

# Find includes in the build directories and the include directory
set(CMAKE_INCLUDE_CURRENT_DIR ON)

execute_process(
        COMMAND ${CMAKE_CXX_COMPILER} -dumpfullversion -dumpversion OUTPUT_VARIABLE GCC_VERSION)
if(GCC_VERSION VERSION_LESS 5.3.1)
    message(FATAL_ERROR "The installed g++ version is ${GCC_VERSION}. ${PROJECT_NAME} requires g++ 5.3.1 or greater.")
endif()

# Find the QtWidgets library
find_package(Qt5 COMPONENTS Widgets Core Quick REQUIRED)

# Enable automatic UIC compilation
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_FLAGS_RELEASE "-O2 -DNDEBUG")
set(CMAKE_CXX_FLAGS_DEBUG "-g -fno-omit-frame-pointer -fno-inline-functions -fno-inline-functions-called-once -fno-optimize-sibling-calls")
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELEASE} ${CMAKE_CXX_FLAGS_DEBUG}")
set(CMAKE_CXX_FLAGS_PROFILE "${CMAKE_CXX_FLAGS_DEBUG} -pg")
set(CMAKE_SHARED_LINKER_FLAGS "-Wl,--no-undefined")

set(ECAT_HEADERS_DIR ${PROJECT_SOURCE_DIR}/include/EcatSystem)
set(INTP_HEADERS_DIR ${PROJECT_SOURCE_DIR}/include/Interpolator)
set(ROBOT_HEADERS_DIR ${PROJECT_SOURCE_DIR}/include/Robot)
set(QTWIDGET_HEADERS_DIR ${PROJECT_SOURCE_DIR}/include/QtWidgets)
set(DARKSTYLE_HEADERS_DIR ${PROJECT_SOURCE_DIR}/include/DarkStyleWindow)

include_directories(${PROJ_INSTALL_INCLUDE_DIR})
include_directories(
        ${KERNELHEADERS_DIR}
        ${ECAT_HEADERS_DIR}
        ${INTP_HEADERS_DIR}
        ${ROBOT_HEADERS_DIR}
        ${QTWIDGET_HEADERS_DIR}
        ${DARKSTYLE_HEADERS_DIR}
)

# ECAT
add_subdirectory(include/EcatSystem)

add_subdirectory(include/Interpolator)
add_subdirectory(include/Robot)

# QT
add_subdirectory(include/QtWidgets)
add_subdirectory(include/DarkStyleWindow)


# Source files
set(CTRL_SOURCE_FILES
    src/RTECAT_Mobile_Client.cpp
    src/mainwindow.cpp
    src/mainwindow.ui
)
set(SIM_SOURCE_FILES
    src/bullet_server.cpp
)
#bullet3
set(BULLET_PHYSICS_SOURCE_DIR  "/opt/bullet3")
include_directories(
    ${BULLET_PHYSICS_SOURCE_DIR}/src
    ${BULLET_PHYSICS_SOURCE_DIR}/examples
    ${BULLET_PHYSICS_SOURCE_DIR}/examples/SharedMemory
    ${BULLET_PHYSICS_SOURCE_DIR}/examples/RobotSimulator		
    ${BULLET_PHYSICS_SOURCE_DIR}/examples/ThirdPartyLibs
    ${BULLET_PHYSICS_SOURCE_DIR}/examples/ThirdPartyLibs/enet/include
    ${BULLET_PHYSICS_SOURCE_DIR}/examples/ThirdPartyLibs/clsocket/src
    ${BULLET_PHYSICS_SOURCE_DIR}/examples/CommonInterfaces
)
SET(RobotSimulator_SRCS_GUI
    ${BULLET_PHYSICS_SOURCE_DIR}/examples/RobotSimulator/b3RobotSimulatorClientAPI.cpp
    ${BULLET_PHYSICS_SOURCE_DIR}/examples/RobotSimulator/b3RobotSimulatorClientAPI.h	
    ${BULLET_PHYSICS_SOURCE_DIR}/examples/RobotSimulator/MinitaurSetup.cpp
    ${BULLET_PHYSICS_SOURCE_DIR}/examples/RobotSimulator/MinitaurSetup.h
    ${BULLET_PHYSICS_SOURCE_DIR}/examples/ExampleBrowser/InProcessExampleBrowser.cpp
    ${BULLET_PHYSICS_SOURCE_DIR}/examples/SharedMemory/GraphicsServerExample.cpp
    ${BULLET_PHYSICS_SOURCE_DIR}/examples/SharedMemory/GraphicsClientExample.cpp
    ${BULLET_PHYSICS_SOURCE_DIR}/examples/SharedMemory/RemoteGUIHelper.cpp
    ${BULLET_PHYSICS_SOURCE_DIR}/examples/SharedMemory/GraphicsServerExample.h
    ${BULLET_PHYSICS_SOURCE_DIR}/examples/SharedMemory/GraphicsClientExample.h
    ${BULLET_PHYSICS_SOURCE_DIR}/examples/SharedMemory/RemoteGUIHelper.h
    ${BULLET_PHYSICS_SOURCE_DIR}/examples/SharedMemory/GraphicsSharedMemoryCommands.h
    ${BULLET_PHYSICS_SOURCE_DIR}/examples/SharedMemory/GraphicsSharedMemoryPublic.h
    ${BULLET_PHYSICS_SOURCE_DIR}/examples/SharedMemory/PhysicsServerExample.cpp
    ${BULLET_PHYSICS_SOURCE_DIR}/examples/SharedMemory/PhysicsServerExampleBullet2.cpp
    ${BULLET_PHYSICS_SOURCE_DIR}/examples/SharedMemory/SharedMemoryInProcessPhysicsC_API.cpp	
)
# Add a executable with the above sources
set(CTRL_EXECUTABLE ${PROJECT_NAME}_CTRL)
set(SIM_EXECUTABLE ${PROJECT_NAME}_SIM)
add_executable(${CTRL_EXECUTABLE} ${CTRL_SOURCE_FILES})
add_executable(${SIM_EXECUTABLE} ${SIM_SOURCE_FILES} ${RobotSimulator_SRCS_GUI})

target_link_libraries(${CTRL_EXECUTABLE}
        sub::ECAT
        sub::Interpolator
        sub::Robot
        dl
        sub::QtWidgets
        sub::DarkStyleWindow
        Qt5::Widgets 
        Qt5::Core
        Qt5::Gui
        Qt5::Network
        Qt5::Quick
)
target_link_libraries(${SIM_EXECUTABLE}
    sub::Robot
    pthread
)

# jsoncpp
find_package(jsoncpp REQUIRED)
target_link_libraries(${CTRL_EXECUTABLE} jsoncpp_lib)
list(APPEND INCLUDE_DIRS_FOR_DYNAMIC_VIEW ${jSONCPP_INCLUDE_DIRS})

# Test for XDDP
# Source files
set(XDDP_NRT_SOURCE_FILES
    src/xddp_nrt.cpp
)
set(XDDP_NRT ${PROJECT_NAME}_XDDP_NRT)
add_executable(${XDDP_NRT} ${XDDP_NRT_SOURCE_FILES})

target_link_libraries(${XDDP_NRT}
    pthread
)

# xenomai
find_package(Xenomai 3.2.3 REQUIRED COBALT)
message(STATUS "Xenomai found with theses skins: ${Xenomai_SKINS}")
if(Xenomai_FOUND)
    target_include_directories(${CTRL_EXECUTABLE} PUBLIC ${KERNELHEADERS_DIR})
    foreach(comp_lists ${Xenomai_FOUND_COMP})
        #message(STATUS "Xenomai ${comp_lists} includes : ${Xenomai_${comp_lists}_INCLUDE_DIRS}" )
        #message(STATUS "Xenomai ${comp_lists} definitions: ${Xenomai_${comp_lists}_DEFINITIONS}")
        #message(STATUS "Xenomai ${comp_lists} LIBRARY_DIRS : ${Xenomai_${comp_lists}_LIBRARY_DIRS} ")
        #message(STATUS "Xenomai ${comp_lists} LIBRARIES : ${Xenomai_${comp_lists}_LIBRARIES}")
        #message(STATUS "Xenomai ${comp_lists} LDFLAGS : ${Xenomai_${comp_lists}_LDFLAGS}")
        #message(STATUS "Xenomai ${comp_lists} DL_FLAGS : ${Xenomai_${comp_lists}_DL_FLAGS}")

        target_include_directories(${CTRL_EXECUTABLE} PUBLIC ${Xenomai_${comp_lists}_INCLUDE_DIRS})
        # target_link_libraries(${PROJECT_NAME} PUBLIC ${Xenomai_${comp_lists}_LIBRARY_DIRS} ${Xenomai_${comp_lists}_LIBRARIES})
        target_link_libraries(${CTRL_EXECUTABLE} ${Xenomai_${comp_lists}_DL_FLAGS})
        set_target_properties(${CTRL_EXECUTABLE} PROPERTIES COMPILE_FLAGS ${Xenomai_${comp_lists}_DEFINITIONS})
    endforeach(comp_lists ${Xenomai_SKINS})
endif(Xenomai_FOUND)

add_compile_definitions(__BULLET__)
find_package(Bullet)
if (BULLET_FOUND)
    MESSAGE(STATUS "BULLET_INCLUDE_DIRS = ${BULLET_INCLUDE_DIRS}")
    MESSAGE(STATUS "BULLET_LIBRARIES = ${BULLET_LIBRARIES}")
    target_include_directories(${SIM_EXECUTABLE} PUBLIC ${BULLET_INCLUDE_DIRS})
    target_link_libraries(${SIM_EXECUTABLE} ${BULLET_LIBRARIES})
endif()
target_compile_definitions(${SIM_EXECUTABLE} PUBLIC -DBT_USE_DOUBLE_PRECISION)


# casadi
add_compile_definitions(__CASADI__)
find_package(CASADI REQUIRED)
if (CASADI_FOUND)
    MESSAGE(STATUS "CASADI_INCLUDE_DIRS = ${CASADI_INCLUDE_DIR}")
    MESSAGE(STATUS "CASADI_LIBRARIES = ${CASADI_LIBRARIES}")
    target_include_directories(${CTRL_EXECUTABLE} PUBLIC ${CASADI_INCLUDE_DIR})
    target_link_libraries(${CTRL_EXECUTABLE} ${CASADI_LIBRARIES})
endif ()

set( INDY7_CASADI
    indy7_M
    indy7_Minv
    indy7_C
    indy7_G
    indy7_id
    indy7_fd
    indy7_fk
    indy7_fk_ee
    indy7_fkrot_ee
    indy7_J_s
    indy7_J_b
    indy7_J_fd
    indy7_J_id
)

set( INDYRP_CASADI
    indyrp2_M
    indyrp2_Minv
    indyrp2_C
    indyrp2_G
    indyrp2_id
    indyrp2_fd
    indyrp2_fk
    indyrp2_fk_ee
    indyrp2_fkrot_ee
    indyrp2_J_s
    indyrp2_J_b
    indyrp2_J_fd
    indyrp2_J_id
)

link_directories(
    "${CMAKE_CURRENT_SOURCE_DIR}/lib/URDF2CASADI/indy7"
    "${CMAKE_CURRENT_SOURCE_DIR}/lib/URDF2CASADI/indyRP"
)

# check if Doxygen is installed
#find_package(Doxygen)
#if (DOXYGEN_FOUND)
#    # set input and output files
#    set(DOXYGEN_IN ${CMAKE_CURRENT_SOURCE_DIR}/ doc/Doxyfile.in)
#    set(DOXYGEN_OUT ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile)
#
#    # request to configure the file
#    configure_file(${DOXYGEN_IN} ${DOXYGEN_OUT} @ONLY)
#    message("Doxygen build started")
#
#    # note the option ALL which allows to build the docs together with the application
#    add_custom_target( doc_doxygen ALL
#                       COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYGEN_OUT}
#                       WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
#                       COMMENT "Generating API documentation with Doxygen"
#                       VERBATIM )
#else (DOXYGEN_FOUND)
#    message("Doxygen need to be installed to generate the doxygen documentation")
#endif (DOXYGEN_FOUND)

#===============================================================================
# Print build summary
#===============================================================================
if(RTECAT_Mobile_VERBOSE)
    message(STATUS "")
    message(STATUS "[ Build summary ]")
    message(STATUS "CMAKE_GENERATOR  : ${CMAKE_GENERATOR}")
    message(STATUS "Compiler ID      : ${CMAKE_CXX_COMPILER_ID}")
    message(STATUS "Compiler version : ${CMAKE_CXX_COMPILER_VERSION}")
    message(STATUS "Build type       : ${CMAKE_BUILD_TYPE}")
    message(STATUS "BUILD_SHARED_LIBS: ${BUILD_SHARED_LIBS}")
    message(STATUS "Install path     : ${CMAKE_INSTALL_PREFIX}")
    message(STATUS "CXX_FLAGS        : ${CMAKE_CXX_FLAGS}")
    if(${CMAKE_BUILD_TYPE_UPPERCASE} STREQUAL "RELEASE")
        message(STATUS "CXX_FLAGS_RELEASE: ${CMAKE_CXX_FLAGS_RELEASE}")
    elseif(${CMAKE_BUILD_TYPE_UPPERCASE} STREQUAL "DEBUG")
        message(STATUS "CXX_FLAGS_DEBUG  : ${CMAKE_CXX_FLAGS_DEBUG}")
    elseif(${CMAKE_BUILD_TYPE_UPPERCASE} STREQUAL "RELWITHDEBINFO")
        message(STATUS "CXX_FLAGS_RELWITHDEBINFO: ${CMAKE_CXX_FLAGS_RELWITHDEBINFO}")
    elseif(${CMAKE_BUILD_TYPE_UPPERCASE} STREQUAL "PROFILE")
        message(STATUS "CXX_FLAGS_PROFILE: ${CMAKE_CXX_FLAGS_PROFILE}")
    endif()
    message(STATUS "CMAKE_SOURCE_DIR : ${CMAKE_SOURCE_DIR}")
    message(STATUS "CMAKE_BINARY_DIR : ${CMAKE_BINARY_DIR}")
    message(STATUS "")
endif(RTECAT_Mobile_VERBOSE)
