cmake_minimum_required(VERSION 3.10)
project(DeviceNetworkDemoCpp)
set(CMAKE_CXX_STANDARD 17)
include_directories(${CMAKE_SOURCE_DIR}/AntilatencySdk/Api)
add_executable(DeviceNetworkDemoCpp DeviceNetworkDemoCpp.cpp)

if (WIN32)
    file(GLOB ANTILATENCY_LIBS ${CMAKE_SOURCE_DIR}/AntilatencySdk/Bin/WindowsDesktop/x64/*.dll)
	
	if(MSVC)
		set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT ${PROJECT_NAME})
	endif(MSVC)
endif (WIN32)
if (UNIX)
    target_link_libraries(${PROJECT_NAME} dl stdc++fs pthread)

    EXECUTE_PROCESS( COMMAND uname -m COMMAND tr -d '\n' OUTPUT_VARIABLE ARCHITECTURE )
    message( STATUS "Architecture: ${ARCHITECTURE}" )

    if( ${ARCHITECTURE} STREQUAL "aarch64" )
        file(GLOB ANTILATENCY_LIBS ${CMAKE_SOURCE_DIR}/AntilatencySdk/Bin/Linux/aarch64_linux_gnu/*.so)
    elseif( ${ARCHITECTURE} STREQUAL "armv7l" )
        file(GLOB ANTILATENCY_LIBS ${CMAKE_SOURCE_DIR}/AntilatencySdk/Bin/Linux/arm_linux_gnueabihf/*.so)
    elseif( ${ARCHITECTURE} STREQUAL "x86_64" )
        file(GLOB ANTILATENCY_LIBS ${CMAKE_SOURCE_DIR}/AntilatencySdk/Bin/Linux/x86_64/*.so)
    endif()

endif (UNIX)

add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different ${ANTILATENCY_LIBS} $<TARGET_FILE_DIR:${PROJECT_NAME}>)