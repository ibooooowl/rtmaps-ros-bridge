cmake_minimum_required(VERSION 2.8.12)


set(PCK rtmaps_ros_bridge)
project(${PCK})

# Set the RTMaps SDK directory
if (NOT DEFINED RTMAPS_SDKDIR)
    if (DEFINED ENV{RTMAPS_SDKDIR})
        set(RTMAPS_SDKDIR "$ENV{RTMAPS_SDKDIR}" CACHE PATH "")  # Use the default RTMaps installation
    else()
        message(FATAL_ERROR "RTMAPS_SDKDIR not defined. Either pass -D\"RTMAPS_SDKDIR=...\" to CMake or define an RTMAPS_SDKDIR environment variable")
    endif()
endif()

# Include the RTMaps CMake configuration file
include("${RTMAPS_SDKDIR}/templates.u/rtmaps.cmake")

# Create the pck target
# NB: dir(pckInfoFile)/local_interfaces, dir(pckInfoFile)/protected_interfaces and dir(pckInfoFile)/public_interfaces
#     will be automatically added to the list of include directories
add_rtmaps_package(${PCK} PCKINFO "rtmaps_ros_bridge.pckinfo"
    # add sources, headers and/or directories
    "local_interfaces"
    "src"
)
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs sensor_msgs nav_msgs geometry_msgs)
include_directories(include ../rtmaps_ros_bridge_core_function.u/public_interfaces ${catkin_INCLUDE_DIRS})

target_link_libraries(${PCK} ${catkin_LIBRARIES})

# Append good compiler warnings
if (MSVC)
    # MSVC https://msdn.microsoft.com/en-us/library/thxezb7y.aspx
    target_compile_options(${PCK} PRIVATE "/W3" "/WX")
else()
    # Gcc   https://gcc.gnu.org/onlinedocs/gcc/Warning-Options.html
    # Clang http://clang.llvm.org/docs/UsersManual.html
    target_compile_options(${PCK} PRIVATE -fdiagnostics-show-option -Wall -Wextra -pedantic -Wno-unused-variable -Werror)
endif()
