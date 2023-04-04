cmake_minimum_required(VERSION 2.8.12)


set(PCK rtmaps_ros_bridge_core_function)
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
add_rtmaps_package(${PCK} PCKINFO "rtmaps_ros_bridge_core_function.pckinfo"
    # add sources, headers and/or directories
    "public_interfaces"
    "local_interfaces"
    "src"
)
set(rmp_msgs_DIR ../../segwayrmp_LE440/devel/share/rmp_msgs/cmake)
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs sensor_msgs nav_msgs geometry_msgs rmp_msgs)
include_directories(include ${catkin_INCLUDE_DIRS})

target_link_libraries(${PCK} ${catkin_LIBRARIES})

# Append good compiler warnings
if (MSVC)
    # MSVC https://msdn.microsoft.com/en-us/library/thxezb7y.aspx
    target_compile_options(${PCK} PRIVATE "/W3" "/WX")
else()
    # Gcc   https://gcc.gnu.org/onlinedocs/gcc/Warning-Options.html
    # Clang http://clang.llvm.org/docs/UsersManual.html
    target_compile_options(${PCK} PRIVATE -fdiagnostics-show-option -Wall -Wextra -pedantic -Werror)
endif()
