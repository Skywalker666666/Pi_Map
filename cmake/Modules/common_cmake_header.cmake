set(COMMON_CMAKE_HEADER_CMKAE 1)


#####################
### System Config ###
#####################

#set(CMAKE_CXX_STANDARD 14) #add_compile_options(-std=c++14)
#set(CMAKE_BUILD_TYPE debug)

set(CMAKE_C_FLAGS "-std=c99")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O3 -std=c++11")
#set(EXT_LIBS -lpthread)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif(NOT CMAKE_BUILD_TYPE)

if (CMAKE_BUILD_TYPE STREQUAL debug)
    add_definitions(-DDEBUG)
endif()

if(CMAKE_BUILD_TYPE STREQUAL Release)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -s -ffunction-sections -fdata-sections -fvisibility=hidden")
    message("**************************RELEASE****************************")
else()
    message("**************************DEBUG****************************")
endif ()

## Extra module paths, used by `find_package`
list(APPEND CMAKE_MODULE_PATH ${PM_ROOT_DIR}/cmake/Modules)

set(PM_3RDPARTY_ROOT_DIR "" CACHE PATH "Root forder for third_party")

set(PM_THIRDPARTY_DIR ${PM_ROOT_DIR}/third_party)

message(STATUS "PM_3RDPARTY_ROOT_DIR ${PM_3RDPARTY_ROOT_DIR}")
if("${CMAKE_SYSTEM_PROCESSOR}" MATCHES "^x86.*")
    set(PM_3RDPARTY_DIR ${PM_3RDPARTY_ROOT_DIR}/Linux-X64)
else()
    set(PM_3RDPARTY_DIR ${PM_3RDPARTY_ROOT_DIR}/Linux-ARM)
endif()

if(NOT THIRD_PARTY_CMAKE)
    include(third_party)
endif()
