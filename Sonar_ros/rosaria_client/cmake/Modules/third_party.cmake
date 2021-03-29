set(THIRD_PARTY_CMAKE 1)

# Boost
if(PM_3RDPARTY_ROOT_DIR)
    set(BOOST_ROOT_DIR ${PM_3RDPARTY_DIR}/boost)
    set(Boost_INCLUDE_DIR ${BOOST_ROOT_DIR}/include/)
    set(Boost_LIBRARIES
            ${BOOST_ROOT_DIR}/lib/libboost_filesystem.a
            ${BOOST_ROOT_DIR}/lib/libboost_system.a
            ${BOOST_ROOT_DIR}/lib/libboost_thread.a
            ${BOOST_ROOT_DIR}/lib/libboost_regex.a
            ${BOOST_ROOT_DIR}/lib/libboost_chrono.a
            ${BOOST_ROOT_DIR}/lib/libboost_date_time.a
            ${BOOST_ROOT_DIR}/lib/libboost_atomic.a
            )
else()
    find_package(Boost REQUIRED COMPONENTS filesystem system thread regex chrono date_time atomic)
endif()
add_library(boost INTERFACE)
target_include_directories(boost INTERFACE ${Boost_INCLUDE_DIR})
target_link_libraries(boost INTERFACE ${Boost_LIBRARIES} pthread)
target_compile_options(boost INTERFACE "-Wno-deprecated-declarations;")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated-declarations")
#string(REPLACE " " ";" REPLACED_FLAGS ${CMAKE_CXX_FLAGS})
#target_compile_options(boost INTERFACE ${REPLACED_FLAGS})
message(STATUS "Boost library status:")
message(STATUS "    Boost_INCLUDE_DIR: ${Boost_INCLUDE_DIR}")
message(STATUS "    Boost_LIBRARIES: ${Boost_LIBRARIES}")

# Glog
if(PM_3RDPARTY_ROOT_DIR)
    set(GLOG_INCLUDE_DIRS ${PM_3RDPARTY_DIR}/glog/include)
    set(GLOG_LIBRARIES
            ${PM_3RDPARTY_DIR}/glog/lib/libglog.a

            ${PM_3RDPARTY_DIR}/unwind/lib/libunwind.a
            #-lunwind
            #-l:libunwind.so.8
            -l:liblzma.so.5
            #-llzma
            )
else()
    find_package(Glog REQUIRED)
endif()

add_library(glog INTERFACE)
target_include_directories(glog INTERFACE ${GLOG_INCLUDE_DIRS})
target_link_libraries(glog INTERFACE ${GLOG_LIBRARIES} gflags)
message(STATUS "Glog library status:")
message(STATUS "    GLOG_INCLUDE_DIRS: ${GLOG_INCLUDE_DIRS}")
message(STATUS "    GLOG_LIBRARIES: ${GLOG_LIBRARIES}")


# GFlags
if(PM_3RDPARTY_ROOT_DIR)
    set(GFLAGS_INCLUDE_DIRS ${PM_3RDPARTY_DIR}/gflags/include)
    set(GFLAGS_LIBRARIES ${PM_3RDPARTY_DIR}/gflags/lib/libgflags.a)
else()
    find_package(GFlags REQUIRED)
endif()

add_library(gflags INTERFACE)
target_include_directories(gflags INTERFACE ${GFLAGS_INCLUDE_DIRS})
target_link_libraries(gflags INTERFACE ${GFLAGS_LIBRARIES})
message(STATUS "Gflags library status:")
message(STATUS "    GFLAGS_INCLUDE_DIRS: ${GFLAGS_INCLUDE_DIRS}")
message(STATUS "    GFLAGS_LIBRARIES: ${GFLAGS_LIBRARIES}")


#GTest
find_package(GTest REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS})

#OpenCV
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})


# nanomsg
if(PM_3RDPARTY_ROOT_DIR)
    set(Nanomsg_INCLUDE_DIR ${PM_3RDPARTY_DIR}/nanomsg/include/)
    set(Nanomsg_LIBRARIES ${PM_3RDPARTY_DIR}/nanomsg/lib/libnanomsg.a)

    add_library(nanomsg INTERFACE)
    target_include_directories(nanomsg INTERFACE ${Nanomsg_INCLUDE_DIR})
    target_link_libraries(nanomsg INTERFACE
            ${Nanomsg_LIBRARIES}
            -lanl
            )
    message(STATUS "Nanomsg library status:")
    message(STATUS "    Nanomsg_INCLUDE_DIR: ${Nanomsg_INCLUDE_DIR}")
    message(STATUS "    Nanomsg_LIBRARIES: ${Nanomsg_LIBRARIES}")
else()
    message(STATUS "Nanomsg library status:")
    message(STATUS "    Assume nanomsg install into system directory (Ex. '/usr/include' /usr/local/include' '/usr/local/lib' '/usr/lib') by apt-get")
endif()

# msg_utils
add_library(msg_utils STATIC
        ${PM_THIRDPARTY_DIR}/msg_utils/src/pi_msg_adaptor.cpp)
target_include_directories(msg_utils PUBLIC ${PM_THIRDPARTY_DIR}/msg_utils/include)
target_link_libraries(msg_utils pthread m nanomsg boost)
