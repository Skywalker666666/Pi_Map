cmake_minimum_required(VERSION 2.8.3)
project(sonar_mapping)

#############################
# Common cmake header:      #
# 1.System Config           #
# 2.Extra PATHS             #
# 3.Additional Dependencies #
#############################
if(NOT START_FROM_ROOT)
    set(PM_ROOT_DIR ${CMAKE_CURRENT_LIST_DIR}/../)
    if(NOT COMMON_CMAKE_HEADER_CMKAE)
        include(${PM_ROOT_DIR}/cmake/Modules/common_cmake_header.cmake)
    endif()
endif()

# mapping
set(MappingSrc
  my_kbhit.cpp
  common_helper.cpp
  CRC16.cpp
  bot.cpp
  serial.cpp
  circle_buffer.cpp
  our_bot.cpp
  our_bot_comm.cpp
  aux_func.cpp
  mapping.cpp
  occupancy_grid_map.cc
  pose_2d.cpp
  sonar_mapping.cpp
  )

include_directories (
        ./
        ${CMAKE_CURRENT_LIST_DIR}/../third_party/msg_utils/include)

add_executable(test_sonar_mapping test_sonar_mapping ${MappingSrc} ${CommonSrc})

target_link_libraries(test_sonar_mapping glog gflags gtest nanomsg msg_utils ${OpenCV_LIBS} pthread)
