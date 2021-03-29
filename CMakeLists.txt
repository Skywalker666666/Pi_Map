cmake_minimum_required(VERSION 2.8.3)
project(PiMap)

#############################
# Common cmake header:      #
# 1.System Config           #
# 2.Extra PATHS             #
# 3.Additional Dependencies #
#############################

set(START_FROM_ROOT 1)
set(PM_ROOT_DIR ${CMAKE_SOURCE_DIR})
include(${PM_ROOT_DIR}/cmake/Modules/common_cmake_header.cmake)


##################
# Subdirectory   #
##################

add_subdirectory(${PM_ROOT_DIR}/Sonar sonar_mapping)

############
# Install  #
############
