cmake_minimum_required(VERSION 2.8.3)
project(pursuit_evasion)

find_package(catkin_simple REQUIRED)


catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=c++14)

catkin_package()


cs_add_executable(evasion
src/evasion.cpp
)  
      
cs_add_executable(pursuit
src/pursuit.cpp
)              

target_link_libraries(evasion ${catkin_LIBRARIES})
target_link_libraries(pursuit ${catkin_LIBRARIES})