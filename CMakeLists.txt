cmake_minimum_required(VERSION 3.16)
project(Sim3)

set(CMAKE_CXX_STANDARD 23)

add_executable(Sim3 main.cpp Simulation/Setup.cpp Simulation/Config.cpp Simulation/Sim3.cpp Simulation/SimData.cpp)

find_package(Bullet CONFIG REQUIRED)
target_link_libraries(Sim3 PRIVATE ${BULLET_LIBRARIES})
target_include_directories(Sim3 PRIVATE ${BULLET_INCLUDE_DIRS})