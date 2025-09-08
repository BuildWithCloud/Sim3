cmake_minimum_required(VERSION 3.16)  # 4.0 doesn't exist yet
project(Sim3)

set(CMAKE_CXX_STANDARD 20)

add_executable(Sim3 main.cpp Simulation/Setup.cpp Simulation/Config.cpp Simulation/Sim3.cpp)

find_package(Bullet CONFIG REQUIRED)
target_link_libraries(Sim3 PRIVATE ${BULLET_LIBRARIES})
target_include_directories(Sim3 PRIVATE ${BULLET_INCLUDE_DIRS})