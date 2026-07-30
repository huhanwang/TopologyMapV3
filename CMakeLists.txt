cmake_minimum_required(VERSION 3.16)
project(TopologyMapV3 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_subdirectory(algorithms/topology_v3)
add_subdirectory(applications/topology_v3_replay)
