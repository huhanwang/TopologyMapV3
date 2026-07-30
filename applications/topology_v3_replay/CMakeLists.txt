add_executable(topology_v3_replay
    src/main.cpp
)

find_package(SQLite3 REQUIRED)

target_include_directories(topology_v3_replay PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../../algorithms/topology_v3/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../visualization-backend/3rdparty/viz_core
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../dependency/type-protocol/1.0.0_linux_ubuntu_gcc9.4.0/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../dependency/auto-protocol/1.0.0_linux_ubuntu_gcc9.4.0/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../dependency/thirdparty/protobuf/3.5.0_linux_ubuntu_gcc9.4.0/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../noah-env/protocol
)

find_library(TOPOLOGY_V3_REPLAY_PROTOBUF
    NAMES protobuf libprotobuf.so.15.0.0
    PATHS ${CMAKE_CURRENT_SOURCE_DIR}/../../../dependency/thirdparty/protobuf/3.5.0_linux_ubuntu_gcc9.4.0/lib
    NO_DEFAULT_PATH
    REQUIRED)
find_library(TOPOLOGY_V3_REPLAY_ADTYPEPROTOCOL
    NAMES adtypeprotocol libadtypeprotocol.so.1.0.0
    PATHS ${CMAKE_CURRENT_SOURCE_DIR}/../../../dependency/type-protocol/1.0.0_linux_ubuntu_gcc9.4.0/lib
    NO_DEFAULT_PATH
    REQUIRED)
find_library(TOPOLOGY_V3_REPLAY_AUTOPROTOCOL
    NAMES autoprotocol libautoprotocol.so.1.0.0
    PATHS ${CMAKE_CURRENT_SOURCE_DIR}/../../../dependency/auto-protocol/1.0.0_linux_ubuntu_gcc9.4.0/lib
    NO_DEFAULT_PATH
    REQUIRED)

target_link_libraries(topology_v3_replay PRIVATE
    topology_v3_core
    SQLite::SQLite3
    ${TOPOLOGY_V3_REPLAY_ADTYPEPROTOCOL}
    ${TOPOLOGY_V3_REPLAY_AUTOPROTOCOL}
    ${TOPOLOGY_V3_REPLAY_PROTOBUF}
)

set_target_properties(topology_v3_replay PROPERTIES
    BUILD_RPATH "${CMAKE_CURRENT_SOURCE_DIR}/../../../dependency/type-protocol/1.0.0_linux_ubuntu_gcc9.4.0/lib;${CMAKE_CURRENT_SOURCE_DIR}/../../../dependency/auto-protocol/1.0.0_linux_ubuntu_gcc9.4.0/lib;${CMAKE_CURRENT_SOURCE_DIR}/../../../dependency/thirdparty/protobuf/3.5.0_linux_ubuntu_gcc9.4.0/lib"
)
