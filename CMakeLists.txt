# Mirrors the Makefile's flags and targets. The Makefile is the primary build.
cmake_minimum_required(VERSION 3.16)
project(amc8270_mesh LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(amc8270_mesh STATIC
  src/coordination.cpp
  src/crypto.cpp
  src/geo.cpp
  src/gmcs_alert_consumer.cpp
  src/mesh_packet.cpp
  src/mesh_security.cpp
  src/pack_mesh_manager.cpp
  src/situational_awareness.cpp
  src/validation.cpp)
target_include_directories(amc8270_mesh PUBLIC include)
# Flight-library rules: no exceptions, no RTTI.
target_compile_options(amc8270_mesh PUBLIC -fno-exceptions -fno-rtti)
target_compile_options(amc8270_mesh PRIVATE -Wall -Wextra -Wpedantic -Wshadow -Werror)

add_executable(mesh_tests tests/test_main.cpp)
target_link_libraries(mesh_tests PRIVATE amc8270_mesh)
target_compile_options(mesh_tests PRIVATE -Wall -Wextra -Wpedantic -Wshadow -Werror)

find_package(Threads REQUIRED)
add_executable(pack_sim sim/pack_sim.cpp sim/scenario.cpp sim/world.cpp)
target_link_libraries(pack_sim PRIVATE amc8270_mesh Threads::Threads)
target_compile_options(pack_sim PRIVATE -Wall -Wextra -Wpedantic -Wshadow -Werror)

enable_testing()
add_test(NAME mesh_tests COMMAND mesh_tests)
foreach(scn ideal6 storyline6 pack16)
  add_test(NAME sim_${scn} COMMAND pack_sim --quiet ${CMAKE_SOURCE_DIR}/sim/scenarios/${scn}.scn)
endforeach()
add_executable(fuzz_replay fuzz/fuzz_packet.cpp fuzz/fuzz_main.cpp)
target_link_libraries(fuzz_replay PRIVATE amc8270_mesh)
add_test(NAME fuzz_corpus COMMAND fuzz_replay ${CMAKE_SOURCE_DIR}/fuzz/corpus)
