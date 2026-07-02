cmake_minimum_required(VERSION 3.10)
project(robot_slamd CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(robot_slamd main.cpp)
target_link_libraries(robot_slamd m dl)



enable_testing()
add_executable(test_cjc_bl4820 test_cjc_bl4820.cpp)
target_link_libraries(test_cjc_bl4820 m dl)
add_test(NAME test_cjc_bl4820 COMMAND test_cjc_bl4820)
add_executable(test_map_quality test_map_quality.cpp)
target_link_libraries(test_map_quality m dl)
add_test(NAME test_map_quality COMMAND test_map_quality)
add_executable(test_mapping_supervisor test_mapping_supervisor.cpp)
target_link_libraries(test_mapping_supervisor m dl)
add_test(NAME test_mapping_supervisor COMMAND test_mapping_supervisor)
add_executable(test_active_scan_manager test_active_scan_manager.cpp)
target_link_libraries(test_active_scan_manager m dl)
add_test(NAME test_active_scan_manager COMMAND test_active_scan_manager)
add_executable(test_active_scan_command_planner test_active_scan_command_planner.cpp)
target_link_libraries(test_active_scan_command_planner m dl)
add_test(NAME test_active_scan_command_planner COMMAND test_active_scan_command_planner)

