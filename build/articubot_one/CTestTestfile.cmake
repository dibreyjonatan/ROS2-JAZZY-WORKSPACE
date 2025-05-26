# CMake generated Testfile for 
# Source directory: /home/jonatan/ros2_ws/src/articubot_one
# Build directory: /home/jonatan/ros2_ws/build/articubot_one
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(laser_obstacle_test "/usr/bin/python3" "-u" "/opt/ros/jazzy/share/ament_cmake_test/cmake/run_test.py" "/home/jonatan/ros2_ws/build/articubot_one/test_results/articubot_one/laser_obstacle_test.xunit.xml" "--package-name" "articubot_one" "--output-file" "/home/jonatan/ros2_ws/build/articubot_one/ament_cmake_pytest/laser_obstacle_test.txt" "--command" "/usr/bin/python3" "-u" "-m" "pytest" "/home/jonatan/ros2_ws/src/articubot_one/test/pytest/test_laserObstacle.py" "-o" "cache_dir=/home/jonatan/ros2_ws/build/articubot_one/ament_cmake_pytest/laser_obstacle_test/.cache" "--junit-xml=/home/jonatan/ros2_ws/build/articubot_one/test_results/articubot_one/laser_obstacle_test.xunit.xml" "--junit-prefix=articubot_one")
set_tests_properties(laser_obstacle_test PROPERTIES  LABELS "pytest" TIMEOUT "120" WORKING_DIRECTORY "/home/jonatan/ros2_ws/build/articubot_one" _BACKTRACE_TRIPLES "/opt/ros/jazzy/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/opt/ros/jazzy/share/ament_cmake_pytest/cmake/ament_add_pytest_test.cmake;177;ament_add_test;/home/jonatan/ros2_ws/src/articubot_one/CMakeLists.txt;46;ament_add_pytest_test;/home/jonatan/ros2_ws/src/articubot_one/CMakeLists.txt;0;")
