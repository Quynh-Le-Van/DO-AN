# CMake generated Testfile for 
# Source directory: /home/proteenteen/Documents/DO-AN/DO-AN/ros/src/unique_identifier/unique_id/tests
# Build directory: /home/proteenteen/Documents/DO-AN/DO-AN/ros/build/unique_identifier/unique_id/tests
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(_ctest_unique_id_gtest_test_unique_id "/home/proteenteen/Documents/DO-AN/DO-AN/ros/build/catkin_generated/env_cached.sh" "/usr/bin/python3" "/opt/ros/noetic/share/catkin/cmake/test/run_tests.py" "/home/proteenteen/Documents/DO-AN/DO-AN/ros/build/test_results/unique_id/gtest-test_unique_id.xml" "--return-code" "/home/proteenteen/Documents/DO-AN/DO-AN/ros/devel/lib/unique_id/test_unique_id --gtest_output=xml:/home/proteenteen/Documents/DO-AN/DO-AN/ros/build/test_results/unique_id/gtest-test_unique_id.xml")
set_tests_properties(_ctest_unique_id_gtest_test_unique_id PROPERTIES  _BACKTRACE_TRIPLES "/opt/ros/noetic/share/catkin/cmake/test/tests.cmake;160;add_test;/opt/ros/noetic/share/catkin/cmake/test/gtest.cmake;98;catkin_run_tests_target;/opt/ros/noetic/share/catkin/cmake/test/gtest.cmake;37;_catkin_add_google_test;/home/proteenteen/Documents/DO-AN/DO-AN/ros/src/unique_identifier/unique_id/tests/CMakeLists.txt;6;catkin_add_gtest;/home/proteenteen/Documents/DO-AN/DO-AN/ros/src/unique_identifier/unique_id/tests/CMakeLists.txt;0;")
add_test(_ctest_unique_id_nosetests_test_unique_id.py "/home/proteenteen/Documents/DO-AN/DO-AN/ros/build/catkin_generated/env_cached.sh" "/usr/bin/python3" "/opt/ros/noetic/share/catkin/cmake/test/run_tests.py" "/home/proteenteen/Documents/DO-AN/DO-AN/ros/build/test_results/unique_id/nosetests-test_unique_id.py.xml" "--return-code" "\"/usr/bin/cmake\" -E make_directory /home/proteenteen/Documents/DO-AN/DO-AN/ros/build/test_results/unique_id" "/usr/bin/nosetests3 -P --process-timeout=60 /home/proteenteen/Documents/DO-AN/DO-AN/ros/src/unique_identifier/unique_id/tests/test_unique_id.py --with-xunit --xunit-file=/home/proteenteen/Documents/DO-AN/DO-AN/ros/build/test_results/unique_id/nosetests-test_unique_id.py.xml")
set_tests_properties(_ctest_unique_id_nosetests_test_unique_id.py PROPERTIES  _BACKTRACE_TRIPLES "/opt/ros/noetic/share/catkin/cmake/test/tests.cmake;160;add_test;/opt/ros/noetic/share/catkin/cmake/test/nosetests.cmake;83;catkin_run_tests_target;/home/proteenteen/Documents/DO-AN/DO-AN/ros/src/unique_identifier/unique_id/tests/CMakeLists.txt;15;catkin_add_nosetests;/home/proteenteen/Documents/DO-AN/DO-AN/ros/src/unique_identifier/unique_id/tests/CMakeLists.txt;0;")
