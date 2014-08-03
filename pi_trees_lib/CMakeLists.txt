cmake_minimum_required(VERSION 2.8.3)

project(pi_trees_lib)

find_package(catkin REQUIRED)

catkin_package()

catkin_python_setup()

catkin_install_python(PROGRAMS examples/parallel_example.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
