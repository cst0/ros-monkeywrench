cmake_minimum_required(VERSION 3.0.2)

project(monkeywrench)

find_package(catkin REQUIRED COMPONENTS message_generation rospy)
catkin_python_setup()

add_service_files(FILES TriggerError.srv)
generate_messages(DEPENDENCIES)
catkin_package(CATKIN_DEPENDS message_generation rospy)
include_directories(${catkin_INCLUDE_DIRS})

