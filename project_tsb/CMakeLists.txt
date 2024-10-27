cmake_minimum_required(VERSION 3.8)
project(project_tsb)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies c++
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(project_tsb_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)


add_executable(simulator src/simulator.cpp)
ament_target_dependencies(simulator rclcpp project_tsb_msgs std_srvs geometry_msgs tf2 tf2_ros tf2_geometry_msgs) #LEMBRAR DE METER DEPENDENCIAS AQUI
add_executable(pid src/pid.cpp)
ament_target_dependencies(pid rclcpp project_tsb_msgs) #LEMBRAR DE METER DEPENDENCIAS AQUI

install(TARGETS
  simulator
  pid
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python executables
install(PROGRAMS
  scripts/plotter.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install RViz config files
install(DIRECTORY
  rviz
  DESTINATION share/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY 
  launch
  DESTINATION share/${PROJECT_NAME})


# Install Python modules
ament_python_install_package(${PROJECT_NAME})

ament_package()
