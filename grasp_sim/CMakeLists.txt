cmake_minimum_required(VERSION 3.8)
project(grasp_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)

install(DIRECTORY launch worlds models config scripts
  DESTINATION share/${PROJECT_NAME}
)

# 把 scripts/grasp_planner.py 装到 lib/grasp_sim/ 下，让 ros2 run grasp_sim grasp_planner 能找到
# install(PROGRAMS ...) 默认装到 lib/<pkg>/，正好匹配 Node(executable="...") 的查找路径
install(PROGRAMS scripts/grasp_planner.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME grasp_planner
)

pluginlib_export_plugin_description_file(hardware_interface config/hardware_plugins.xml)

ament_package()
