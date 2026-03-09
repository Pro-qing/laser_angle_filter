cmake_minimum_required(VERSION 3.0.2)
project(laser_angle_filter)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  dynamic_reconfigure
)

# 编译动态调参 cfg
generate_dynamic_reconfigure_options(
  cfg/LaserFilter.cfg
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES laser_angle_filter
#  CATKIN_DEPENDS roscpp sensor_msgs dynamic_reconfigure
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

# 必须添加依赖保证 cfg 先编译
add_executable(laser_filter_node src/laser_filter_node.cpp)
add_dependencies(laser_filter_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(laser_filter_node
  ${catkin_LIBRARIES}
)

# ============== 追加安装规则 ==============
# 1. 安装可执行文件
install(TARGETS laser_filter_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# 2. 安装 launch 文件夹
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

# 3. 安装 cfg 文件夹
install(DIRECTORY cfg/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/cfg
)