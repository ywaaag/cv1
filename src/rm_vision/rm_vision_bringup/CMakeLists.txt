cmake_minimum_required(VERSION 3.8)
project(rm_vision_bringup)

find_package(ament_cmake_auto REQUIRED)
# 安装弹道解算配置文件
install(
  FILES config/ballistic_solver.yaml
  DESTINATION share/${PROJECT_NAME}/config
)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  launch
  config
)
