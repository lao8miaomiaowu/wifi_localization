cmake_minimum_required(VERSION 3.5)
project(wifi_localization_cpp)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
#find_package(onnxruntime REQUIRED)  # 添加ONNX Runtime支持

# 2. 手动给出头文件和库路径
set(ONNXRuntime_INCLUDE_DIRS "/home/hao/onnxruntime-linux-x64-1.12.1/include")
set(ONNXRuntime_LIBRARIES    "/home/hao/onnxruntime-linux-x64-1.12.1/lib/libonnxruntime.so")

# 3. 让编译器能找到它们
include_directories(${ONNXRuntime_INCLUDE_DIRS})
link_directories   (/opt/onnxruntime/lib)

include_directories(include)
add_executable(wifi_localizer_node src/WIFI-Loc.cpp)

ament_target_dependencies(wifi_localizer_node
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
)

target_link_libraries(wifi_localizer_node
  ${ONNXRuntime_LIBRARIES}
)

# 设置安装的规则，这样ros2 run命令才找得到可执行文件
install(TARGETS wifi_localizer_node
DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME})

  install(DIRECTORY
 Net/
  DESTINATION share/${PROJECT_NAME}/Net)

  link_directories(/opt/onnxruntime/lib)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
