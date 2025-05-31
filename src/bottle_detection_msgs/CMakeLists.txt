cmake_minimum_required(VERSION 3.8)
set(PYTHON_EXECUTABLE /usr/bin/python3)
project(bottle_detection_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖包
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

# 生成接口
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BottleDetection.msg"
  DEPENDENCIES std_msgs
)

ament_package()