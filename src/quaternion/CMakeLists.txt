cmake_minimum_required(VERSION 3.0.2)
project(quaternion)

# set(CMAKE_CXX_STANDARD 17)
# set(CMAKE_CXX_STANDARD_REQUIRED ON)
find_package(catkin REQUIRED COMPONENTS
  std_msgs
  roscpp
)

catkin_package(
  INCLUDE_DIRS # include目录为空，因为头文件不在特定的子目录下
  LIBRARIES quaternionlib
  CATKIN_DEPENDS roscpp std_msgs

  # DEPENDS system_lib
)

# ROS包的其他依赖库
include_directories(${catkin_INCLUDE_DIRS})

# 包的根目录 包含自定义头文件
include_directories(${PROJECT_SOURCE_DIR})

# 声明一个cpp库
add_library(quaternionlib src/quaternion.cc)

# 链接catkin库
target_link_libraries(quaternionlib ${catkin_LIBRARIES})