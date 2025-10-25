cmake_minimum_required(VERSION 3.22)
project(serial_comm)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release")
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_DIR build)

set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g -ggdb")
set(CMAKE_CXX_FLAGS_RELEASE "-O2 -DNDEBUG")

set(FINISH_RECEIVER ON)
set(TEST_RECEIVER ON)
set(FINISH_PUBLISHER ON)

add_compile_options(-Wall -Wpedantic -Wextra)

# --- 依赖包查找 ---
find_package(ament_cmake REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(spdlog REQUIRED)
# find_package(fmt REQUIRED)  <-- 已删除
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# --- 依赖包分组 ---
set(THIRD_DEPS
  Boost::system
  spdlog::spdlog
  # fmt::fmt  <-- 已删除
)

set(ROS_DEPS
  rclcpp
  sensor_msgs
)

include_directories(include)

# --- 目标：serial_sender ---
add_executable(serial_sender src/serial_sender.cpp)
target_link_libraries(serial_sender PRIVATE ${THIRD_DEPS})

# --- 检查编译选项 ---
if(TEST_RECEIVER AND NOT FINISH_RECEIVER)
  message(WARNING "TEST_RECEIVER requires FINISH_RECEIVER. Disabling TEST_RECEIVER.")
  set(TEST_RECEIVER OFF)
endif()

if(FINISH_PUBLISHER AND NOT FINISH_RECEIVER)
  message(WARNING "FINISH_PUBLISHER requires FINISH_RECEIVER. Disabling FINISH_PUBLISHER.")
  set(FINISH_PUBLISHER OFF)
endif()

# --- 目标：lib_serial_receiver (库) ---
if(FINISH_RECEIVER)
  add_library(lib_serial_receiver STATIC src/serial_receiver.cpp)
  target_link_libraries(lib_serial_receiver PUBLIC ${THIRD_DEPS})
endif()

# --- 目标：test_serial_receiver (测试) ---
if(TEST_RECEIVER)
  add_executable(test_serial_receiver src/test_serial_receiver.cpp)
  target_link_libraries(test_serial_receiver PRIVATE lib_serial_receiver)
endif()

# --- 目标：serial_relay_node (ROS 节点) ---
if(FINISH_PUBLISHER)
  add_executable(serial_relay_node src/msg_publisher.cpp)
  
  # 链接本地库（这个库已经包含了 THIRD_DEPS）
  # 注意：移除 PRIVATE 以匹配 ament_target_dependencies 的 "plain" 风格
  target_link_libraries(serial_relay_node lib_serial_receiver)
  
  # 专门使用 ament 命令链接 ROS 依赖 (注意：这里不使用 PRIVATE/PUBLIC)
  ament_target_dependencies(serial_relay_node ${ROS_DEPS})
endif()

ament_package()

