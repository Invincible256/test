# hik_camera_driver —— 核心库 + SDK 驱动类 + ROS2 组件节点（第 1–5 步）
#
# 两种编译方式：
#   1. colcon build（ROS2 环境下）：编译全部内容，包括组件节点 hik_camera_driver::HikCameraNode
#   2. 普通 cmake（没有 ROS2 时）：只编译核心库、驱动库、单元测试和工具
#   - 核心库 hik_camera_core：纯逻辑，不依赖 SDK
#   - 驱动库 hik_camera_sdk：SdkContext + HikCamera，需要海康 MVS SDK（找不到时跳过）
#   - 单元测试：test_core（纯逻辑）、test_driver（驱动类 + 假 SDK，不需要相机和 SDK）
#   - 工具：clock_sync_replay（回放 hik_probe 数据）、hik_camera_check（真机 / 虚拟相机验收）
cmake_minimum_required(VERSION 3.18)
project(hik_camera_driver C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()
find_package(Threads REQUIRED)

# ---------------- 核心库（第 1 步）----------------
add_library(hik_camera_core STATIC
  src/clock_sync.cpp
  src/pixel_format.cpp
  src/param_table.cpp
  src/stereo_pairer.cpp)
target_include_directories(hik_camera_core PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_compile_options(hik_camera_core PRIVATE -Wall -Wextra)
set_target_properties(hik_camera_core PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_link_libraries(hik_camera_core PUBLIC Threads::Threads)

# ---------------- 单元测试 ----------------
enable_testing()
add_executable(test_core
  test/test_main.cpp test/test_clock_sync.cpp test/test_pixel_format.cpp test/test_param_table.cpp test/test_stereo_pairer.cpp)
target_link_libraries(test_core PRIVATE hik_camera_core)
add_test(NAME test_core COMMAND test_core)

# 驱动类测试：同一份 hik_camera.cpp / sdk_context.cpp 链接“假 SDK”编译，不需要相机，也不需要安装 SDK
add_executable(test_driver
  test/test_main.cpp test/test_hik_camera.cpp
  src/sdk_context.cpp src/hik_camera.cpp
  test/fake_mvs/fake_mvs.cpp)
target_include_directories(test_driver PRIVATE test test/fake_mvs)  # 假 SDK 头文件，不会用到 /opt/MVS/include
target_compile_options(test_driver PRIVATE -Wall -Wextra)
target_link_libraries(test_driver PRIVATE hik_camera_core)
add_test(NAME test_driver COMMAND test_driver)

add_executable(clock_sync_replay tools/clock_sync_replay.cpp)
target_link_libraries(clock_sync_replay PRIVATE hik_camera_core)

# ---------------- 驱动库（第 2 步，需要 MVS SDK）----------------
if(DEFINED ENV{MVCAM_SDK_PATH})
  set(MVS_ROOT "$ENV{MVCAM_SDK_PATH}" CACHE PATH "MVS install root")
else()
  set(MVS_ROOT "/opt/MVS" CACHE PATH "MVS install root")
endif()
set(MVS_LIB_DIR "${MVS_ROOT}/lib/64")
find_library(MVCAMERACONTROL_LIB NAMES MvCameraControl PATHS "${MVS_LIB_DIR}" NO_DEFAULT_PATH)

if(MVCAMERACONTROL_LIB AND EXISTS "${MVS_ROOT}/include/MvCameraControl.h")
  message(STATUS "找到海康 SDK：${MVCAMERACONTROL_LIB}")
  add_library(mvs::MvCameraControl SHARED IMPORTED)
  set_target_properties(mvs::MvCameraControl PROPERTIES
    IMPORTED_LOCATION "${MVCAMERACONTROL_LIB}"
    INTERFACE_INCLUDE_DIRECTORIES "${MVS_ROOT}/include")

  add_library(hik_camera_sdk STATIC src/sdk_context.cpp src/hik_camera.cpp)
  target_compile_options(hik_camera_sdk PRIVATE -Wall -Wextra)
  set_target_properties(hik_camera_sdk PROPERTIES POSITION_INDEPENDENT_CODE ON)
  target_link_libraries(hik_camera_sdk PUBLIC hik_camera_core mvs::MvCameraControl)

  add_executable(hik_camera_check tools/hik_camera_check.cpp)
  target_link_libraries(hik_camera_check PRIVATE hik_camera_sdk)
  set_target_properties(hik_camera_check PROPERTIES BUILD_RPATH "${MVS_LIB_DIR}" INSTALL_RPATH "${MVS_LIB_DIR}")
else()
  message(WARNING "没有找到海康 SDK（${MVS_ROOT}），跳过 hik_camera_sdk、hik_camera_check 和 ROS 节点；单元测试仍可编译")
endif()

# ---------------- ROS2 组件节点（第 3 步，需要 ament 和 MVS SDK）----------------
find_package(ament_cmake QUIET)
if(ament_cmake_FOUND)
  find_package(rclcpp REQUIRED)
  find_package(rclcpp_components REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(std_srvs REQUIRED)
  find_package(hik_camera_interfaces REQUIRED)

  if(TARGET hik_camera_sdk)
    add_library(hik_camera_component SHARED src/hik_camera_node.cpp src/stereo_coordinator_node.cpp)
    target_compile_options(hik_camera_component PRIVATE -Wall -Wextra)
    target_link_libraries(hik_camera_component hik_camera_sdk)
    # ament_target_dependencies 在 Humble 和 Jazzy 上都可用
    ament_target_dependencies(hik_camera_component rclcpp rclcpp_components sensor_msgs std_srvs hik_camera_interfaces)
    set_target_properties(hik_camera_component PROPERTIES INSTALL_RPATH "${MVS_LIB_DIR}" BUILD_RPATH "${MVS_LIB_DIR}")
    rclcpp_components_register_node(hik_camera_component
      PLUGIN "hik_camera_driver::HikCameraNode"
      EXECUTABLE hik_camera_node)
    rclcpp_components_register_node(hik_camera_component
      PLUGIN "hik_camera_driver::StereoCoordinator"
      EXECUTABLE stereo_coordinator)

    install(TARGETS hik_camera_component
      ARCHIVE DESTINATION lib
      LIBRARY DESTINATION lib
      RUNTIME DESTINATION bin)
  endif()

  # 工具也安装，方便在工作空间里直接运行：ros2 run 不适用，用 install/hik_camera_driver/lib/hik_camera_driver/<工具名>
  set(_tools clock_sync_replay)
  if(TARGET hik_camera_check)
    list(APPEND _tools hik_camera_check)
  endif()
  install(TARGETS ${_tools} test_core test_driver DESTINATION lib/${PROJECT_NAME})
  install(PROGRAMS tools/camstat.py DESTINATION lib/${PROJECT_NAME})   # ros2 run hik_camera_driver camstat.py
  install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
  ament_package()
endif()
