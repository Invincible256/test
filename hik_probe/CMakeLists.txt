cmake_minimum_required(VERSION 3.18)          # find_library(... REQUIRED) 需要 3.18+
project(hik_probe CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

# MVS 安装根目录：优先用 set_env_path.sh 导出的 MVCAM_SDK_PATH，否则默认 /opt/MVS
# （写法来自 sdk_notes.md 第 5 项，已在笔记本上验证可编译运行）
if(DEFINED ENV{MVCAM_SDK_PATH})
  set(MVS_ROOT "$ENV{MVCAM_SDK_PATH}" CACHE PATH "MVS install root")
else()
  set(MVS_ROOT "/opt/MVS" CACHE PATH "MVS install root")
endif()
set(MVS_LIB_DIR "${MVS_ROOT}/lib/64")

find_library(MVCAMERACONTROL_LIB NAMES MvCameraControl
             PATHS "${MVS_LIB_DIR}" NO_DEFAULT_PATH REQUIRED)
add_library(mvs::MvCameraControl SHARED IMPORTED)
set_target_properties(mvs::MvCameraControl PROPERTIES
  IMPORTED_LOCATION "${MVCAMERACONTROL_LIB}"
  INTERFACE_INCLUDE_DIRECTORIES "${MVS_ROOT}/include")

find_package(Threads REQUIRED)

add_executable(hik_probe src/hik_probe.cpp)
target_compile_options(hik_probe PRIVATE -Wall -Wextra)
target_link_libraries(hik_probe PRIVATE mvs::MvCameraControl Threads::Threads)
set_target_properties(hik_probe PROPERTIES
  BUILD_RPATH   "${MVS_LIB_DIR}"
  INSTALL_RPATH "${MVS_LIB_DIR}")
