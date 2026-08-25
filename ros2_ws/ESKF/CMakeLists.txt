cmake_minimum_required(VERSION 3.20)
project(eskf VERSION 0.1 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 告诉 CMake 去哪找 Eigen（我们已解压的目录）
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/third_party/eigen)
# 共享 ESKF 核心（阶段6 起 stage6 和 ROS2 节点共用）
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

# 数据目录以编译期宏注入，程序不再依赖启动时的当前目录
set(ESKF_DATA_DIR "${CMAKE_CURRENT_SOURCE_DIR}/data")

# ============================================================
#  每个课程程序 = 一个可执行目标（一个程序只能有一个 main）
#  以后每进入一个新阶段，就在这里"加一行 eskf_stage(...)"
#  MSVC 才需要 /utf-8 把 UTF-8 注释当源码字符集；g++/clang 默认就是。
# ============================================================
function(eskf_stage name src)
  add_executable(${name} ${src})
  target_compile_definitions(${name} PRIVATE DATA_DIR="${ESKF_DATA_DIR}")
  if(MSVC)
    target_compile_options(${name} PRIVATE /utf-8)
  endif()
endfunction()

eskf_stage(stage0_traj     src/stage0_gen_data.cpp)
eskf_stage(stage0_check    src/stage0_check.cpp)
eskf_stage(stage1_integrate src/stage1_integrate.cpp)
eskf_stage(stage2_kf1d     src/stage2_kf1d.cpp)
eskf_stage(stage3_quat     src/stage3_quat.cpp)
eskf_stage(stage3_perturb  src/stage3_perturb.cpp)
eskf_stage(stage4_eskf     src/stage4_eskf.cpp)
eskf_stage(stage5_eval     src/stage5_eval.cpp)
eskf_stage(stage6_euroc    src/stage6_euroc.cpp)
