# 设置最低CMake版本要求
cmake_minimum_required(VERSION 3.0 FATAL_ERROR)

# 设置项目名称
project(ir_photo_calib)

# 设置编译器选项
set(CMAKE_CXX_FLAGS "-march=native -msse4.2 -mfpmath=sse")
set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_STANDARD 11)
set(OpenCV_DIR /usr/local/opencv420/share/opencv4)
# 查找OpenCV
find_package(OpenCV 4.2.0 REQUIRED)

# 查找Boost库，确保Boost系统和文件系统组件
find_package(Boost REQUIRED COMPONENTS system filesystem)

# 查找Ceres Solver
find_package(Ceres REQUIRED)

# 查找ROS
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  cv_bridge
  rosbag
  tf2_msgs
  geometry_msgs
)

# 启用OpenMP
option(USE_OpenMP "Use OpenMP" ON)
if(USE_OpenMP)
    find_package(OpenMP)
    if(OPENMP_FOUND)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
        set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    endif()
endif()

# 输出OpenMP的检测信息
if(OPENMP_FOUND)
    message("OpenMP found")
else()
    message("OpenMP not found")
endif()

# 添加包含目录
include_directories(
  ${OpenCV_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}  # ROS 包包含路径
  include
)

# 设置链接的库
link_directories(${catkin_LIBRARY_DIRS})

# 添加可执行文件并链接相关库
add_executable(main main.cpp irPhotoCalib.cpp)
target_link_libraries(main
    ${OpenCV_LIBRARIES}
    ${CERES_LIBRARIES}
    ${Boost_LIBRARIES}
    ${catkin_LIBRARIES}  # 链接 ROS 库
    ${OpenMP_LIBRARIES}
)

