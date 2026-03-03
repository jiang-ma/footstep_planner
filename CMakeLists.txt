# cmake_minimum_required(VERSION 3.8)
# project(footstep_planner)

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()

# set(CMAKE_CXX_STANDARD 17)
# set(CMAKE_CXX_STANDARD_REQUIRED ON)

# # find dependencies
# find_package(ament_cmake REQUIRED) # 查找 ROS 2 的 CMake 工具
# find_package(matplotlib_cpp REQUIRED) # 查找 matplotlib_cpp 库
# find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# find_package(OsqpEigen REQUIRED)

# find_package(casadi REQUIRED)

# find_package(glog REQUIRED)

# find_package(Python3 3.10 EXACT COMPONENTS Interpreter Development REQUIRED)
# find_package(Python3 3.10 EXACT COMPONENTS NumPy)
# find_package(PythonLibs REQUIRED)
# include_directories( # 指定头文件搜索路径
#     include
#     ${Eigen3_inlude_directory} # 这里有拼写错误
#     ${PYTHON_INCLUDE_DIRS}
#     ${Python3_INCLUDE_DIRS}
#     ${matplotlib_cpp_inlude_directory} # 这里有拼写错误
# )

# add_executable(footstep_planner # 添加一个可执行文件
#     src/footstep_planner_casadi.cpp
# )

# add_executable(footstep_planner_double_layer # 添加一个可执行文件
#     src/footstep_planner_double_layer.cpp
# )

# target_link_libraries(footstep_planner # 链接库
#     Eigen3::Eigen 
#     ${PYTHON_LIBRARIES}
#     ${matplotlib_cpp_library} # 这里有拼写错误
#     casadi
#     glog::glog
#     # OsqpEigen::OsqpEigen
# )

# target_link_libraries(footstep_planner_double_layer # 链接库
#     Eigen3::Eigen 
#     ${PYTHON_LIBRARIES}
#     ${matplotlib_cpp_library} # 这里有拼写错误
#     casadi
#     glog::glog
#     # OsqpEigen::OsqpEigen
# )

# message(STATUS "Eigen3 found at: ${EIGEN3_INCLUDE_DIR}")
# message(STATUS "CasADi found version: ${casadi_VERSION}")

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   # the following line skips the linter which checks for copyrights
#   # comment the line when a copyright and license is added to all source files
#   set(ament_cmake_copyright_FOUND TRUE)
#   # the following line skips cpplint (only works in a git repo)
#   # comment the line when this package is in a git repo and when
#   # a copyright and license is added to all source files
#   set(ament_cmake_cpplint_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()
# endif()

# ament_package()



cmake_minimum_required(VERSION 3.8)
project(footstep_planner)

# 默认使用C++17
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找ament_cmake包
find_package(ament_cmake REQUIRED)

# 查找依赖包
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

# 查找glog
find_package(PkgConfig REQUIRED)
pkg_check_modules(glog REQUIRED libglog)

# 查找matplotlib-cpp（假设已安装）
find_path(MATPLOTLIBCPP_INCLUDE_DIR matplotlibcpp.h
  PATHS /usr/local/include /usr/include
)

# 查找CasADi（需要手动安装）
find_path(CASADI_INCLUDE_DIR casadi/casadi.hpp
  PATHS /usr/local/include /usr/include
  PATH_SUFFIXES casadi
)
find_library(CASADI_LIBRARY casadi
  PATHS /usr/local/lib /usr/lib
)

# 检查CasADi是否找到
if(NOT CASADI_INCLUDE_DIR OR NOT CASADI_LIBRARY)
  message(WARNING "CasADi not found. Please install CasADi manually.")
endif()

# 设置包含目录
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${glog_INCLUDE_DIRS}
  ${MATPLOTLIBCPP_INCLUDE_DIR}
  ${CASADI_INCLUDE_DIR}
)

# 添加可执行文件
add_executable(stair_footstep_planner
  src/stair_footstep_planner.cpp
  src/stair_geometry.cpp
)

# 链接库
target_link_libraries(stair_footstep_planner
  ${glog_LIBRARIES}
)

# 如果找到CasADi，链接它
if(CASADI_LIBRARY)
  target_link_libraries(stair_footstep_planner ${CASADI_LIBRARY})
endif()

# 添加Python支持（用于matplotlib）
find_package(Python3 COMPONENTS Interpreter Development)
if(Python3_FOUND)
  target_include_directories(stair_footstep_planner PRIVATE ${Python3_INCLUDE_DIRS})
  target_link_libraries(stair_footstep_planner ${Python3_LIBRARIES})
endif()

# 设置编译选项
target_compile_options(stair_footstep_planner PRIVATE -O2)

# 添加测试可执行文件（可选）
add_executable(test_stair_geometry
  src/test_stair_geometry.cpp
  src/stair_geometry.cpp
)

target_link_libraries(test_stair_geometry
  ${glog_LIBRARIES}
)

# 如果找到CasADi，链接它
if(CASADI_LIBRARY)
  target_link_libraries(test_stair_geometry ${CASADI_LIBRARY})
endif()

# 安装目标
install(TARGETS
  stair_footstep_planner
  test_stair_geometry
  DESTINATION lib/${PROJECT_NAME}
)

# 安装头文件
install(DIRECTORY include/
  DESTINATION include/
)

# 导出依赖项
ament_export_include_directories(include)
ament_export_libraries(stair_footstep_planner)
ament_package()