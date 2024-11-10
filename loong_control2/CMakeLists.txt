cmake_minimum_required(VERSION 3.8)
project(loong_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_typesupport_c REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
# find_package(azureloong_description REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)



IF (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release) 
ENDIF ()
add_compile_options(-std=c++17)

#系统eigen版本可能老旧，使用自供eigen
set(dirEigen "third_party/eigen3")
set(dirGlfw "third_party/glfw")
set(dirPino "third_party/pinocchio")
set(dirJson "third_party/jsoncpp")
set(dirQuill "third_party/quill")
set(dirQP "third_party/qpOASES")

set(libURDF "third_party/urdfdom")

set(incFmt "third_party/quill/quill/bundled")
set(incBoost "third_party/boost")
set(incMujoco "third_party/mujoco")


set(allDir ${dirEigen} ${dirGlfw} ${dirPino} ${dirJson} ${dirQuill} ${dirQP} )
set(allLib ${allDir} ${libURDF})
set(allInc ${allDir} ${incFmt} ${incBoost} ${incMujoco})

include_directories(${allInc})
include_directories("algorithm")
include_directories("common")
include_directories("math")
include_directories("sim_interface")

message(${CMAKE_SYSTEM_PROCESSOR})
# if(${CMAKE_CXX_COMPILER} MATCHES "aarch64" OR ${CMAKE_SYSTEM_PROCESSOR} MATCHES "aarch64")
# 	list(APPEND allLib "third_party/mujoco/lin_arm64")
# else()
# 	list(APPEND allLib "third_party/mujoco/lin_x64")
# endif()
list(APPEND allLib "third_party/mujoco/lin_x64")

link_directories(${allLib})

file(GLOB C_SOURCES *.c)
file(GLOB CPP_SOURCES *.cpp algorithm/*.cpp common/*.cpp math/*.cpp sim_interface/*.cpp)
file(GLOB HEADER_FILES *.h algorithm/*.h common/*.h math/*.h sim_interface/*.h)
set(SOURCES ${C_SOURCES} ${CPP_SOURCES} ${HEADER_FILES})

#链接库
set(corLibs pinocchio urdfdom_model tinyxml console_bridge jsoncpp quill qpOASES)
set(simLibs glfw3)#mujoco区分x86/arm的方式不一样，在上面按link目录区分

#根据架构，处理连接库
set(sysCoreLibs)
set(sysSimLibs)

foreach(lib ${corLibs})
    list(APPEND sysCoreLibs ${lib}_lin_x64)
endforeach()
foreach(lib ${simLibs})
    list(APPEND sysSimLibs ${lib}_lin_x64)
endforeach()

message(${sysCoreLibs})

#生成控制核心库
add_library(core ${SOURCES})
target_link_libraries(core ${sysCoreLibs} pthread)


add_executable(loong_control2 src/loong_control2.cpp)
target_link_libraries(loong_control2 core mujoco ${sysSimLibs} dl)

ament_target_dependencies(
  loong_control2
  "rclcpp"
  "rclpy"
  "tf2"
  "tf2_ros"
  "geometry_msgs"
  # "azureloong_description"
  "rclcpp_action"
  "nav_msgs"
  "sensor_msgs"
  "tf2_geometry_msgs"
  "ament_index_cpp"
  "rosidl_typesupport_c"

)
# 导出依赖项
# ament_export_dependencies(
#   rclcpp
#   sensor_msgs
#   nav_msgs
#   geometry_msgs
#   tf2
#   tf2_ros
#   tf2_geometry_msgs
# )

install(TARGETS 
  core
  loong_control2
  DESTINATION lib/${PROJECT_NAME})

target_include_directories(loong_control2 PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(loong_control2 PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)



install(DIRECTORY 
  launch
  algorithm
  models
  sim_interface
  common
  third_party
  math
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
