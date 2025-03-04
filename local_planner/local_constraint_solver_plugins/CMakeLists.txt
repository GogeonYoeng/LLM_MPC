add_library(forward_trajectory_plugin SHARED
  src/forward_trajectory.cpp
  src/mpc_controller.cpp
)

set_target_properties(forward_trajectory_plugin PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")

# ✅ OSQP 라이브러리 경로 추가
find_package(osqp REQUIRED PATHS /usr/local/lib/cmake/osqp /usr/local/share/osqp/cmake)

# ✅ tf2_eigen 추가 (MoveIt! 최신 버전에서 필요)
find_package(tf2_eigen REQUIRED)

# ✅ OSQP 및 기타 헤더 포함
include_directories(/usr/local/include)
include_directories(${tf2_eigen_INCLUDE_DIRS})

# ✅ 패키지 종속성 추가
ament_target_dependencies(forward_trajectory_plugin ${THIS_PACKAGE_INCLUDE_DEPENDS} osqp tf2_eigen)

target_include_directories(forward_trajectory_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# ✅ OSQP 라이브러리 링크
target_link_libraries(forward_trajectory_plugin osqp)