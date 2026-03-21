cmake_minimum_required(VERSION 3.15)

project(arena_camera_node)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
	# linked GenICam hearders give warnings No such file or directory
	# GENICAM_USER_ACCEPTS_ANY_COMPILER flag need tro be defined 
	# 
	#add_compile_options(-Wall -Wextra -Wpedantic -Wunknown-pragmas)
endif()

# -----------------------------------------------------------------------------
#
# ALL PKGS SETTIGNS
#
# -----------------------------------------------------------------------------


# ------------------
# CUSTOM DEPS PATHS
# ------------------

# arenasdk find file dir
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake/")

# ----------
# FIND DEPS
# ----------

# finds dependencies
# - same as the ones mentioned in package.xml under
#   the tags <buildtool_depend>, <build_depend>, <exec_depend>
# - for pakages used in #includes
# - even if a dependency is used in one package, it has to be found
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(arena_sdk REQUIRED)
find_package(OpenCV REQUIRED)
find_package(camera_info_manager REQUIRED)

# ------------
# EXPORT DEPS
# ------------

# export dependencies
# For packages that depends on arena_camera_node
# basically the dependecies of the depencey
# these are the one mentioned in
# - "find_package"
# - "ament_target_dependencies"
# - forgetting to export a depencey would not show at the time of
#   building the package but when other packages use this package
#ament_export_dependencies(ament_cmake)
#ament_export_dependencies(ament_cmake_python)
#ament_export_dependencies(rclcpp)
#ament_export_dependencies(std_msgs)
# Export package dependencies
#ament_export_dependencies(ament_cmake)
#ament_export_dependencies(ament_cmake_python)
#ament_export_dependencies(rclcpp)
#ament_export_dependencies(rclpy)
#ament_export_dependencies(sensor_msgs)
#ament_export_dependencies(std_srvs)

# -------------
# LAUNCH FILES
# -------------

# install launch files.
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# --------
# HEADERS
# --------

# MV .hpp (s) to include dir
#install(
#  DIRECTORY include/${PROJECT_NAME}/
#  DESTINATION include/${PROJECT_NAME}
#  FILES_MATCHING PATTERN "*.hpp")

#
# EXPORT HEADERS should it before the "ament_export_dependencies" ?
#
#ament_export_include_directories(include)
#ament_export_include_directories(include ${rclcpp_INCLUDE_DIRS})


# -----------------------------------------------------------------------------
#
# ROS2 RUN ARENA_CAMERA_NODE <>
#
# -----------------------------------------------------------------------------

set(target_name start)

# CPP DEPS
add_executable(${target_name}
    src/main.cpp
    src/ArenaCameraNode.cpp
    src/camera_config.cpp
    src/diagnostics.cpp
    src/light_arena/deviceinfo_helper.cpp
    src/rclcpp_adapter/pixelformat_translation.cpp
    src/rclcpp_adapter/quality_of_service_translation.cpp
)

target_include_directories(${target_name}
  PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
    ${ARENA_SDK_INCLUDE_DIRS}
)

# Ensure C++17 for this target so std::is_convertible_v is available
target_compile_features(${target_name} PUBLIC cxx_std_17)

# CPP -> .a file
# creates :
# - build/arena_camera_node/CMakeFiles/arena_camera_node.dir and sub dirs
# - build/arena_camera_node/libarena_camera_node.a
# add_library(${PROJECT_NAME} src/main.cpp)

# EXPORT .a file for other pakges ?
#ament_export_libraries(${PROJECT_NAME} ${rclcpp_LIBRARIES})

# ROS DEP
# - this way, it will find our header libs automatically
# - for Arenasdk the cmake way is used no ament
ament_target_dependencies(${target_name}  rclcpp
                                          std_msgs
                                          std_srvs
                                          sensor_msgs
                                          diagnostic_updater
                                          camera_info_manager
                                          arena_sdk
                                          )

# Link yaml-cpp
target_link_libraries(${target_name} yaml-cpp)

# OpenCV is required by ArenaSDK's libarena.so
target_link_libraries(${target_name} ${OpenCV_LIBS})

# --------------------------------------------------------------------------
# Optional: CUDA — nvJPEG hardware JPEG encoding + fused polarization kernel
# nvjpeg_encoder.cpp is pure C++ — no nvcc needed.
# polarization_kernels.cu requires nvcc (CUDA language).
# On JetPack, headers land in /usr/local/cuda-<ver>/targets/<arch>/include/
# after: sudo apt install cuda-cudart-dev-12-6 libnvjpeg-dev-12-6 cuda-crt-12-6
# Gracefully skipped on x86 hosts that don't have CUDA installed.
# --------------------------------------------------------------------------

# Discover versioned CUDA target dirs (e.g. cuda-12.6/targets/aarch64-linux)
# so we don't hard-code the CUDA version or architecture.
file(GLOB _cuda_inc_hints
  "/usr/local/cuda/targets/*/include"
  "/usr/local/cuda-*/targets/*/include"
  "/usr/local/cuda/include")
file(GLOB _cuda_lib_hints
  "/usr/local/cuda/targets/*/lib"
  "/usr/local/cuda-*/targets/*/lib"
  "/usr/local/cuda/lib64")

# These are CACHE variables — unset them so each configure re-evaluates
# (prevents stale NOTFOUND values from a prior failed configure).
unset(CUDA_INCLUDE_DIR CACHE)
unset(NVJPEG_LIB CACHE)
unset(CUDART_LIB CACHE)

find_path(CUDA_INCLUDE_DIR cuda_runtime.h
  HINTS ${_cuda_inc_hints} ENV CUDA_PATH
  PATHS /usr/local/cuda/include /usr/include
  NO_DEFAULT_PATH)

find_library(NVJPEG_LIB nvjpeg
  HINTS ${_cuda_lib_hints} ENV CUDA_PATH
  PATHS /usr/local/cuda/lib64 /usr/lib/aarch64-linux-gnu/nvidia /usr/lib/aarch64-linux-gnu
  NO_DEFAULT_PATH)

find_library(CUDART_LIB cudart
  HINTS ${_cuda_lib_hints} ENV CUDA_PATH
  PATHS /usr/local/cuda/lib64 /usr/lib/aarch64-linux-gnu
  NO_DEFAULT_PATH)

if(CUDA_INCLUDE_DIR AND NVJPEG_LIB AND CUDART_LIB)
  message(STATUS "nvJPEG hardware JPEG encoding enabled")
  message(STATUS "  cuda headers : ${CUDA_INCLUDE_DIR}")
  message(STATUS "  libnvjpeg    : ${NVJPEG_LIB}")
  message(STATUS "  libcudart    : ${CUDART_LIB}")
  target_compile_definitions(${target_name} PRIVATE HAS_CUDA)
  target_include_directories(${target_name} PRIVATE ${CUDA_INCLUDE_DIR})
  target_link_libraries(${target_name} ${NVJPEG_LIB} ${CUDART_LIB})
  target_sources(${target_name} PRIVATE src/nvjpeg_encoder.cpp)

  # Fused polarization kernel — requires nvcc
  include(CheckLanguage)
  check_language(CUDA)
  if(CMAKE_CUDA_COMPILER)
    enable_language(CUDA)
    # Orin is SM 8.7; allow override via -DCMAKE_CUDA_ARCHITECTURES=<arch>
    if(NOT DEFINED CMAKE_CUDA_ARCHITECTURES)
      set(CMAKE_CUDA_ARCHITECTURES 87)
    endif()
    message(STATUS "CUDA polarization kernel enabled (SM ${CMAKE_CUDA_ARCHITECTURES})")
    target_sources(${target_name} PRIVATE src/polarization_kernels.cu)
    set_source_files_properties(src/polarization_kernels.cu
      PROPERTIES COMPILE_OPTIONS "--expt-relaxed-constexpr")
    set_target_properties(${target_name} PROPERTIES CUDA_ARCHITECTURES "${CMAKE_CUDA_ARCHITECTURES}")
    target_compile_definitions(${target_name} PRIVATE HAS_POLAR_KERNEL)
  else()
    message(STATUS "nvcc not found — polarization kernel disabled (CPU fallback active)")
  endif()
else()
  message(STATUS "nvJPEG not available — software JPEG encoding + CPU polarization (cv::imencode)")
  if(NOT CUDA_INCLUDE_DIR)
    message(STATUS "  missing: cuda_runtime.h  →  sudo apt install cuda-cudart-dev-12-6")
  endif()
  if(NOT NVJPEG_LIB)
    message(STATUS "  missing: libnvjpeg.so    →  sudo apt install libnvjpeg-dev-12-6")
  endif()
  if(NOT CUDART_LIB)
    message(STATUS "  missing: libcudart.so    →  sudo apt install cuda-cudart-dev-12-6")
  endif()
endif()

# MV EXEC to lib dir
install(TARGETS
  ${target_name} 
    DESTINATION lib/${PROJECT_NAME}
)

# -----------------------------------------------------------------------------
#
# ROS2 RUN TRIGGER_IMAGE <>
#
# -----------------------------------------------------------------------------

set(trigger_image trigger_image)

# CPP -> EXECUTABLE
add_executable(${trigger_image} src/trigger_image_client.cpp)

# CPP -> .a file

# EXPORT .a file for other pakges ?
#ament_export_libraries(${PROJECT_NAME} ${rclcpp_LIBRARIES})

# ROS DEP
# - this way it will find our header libs automatically
# - for ArenAsdk the cmake way is used no ament
ament_target_dependencies(${trigger_image}  rclcpp 
                                          std_srvs)

# HEADERS DEP
target_include_directories(${trigger_image}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Ensure C++17 for the trigger_image target as well
target_compile_features(${trigger_image} PUBLIC cxx_std_17)

# MV EXEC to lib dir
install(TARGETS
  ${trigger_image} 
    DESTINATION lib/${PROJECT_NAME}
)

# -----------------------------------------------------------------------------

#
#
# PYTHON
#
#

#ament_cmake_python()


# invistifate ----------------------------
# Install python scripts
#install(DIRECTORY arena_camera_node
#        DESTINATION lib
#        FILES_MATCHING REGEX "arena_camera_node/.+\.py")

#install(DIRECTORY arena_camera_node
#        DESTINATION ../build
#        FILES_MATCHING REGEX "arena_camera_node/.+\.py")
# TESTING ---------------------------------------------------------------------

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  # Pixel format translation tests
  ament_add_gtest(test_pixelformat_translation
    test/test_pixelformat_translation.cpp
    src/rclcpp_adapter/pixelformat_translation.cpp
  )
  target_include_directories(test_pixelformat_translation PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
  )
  ament_target_dependencies(test_pixelformat_translation
    rclcpp
    sensor_msgs
    arena_sdk
  )

  # QoS translation tests
  ament_add_gtest(test_qos_translation
    test/test_qos_translation.cpp
    src/rclcpp_adapter/quality_of_service_translation.cpp
  )
  target_include_directories(test_qos_translation PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
  )
  ament_target_dependencies(test_qos_translation
    rclcpp
  )

  # Arena image RAII wrapper tests
  ament_add_gtest(test_arena_image_raii
    test/test_arena_image_raii.cpp
  )
  target_include_directories(test_arena_image_raii PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
  )
  ament_target_dependencies(test_arena_image_raii
    arena_sdk
    OpenCV
  )

  # Config helper and pixel format utility tests
  ament_add_gtest(test_config_helpers
    test/test_config_helpers.cpp
  )
  target_include_directories(test_config_helpers PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
  )
  target_link_libraries(test_config_helpers yaml-cpp)
endif()

ament_package()

#message(FATAL_ERROR ********************************************)