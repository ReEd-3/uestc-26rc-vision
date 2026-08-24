cmake_minimum_required(VERSION 3.16)
project(kfs_vision LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(custom_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Threads REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(nlohmann_json 3.2 REQUIRED)
find_package(OpenCV 4 REQUIRED COMPONENTS core imgproc highgui)
find_package(PCL 1.14 REQUIRED COMPONENTS common)
find_package(OrbbecSDK REQUIRED CONFIG)

set(ONNXRUNTIME_ROOT
    "${CMAKE_CURRENT_SOURCE_DIR}/third_party/onnxruntime-linux-x64-gpu_cuda13-1.29.0"
    CACHE PATH "ONNX Runtime CUDA 13 SDK root")
if(NOT EXISTS "${ONNXRUNTIME_ROOT}/include/onnxruntime_cxx_api.h" OR
   NOT EXISTS "${ONNXRUNTIME_ROOT}/lib/libonnxruntime.so" OR
   NOT EXISTS "${ONNXRUNTIME_ROOT}/lib/libonnxruntime_providers_shared.so" OR
   NOT EXISTS "${ONNXRUNTIME_ROOT}/lib/libonnxruntime_providers_cuda.so")
  message(FATAL_ERROR
    "ONNX Runtime CUDA 13 SDK is missing or incomplete: ${ONNXRUNTIME_ROOT}\n"
    "Run: ${CMAKE_CURRENT_SOURCE_DIR}/scripts/setup_onnxruntime.sh")
endif()

get_target_property(ORBBECSDK_LIBRARY ob::OrbbecSDK IMPORTED_LOCATION_RELEASE)
if(NOT ORBBECSDK_LIBRARY)
  get_target_property(ORBBECSDK_LIBRARY ob::OrbbecSDK IMPORTED_LOCATION)
endif()
if(NOT ORBBECSDK_LIBRARY OR NOT EXISTS "${ORBBECSDK_LIBRARY}")
  message(FATAL_ERROR "Could not resolve the OrbbecSDK imported runtime library")
endif()
get_filename_component(ORBBECSDK_LIBRARY_DIR "${ORBBECSDK_LIBRARY}" DIRECTORY)
set(ORBBECSDK_EXTENSIONS_DIR "${ORBBECSDK_LIBRARY_DIR}/extensions")
if(NOT EXISTS
   "${ORBBECSDK_EXTENSIONS_DIR}/frameprocessor/libob_frame_processor.so" OR
   NOT EXISTS
   "${ORBBECSDK_EXTENSIONS_DIR}/filters/libob_priv_filter.so" OR
   NOT EXISTS
   "${ORBBECSDK_EXTENSIONS_DIR}/filters/libFilterProcessor.so")
  message(FATAL_ERROR
    "OrbbecSDK runtime extensions are missing below: "
    "${ORBBECSDK_EXTENSIONS_DIR}\n"
    "Install the complete OrbbecSDK runtime, not only libOrbbecSDK.so.")
endif()

add_library(kfs_geometry
  src/config.cpp
  src/plane_pose.cpp
)
target_include_directories(kfs_geometry PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)
target_link_libraries(kfs_geometry PUBLIC
  Eigen3::Eigen
  nlohmann_json::nlohmann_json
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
)

add_library(kfs_onnx src/yolo_seg.cpp)
target_include_directories(kfs_onnx PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${ONNXRUNTIME_ROOT}/include
  ${OpenCV_INCLUDE_DIRS}
)
target_link_directories(kfs_onnx PUBLIC "${ONNXRUNTIME_ROOT}/lib")
target_link_libraries(kfs_onnx PUBLIC ${OpenCV_LIBS} onnxruntime)

add_library(kfs_target_message src/target_message.cpp)
target_include_directories(kfs_target_message PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(kfs_target_message kfs_geometry kfs_onnx)
ament_target_dependencies(kfs_target_message custom_msgs)

add_executable(kfs_vision_node
  src/main.cpp
  src/kfs_vision_node.cpp
  src/debug_view.cpp
)
target_include_directories(kfs_vision_node PRIVATE
  include
  ${ONNXRUNTIME_ROOT}/include
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)
target_link_directories(kfs_vision_node PRIVATE "${ONNXRUNTIME_ROOT}/lib")
target_link_libraries(kfs_vision_node
  kfs_geometry
  kfs_onnx
  kfs_target_message
  ob::OrbbecSDK
  Threads::Threads
)
ament_target_dependencies(kfs_vision_node
  ament_index_cpp
  custom_msgs
  rclcpp
)

add_executable(kfs_model_check src/model_check.cpp)
target_include_directories(kfs_model_check PRIVATE
  include
  ${ONNXRUNTIME_ROOT}/include
  ${OpenCV_INCLUDE_DIRS}
)
target_link_directories(kfs_model_check PRIVATE "${ONNXRUNTIME_ROOT}/lib")
target_link_libraries(kfs_model_check kfs_onnx)
ament_target_dependencies(kfs_model_check ament_index_cpp)

foreach(target kfs_vision_node kfs_model_check)
  set_target_properties(${target} PROPERTIES
    BUILD_RPATH "${ONNXRUNTIME_ROOT}/lib;${ORBBECSDK_LIBRARY_DIR}"
    INSTALL_RPATH "$ORIGIN/.."
  )
  target_link_options(${target} PRIVATE "-Wl,--disable-new-dtags")
endforeach()

install(TARGETS kfs_vision_node kfs_model_check
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include
)
install(DIRECTORY config launch models scripts
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
)

# Keep the installed node independent of the build/source directory. The exact
# Orbbec runtime selected by find_package, its runtime-loaded extensions, and
# the required ONNX Runtime CUDA libraries are installed with the package.
# The Orbbec SDK resolves extensions relative to its runtime library directory.
install(FILES "${ORBBECSDK_LIBRARY}"
  DESTINATION lib
  RENAME libOrbbecSDK.so.2
)
install(DIRECTORY "${ORBBECSDK_EXTENSIONS_DIR}/"
  DESTINATION lib/extensions
  USE_SOURCE_PERMISSIONS
  FILES_MATCHING PATTERN "*.so*"
)
if(EXISTS "${ORBBECSDK_LIBRARY_DIR}/OrbbecSDKConfig.xml")
  install(FILES "${ORBBECSDK_LIBRARY_DIR}/OrbbecSDKConfig.xml"
    DESTINATION lib
  )
endif()
install(DIRECTORY "${ONNXRUNTIME_ROOT}/lib/"
  DESTINATION lib
  USE_SOURCE_PERMISSIONS
  FILES_MATCHING
    PATTERN "libonnxruntime.so*"
    PATTERN "libonnxruntime_providers_shared.so"
    PATTERN "libonnxruntime_providers_cuda.so"
    PATTERN "cmake" EXCLUDE
    PATTERN "pkgconfig" EXCLUDE
)

include(CTest)
if(BUILD_TESTING)
  add_executable(geometry_test tests/geometry_test.cpp)
  target_link_libraries(geometry_test kfs_geometry)
  add_test(NAME geometry_test COMMAND geometry_test)

  add_executable(target_message_test tests/target_message_test.cpp)
  target_link_libraries(target_message_test kfs_target_message)
  ament_target_dependencies(target_message_test custom_msgs)
  add_test(NAME target_message_test COMMAND target_message_test)
endif()

ament_package()
