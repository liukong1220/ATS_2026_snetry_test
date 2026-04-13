cmake_minimum_required(VERSION 3.8)
project(hik_camera_ros2_driver)

## Use C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
## enforcing cleaner code.
add_definitions(-Wall -Werror)

## Export compile commands for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/hik_camera_node.cpp
)

set(HIKROBOT_SDK_ROOT "" CACHE PATH "Root directory of the HikRobot MVS SDK")
set(HIKROBOT_SDK_HINT_PATHS
  "${HIKROBOT_SDK_ROOT}"
  "$ENV{HIKROBOT_SDK_ROOT}"
  "$ENV{MVCAM_SDK_ROOT}"
  "$ENV{MVCAM_COMMON_RUNENV}"
  /opt/MVS
  /usr/local/MVS
  /opt/hikrobot/MVS
  /opt/HikRobot/MVS
  ${CMAKE_CURRENT_SOURCE_DIR}/hikSDK
)

if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
  set(HIKROBOT_LIB_SUFFIXES lib lib/64 lib/amd64 MvImport/lib/64 MvImport/lib/amd64)
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
  set(HIKROBOT_LIB_SUFFIXES lib lib/arm64 MvImport/lib/arm64)
else()
  message(FATAL_ERROR "Unsupported host system architecture: ${CMAKE_HOST_SYSTEM_PROCESSOR}!")
endif()

set(HIKROBOT_REQUIRED_LIBS
  FormatConversion
  MediaProcess
  MvCameraControl
  MVRender
  MvUsb3vTL
)

set(HIKROBOT_INCLUDE_DIR "")
set(HIKROBOT_LIBRARIES "")
set(HIKROBOT_LIBRARY_DIR "")

foreach(_hikrobot_root IN LISTS HIKROBOT_SDK_HINT_PATHS)
  if(NOT _hikrobot_root)
    continue()
  endif()

  unset(_hikrobot_include_candidate CACHE)
  find_path(_hikrobot_include_candidate
    NAMES MvCameraControl.h
    PATHS "${_hikrobot_root}"
    PATH_SUFFIXES include MvImport/include
    NO_DEFAULT_PATH
  )

  if(NOT _hikrobot_include_candidate)
    continue()
  endif()

  set(_hikrobot_found_all_libs TRUE)
  set(_hikrobot_library_candidates "")

  foreach(_hikrobot_lib IN LISTS HIKROBOT_REQUIRED_LIBS)
    unset(_hikrobot_library_candidate CACHE)
    find_library(_hikrobot_library_candidate
      NAMES "${_hikrobot_lib}"
      PATHS "${_hikrobot_root}"
      PATH_SUFFIXES ${HIKROBOT_LIB_SUFFIXES}
      NO_DEFAULT_PATH
    )

    if(NOT _hikrobot_library_candidate)
      set(_hikrobot_found_all_libs FALSE)
      break()
    endif()

    list(APPEND _hikrobot_library_candidates "${_hikrobot_library_candidate}")
  endforeach()

  if(_hikrobot_found_all_libs)
    set(HIKROBOT_INCLUDE_DIR "${_hikrobot_include_candidate}")
    set(HIKROBOT_LIBRARIES ${_hikrobot_library_candidates})
    list(GET HIKROBOT_LIBRARIES 0 _hikrobot_first_library)
    get_filename_component(HIKROBOT_LIBRARY_DIR "${_hikrobot_first_library}" DIRECTORY)
    break()
  endif()
endforeach()

if(NOT HIKROBOT_INCLUDE_DIR OR NOT HIKROBOT_LIBRARIES)
  message(FATAL_ERROR
    "HikRobot SDK not found. Set HIKROBOT_SDK_ROOT to the SDK root directory, "
    "or place the SDK under src/dependencies/hik_camera_ros2_driver/hikSDK. "
    "The SDK directory must contain MvCameraControl.h and the required shared libraries.")
endif()

message(STATUS "Using HikRobot SDK include dir: ${HIKROBOT_INCLUDE_DIR}")
message(STATUS "Using HikRobot SDK library dir: ${HIKROBOT_LIBRARY_DIR}")

target_include_directories(${PROJECT_NAME} PUBLIC "${HIKROBOT_INCLUDE_DIR}")
target_link_libraries(${PROJECT_NAME} ${HIKROBOT_LIBRARIES})

install(
  DIRECTORY "${HIKROBOT_LIBRARY_DIR}/"
  DESTINATION lib
  FILES_MATCHING
  PATTERN "*.so"
  PATTERN "*.so.*"
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN hik_camera_ros2_driver::HikCameraRos2DriverNode
  EXECUTABLE ${PROJECT_NAME}_node
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_copyright
    ament_cmake_cpplint
    ament_cmake_uncrustify
    ament_cmake_flake8
  )
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  launch
  config
)
