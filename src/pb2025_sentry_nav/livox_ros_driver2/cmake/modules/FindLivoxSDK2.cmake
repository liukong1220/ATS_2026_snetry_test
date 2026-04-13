include(FindPackageHandleStandardArgs)

set(_livox_sdk2_candidate_roots
  "${LIVOX_SDK2_ROOT}"
  "$ENV{LIVOX_SDK2_ROOT}"
  "$ENV{LIVOX_SDK_ROOT}"
  "${LivoxSDK2_BUNDLED_ROOT}"
)

if(CMAKE_PREFIX_PATH)
  list(APPEND _livox_sdk2_candidate_roots ${CMAKE_PREFIX_PATH})
endif()

list(APPEND _livox_sdk2_candidate_roots
  /usr/local
  /usr
)

set(_livox_sdk2_library_suffixes lib lib64)
if(CMAKE_LIBRARY_ARCHITECTURE)
  list(APPEND _livox_sdk2_library_suffixes "lib/${CMAKE_LIBRARY_ARCHITECTURE}")
endif()

if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
  list(APPEND _livox_sdk2_library_suffixes lib/64 lib/amd64)
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
  list(APPEND _livox_sdk2_library_suffixes lib/arm64)
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "riscv64")
  list(APPEND _livox_sdk2_library_suffixes lib/riscv64)
endif()

set(_livox_sdk2_normalized_roots)
foreach(_livox_sdk2_root IN LISTS _livox_sdk2_candidate_roots)
  if(_livox_sdk2_root)
    file(TO_CMAKE_PATH "${_livox_sdk2_root}" _livox_sdk2_root_normalized)
    list(APPEND _livox_sdk2_normalized_roots "${_livox_sdk2_root_normalized}")
  endif()
endforeach()
list(REMOVE_DUPLICATES _livox_sdk2_normalized_roots)

foreach(_livox_sdk2_root IN LISTS _livox_sdk2_normalized_roots)
  unset(_livox_sdk2_include_candidate CACHE)
  unset(_livox_sdk2_library_candidate CACHE)

  find_path(_livox_sdk2_include_candidate
    NAMES livox_lidar_api.h
    PATHS "${_livox_sdk2_root}"
    PATH_SUFFIXES include
    NO_DEFAULT_PATH
  )

  find_library(_livox_sdk2_library_candidate
    NAMES livox_lidar_sdk_shared
    PATHS "${_livox_sdk2_root}"
    PATH_SUFFIXES ${_livox_sdk2_library_suffixes}
    NO_DEFAULT_PATH
  )

  if(_livox_sdk2_include_candidate AND _livox_sdk2_library_candidate)
    set(LivoxSDK2_ROOT_DIR "${_livox_sdk2_root}")
    set(LivoxSDK2_INCLUDE_DIR "${_livox_sdk2_include_candidate}")
    set(LivoxSDK2_LIBRARY "${_livox_sdk2_library_candidate}")
    break()
  endif()
endforeach()

if(LivoxSDK2_LIBRARY)
  get_filename_component(LivoxSDK2_LIBRARY_DIR "${LivoxSDK2_LIBRARY}" DIRECTORY)
endif()

find_package_handle_standard_args(LivoxSDK2
  REQUIRED_VARS LivoxSDK2_INCLUDE_DIR LivoxSDK2_LIBRARY
)

if(NOT LivoxSDK2_FOUND)
  message(FATAL_ERROR
    "Livox SDK2 not found. Set LIVOX_SDK2_ROOT to a valid Livox-SDK2 install prefix, "
    "or run src/pb2025_sentry_nav/livox_ros_driver2/scripts/setup_livox_sdk2.sh "
    "to install the SDK into the workspace.")
endif()

if(NOT TARGET LivoxSDK2::sdk)
  add_library(LivoxSDK2::sdk SHARED IMPORTED)
  set_target_properties(LivoxSDK2::sdk PROPERTIES
    IMPORTED_LOCATION "${LivoxSDK2_LIBRARY}"
    INTERFACE_INCLUDE_DIRECTORIES "${LivoxSDK2_INCLUDE_DIR}"
  )
endif()

mark_as_advanced(
  LivoxSDK2_INCLUDE_DIR
  LivoxSDK2_LIBRARY
  LivoxSDK2_ROOT_DIR
  LivoxSDK2_LIBRARY_DIR
)
