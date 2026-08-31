#!/usr/bin/env bash

# Compare a package source tree with the explicit artifact that contains its
# runtime code. Component wrappers and unrelated libraries are not valid
# freshness references because their timestamps can move independently.
runtime_binary_is_fresh() {
  local package_name="$1" executable_path="$2" source_dir="$3" runtime_artifact="$4"
  local newer_source scan_status

  if [ ! -x "${executable_path}" ]; then
    printf '%s executable_missing path=%s\n' "${package_name}" "${executable_path}"
    return 1
  fi
  if [ ! -d "${source_dir}" ]; then
    printf '%s source_missing path=%s\n' "${package_name}" "${source_dir}"
    return 1
  fi
  if [ ! -f "${runtime_artifact}" ]; then
    printf '%s artifact_missing path=%s\n' "${package_name}" "${runtime_artifact}"
    return 1
  fi

  newer_source="$(find "${source_dir}" -type f \
    \( \
      \( \
        \( -path "${source_dir}/src/*" -o -path "${source_dir}/include/*" \) \
        -a \( \
          -name '*.cpp' -o -name '*.hpp' -o -name '*.h' -o -name '*.hh' \
          -o -name '*.hxx' -o -name '*.c' -o -name '*.cc' -o -name '*.cxx' \
        \) \
      \) \
      -o -name 'CMakeLists.txt' -o -name 'package.xml' \
    \) \
    -newer "${runtime_artifact}" -print -quit 2>/dev/null)"
  scan_status=$?
  if [ "${scan_status}" -ne 0 ]; then
    printf '%s source_scan_failed path=%s\n' "${package_name}" "${source_dir}"
    return 1
  fi
  if [ -n "${newer_source}" ]; then
    printf '%s stale_binary source=%s artifact=%s\n' \
      "${package_name}" "${newer_source}" "${runtime_artifact}"
    return 1
  fi
  printf '%s fresh artifact=%s\n' "${package_name}" "${runtime_artifact}"
}

# A static library is fresh only when its source is not newer and each listed
# runtime dependent has been relinked after the library.
linked_library_is_propagated() {
  local library_name="$1" library_path="$2" source_dir="$3"
  shift 3
  local newer_source dependent stale=0 scan_status

  if [ ! -d "${source_dir}" ]; then
    printf '%s source_missing path=%s\n' "${library_name}" "${source_dir}"
    return 1
  fi
  if [ ! -f "${library_path}" ]; then
    printf '%s library_missing path=%s\n' "${library_name}" "${library_path}"
    return 1
  fi
  newer_source="$(find "${source_dir}" -type f \
    \( \
      \( \
        \( -path "${source_dir}/src/*" -o -path "${source_dir}/include/*" \) \
        -a \( \
          -name '*.cpp' -o -name '*.hpp' -o -name '*.h' -o -name '*.hh' \
          -o -name '*.hxx' -o -name '*.c' -o -name '*.cc' -o -name '*.cxx' \
        \) \
      \) \
      -o -name 'CMakeLists.txt' -o -name 'package.xml' \
    \) \
    -newer "${library_path}" -print -quit 2>/dev/null)"
  scan_status=$?
  if [ "${scan_status}" -ne 0 ]; then
    printf '%s source_scan_failed path=%s\n' "${library_name}" "${source_dir}"
    return 1
  fi
  if [ -n "${newer_source}" ]; then
    printf '%s stale_binary source=%s artifact=%s\n' \
      "${library_name}" "${newer_source}" "${library_path}"
    return 1
  fi
  for dependent in "$@"; do
    if [ ! -f "${dependent}" ]; then
      printf '%s dependent_missing path=%s\n' "${library_name}" "${dependent}"
      stale=1
      continue
    fi
    if [ "${library_path}" -nt "${dependent}" ]; then
      printf '%s stale_binary source=%s artifact=%s\n' \
        "${library_name}" "${library_path}" "${dependent}"
      stale=1
      continue
    fi
    printf '%s propagated artifact=%s\n' "${library_name}" "${dependent}"
  done
  return "${stale}"
}
