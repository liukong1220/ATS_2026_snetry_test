#!/usr/bin/env bash

set -u -o pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck source=scripts/runtime_binary_freshness.sh
source "${ROOT_DIR}/scripts/runtime_binary_freshness.sh"

TEST_DIR="$(mktemp -d /tmp/ats_runtime_freshness.XXXXXX)"
trap 'rm -rf -- "${TEST_DIR}"' EXIT

fail() {
  echo "FAIL: $*" >&2
  exit 1
}

assert_result() {
  local expected_rc="$1" expected_text="$2"
  shift 2
  local output rc
  set +e
  output="$("$@")"
  rc=$?
  set -e
  [ "${rc}" -eq "${expected_rc}" ] || \
    fail "expected rc=${expected_rc}, got rc=${rc}: ${output}"
  [[ "${output}" == *"${expected_text}"* ]] || \
    fail "expected '${expected_text}' in '${output}'"
}

set -e
mkdir -p "${TEST_DIR}/pkg/src" "${TEST_DIR}/build"
touch "${TEST_DIR}/pkg/src/node.cpp"
touch "${TEST_DIR}/build/node" "${TEST_DIR}/build/libtarget.so"
chmod +x "${TEST_DIR}/build/node"
touch -t 202601010000 "${TEST_DIR}/pkg/src/node.cpp"
touch -t 202601010001 "${TEST_DIR}/build/node" "${TEST_DIR}/build/libtarget.so"

assert_result 0 "pkg fresh artifact=${TEST_DIR}/build/libtarget.so" \
  runtime_binary_is_fresh pkg "${TEST_DIR}/build/node" \
  "${TEST_DIR}/pkg" "${TEST_DIR}/build/libtarget.so"

# A newer unrelated library cannot hide an old target artifact.
touch -t 202601010002 "${TEST_DIR}/pkg/src/node.cpp"
touch -t 202601010003 "${TEST_DIR}/build/libunrelated.so"
assert_result 1 "pkg stale_binary" \
  runtime_binary_is_fresh pkg "${TEST_DIR}/build/node" \
  "${TEST_DIR}/pkg" "${TEST_DIR}/build/libtarget.so"

assert_result 1 "source_missing" \
  runtime_binary_is_fresh pkg "${TEST_DIR}/build/node" \
  "${TEST_DIR}/missing_source" "${TEST_DIR}/build/libtarget.so"
assert_result 1 "artifact_missing" \
  runtime_binary_is_fresh pkg "${TEST_DIR}/build/node" \
  "${TEST_DIR}/pkg" "${TEST_DIR}/build/missing.so"

touch "${TEST_DIR}/build/libstatic.a" "${TEST_DIR}/build/dependent"
touch -t 202601010004 "${TEST_DIR}/build/libstatic.a"
touch -t 202601010005 "${TEST_DIR}/build/dependent"
touch -t 202601010003 "${TEST_DIR}/pkg/src/node.cpp"
assert_result 0 "propagated artifact=${TEST_DIR}/build/dependent" \
  linked_library_is_propagated static "${TEST_DIR}/build/libstatic.a" \
  "${TEST_DIR}/pkg" "${TEST_DIR}/build/dependent"

touch -t 202601010002 "${TEST_DIR}/build/dependent"
assert_result 1 "static stale_binary" \
  linked_library_is_propagated static "${TEST_DIR}/build/libstatic.a" \
  "${TEST_DIR}/pkg" "${TEST_DIR}/build/dependent"

# Independent Gazebo targets must be checked against their own installed code.
mkdir -p "${TEST_DIR}/gazebo/src" "${TEST_DIR}/gazebo/include" \
  "${TEST_DIR}/gazebo/build" "${TEST_DIR}/gazebo/install"
GZ="${TEST_DIR}/gazebo"
touch "${GZ}/src/recorder.cpp" "${GZ}/include/recorder.hpp" \
  "${GZ}/src/plugin.cpp" "${GZ}/include/kinematics.hpp" \
  "${GZ}/CMakeLists.txt" "${GZ}/package.xml" \
  "${GZ}/build/recorder.flags" "${GZ}/build/recorder.link" \
  "${GZ}/build/plugin.flags" "${GZ}/build/plugin.link" \
  "${GZ}/build/recorder" "${GZ}/build/plugin.so" "${GZ}/build/Makefile"
touch -t 202601010000 "${GZ}/src/"* "${GZ}/include/"* \
  "${GZ}/CMakeLists.txt" "${GZ}/package.xml" \
  "${GZ}/build/recorder.flags" "${GZ}/build/recorder.link" \
  "${GZ}/build/plugin.flags" "${GZ}/build/plugin.link"
touch -t 202601010001 "${GZ}/build/recorder" "${GZ}/build/plugin.so" \
  "${GZ}/build/Makefile"
ln -s "${GZ}/build/recorder" "${GZ}/install/recorder"
ln -s "${GZ}/build/plugin.so" "${GZ}/install/plugin.so"

recorder_fresh() {
  runtime_artifact_is_fresh recorder "${GZ}/install/recorder" \
    "${GZ}/src/recorder.cpp" "${GZ}/include/recorder.hpp" \
    "${GZ}/build/recorder.flags" "${GZ}/build/recorder.link"
}
plugin_fresh() {
  runtime_artifact_is_fresh plugin "${GZ}/install/plugin.so" \
    "${GZ}/src/plugin.cpp" "${GZ}/include/kinematics.hpp" \
    "${GZ}/build/plugin.flags" "${GZ}/build/plugin.link"
}
configuration_fresh() {
  runtime_artifact_is_fresh configuration "${GZ}/build/Makefile" \
    "${GZ}/CMakeLists.txt" "${GZ}/package.xml"
}
assert_result 0 "recorder fresh" recorder_fresh
assert_result 0 "plugin fresh artifact=${GZ}/build/plugin.so" plugin_fresh

# New symlink inode / unrelated recorder cannot hide an outdated native plugin.
touch -t 202601010002 "${GZ}/src/plugin.cpp"
touch -h -t 202601010005 "${GZ}/install/plugin.so"
assert_result 1 "plugin stale_binary" plugin_fresh
assert_result 0 "recorder fresh" recorder_fresh
touch -t 202601010003 "${GZ}/build/plugin.so"
assert_result 0 "plugin fresh" plugin_fresh

touch -t 202601010002 "${GZ}/include/recorder.hpp"
assert_result 1 "recorder stale_binary" recorder_fresh
assert_result 0 "plugin fresh" plugin_fresh
touch -t 202601010003 "${GZ}/build/recorder"

# Test-only CMake edits require configure evidence, not unrelated binary relinks.
touch -t 202601010004 "${GZ}/CMakeLists.txt" "${GZ}/package.xml"
assert_result 1 "configuration stale_binary" configuration_fresh
touch -t 202601010005 "${GZ}/build/Makefile"
assert_result 0 "configuration fresh" configuration_fresh
assert_result 0 "recorder fresh" recorder_fresh
assert_result 0 "plugin fresh" plugin_fresh

# Actual target command changes still demand a rebuilt corresponding artifact.
touch -t 202601010004 "${GZ}/build/plugin.flags"
assert_result 1 "plugin stale_binary" plugin_fresh
assert_result 0 "recorder fresh" recorder_fresh
touch -t 202601010006 "${GZ}/build/plugin.so"
assert_result 0 "plugin fresh" plugin_fresh
assert_result 1 "source_missing" runtime_artifact_is_fresh plugin \
  "${GZ}/install/plugin.so" "${GZ}/src/missing.cpp"
assert_result 1 "artifact_missing" runtime_artifact_is_fresh plugin \
  "${GZ}/install/missing.so" "${GZ}/src/plugin.cpp"
assert_result 1 "source_missing" runtime_artifact_is_fresh plugin \
  "${GZ}/install/plugin.so"

echo "PASS: runtime binary freshness contract"
