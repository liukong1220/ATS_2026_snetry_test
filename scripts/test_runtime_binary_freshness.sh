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

echo "PASS: runtime binary freshness contract"
