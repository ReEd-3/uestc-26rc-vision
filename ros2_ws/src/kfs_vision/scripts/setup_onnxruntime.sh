#!/usr/bin/env bash
set -euo pipefail

readonly ORT_VERSION="1.29.0"
readonly ORT_DIR="onnxruntime-linux-x64-gpu_cuda13-${ORT_VERSION}"
readonly ORT_ARCHIVE="${ORT_DIR}.tgz"
readonly ORT_URL="https://github.com/microsoft/onnxruntime/releases/download/v${ORT_VERSION}/${ORT_ARCHIVE}"
readonly ORT_SHA256="844c64acfc43ab9423215c26493055ea229268e28283146cc644ecef0bdae048"

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PACKAGE_DIR="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
DESTINATION="${PACKAGE_DIR}/third_party/${ORT_DIR}"
SOURCE_DIR=""

usage() {
  echo "Usage: $0 [--from /absolute/path/to/extracted/${ORT_DIR}]"
  echo "Without --from, the pinned official archive is downloaded and SHA-256 verified."
}

if [[ $# -gt 0 ]]; then
  if [[ $# -ne 2 || $1 != "--from" ]]; then
    usage >&2
    exit 2
  fi
  SOURCE_DIR=$2
fi

if [[ $(uname -m) != "x86_64" ]]; then
  echo "This package requires the Linux x86_64 ONNX Runtime build." >&2
  exit 1
fi

is_complete() {
  local root=$1
  [[ -f "${root}/include/onnxruntime_cxx_api.h" &&
     -e "${root}/lib/libonnxruntime.so" &&
     -f "${root}/lib/libonnxruntime_providers_shared.so" &&
     -f "${root}/lib/libonnxruntime_providers_cuda.so" ]]
}

if is_complete "${DESTINATION}"; then
  echo "ONNX Runtime is already ready: ${DESTINATION}"
  exit 0
fi
if [[ -e "${DESTINATION}" ]]; then
  echo "Destination exists but is incomplete: ${DESTINATION}" >&2
  echo "Move it aside, then run this script again." >&2
  exit 1
fi

mkdir -p "${PACKAGE_DIR}/third_party"
TEMP_DIR="$(mktemp -d "${TMPDIR:-/tmp}/kfs-onnxruntime.XXXXXX")"
trap 'rm -rf -- "${TEMP_DIR}"' EXIT

if [[ -n ${SOURCE_DIR} ]]; then
  if [[ ${SOURCE_DIR} != /* ]]; then
    echo "--from path must be absolute: ${SOURCE_DIR}" >&2
    exit 1
  fi
  if ! is_complete "${SOURCE_DIR}"; then
    echo "Source SDK is incomplete: ${SOURCE_DIR}" >&2
    exit 1
  fi
  mkdir "${TEMP_DIR}/${ORT_DIR}"
  cp -a "${SOURCE_DIR}/." "${TEMP_DIR}/${ORT_DIR}/"
else
  echo "Downloading ${ORT_URL}"
  curl --fail --location --retry 3 --output "${TEMP_DIR}/${ORT_ARCHIVE}" "${ORT_URL}"
  echo "${ORT_SHA256}  ${TEMP_DIR}/${ORT_ARCHIVE}" | sha256sum --check --status
  echo "SHA-256 verified: ${ORT_SHA256}"
  tar -xzf "${TEMP_DIR}/${ORT_ARCHIVE}" -C "${TEMP_DIR}"
fi

if ! is_complete "${TEMP_DIR}/${ORT_DIR}"; then
  echo "Prepared SDK is incomplete after extraction/copy." >&2
  exit 1
fi

mv "${TEMP_DIR}/${ORT_DIR}" "${DESTINATION}"
echo "ONNX Runtime installed at: ${DESTINATION}"

