#!/bin/bash

# YAQLE release preparation script.
# Usage: ./tools/prepare-release.sh [VERSION]
#
# Produces in release-artifacts/:
#   yaqle-<VERSION>-source.zip                      source code (git HEAD)
#   yaqle-<VERSION>-linux-x86_64.tar.gz             libyaqle.so + libyaqle.a + headers
#   yaqle-<VERSION>-linux-aarch64.tar.gz            libyaqle.so + libyaqle.a + headers (needs aarch64-linux-gnu-g++)
#   yaqle-<VERSION>-arm-none-eabi-hard.tar.gz        libyaqle.a + headers (bare-metal, hard float, no .so possible)
#
# Missing cross toolchains are skipped with a warning.

set -euo pipefail

PROJECT_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
VERSION="${1:-1.0.1}"
BUILD_DIR="$PROJECT_ROOT/build-release"
ARTIFACTS_DIR="$PROJECT_ROOT/release-artifacts"
CORTEX_M4_FLAGS="-mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard"
LINUX_ARM_PREFIX="${LINUX_ARM_PREFIX:-aarch64-linux-gnu-}"
BARE_ARM_PREFIX="${BARE_ARM_PREFIX:-arm-none-eabi-}"

# A cache in the source tree makes CMake ignore -B and reuse stale settings.
if [ -e "$PROJECT_ROOT/CMakeCache.txt" ]; then
    echo "error: stale in-source $PROJECT_ROOT/CMakeCache.txt, remove it (and CMakeFiles/)" >&2
    exit 1
fi

rm -rf "$BUILD_DIR" "$ARTIFACTS_DIR"
mkdir -p "$BUILD_DIR" "$ARTIFACTS_DIR"

# build_package <name> <targets> <libs> [cmake args...]
#   targets: space-separated CMake targets to build
#   libs:    space-separated library file names expected in the build dir
build_package() {
    local name="$1" targets="$2" libs="$3"
    shift 3
    local build="$BUILD_DIR/$name-build"
    local pkg="$BUILD_DIR/yaqle-${VERSION}-${name}"

    # -Wno-deprecated hides ETL's own old cmake_minimum_required warning
    cmake -Wno-deprecated -S "$PROJECT_ROOT" -B "$build" \
        -DCMAKE_BUILD_TYPE=Release \
        -DYAQLE_BUILD_TESTS=OFF \
        -DYAQLE_USE_COUT=OFF \
        "$@"
    # shellcheck disable=SC2086
    cmake --build "$build" --target $targets

    mkdir -p "$pkg/include" "$pkg/lib"
    cp "$PROJECT_ROOT"/include/*.hpp "$pkg/include/"
    for lib in $libs; do
        cp -a "$build/$lib" "$pkg/lib/"
    done
    cp "$PROJECT_ROOT/LICENSE.md" "$PROJECT_ROOT/README.md" "$pkg/"

    tar -czf "$ARTIFACTS_DIR/yaqle-${VERSION}-${name}.tar.gz" -C "$BUILD_DIR" "yaqle-${VERSION}-${name}"
    echo "✓ yaqle-${VERSION}-${name}.tar.gz"
}

echo "=== Linux x86_64 ==="
build_package linux-x86_64 "libyaqle libyaqle_shared" "libyaqle.a libyaqle.so" \
    -DCMAKE_C_COMPILER=gcc -DCMAKE_CXX_COMPILER=g++

echo ""
echo "=== Linux ARM (aarch64) ==="
if command -v "${LINUX_ARM_PREFIX}g++" >/dev/null 2>&1; then
    build_package linux-aarch64 "libyaqle libyaqle_shared" "libyaqle.a libyaqle.so" \
        -DCMAKE_SYSTEM_NAME=Linux \
        -DCMAKE_SYSTEM_PROCESSOR=aarch64 \
        -DCMAKE_C_COMPILER="${LINUX_ARM_PREFIX}gcc" \
        -DCMAKE_CXX_COMPILER="${LINUX_ARM_PREFIX}g++"
else
    echo "⚠ ${LINUX_ARM_PREFIX}g++ not found, skipping (override with LINUX_ARM_PREFIX)"
fi

echo ""
echo "=== ARM bare-metal (hard float) ==="
if command -v "${BARE_ARM_PREFIX}g++" >/dev/null 2>&1; then
    build_package arm-none-eabi-hard "libyaqle" "libyaqle.a" \
        -DCMAKE_SYSTEM_NAME=Generic \
        -DCMAKE_SYSTEM_PROCESSOR=arm \
        -DCMAKE_CXX_COMPILER="${BARE_ARM_PREFIX}g++" \
        -DCMAKE_CXX_FLAGS="$CORTEX_M4_FLAGS" \
        -DCMAKE_TRY_COMPILE_TARGET_TYPE=STATIC_LIBRARY
else
    echo "⚠ ${BARE_ARM_PREFIX}g++ not found, skipping (override with BARE_ARM_PREFIX)"
fi

echo ""
echo "=== Source archive ==="
if [ -n "$(git -C "$PROJECT_ROOT" status --porcelain --untracked-files=no)" ]; then
    echo "⚠ uncommitted changes: the archive only contains what is committed in HEAD"
fi
git -C "$PROJECT_ROOT" archive --format=zip --prefix="yaqle-${VERSION}/" \
    -o "$ARTIFACTS_DIR/yaqle-${VERSION}-source.zip" HEAD
echo "✓ yaqle-${VERSION}-source.zip"

echo ""
echo "Artifacts in: $ARTIFACTS_DIR"
ls -lh "$ARTIFACTS_DIR"
