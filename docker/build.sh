#!/usr/bin/env bash
# Build a Docker image for arena_camera_node.
#
# Usage:
#   ./docker/build.sh [amd64|arm64] [image-tag]
#
# Examples:
#   ./docker/build.sh amd64                  # build x86-64 image
#   ./docker/build.sh arm64                  # build ARM64 image (via QEMU on x86)
#   ./docker/build.sh arm64 my_camera:latest # custom tag
#
# Prerequisites (one-time setup):
#   sudo apt install docker.io qemu-user-static binfmt-support
#   sudo usermod -aG docker $USER   # then log out/in
#   docker run --rm --privileged multiarch/qemu-user-static --reset -p yes

set -euo pipefail

ARCH=${1:-amd64}
TAG=${2:-arena_camera_node}
REPO_ROOT=$(cd "$(dirname "$0")/.." && pwd)

case "$ARCH" in
  amd64)
    PLATFORM=linux/amd64
    SDK=/opt/ArenaSDK_Linux_x64
    ;;
  arm64)
    PLATFORM=linux/arm64
    SDK=/opt/ArenaSDK_Linux_ARM64
    ;;
  *)
    echo "Usage: $0 [amd64|arm64] [tag]"
    exit 1
    ;;
esac

if [[ ! -d "$SDK" ]]; then
  echo "ERROR: ArenaSDK not found at $SDK"
  echo "  For amd64: install from resources/ArenaSDK/linux64/"
  echo "  For arm64: install from resources/ArenaSDK/linux_arm64/ (see docs/orin-deployment.md)"
  exit 1
fi

# Ensure BuildKit is enabled (required for --build-context)
export DOCKER_BUILDKIT=1

echo "==> Building ${TAG}:${ARCH}  (platform: ${PLATFORM})"
echo "    SDK: ${SDK}"
echo "    Source: ${REPO_ROOT}"
echo ""

docker buildx build \
  --platform "$PLATFORM" \
  --build-context sdk="$SDK" \
  -t "${TAG}:${ARCH}" \
  -f "$REPO_ROOT/docker/Dockerfile.${ARCH}" \
  "$REPO_ROOT"

echo ""
echo "==> Done: ${TAG}:${ARCH}"
echo ""
echo "Run the node (waits for camera):"
echo "  docker run --rm --network host ${TAG}:${ARCH}"
echo ""
echo "Interactive shell:"
echo "  docker run --rm -it --network host --entrypoint bash ${TAG}:${ARCH}"
