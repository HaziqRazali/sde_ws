#!/bin/bash
set -euo pipefail

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )/.." && pwd )"

IMAGE="u22_gpu_humble_sde:latest"
CONTAINER="u22_humble_sde"
DOCKERFILE="$DIR/docker/Dockerfile-deploy-sde-gpu_humble"

echo "Allowing X11 access..."
xhost +si:localuser:$USER
xhost +local:docker
xhost +local:root

echo "Building $IMAGE ..."
DOCKER_BUILDKIT=1 docker build -t "$IMAGE" \
  --build-arg UID="$(id -u)" \
  --build-arg GID="$(id -g)" \
  -f "$DOCKERFILE" \
  "$DIR"

echo "Removing old container (if any)..."
docker rm -f "$CONTAINER" >/dev/null 2>&1 || true

echo "Starting $CONTAINER ..."
docker run -d \
  --name "$CONTAINER" \
  --restart unless-stopped \
  --privileged \
  --net=host \
  --ipc=host \
  --runtime=nvidia \
  --gpus all \
  -e NVIDIA_VISIBLE_DEVICES=all \
  -e NVIDIA_DRIVER_CAPABILITIES=all \
  -e DISPLAY="$DISPLAY" \
  -e QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v "$DIR":/workspace/sde_ws \
  -w /workspace/sde_ws \
  "$IMAGE" \
  tail -f /dev/null

echo "Container started: $CONTAINER"
echo "Run: ./docker_scripts/join_humble.sh"