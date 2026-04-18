#!/bin/bash
set -euo pipefail

CONTAINER="u22_humble_sde"

echo "Joining $CONTAINER ..."
docker start "$CONTAINER" >/dev/null
docker exec -it "$CONTAINER" bash