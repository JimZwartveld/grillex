#!/bin/bash
# =============================================================================
# Build Linux wheel for grillex using Docker
# =============================================================================
# This script builds a manylinux-compatible wheel inside Docker and copies
# it to the host machine for use in other projects.
#
# Usage:
#   ./scripts/build-linux-wheel.sh [output-dir]
#
# Arguments:
#   output-dir  Directory to output the wheel (default: ./dist-linux)
#
# Example:
#   ./scripts/build-linux-wheel.sh
#   ./scripts/build-linux-wheel.sh ../grillex-webapp/wheels
# =============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
OUTPUT_DIR="${1:-$PROJECT_ROOT/dist-linux}"

echo "=== Building Linux wheel for grillex ==="
echo "Project root: $PROJECT_ROOT"
echo "Output dir: $OUTPUT_DIR"

# Get version from git
cd "$PROJECT_ROOT"
GIT_DESCRIBE=$(git describe --tags --always 2>/dev/null || echo "")
if [[ "$GIT_DESCRIBE" =~ ^v?[0-9]+\.[0-9]+ ]]; then
    # Valid semver tag found
    VERSION="${GIT_DESCRIBE#v}"
else
    # No valid tag, use dev version with commit hash
    COMMIT=$(git rev-parse --short HEAD 2>/dev/null || echo "unknown")
    VERSION="0.0.0.dev0+g${COMMIT}"
fi
echo "Version: $VERSION"

# Create output directory
mkdir -p "$OUTPUT_DIR"

# Build the wheel using Docker
echo ""
echo "=== Building wheel in Docker ==="
docker build \
    --target builder \
    --build-arg SETUPTOOLS_SCM_PRETEND_VERSION="$VERSION" \
    -f mcp/Dockerfile \
    -t grillex-builder:latest \
    "$PROJECT_ROOT"

# Extract the wheel from the builder image
echo ""
echo "=== Extracting wheel ==="
CONTAINER_ID=$(docker create grillex-builder:latest)
docker cp "$CONTAINER_ID:/build/dist/." "$OUTPUT_DIR/"
docker rm "$CONTAINER_ID"

# List the built wheels
echo ""
echo "=== Built wheels ==="
ls -la "$OUTPUT_DIR"/*.whl

echo ""
echo "=== Done ==="
echo "Wheel(s) available in: $OUTPUT_DIR"
