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

# Get version from git (must be PEP 440 compliant)
cd "$PROJECT_ROOT"
GIT_DESCRIBE=$(git describe --tags --always 2>/dev/null || echo "")

# Convert git describe to PEP 440 format
# Examples:
#   v0.1.0          -> 0.1.0
#   0.1.0           -> 0.1.0
#   v0.1.0-17-gaf70 -> 0.1.0.dev17+gaf70
#   0.1.0-17-gaf70  -> 0.1.0.dev17+gaf70
#   af70da3         -> 0.0.0.dev0+gaf70da3

# Strip leading 'v' if present
GIT_DESCRIBE="${GIT_DESCRIBE#v}"

if echo "$GIT_DESCRIBE" | grep -qE '^[0-9]+\.[0-9]+\.[0-9]+$'; then
    # Exact semver tag (e.g., 0.1.0)
    VERSION="$GIT_DESCRIBE"
elif echo "$GIT_DESCRIBE" | grep -qE '^[0-9]+\.[0-9]+\.[0-9]+-[0-9]+-g[a-f0-9]+$'; then
    # Tag with commits after (e.g., 0.1.0-17-gaf70da3)
    # Convert to PEP 440: 0.1.0.dev17+gaf70da3
    BASE_VERSION=$(echo "$GIT_DESCRIBE" | sed -E 's/^([0-9]+\.[0-9]+\.[0-9]+)-[0-9]+-g[a-f0-9]+$/\1/')
    COMMIT_COUNT=$(echo "$GIT_DESCRIBE" | sed -E 's/^[0-9]+\.[0-9]+\.[0-9]+-([0-9]+)-g[a-f0-9]+$/\1/')
    COMMIT_HASH=$(echo "$GIT_DESCRIBE" | sed -E 's/^[0-9]+\.[0-9]+\.[0-9]+-[0-9]+-g([a-f0-9]+)$/\1/')
    VERSION="${BASE_VERSION}.dev${COMMIT_COUNT}+g${COMMIT_HASH}"
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
