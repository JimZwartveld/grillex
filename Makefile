# =============================================================================
# Grillex Makefile
# =============================================================================
# Convenient commands for building, testing, and deploying Grillex.
#
# Usage:
#   make build          # Build C++ extension
#   make test           # Run tests
#   make docker-mcp     # Build MCP server Docker image
#   make docker-backend # Build backend Docker image
#   make wheel          # Build Python wheel
# =============================================================================

.PHONY: build test clean wheel docker-mcp docker-backend docker-all version help

# Get version from git
VERSION := $(shell git describe --tags --always --dirty 2>/dev/null | sed 's/^v//' || echo "0.0.0.dev0")

# Default target
help:
	@echo "Grillex Build System"
	@echo "===================="
	@echo "Current version: $(VERSION)"
	@echo ""
	@echo "Build targets:"
	@echo "  make build           - Build C++ extension"
	@echo "  make wheel           - Build Python wheel"
	@echo "  make test            - Run tests"
	@echo "  make clean           - Clean build artifacts"
	@echo ""
	@echo "Docker targets:"
	@echo "  make docker-mcp      - Build MCP server image"
	@echo "  make docker-backend  - Build backend image (from source)"
	@echo "  make docker-frontend - Build frontend image"
	@echo "  make docker-all      - Build all Docker images"
	@echo ""
	@echo "Version: $(VERSION)"

# Show current version
version:
	@echo $(VERSION)

# Build C++ extension
build:
	cmake -B build -DCMAKE_BUILD_TYPE=Release
	cmake --build build -j$$(nproc 2>/dev/null || sysctl -n hw.ncpu 2>/dev/null || echo 4)

# Build Python wheel
wheel: build
	SETUPTOOLS_SCM_PRETEND_VERSION=$(VERSION) python -m build --wheel

# Run tests
test: build
	pytest tests/python -v --ignore=tests/python/test_gmsh*.py

# Clean build artifacts
clean:
	rm -rf build/ dist/ *.egg-info/ .pytest_cache/ .tox/
	find . -type d -name __pycache__ -exec rm -rf {} + 2>/dev/null || true
	find . -type f -name "*.so" -delete 2>/dev/null || true
	find . -type f -name "*.pyd" -delete 2>/dev/null || true

# =============================================================================
# Docker targets
# =============================================================================

# Build MCP server Docker image
docker-mcp:
	docker build \
		--build-arg GRILLEX_VERSION=$(VERSION) \
		-f mcp/Dockerfile \
		-t grillex-mcp:$(VERSION) \
		-t grillex-mcp:latest \
		.

# Build backend Docker image (from source)
docker-backend:
	docker build \
		--build-arg GRILLEX_VERSION=$(VERSION) \
		-f webapp/backend/Dockerfile \
		-t grillex-backend:$(VERSION) \
		-t grillex-backend:latest \
		.

# Build frontend Docker image
docker-frontend:
	docker build \
		-f webapp/frontend/Dockerfile \
		-t grillex-frontend:$(VERSION) \
		-t grillex-frontend:latest \
		webapp/frontend/

# Build all Docker images
docker-all: docker-mcp docker-backend docker-frontend
	@echo "Built all images with version: $(VERSION)"

# =============================================================================
# Development helpers
# =============================================================================

# Install in development mode
dev-install: build
	pip install -e .[testing,mcp]

# Format code
format:
	black src/grillex tests/
	ruff check --fix src/grillex tests/

# Lint code
lint:
	ruff check src/grillex tests/
	black --check src/grillex tests/
