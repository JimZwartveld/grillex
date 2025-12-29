# =============================================================================
# Grillex MCP Server Dockerfile
# =============================================================================
# Multi-stage build for efficient MCP server deployment
#
# Usage:
#   docker build -f mcp/Dockerfile -t grillex-mcp .
#   docker run -i grillex-mcp
#
# For Claude Desktop integration, add to claude_desktop_config.json:
#   {
#     "mcpServers": {
#       "grillex": {
#         "command": "docker",
#         "args": ["run", "-i", "--rm", "grillex-mcp"]
#       }
#     }
#   }
# =============================================================================

# -----------------------------------------------------------------------------
# Stage 1: Build the C++ extension and Python wheel
# -----------------------------------------------------------------------------
FROM python:3.11-slim AS builder

WORKDIR /build

# Install build dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    libeigen3-dev \
    git \
    && rm -rf /var/lib/apt/lists/*

# Copy source files needed for build
COPY pyproject.toml setup.py setup.cfg CMakeLists.txt ./
COPY src/ src/
COPY cpp/ cpp/

# Set fallback version since git history is not available in Docker
ARG SETUPTOOLS_SCM_PRETEND_VERSION=0.0.0
ENV SETUPTOOLS_SCM_PRETEND_VERSION=${SETUPTOOLS_SCM_PRETEND_VERSION}

# Install build dependencies including pybind11 (provides CMake config)
RUN pip install --no-cache-dir build wheel setuptools setuptools_scm pybind11

# Build C++ extension first
# Use pybind11's CMake config from pip install
RUN cmake -B build -DCMAKE_BUILD_TYPE=Release \
    -Dpybind11_DIR=$(python -c "import pybind11; print(pybind11.get_cmake_dir())") \
    && cmake --build build -j$(nproc)

# Build Python wheel
RUN python -m build --wheel

# Verify wheel was created
RUN ls -la dist/ && test -f dist/*.whl

# -----------------------------------------------------------------------------
# Stage 2: Runtime image
# -----------------------------------------------------------------------------
FROM python:3.11-slim AS runtime

WORKDIR /app

# Install minimal runtime dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    libgomp1 \
    && rm -rf /var/lib/apt/lists/* \
    && apt-get clean

# Create non-root user for security
RUN useradd --create-home --shell /bin/bash grillex

# Copy built wheel from builder stage
COPY --from=builder /build/dist/ /tmp/dist/

# Install the grillex package with MCP dependencies
RUN pip install --no-cache-dir /tmp/dist/grillex-*.whl[mcp] \
    && rm -rf /tmp/dist \
    && pip cache purge

# Switch to non-root user
USER grillex

# Set environment variables
ENV PYTHONUNBUFFERED=1
ENV PYTHONDONTWRITEBYTECODE=1

# Health check (MCP servers use stdio, so we just check the module loads)
HEALTHCHECK --interval=30s --timeout=10s --start-period=5s --retries=3 \
    CMD python -c "from grillex.mcp import main; print('OK')" || exit 1

# Default command runs the MCP server
ENTRYPOINT ["python", "-m", "grillex.mcp.server"]

# Metadata labels
LABEL org.opencontainers.image.title="Grillex MCP Server"
LABEL org.opencontainers.image.description="MCP server for Grillex structural analysis"
LABEL org.opencontainers.image.source="https://github.com/JimZwartveld/grillex"
LABEL org.opencontainers.image.licenses="MIT"
