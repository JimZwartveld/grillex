FROM python:3.11-slim

WORKDIR /app

# Install system dependencies for C++ extension
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    libeigen3-dev \
    curl \
    && rm -rf /var/lib/apt/lists/*

# Copy and install Python package
COPY pyproject.toml setup.py setup.cfg CMakeLists.txt ./
COPY src/ src/
COPY cpp/ cpp/

# Build C++ extension
# Set fallback version since git history is not available in Docker
ARG SETUPTOOLS_SCM_PRETEND_VERSION=0.0.0
ENV SETUPTOOLS_SCM_PRETEND_VERSION=${SETUPTOOLS_SCM_PRETEND_VERSION}

RUN pip install --no-cache-dir build wheel setuptools setuptools_scm

# Build C++ extension first
RUN cmake -B build -DCMAKE_BUILD_TYPE=Release \
    && cmake --build build -j$(nproc)

# Build and install Python wheel
RUN python -m build --wheel
RUN pip install --no-cache-dir dist/*.whl

# Copy webapp backend
COPY webapp/backend/ webapp/backend/

# Install webapp dependencies
RUN pip install --no-cache-dir fastapi uvicorn anthropic python-multipart pydantic-settings

EXPOSE 8000

HEALTHCHECK --interval=30s --timeout=10s --retries=3 \
    CMD curl -f http://localhost:8000/api/health || exit 1

CMD ["uvicorn", "webapp.backend.main:app", "--host", "0.0.0.0", "--port", "8000"]
