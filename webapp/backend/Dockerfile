FROM python:3.11-slim

WORKDIR /app

# Install system dependencies for C++ extension
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    libeigen3-dev \
    curl \
    git \
    && rm -rf /var/lib/apt/lists/*

# Install pybind11 for C++ bindings
RUN pip install --no-cache-dir pybind11

# Copy and install Python package
COPY pyproject.toml .
COPY src/ src/
COPY cpp/ cpp/
COPY CMakeLists.txt .

# Build C++ extension with CMake first
RUN mkdir -p build && cd build && cmake .. && make -j4

# Set version for setuptools-scm (git history not available in Docker)
ENV SETUPTOOLS_SCM_PRETEND_VERSION=2.0.0

# Build and install the Python package
RUN pip install --no-cache-dir build
RUN python -m build --wheel
RUN pip install --no-cache-dir dist/*.whl

# Install grillex runtime dependencies
RUN pip install --no-cache-dir "numpy>=1.24.0" "scipy>=1.10.0" "pyyaml>=6.0" "pydantic>=2.0"

# Copy webapp backend
COPY webapp/__init__.py webapp/
COPY webapp/backend/ webapp/backend/

# Install webapp dependencies
RUN pip install --no-cache-dir fastapi uvicorn anthropic python-multipart pydantic-settings

EXPOSE 8000

HEALTHCHECK --interval=30s --timeout=10s --retries=3 \
    CMD curl -f http://localhost:8000/api/health || exit 1

CMD ["uvicorn", "webapp.backend.main:app", "--host", "0.0.0.0", "--port", "8000"]
