# Private PyPI Server

This directory contains configuration for running a private PyPI server using devpi.

## Quick Start

```bash
# Start the server
docker-compose -f deployment/pypi/docker-compose.yml up -d

# Wait for it to be ready
sleep 10

# Initialize (first time only)
docker exec devpi-server devpi-init
```

## Server URLs

| Service | URL |
|---------|-----|
| Web UI | http://localhost:3141 |
| Simple API | http://localhost:3141/root/pypi/+simple/ |

## Configuration

### 1. Create a User and Index

```bash
# Install devpi-client
pip install devpi-client

# Connect to server
devpi use http://localhost:3141

# Login as root (default password: changeme)
devpi login root --password changeme

# Change root password
devpi user -m root password=YOUR_SECURE_PASSWORD

# Create a user
devpi user -c myuser password=mypassword email=me@example.com

# Login as your user
devpi login myuser

# Create an index (inherits from PyPI for dependencies)
devpi index -c dev bases=root/pypi
```

### 2. Configure pip to Use Your Server

**Option A: pip.conf (global)**

```ini
# ~/.pip/pip.conf (Linux/macOS)
# %APPDATA%\pip\pip.ini (Windows)

[global]
index-url = http://localhost:3141/myuser/dev/+simple/
trusted-host = localhost

[search]
index = http://localhost:3141/myuser/dev/
```

**Option B: Environment variable**

```bash
export PIP_INDEX_URL=http://localhost:3141/myuser/dev/+simple/
export PIP_TRUSTED_HOST=localhost
```

**Option C: Per-command**

```bash
pip install --index-url http://localhost:3141/myuser/dev/+simple/ grillex
```

### 3. Configure twine for Uploads

Create `~/.pypirc`:

```ini
[distutils]
index-servers =
    local
    testpypi
    pypi

[local]
repository = http://localhost:3141/myuser/dev/
username = myuser
password = mypassword

[testpypi]
repository = https://test.pypi.org/legacy/
username = __token__
password = pypi-xxx

[pypi]
repository = https://upload.pypi.org/legacy/
username = __token__
password = pypi-xxx
```

## Uploading Packages

```bash
# Build wheels
python -m build

# Upload to local server
twine upload -r local dist/*

# Or with devpi-client
devpi upload dist/*
```

## CI/CD Integration

Add to your GitHub Actions workflow:

```yaml
- name: Publish to Private PyPI
  env:
    TWINE_USERNAME: ${{ secrets.PRIVATE_PYPI_USER }}
    TWINE_PASSWORD: ${{ secrets.PRIVATE_PYPI_PASS }}
    TWINE_REPOSITORY_URL: ${{ secrets.PRIVATE_PYPI_URL }}
  run: |
    twine upload dist/*
```

## Caching Public PyPI

devpi automatically caches packages from PyPI. First install from your server:

```bash
# This fetches from PyPI and caches locally
pip install --index-url http://localhost:3141/root/pypi/+simple/ numpy
```

Subsequent installs (by any user) will use the cached version.

## Backup

```bash
# Export all data
docker exec devpi-server devpi-export /tmp/backup
docker cp devpi-server:/tmp/backup ./devpi-backup-$(date +%Y%m%d)

# Import backup
docker cp ./devpi-backup-20240101 devpi-server:/tmp/backup
docker exec devpi-server devpi-import /tmp/backup
```

## SSL/TLS Setup

For production, add SSL:

1. Place certificates in `deployment/pypi/certs/`
2. Create `nginx.conf`:

```nginx
server {
    listen 443 ssl;
    server_name pypi.yourcompany.com;

    ssl_certificate /etc/nginx/certs/cert.pem;
    ssl_certificate_key /etc/nginx/certs/key.pem;

    location / {
        proxy_pass http://devpi:3141;
        proxy_set_header Host $host;
        proxy_set_header X-Real-IP $remote_addr;
        proxy_set_header X-Forwarded-Proto https;
    }
}
```

3. Start with SSL profile:
```bash
docker-compose --profile ssl up -d
```
