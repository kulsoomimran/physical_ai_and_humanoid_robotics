# Use Python 3.12 slim image as the base (compatible with PyTorch)
FROM python:3.12-slim

# Set environment variables
ENV PYTHONDONTWRITEBYTECODE=1 \
    PYTHONUNBUFFERED=1 \
    PYTHONPATH=/app \
    PIP_DEFAULT_TIMEOUT=300

# Set work directory
WORKDIR /app

# Install system dependencies
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        build-essential \
        curl \
        git \
        libpq-dev && \
    rm -rf /var/lib/apt/lists/*

# Use stable faster PyPI mirror + retries
RUN pip config set global.index-url https://pypi.org/simple && \
    pip config set global.timeout 300 && \
    pip config set global.retries 10

# Copy requirements first to leverage Docker cache
COPY requirements.txt .

# Install Python dependencies with PyTorch CPU index prioritized to ensure pre-compiled wheels
RUN pip install --no-cache-dir --prefer-binary -r requirements.txt \
    --index-url https://pypi.org/simple \
    --extra-index-url https://download.pytorch.org/whl/cpu

# Copy project
COPY . .

# Create non-root user
RUN adduser --disabled-password --gecos '' appuser && \
    chown -R appuser:appuser /app
USER appuser

# Expose port
EXPOSE 8000

# Run the application
CMD ["uvicorn", "src.main:app", "--host", "0.0.0.0", "--port", "8000"]
