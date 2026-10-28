FROM python:3.12-slim

ENV PIP_DISABLE_PIP_VERSION_CHECK=1 \
    PYTHONDONTWRITEBYTECODE=1 \
    PYTHONUNBUFFERED=1

WORKDIR /topical-lectures
COPY requirements-simulation.txt .
RUN python -m pip install --no-cache-dir -r requirements-simulation.txt

COPY code code
COPY examples examples
COPY docs docs
COPY scripts scripts
COPY README.md .

EXPOSE 8888
CMD ["python", "-m", "jupyterlab", "--ip=0.0.0.0", "--port=8888", "--no-browser", "--allow-root", "--ServerApp.token=", "--ServerApp.password="]
