#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PYTHON="${PYTHON:-python3.12}"
DOCKER_SETUP=""

usage() {
  cat <<'EOF'
Usage: ./scripts/setup-unix.sh [DOCKER OPTION]

Docker options:
  --with-docker          Install Docker Engine on supported Linux distributions.
  --with-docker-desktop  Install Docker Desktop on macOS using Homebrew.
  --with-colima          Install Docker, Compose, and Colima on macOS using MacPorts.
EOF
}

for argument in "$@"; do
  case "$argument" in
    --with-docker)
      DOCKER_SETUP="engine"
      ;;
    --with-docker-desktop)
      DOCKER_SETUP="desktop"
      ;;
    --with-colima)
      DOCKER_SETUP="colima"
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown argument: $argument" >&2
      usage >&2
      exit 2
      ;;
  esac
done

if [[ $# -gt 1 ]]; then
  echo "Specify at most one Docker option." >&2
  usage >&2
  exit 2
fi

docker_is_ready() {
  command -v docker >/dev/null 2>&1 \
    && docker compose version >/dev/null 2>&1 \
    && docker info >/dev/null 2>&1
}

install_docker_desktop() {
  if ! command -v brew >/dev/null 2>&1; then
    echo "Homebrew is required for --with-docker-desktop." >&2
    echo "Alternatively use --with-colima with MacPorts." >&2
    exit 1
  fi

  brew install --cask docker-desktop
  echo "Docker Desktop installed. Start Docker.app and accept its first-run setup."
  echo 'If an older docker CLI takes precedence, put $HOME/.docker/bin first in PATH.'
}

install_docker_with_colima() {
  if ! command -v port >/dev/null 2>&1; then
    echo "MacPorts is required for --with-colima." >&2
    echo "Alternatively use --with-docker-desktop with Homebrew." >&2
    exit 1
  fi

  # The MacPorts docker port is only a client. Compose supplies the CLI
  # plugin, while Colima runs the Linux VM containing the Docker daemon.
  sudo port selfupdate
  sudo port install docker docker-compose-plugin colima
  colima start
}

install_docker_with_apt() {
  # Follow Docker's official repository installation rather than piping a
  # downloaded convenience script into a privileged shell.
  . /etc/os-release
  local distribution="$ID"
  local codename="${UBUNTU_CODENAME:-${VERSION_CODENAME:-}}"
  if [[ "$distribution" != "ubuntu" && "$distribution" != "debian" ]]; then
    echo "Automatic apt installation supports Debian and Ubuntu only." >&2
    exit 1
  fi
  if [[ -z "$codename" ]]; then
    echo "Could not determine the Linux release codename." >&2
    exit 1
  fi

  sudo apt-get update
  sudo apt-get install -y ca-certificates curl
  sudo install -m 0755 -d /etc/apt/keyrings
  sudo curl -fsSL \
    "https://download.docker.com/linux/$distribution/gpg" \
    -o /etc/apt/keyrings/docker.asc
  sudo chmod a+r /etc/apt/keyrings/docker.asc

  local architecture
  architecture="$(dpkg --print-architecture)"
  printf '%s\n' \
    "Types: deb" \
    "URIs: https://download.docker.com/linux/$distribution" \
    "Suites: $codename" \
    "Components: stable" \
    "Architectures: $architecture" \
    "Signed-By: /etc/apt/keyrings/docker.asc" \
    | sudo tee /etc/apt/sources.list.d/docker.sources >/dev/null

  sudo apt-get update
  sudo apt-get install -y \
    docker-ce \
    docker-ce-cli \
    containerd.io \
    docker-buildx-plugin \
    docker-compose-plugin
  sudo systemctl enable --now docker
}

install_docker_with_dnf() {
  sudo dnf -y install dnf-plugins-core
  sudo dnf config-manager addrepo \
    --from-repofile https://download.docker.com/linux/fedora/docker-ce.repo
  sudo dnf -y install \
    docker-ce \
    docker-ce-cli \
    containerd.io \
    docker-buildx-plugin \
    docker-compose-plugin
  sudo systemctl enable --now docker
}

install_docker_on_linux() {
  if [[ ! -r /etc/os-release ]]; then
    echo "Cannot identify this Linux distribution." >&2
    exit 1
  fi

  . /etc/os-release
  case "$ID" in
    ubuntu|debian)
      install_docker_with_apt
      ;;
    fedora)
      install_docker_with_dnf
      ;;
    *)
      echo "Automatic Docker installation is not supported for $ID." >&2
      echo "Use https://docs.docker.com/engine/install/, then rerun setup." >&2
      exit 1
      ;;
  esac

  # Docker group membership grants root-equivalent daemon access. Make the
  # change explicit here and tell the user when it becomes effective.
  sudo usermod -aG docker "$USER"
  echo "Added $USER to the docker group. Log out and back in before using Docker."
}

install_docker() {
  if docker_is_ready; then
    echo "Docker, Docker Compose, and the Docker daemon are already available."
    return
  fi

  local operating_system
  operating_system="$(uname -s)"
  case "$DOCKER_SETUP:$operating_system" in
    desktop:Darwin)
      install_docker_desktop
      ;;
    colima:Darwin)
      install_docker_with_colima
      ;;
    engine:Linux)
      install_docker_on_linux
      ;;
    engine:Darwin)
      echo "On macOS, choose --with-docker-desktop or --with-colima." >&2
      exit 2
      ;;
    *)
      echo "Docker option '$DOCKER_SETUP' is unsupported on $operating_system." >&2
      exit 2
      ;;
  esac
}

if [[ -n "$DOCKER_SETUP" ]]; then
  install_docker
fi

if ! command -v "$PYTHON" >/dev/null 2>&1; then
  echo "Python 3.12 is the default. Install it, or set PYTHON to a Python 3.12+ executable." >&2
  exit 1
fi

"$PYTHON" "$ROOT/scripts/check_python_version.py"
"$PYTHON" -m venv "$ROOT/.venv"
"$ROOT/.venv/bin/python" -m pip install --upgrade pip
"$ROOT/.venv/bin/python" -m pip install -r "$ROOT/requirements-hardware.txt" -r "$ROOT/requirements-simulation.txt"
"$ROOT/.venv/bin/python" "$ROOT/scripts/validate_environment.py"

echo
echo "Setup complete. Activate with: source $ROOT/.venv/bin/activate"
echo "Start cfclient with: cfclient"
echo "Run the radio check with: python scripts/validate_environment.py --radio"
if [[ -n "$DOCKER_SETUP" ]]; then
  echo "After Docker is running, check it with: python scripts/validate_environment.py --docker"
fi
