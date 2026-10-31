# Optional container setup

[Software setup](software_setup.md) | [Documentation guide](README.md)

Docker provides an alternative environment for simulations and notebooks.
Use the [native Python environment](software_setup.md#native-setup) for cfclient
and Crazyradio access. Run the commands below from the project root.

## Install and start Docker

On macOS, choose either **Docker Desktop or Colima**. Colima provides a Linux VM
for Docker. The setup scripts can install container tools:

| Platform | Setup command | Runtime |
| --- | --- | --- |
| macOS with Homebrew | `bash scripts/setup-unix.sh --with-docker-desktop` | Docker Desktop |
| macOS with MacPorts | `bash scripts/setup-unix.sh --with-colima` | Colima |
| Ubuntu, Debian, Fedora | `bash scripts/setup-unix.sh --with-docker` | Docker Engine |
| Windows PowerShell | `.\scripts\setup-windows.ps1 -WithDocker` | Docker Desktop |

These options also prepare the native Python environment. If you selected a
custom `PYTHON` in the software setup, use the same setting here. For other Linux
distributions, follow [Docker's installation guide](https://docs.docker.com/engine/install/).

Automatic installation uses `brew` on the Homebrew route, `port` plus `sudo`
on the MacPorts route, `sudo` plus `apt-get`/`dnf` on Linux, and `winget` on
Windows. Install the relevant package manager first. Linux automation supports
Debian, Ubuntu, and Fedora (`dnf config-manager addrepo` syntax); check
[Docker's supported releases](https://docs.docker.com/engine/install/) and
resolve conflicting older packages before running it. Windows Docker Desktop
also needs its [Windows/WSL 2 prerequisites](https://docs.docker.com/desktop/setup/install/windows-install/),
and installation may require administrator approval or a restart.

If Docker is already installed but stopped, start its existing runtime first
and use native-only setup; the scripts do not repair partial Docker setups.
The Unix installer skips container installation only if the CLI, Compose, and
daemon are already usable. The Windows installer checks only the CLI and
Compose before skipping installation; start the daemon separately.

Launch Docker Desktop and complete its first-run setup, or start Colima with
`colima start` (the installer starts it initially; `colima stop` stops it).
On Linux, log out and back in after installation so membership of the
`docker` group takes effect. Membership grants root-equivalent access to the
Docker daemon. From the activated native environment, verify the installation:

```bash
python scripts/validate_environment.py --docker
```

## Simulations and notebooks

The simulation container uses Python 3.12. Start it with:

```bash
docker compose build
docker compose up
```

Open `http://127.0.0.1:8888`. The host `code/` directory remains editable and is
part of the project folder mounted into the container, alongside `examples/`,
`docs/` and generated outputs. Open `code/day1/00_jupyter_example.ipynb` for the
Day 1 software check. The inline simulation notebooks use this environment too;
keyboard-controlled Qt windows and radio notebooks use the native setup.
Results written under `day1/`, `E6/` or `plans/` stay in the host project folder
after the container is stopped or recreated. Select the container's **Python 3
(ipykernel)** kernel; here `sys.executable` identifies the container's Python
rather than the native `.venv`.

