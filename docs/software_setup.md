# Software setup and installation

[Repository overview](../README.md) | [Documentation guide](README.md)

Run commands from the project folder unless shown otherwise. This is the folder
containing `README.md`, `scripts/`, `code/`, `requirements-hardware.txt` and
`requirements-simulation.txt`.

Before the lectures, install the native environment and complete the
[software-only checks](#native-simulations-and-notebooks). You can do this
without a drone or Crazyradio. The [USB](#usb-setup) and
[radio checks](#team-radio-configuration) follow when your group has its hardware.

Use the native Python environment for cfclient and Crazyradio access.
**Neither Docker nor Colima is needed to connect to or fly a drone.**

| Activity | Docker needed? |
| --- | --- |
| cfclient, Crazyradio, flight scripts, Matplotlib plots | No |
| Simulations and Jupyter notebooks in the native Python environment | No |
| Optional simulation/notebook container | Yes |

## Native setup

There are two separate steps:

1. Install any missing prerequisite tools for [macOS](#macos), [Linux](#linux)
   or [Windows](#windows). If they are already installed, skip their installation.
2. [Set up this project](#create-the-project-environment) in the cloned folder.
   This step is still needed when Git, Python and the other tools are installed.

If you already cloned the course, open a terminal in that project folder.
You only need to clone once. Neither `git clone` nor `git pull` creates `.venv`;
the project setup script creates it locally and installs the lecture packages.

Install Git before cloning: use `xcode-select --install` on macOS, your
distribution package manager on Linux, or [Git for Windows](https://git-scm.com/downloads/win).
Open a new Terminal or PowerShell window after installation, then run:

```text
git --version
git clone --branch course --single-branch https://gitlab.nikhef.nl/graven/crazyflie-topical-lectures.git
cd crazyflie-topical-lectures
```

The `course` branch contains the notes, exercises and code. While the project
is private, you need a Nikhef GitLab account with repository access. Once public,
it can be cloned without logging in. The [cloning guide](course_git.md#clone-the-course)
also explains SSH access. Keep this terminal in the cloned
`crazyflie-topical-lectures` folder for the commands below. To check your location, use
`pwd` and `ls` on macOS/Linux, or `Get-Location` and `Get-ChildItem` in PowerShell.
You should see `README.md`, `scripts/`, `code/` and the two requirements files.
If you open a new terminal, use `cd` with the path where you cloned the project;
for example `cd "$HOME/Downloads/crazyflie-topical-lectures"` on macOS/Linux or
`cd "$HOME\Downloads\crazyflie-topical-lectures"` in PowerShell if it is inside Downloads.

The [Git workflow](course_git.md) explains team branches, merging course updates
and sharing improvements. If you have an offline ZIP snapshot, extract it into
a writable folder and use the folder containing those same files for setup.


The **project root** (or **package root** for a ZIP download) means this folder,
not `code/` or `scripts/`. Commands such as `bash scripts/setup-unix.sh` look for
files relative to the terminal's current folder. Changing folders in a file
browser does not change an already-open terminal's current folder.

Check your location with `pwd` and list files with `ls` on macOS/Linux, or use
`Get-Location` and `Get-ChildItem` in PowerShell. You should see the files and
folders listed above. Commands that need a different folder explicitly include
a step such as `cd code`.

**Python 3.12.x is the tested default.** Newer versions are allowed with a
warning but have not been validated with the pinned packages, Qt GUI, and USB
access; older versions are rejected. The setup scripts create or reuse `.venv`,
install both `requirements-hardware.txt` and `requirements-simulation.txt`, and
run the environment validator. They do not install Git, Python, native GUI/USB
libraries, USB permissions/drivers, or team radio settings. Docker installation
is optional and requires an explicit flag.

Initial installation needs internet access and space for the Python environment
and downloaded packages. Installing a package manager, Linux libraries or Windows
USB drivers may require administrator access; arrange that in advance on managed
laptops. The installed simulations and supplied notebook then work offline.
Keep the extracted project in a writable folder and open a terminal in that
folder, rather than running files from inside the ZIP viewer.

### macOS

If Git/developer command-line tools are missing, install them first:

```bash
xcode-select --install
```

Choose either [Homebrew](https://brew.sh/) or
[MacPorts](https://www.macports.org/install.php); you do not need both.
Install the package manager first and follow its instructions to add it to
`PATH`. Both routes below include `libusb` for Crazyradio access.

**Homebrew:**

```bash
brew install python@3.12 libusb
"$(brew --prefix)/bin/python3.12" --version
```

**MacPorts:**

```bash
sudo port install python312 libusb
/opt/local/bin/python3.12 --version
```

See also the [Python macOS guide](https://docs.python.org/3.12/using/mac.html).

### Linux

Install Python 3.12 and its `venv` support using your distribution's packages;
see the [Python Unix guide](https://docs.python.org/3.12/using/unix.html).
Also install the system USB library and Qt/X11 prerequisites for cfclient.
For Ubuntu releases providing Python 3.12 (for example Ubuntu 24.04):

```bash
sudo apt-get update
sudo apt-get install git python3.12 python3.12-venv libusb-1.0-0 libxcb-xinerama0 libxcb-cursor0
```

Other distributions use different package names; follow the
[cfclient Linux prerequisites](https://www.bitcraze.io/documentation/repository/crazyflie-clients-python/master/installation/install/).
If your distribution supplies a newer Python, use the explicit `PYTHON`
override below rather than replacing the system Python. The default command
looks specifically for `python3.12`, not `python3`.

Once these prerequisites are available, continue with project setup below.

### Windows

Install the [Python Install Manager](https://docs.python.org/3/using/windows.html),
then run in PowerShell:

```powershell
pymanager install 3.12
py -3.12 --version
```

The project setup script defaults to `py -3.12`; installing Python alone does not install the
Crazyradio driver. Complete the Windows USB steps below separately.
`pymanager` explicitly selects the install manager, even when an older `py`
launcher is also installed. Open a new PowerShell window if the command is not
found immediately after installing the manager.

### Create the project environment

Run this step from the project root even if all prerequisite tools were already
installed. Select the command for your platform; it creates or reuses `.venv`,
installs the hardware and simulation requirements, and runs the validator.
An existing Conda environment, such as `(base)`, is separate from this project's
`.venv` and does not replace this step.

**macOS with Homebrew:**

```bash
PYTHON="$(brew --prefix)/bin/python3.12" bash scripts/setup-unix.sh
```

**macOS with MacPorts:**

```bash
PYTHON=/opt/local/bin/python3.12 bash scripts/setup-unix.sh
```

**Linux, or macOS with `python3.12` already on PATH:**

```bash
python3.12 --version
bash scripts/setup-unix.sh
```

**Windows PowerShell:**

```powershell
Set-ExecutionPolicy -Scope Process Bypass
.\scripts\setup-windows.ps1
```

Wait for setup to finish successfully before activation. If it fails, keep the
error output and resolve that error first. Updating the checkout with `git pull`
preserves an existing environment; rerun setup if the requirements change.

### Select another Python

Leave `PYTHON` unset for the default. To select another executable on macOS/Linux:

```bash
PYTHON=/path/to/python3.14 bash scripts/setup-unix.sh
```

On Windows, set `$env:PYTHON` to the full path of the desired `python.exe`
before running setup; this variable accepts an executable path:

```powershell
$env:PYTHON = 'C:\path\to\python.exe'
.\scripts\setup-windows.ps1
```

Use a fresh course clone or extracted snapshot when changing interpreter versions
so an existing `.venv` is not reused with another Python.

### Activate and check

After successful project setup, activate the environment in each new terminal:

```bash
# macOS/Linux
source .venv/bin/activate
```

```powershell
# Windows PowerShell
Set-ExecutionPolicy -Scope Process Bypass
.\.venv\Scripts\Activate.ps1
```

If activation reports `no such file or directory` or cannot find the script,
check that the terminal is in the project root. If `.venv` is missing there,
return to [Create the project environment](#create-the-project-environment).
Activation selects an existing environment; it does not create one.

The PowerShell policy change lasts only for that terminal; managed policies may
prevent it. Activation affects only the current terminal on all platforms.

With the environment active, run these software-only checks:

```bash
python -c "import sys; print(sys.executable)"
python scripts/validate_environment.py
cfclient
```

These commands check three different things:

1. The first prints the Python interpreter path. It should be inside this
   project's `.venv` folder.
2. `python scripts/validate_environment.py` checks package imports and versions,
   and ends with `Environment ready.` if the checks pass. Its informational
   message `CF_URI is unset` is normal before radio assignment, which is done
   below under [Team radio configuration](#team-radio-configuration).
3. Running `cfclient` opens its window and checks that the graphical interface
   starts correctly. Leave it disconnected from the drone, then close the window
   before continuing. This step checks GUI startup separately from the package
   imports checked by the validator.

If local policy prevents PowerShell activation, the environment's executables
can be run directly: `.\.venv\Scripts\python.exe` and
`.\.venv\Scripts\cfclient.exe`. If policy also prevents running the setup script,
arrange a permitted installation before arrival or share a working group laptop.

## Native simulations and notebooks

The native setup scripts install **the core simulation and notebook dependencies**
alongside cfclient. Neither Docker nor Colima nor a connected drone is needed
for these offline exercises.

From the project root, with `.venv` active, run:

```bash
cd code
python -m day1 --output ../day1/setup-check --no-show
python -m unittest andreas.test_practical
cd ..
python -m ipykernel install --sys-prefix --name crazyflie-lectures --display-name "Crazyflie lectures (.venv)"
jupyter lab
```

**Expected:** the simulation writes CSV files, `comparison.png` and
`settings.json` under `day1/setup-check`; the tests finish with `OK`. Open the
saved PNG to check that the plots render. Use a new output name, such as
`day1/setup-check-2`, when repeating the simulation; existing results are preserved.

Open Jupyter's printed URL and, in its file browser, open
**`code/andreas/day1_control.ipynb`**. Start Jupyter from the project root as
shown above so the notebooks can also open links into `docs/`.
Select the **Crazyflie lectures (.venv)** kernel. Check it by running this in
a notebook cell:

```python
import sys
print(sys.executable)
```

The path should point into this project's `.venv`. If it differs from the
terminal check, select **Crazyflie lectures (.venv)** or close Jupyter and restart
it from the activated terminal. The kernel-registration command above stores
this kernel inside the project's environment. Give `run_id` in the notebook's first code cell
a fresh value before each run, then run the supplied cells. They use simulations
and the included synthetic replay data. Results appear under `day1/notebook/`.

Days 2 and 3 use the same environment and kernel. When you reach those sessions,
open [day2/day2_kalman.ipynb](../code/day2/day2_kalman.ipynb) or
[day3/day3_race.ipynb](../code/day3/day3_race.ipynb) in Jupyter for the scheduled
filter experiments, trajectory planning and recording analysis. Their default paths
work offline; live recordings and measured courses can be selected during the
practicals. There is no extra software installation or pre-lecture assignment.

**Pre-lecture setup is complete** when the validator passes, cfclient opens,
the simulation/test checks pass and the notebook produces its plots using the
intended interpreter. Keep any error output if a check fails and compare it
within your group. Hardware checks can wait until your drone and radio are available.

These commands work in Bash and PowerShell after activation. See the
[Day 1 software guide](day_1_software.md) for the experiments. The additional
Kalman simulations can be checked from `code/` when useful:

```bash
python -m day2.simulate
python -m day1.feedforward
python -m unittest kalman.test_filter
```

Docker remains an [optional simulation alternative](#simulations-and-notebooks).

## USB setup

These steps are for the Crazyradio once hardware is available. Keep the drone's
motors stopped during connection checks. Each group needs one radio and its
assigned drone; software-only pre-lecture checks use neither.

### Windows

1. Download [Zadig](https://zadig.akeo.ie/) and connect the Crazyradio.
2. Run Zadig as administrator. If the radio is not listed, select
   **Options -> List All Devices**.
3. Select **Crazyradio** or **Crazyradio 2.0** in the device drop-down menu at the
   top of the window. Check the device name carefully before changing its driver.
4. Use the driver selector to the right of the green arrow to select
   **libusb-win32**, then click **Install Driver** or **Replace Driver**.
   If the radio is unreliable with this driver, Bitcraze recommends trying
   **libusbK** instead.
5. Unplug and reconnect the radio, then run the radio check again.

See Bitcraze's
[Windows USB-driver instructions](https://www.bitcraze.io/documentation/repository/crazyradio-firmware/master/building/usbwindows/).

### Linux

Add your account to the `plugdev` group. Log out and back in after these commands:

```bash
sudo groupadd -f plugdev
sudo usermod -a -G plugdev "$USER"
```

Create the rule file with the following complete command. It writes this one
file with administrator permissions:

```bash
sudo tee /etc/udev/rules.d/99-bitcraze.rules >/dev/null <<'EOF'
# Crazyradio (normal operation)
SUBSYSTEM=="usb", ATTRS{idVendor}=="1915", ATTRS{idProduct}=="7777", MODE="0664", GROUP="plugdev"
# Crazyradio bootloader
SUBSYSTEM=="usb", ATTRS{idVendor}=="1915", ATTRS{idProduct}=="0101", MODE="0664", GROUP="plugdev"
# Crazyflie connected directly over USB
SUBSYSTEM=="usb", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", MODE="0664", GROUP="plugdev"
EOF
```

Reload the rules, then unplug and reconnect the radio:

```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```

See Bitcraze's
[Linux USB-permission instructions](https://www.bitcraze.io/documentation/repository/crazyflie-lib-python/master/installation/usb_permissions/).

### macOS

The native setup commands above already install `libusb`.

Connect the Crazyradio directly to the Mac where possible. On Apple-silicon
Mac laptops, unlock the Mac and click **Allow** if an accessory prompt appears.
If you denied it, unplug and reconnect the radio while the Mac is unlocked
to request approval again. To change the policy, open **System Settings ->
Privacy & Security -> Accessories -> Allow accessories to connect**; some
macOS versions show the control directly under Privacy & Security.
This setting does not apply to every Mac: its absence on Intel Macs or desktop
Macs is not evidence of a USB-permission problem. Managed laptops may have an
administrator-controlled policy. See [Apple's accessory-access guide](https://support.apple.com/102282).
With a non-standard Homebrew prefix,
follow Bitcraze's
[macOS library-path note](https://www.bitcraze.io/documentation/repository/crazyflie-lib-python/master/installation/install/).

## Team radio configuration

Organizers provide each team with a unique drone [radio URI](glossary.md#radio-uri)
and identity. Assignment records are not part of the public package.

Each team copies the example and enters the assigned URI:

```bash
cp tl-config.env.example tl-config.env
```

On macOS/Linux, load it before an exercise:

```bash
set -a
source tl-config.env
set +a
```

The channel, data rate, and address in `CF_URI` must match the persistent radio
configuration programmed into that team's Crazyflie.

In Windows PowerShell, set the same values from your edited `tl-config.env`
explicitly (the Bash `source` command does not work in PowerShell):

```powershell
$env:CF_URI = 'radio://0/80/2M/E7E7E7E701'
$env:TEAM_NAME = 'team-01'
```

Replace these examples with your assigned values. Environment variables must be
set again in each new terminal; copying the file alone does not load them.

After [USB setup](#usb-setup), plug in the radio and run:

```bash
python scripts/validate_environment.py --radio
cfclient
```

**Expected:** the validator lists the radio's serial number. In cfclient, connect
to the assigned drone following [B2's connection steps](day_1_baseline_checks.md#b2-cfclient-connection-firmware-and-controller-identity),
with motors stopped. Confirm the drone identity and live telemetry. Close
cfclient before any other script uses that radio. A radio-discovery check detects
the USB dongle; the cfclient connection checks the link to the drone.

## Troubleshooting

| Symptom | First checks |
| --- | --- |
| Python missing, wrong version or wrong interpreter | Use the OS-specific Python command above. After activation, `python -c "import sys; print(sys.executable)"` should show the project's `.venv`. |
| `pymanager` missing or `py install` tries to open a file named `install` | Install the linked Python Install Manager, reopen PowerShell and use `pymanager install 3.12`; an older `py` launcher may still be present. |
| pip fails or reports no matching distribution | Keep the first error, check internet access and Python version/architecture, and retry with the tested Python 3.12 in a fresh project copy. Preserve the pinned requirements. |
| cfclient fails with a Qt/xcb plugin error | On Linux, install the Qt libraries listed above and use a graphical desktop session. Retain the full terminal error; an import check alone cannot verify the GUI. |
| macOS reports `NoBackendError` or cannot find libusb | Check that Python and libusb come from the chosen package manager. For MacPorts, try `export DYLD_LIBRARY_PATH="/opt/local/lib${DYLD_LIBRARY_PATH:+:$DYLD_LIBRARY_PATH}"` in the same terminal, then retry. For Homebrew, use the linked library-path note. |
| Radio absent or permission denied | Check the OS-specific driver/udev steps, reconnect the radio, try a direct USB port and close other radio clients. On Linux, confirm the new group membership after logging in again. |
| Simulation/notebook output already exists | Choose a new CLI output directory or change the notebook's `run_id`; keep previous results for comparison. |

If a problem remains, keep the command, working directory, OS, Python version
and full error output. The group can continue on a working laptop while retaining
the unresolved check for later investigation.

## Optional containers

Skip this section for cfclient, flight scripts, and native simulations or
notebooks. The containerized services need a running Docker engine, not just
the Docker command-line client. On macOS, choose **Docker Desktop or Colima**,
not both. Colima runs a Linux VM for Docker; it is not needed on Linux or Windows.

The setup scripts can also install container tools. Choose one option:

| Platform | Setup command | Runtime |
| --- | --- | --- |
| macOS with Homebrew | `bash scripts/setup-unix.sh --with-docker-desktop` | Docker Desktop |
| macOS with MacPorts | `bash scripts/setup-unix.sh --with-colima` | Colima |
| Ubuntu, Debian, Fedora | `bash scripts/setup-unix.sh --with-docker` | Docker Engine |
| Windows PowerShell | `.\scripts\setup-windows.ps1 -WithDocker` | Docker Desktop |

These options also prepare the native Python environment. If you selected a
custom `PYTHON` above, use the same setting here. For other Linux distributions,
follow [Docker's installation guide](https://docs.docker.com/engine/install/).

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
Docker daemon. Verify the installation with:

```bash
python scripts/validate_environment.py --docker
```

### Simulations and notebooks

The optional simulation container uses Python 3.12 and does not provide cfclient
or direct Crazyradio access:

```bash
docker compose build
docker compose up
```

Open `http://127.0.0.1:8888`. The host `code/` directory remains editable and is
part of the project folder mounted into the container, alongside `examples/`,
`docs/` and generated outputs. Open `code/andreas/day1_control.ipynb` for Day 1.
Results written under `day1/`, `E6/` or `plans/` stay in the host project folder
after the container is stopped or recreated. Select the container's **Python 3
(ipykernel)** kernel; here `sys.executable` identifies the container's Python
rather than the native `.venv`.

