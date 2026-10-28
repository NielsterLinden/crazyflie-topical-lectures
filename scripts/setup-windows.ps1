param(
    [switch]$WithDocker
)

$ErrorActionPreference = "Stop"
$Root = Split-Path -Parent $PSScriptRoot
$Python = if ($env:PYTHON) { $env:PYTHON } else { "py" }

function Test-DockerCli {
    if (-not (Get-Command docker -ErrorAction SilentlyContinue)) {
        return $false
    }

    & docker compose version *> $null
    return $LASTEXITCODE -eq 0
}

function Install-DockerDesktop {
    if (Test-DockerCli) {
        Write-Host "Docker and Docker Compose are already installed."
        return
    }

    if (-not (Get-Command winget -ErrorAction SilentlyContinue)) {
        throw "winget is required for automatic Docker Desktop installation. See https://docs.docker.com/desktop/setup/install/windows-install/"
    }

    & winget install `
        --id Docker.DockerDesktop `
        --exact `
        --accept-package-agreements `
        --accept-source-agreements
    if ($LASTEXITCODE -ne 0) {
        throw "Docker Desktop installation failed with exit code $LASTEXITCODE"
    }

    Write-Host "Docker Desktop installed. Start it and complete its first-run setup."
    Write-Host "The recommended backend requires WSL 2."
}

if ($WithDocker) {
    Install-DockerDesktop
}

$PythonArguments = @()
if ($Python -eq "py") {
    $PythonArguments = @("-3.12")
}

& $Python @PythonArguments "$Root\scripts\check_python_version.py"
if ($LASTEXITCODE -ne 0) {
    throw "Python version check failed; install 3.12 or set PYTHON to a Python 3.12+ executable."
}
& $Python @PythonArguments -m venv "$Root\.venv"
if ($LASTEXITCODE -ne 0) {
    throw "Virtual environment creation failed."
}

$VenvPython = "$Root\.venv\Scripts\python.exe"
& $VenvPython -m pip install --upgrade pip
if ($LASTEXITCODE -ne 0) { throw "pip upgrade failed." }
& $VenvPython -m pip install -r "$Root\requirements-hardware.txt" -r "$Root\requirements-simulation.txt"
if ($LASTEXITCODE -ne 0) { throw "Package installation failed." }
& $VenvPython "$Root\scripts\validate_environment.py"
if ($LASTEXITCODE -ne 0) { throw "Environment validation failed." }

Write-Host ""
Write-Host "Setup complete. Activate with: .\.venv\Scripts\Activate.ps1"
Write-Host "Start cfclient with: cfclient"
Write-Host "Run the radio check with: python scripts\validate_environment.py --radio"
if ($WithDocker) {
    Write-Host "After Docker Desktop is running, check it with: python scripts\validate_environment.py --docker"
}
