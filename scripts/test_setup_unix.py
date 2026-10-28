"""Exercise installer orchestration without installing packages or using sudo."""

import json
import os
from pathlib import Path
import shlex
import shutil
import subprocess
import sys
import tempfile
import unittest


SETUP = Path(__file__).with_name("setup-unix.sh")

# Each external command is a fixture executable. Only the fake venv directory
# and call log are written; no package manager or real installer is launched.
COMMAND_STUB = r'''
import json
import os
from pathlib import Path
import shutil
import sys

name = Path(sys.argv[0]).name
arguments = sys.argv[1:]
with open(os.environ["SETUP_CALLS"], "a") as stream:
    stream.write(json.dumps([name, *arguments]) + "\n")
if name in ("python3.12", "python-custom", "python"):
    if arguments[:2] == ["-m", "venv"]:
        target = Path(arguments[2]) / "bin" / "python"
        target.parent.mkdir(parents=True)
        shutil.copy2(sys.argv[0], target)
    if arguments and arguments[0].endswith("check_python_version.py"):
        sys.exit(int(os.environ.get("VERSION_CHECK_EXIT", "0")))
elif name == "uname":
    print(os.environ["SETUP_OS"])
elif name == "dpkg":
    print("amd64")
elif name == "docker":
    sys.exit(0 if os.environ.get("DOCKER_READY") == "1" else 1)
'''


@unittest.skipUnless(os.name == "posix" and shutil.which("bash"), "requires a Unix shell")
class UnixSetupTests(unittest.TestCase):
    def run_setup(self, *arguments, operating_system="Darwin", custom_python=False,
                  docker_ready=False, version_check_exit=0, distribution="ubuntu"):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            scripts = root / "scripts"
            scripts.mkdir()
            release = root / "os-release"
            release.write_text(f"ID={distribution}\nVERSION_CODENAME=test-release\n")
            # Redirect only the fixture's OS lookup, never the host's /etc.
            script = SETUP.read_text().replace("/etc/os-release", shlex.quote(str(release)))
            (scripts / SETUP.name).write_text(script)
            commands = root / "commands"
            commands.mkdir()
            for name in ("python3.12", "python-custom", "uname", "brew", "port",
                         "sudo", "docker", "colima", "dpkg"):
                executable = commands / name
                executable.write_text(f"#!{sys.executable}\n" + COMMAND_STUB)
                executable.chmod(0o755)
            calls_path = root / "calls.jsonl"
            env = dict(os.environ)
            env.pop("PYTHON", None)
            env.update(
                PATH=str(commands) + os.pathsep + env.get("PATH", ""),
                SETUP_CALLS=str(calls_path),
                SETUP_OS=operating_system,
                DOCKER_READY="1" if docker_ready else "0",
                VERSION_CHECK_EXIT=str(version_check_exit),
                USER="test-team",
            )
            if custom_python:
                env["PYTHON"] = str(commands / "python-custom")
            result = subprocess.run(
                ["bash", str(scripts / SETUP.name), *arguments],
                env=env, capture_output=True, text=True,
            )
            calls = [json.loads(line) for line in calls_path.read_text().splitlines()] if calls_path.exists() else []
            return result, calls

    def test_native_setup_uses_both_requirements_and_validator(self):
        for platform in ("Darwin", "Linux"):
            with self.subTest(platform=platform):
                result, calls = self.run_setup(operating_system=platform)
                self.assertEqual(result.returncode, 0, result.stderr)
                self.assertTrue(calls[0][1].endswith("scripts/check_python_version.py"))
                self.assertEqual(calls[1][:3], ["python3.12", "-m", "venv"])
                install = calls[3]
                self.assertEqual(install[:5], ["python", "-m", "pip", "install", "-r"])
                self.assertTrue(install[5].endswith("requirements-hardware.txt"))
                self.assertEqual(install[6], "-r")
                self.assertTrue(install[7].endswith("requirements-simulation.txt"))
                self.assertTrue(calls[4][1].endswith("scripts/validate_environment.py"))
                self.assertEqual(len(calls), 5)

    def test_custom_interpreter_and_failed_version_check(self):
        result, calls = self.run_setup(custom_python=True)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(calls[0][0], "python-custom")
        result, calls = self.run_setup(version_check_exit=1)
        self.assertNotEqual(result.returncode, 0)
        self.assertEqual(len(calls), 1)

    def test_homebrew_and_macports_container_routes(self):
        result, calls = self.run_setup("--with-docker-desktop")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn(["brew", "install", "--cask", "docker-desktop"], calls)
        result, calls = self.run_setup("--with-colima")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn(["sudo", "port", "install", "docker", "docker-compose-plugin", "colima"], calls)
        self.assertIn(["colima", "start"], calls)

    def test_running_docker_skips_installation(self):
        result, calls = self.run_setup("--with-docker", operating_system="Linux", docker_ready=True)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn(["docker", "compose", "version"], calls)
        self.assertIn(["docker", "info"], calls)
        self.assertFalse(any(call[0] == "sudo" for call in calls))

    def test_linux_container_routes(self):
        for distribution in ("ubuntu", "debian", "fedora"):
            with self.subTest(distribution=distribution):
                result, calls = self.run_setup(
                    "--with-docker", operating_system="Linux", distribution=distribution,
                )
                self.assertEqual(result.returncode, 0, result.stderr)
                if distribution == "fedora":
                    self.assertIn(["sudo", "dnf", "config-manager", "addrepo", "--from-repofile",
                                   "https://download.docker.com/linux/fedora/docker-ce.repo"], calls)
                else:
                    self.assertIn(["sudo", "apt-get", "update"], calls)
                    self.assertTrue(any(
                        call[:3] == ["sudo", "curl", "-fsSL"]
                        and f"https://download.docker.com/linux/{distribution}/gpg" in call
                        for call in calls
                    ))
                self.assertIn(["sudo", "systemctl", "enable", "--now", "docker"], calls)
                self.assertIn(["sudo", "usermod", "-aG", "docker", "test-team"], calls)

    def test_invalid_or_multiple_options_fail_without_installing(self):
        for arguments in (("--unknown",), ("--with-colima", "--with-docker-desktop")):
            result, calls = self.run_setup(*arguments)
            self.assertEqual(result.returncode, 2)
            self.assertEqual(calls, [])


if __name__ == "__main__":
    unittest.main()
