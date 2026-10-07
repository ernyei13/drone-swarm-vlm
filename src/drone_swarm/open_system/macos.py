"""Launch the live demo with MuJoCo's native macOS trampoline and this Python.

This avoids the packaged mjpython script's dependency on Apple's otool, and keeps
an activated virtual environment even when a global Anaconda launcher is on PATH.
Verified with Homebrew Python 3.12 and an Anaconda Python 3.11 virtual environment.
"""

import os
import sys
from pathlib import Path

import mujoco


def main():
    if sys.platform != "darwin":
        raise SystemExit("Use python -m drone_swarm.open_system --viewer on Linux / Windows")
    binary = Path(mujoco.__file__).parent / "MuJoCo_(mjpython).app/Contents/MacOS/mjpython"
    if not binary.is_file():
        raise SystemExit("MuJoCo's native macOS launcher is missing; reinstall mujoco")
    environment = os.environ.copy()
    environment["MJPYTHON_BIN"] = str(binary)
    environment["MJPYTHON_LIBPYTHON"] = os.path.realpath(sys.executable)
    if sys.argv[1:] == ["--check"]:
        arguments = [
            "-c",
            "import sys, mujoco, drone_swarm; print(sys.executable); print(mujoco.__version__)",
        ]
    else:
        arguments = ["-m", "drone_swarm.open_system", "--viewer", *sys.argv[1:]]
    os.execve(str(binary), [sys.executable, *arguments], environment)


if __name__ == "__main__":
    main()
