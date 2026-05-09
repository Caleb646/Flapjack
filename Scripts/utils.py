import shutil
import subprocess
import sys


def is_installed(binary):
    """Checks if a binary exists in the system PATH."""
    return shutil.which(binary) is not None


def run_command(command, shell=False, cwd=None):
    """Executes a command and exits on failure."""
    try:
        subprocess.run(command, shell=shell, cwd=cwd, check=True)
    except subprocess.CalledProcessError as e:
        print(f"Error executing command: {e}")
        sys.exit(1)
