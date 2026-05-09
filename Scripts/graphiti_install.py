import os
import subprocess
import sys
import getpass
import pwd

from utils import is_installed, run_command


def check_user_exists(username):
    try:
        pwd.getpwnam(username)
        return True
    except KeyError:
        return False

def check_docker():
    """Checks for Docker and installs it if missing."""
    if is_installed("docker"):
        print("Status: Docker is already installed.")
        # Check if the service is running
        try:
            subprocess.run(["docker", "info"], capture_output=True, check=True)
        except subprocess.CalledProcessError:
            print("Warning: Docker is installed but the daemon is not running or permissions are missing.")
            print("Try: sudo systemctl start docker")
    else:
        print("Status: Docker not found. Starting installation...")
        run_command(["sudo", "apt-get", "update"])
        run_command(["sudo", "apt-get", "install", "-y", "ca-certificates", "curl", "gnupg"])
        
        run_command("sudo install -m 0755 -d /etc/apt/keyrings", shell=True)
        run_command("curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg", shell=True)
        run_command("sudo chmod a+r /etc/apt/keyrings/docker.gpg", shell=True)
        
        repo_cmd = (
            'echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] '
            'https://download.docker.com/linux/ubuntu $(. /etc/os-release && echo \\"$VERSION_CODENAME\\") stable" | '
            'sudo tee /etc/apt/sources.list.d/docker.list > /dev/null'
        )
        run_command(repo_cmd, shell=True)
        run_command(["sudo", "apt-get", "update"])
        run_command(["sudo", "apt-get", "install", "-y", "docker-ce", "docker-ce-cli", "containerd.io", "docker-buildx-plugin", "docker-compose-plugin"])
        
        user = getpass.getuser()
        run_command(["sudo", "usermod", "-aG", "docker", user])
        print(f"Success: Docker installed. User {user} added to docker group.")

def check_claude_code():
    """Checks for Claude Code CLI and installs if missing."""
    if is_installed("claude"):
        print("Status: Claude Code CLI is already installed.")
    else:
        print("Status: Claude Code CLI not found. Starting installation...")
        run_command("curl -fsSL https://claude.ai/install.sh | bash", shell=True)
        
        if not is_installed("npm"):
            print("Installing Node.js/npm for MCP support...")
            run_command(["sudo", "apt-get", "install", "-y", "nodejs", "npm"])

def check_graphiti():
    """Checks for Graphiti repository and sets up the server."""
    target_dir = os.path.expanduser("~/graphiti-mcp")
    mcp_dir = os.path.join(target_dir, "mcp_server")
    
    if os.path.exists(mcp_dir):
        print(f"Status: Graphiti MCP server found at {mcp_dir}.")
    else:
        print("Status: Graphiti not found. Cloning and configuring...")
        run_command(["git", "clone", "https://github.com/getzep/graphiti.git", target_dir])
        
        if not is_installed("uv"):
            run_command("curl -LsSf https://astral.sh/uv/install.sh | sh", shell=True)

        env_path = os.path.join(mcp_dir, ".env")
        if not os.path.exists(env_path):
            api_key = input("Enter Anthropic API Key: ").strip()
            with open(env_path, "w") as f:
                f.write(f"ANTHROPIC_API_KEY={api_key}\n")
                f.write("NEO4J_URI=bolt://localhost:7687\n")
                f.write("NEO4J_USER=neo4j\n")
                f.write("NEO4J_PASSWORD=demodemo\n")

    if not check_user_exists("docker"):
        run_command(["sudo", "groupadd", "docker"])
        run_command(["sudo", "usermod", "-aG", "docker", "$USER"])

    # Ensure the database is running
    print("Ensuring Graphiti database services are active...")
    run_command(["docker", "compose", "up", "-d"], cwd=mcp_dir)
    
    # Refresh dependencies
    run_command(["uv", "sync"], cwd=mcp_dir)

def main():
    if os.geteuid() == 0:
        print("Error: Do not run this script as root. It will request sudo permissions as needed.")
        sys.exit(1)

    print("--- Environment Verification ---")
    check_docker()
    check_claude_code()
    check_graphiti()

    print("\n--- Final Instructions ---")
    print("1. If this is a fresh Docker install, log out and back in to refresh group permissions.")
    print("2. Run 'claude auth login' if you have not authenticated the CLI.")
    print("3. Register the server in Claude Code:")
    print("   claude mcp add graphiti npx -y mcp-remote http://localhost:8000/sse")

if __name__ == "__main__":
    main()