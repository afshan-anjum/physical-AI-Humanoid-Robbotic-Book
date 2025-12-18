---
sidebar_position: 1
---

# Prerequisites and Setup Instructions

To get the most out of the "AI & Humanoid Robotics Textbook", it's important to have your development environment set up correctly. This guide will walk you through the necessary software installations and configurations.

## 1. Operating System

We highly recommend using **Ubuntu 22.04 LTS (Jammy Jellyfish)**. Many of the tools and examples in this textbook (especially ROS 2, Gazebo, and NVIDIA Isaac Sim) are optimized for and best supported on this Linux distribution.

*   **Virtual Machine**: If you are not running Ubuntu natively, we recommend setting up a virtual machine (e.g., using VirtualBox or VMware) with Ubuntu 22.04 LTS.
*   **WSL 2 (Windows Subsystem for Linux)**: For Windows users, WSL 2 with an Ubuntu 22.04 distribution can also be a viable option, though some graphical applications may require additional setup.

## 2. Software Installation

### Node.js and Yarn/npm

Docusaurus requires Node.js. Many web-based tools will also use it.

1.  **Install Node.js (version 20.x or higher)**:
    ```bash
    curl -fsSL https://deb.nodesource.com/setup_20.x | sudo -E bash -
    sudo apt-get install -y nodejs
    ```
2.  **Install Yarn (recommended package manager for this project)**:
    ```bash
    npm install --global yarn
    ```
    Alternatively, you can use `npm` if you prefer.

### Python

Python is the primary programming language for AI and Robotics in this textbook.

1.  **Install Python 3.10 or higher**:
    Ubuntu 22.04 typically comes with Python 3.10 pre-installed. Verify with:
    ```bash
    python3 --version
    ```
2.  **Install `pip` and `venv`**:
    ```bash
    sudo apt install python3-pip python3-venv
    ```

### Git

For version control and cloning repositories.

```bash
sudo apt install git
```

### Docker (Recommended)

Many development environments, especially for ROS 2 and NVIDIA Isaac Sim, can be managed more easily using Docker containers.

```bash
sudo apt-get update
sudo apt-get install ca-certificates curl gnupg
sudo install -m 0755 -d /etc/apt/keyrings
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg
sudo chmod a+r /etc/apt/keyrings/docker.gpg
echo \
  "deb [arch=\"$(dpkg --print-architecture)\" signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu \
  "$(. /etc/os-release && echo "$VERSION_CODENAME")" stable" | \
  sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
sudo apt-get update
sudo apt-get install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
sudo usermod -aG docker $USER # Add your user to the docker group
newgrp docker # Activate the changes (may require logout/login)
```

## 3. Integrated Development Environment (IDE)

**Visual Studio Code (VS Code)** is highly recommended due to its extensive ecosystem of extensions for Python, JavaScript, Markdown, ROS, and Docker.

---

**Note**: More detailed setup instructions for specific tools like ROS 2, Gazebo, and NVIDIA Isaac Sim will be provided in their respective modules.
