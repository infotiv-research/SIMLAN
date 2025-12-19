# Dependencies

## Linux Dependencies

## NVIDIA Driver

Use 'nvidia-smi' to ensure that the right NVIDIA driver is installed.
If you have not installed **Additional Drivers** when installing Ubuntu, you need to manually install NVIDIA drivers.

## Docker

```
sudo apt install curl git
curl -fsSL https://get.docker.com -o get-docker.sh
sudo sh get-docker.sh
sudo groupadd docker
sudo usermod -aG docker $USER
newgrp docker
```

## nvidia-container-toolkit

To install Docker and `nvidia-container-toolkit`, use the following commands:

```
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg \
  && curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
    sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
    sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list

sudo sed -i -e '/experimental/ s/^#//g' /etc/apt/sources.list.d/nvidia-container-toolkit.list

sudo apt-get update

sudo apt-get install -y nvidia-container-toolkit

sudo nvidia-ctk runtime configure --runtime=docker

INFO[0000] Config file does not exist; using empty config
INFO[0000] Wrote updated config to /etc/docker/daemon.json
INFO[0000] It is recommended that docker daemon be restarted.

sudo systemctl restart docker

sudo nvidia-ctk runtime configure --runtime=containerd

INFO[0000] Using config version 1
INFO[0000] Using CRI runtime plugin name "cri"
WARN[0000] could not infer options from runtimes [runc crun]; using defaults
INFO[0000] Wrote updated config to /etc/containerd/config.toml
INFO[0000] It is recommended that containerd daemon be restarted.
```

Restart the computer to apply the group and user changes.

To check for correct installation of Docker's NVIDIA runtime:

```
docker info|grep -i runtime
 Runtimes: nvidia runc
 Default Runtime: runc
```

Otherwise you get the following error message in VS Code: `Error response from daemon: unknown or invalid runtime name: nvidia`

On a **host** machine's terminal (**not** inside Visual Studio Code terminal): `xhost +local:docker`.

## Windows Dependencies

These instructions are tested on Windows 11, Docker Desktop for Windows ARM64 and VS Code.

To get the Docker container up and running, download and install Docker Desktop for your system: https://www.docker.com/products/docker-desktop/

When this is done, clone the repo and open the folder in VS Code. Then you should automatically be prompted to download and install VS Code extensions that are needed and recommended for the project.

Once that is completed, make sure Docker Desktop is running and then you should be able to start the container environment in VS Code by pressing Ctrl+Shift+P and searching for Dev Containers: Rebuild and Reopen in Container.

To make Gazebo's and RViz's GUIs visible from the Docker container on your screen, download and install the following program: https://sourceforge.net/projects/vcxsrv/

When this is done, start the XLaunch program and configure it with these settings:

#### Select display settings

```
◉ Multiple windows
◯ Fullscreen
◯ One large window
◯ One window without titlebar

Display number: -1
```

______________________________________________________________________

#### Select how to start clients

```
◉ Start no client
◯ Start a program
◯ Open session via XDMCP
```

______________________________________________________________________

#### Extra settings

```
☑ Clipboard (optional)
☑ Primary Selection (optional)
☐ Native OpenGL
☑ Disable access control
```

______________________________________________________________________

Before continuing, make sure Docker Desktop is running.
