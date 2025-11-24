# Dependencies

## Linux Dependencies

## nvidia Driver

Use 'nvidia-smi' to ensure that the right nvidia driver is installed.
If you have not installed **Additional Drivers** when installing Ubuntu, you need to manually install nvidia drivers.

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

To install docker and `nvidia-container-toolkit` use the following commands:

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

To check for correct installation docker's nvidia runtime:

```
docker info|grep -i runtime
 Runtimes: nvidia runc
 Default Runtime: runc
```

Otherwise you get the following error message in vscode: `Error response from daemon: unknown or invalid runtime name: nvidia`

On a **host** machine's terminal (**not** inside Visual Studio Code terminal): `xhost +local:docker`.

## Windows Dependencies

These instruction is tested on Windows 11, docker desktop for windows-ARM64 and VScode.

To get the docker container up and running, download and install Docker Desktop for your system: https://www.docker.com/products/docker-desktop/

When this is done, clone the repo and open the folder in VScode. Then you should automatically be prompted to download and install VScode extensions which are needed and recommended for the project.

Once that is completed, make sure Docker Desktop ir running and then you should be able to start the container environment in VScode by pressing Ctrl+Shift+p and searching for Dev Containers: Rebuild and Reopen in Container.

To make Gazebo's and Rviz's GUI be visible from the docker container to your screen, download and install the following program: https://sourceforge.net/projects/vcxsrv/

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
☐ Native opengl
☑ Disable access control
```

______________________________________________________________________

Before continuing, make sure Docker Desktop is running.
