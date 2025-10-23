
## Quick start Windows

These instruction is tested on Windows 11, docker desktop for windows-ARM64 and VScode.

To get the docker container up and running, download and install Docker Desktop for your system: https://www.docker.com/products/docker-desktop/
  
When this is done, clone the repo and open the folder in VScode. Then you should automatically be prompted to download and install VScode extensions which are needed and recommended for the project.
  
Once that is completed, make sure Docker Desktop ir running and then you should be able to start the container environment in VScode by pressing Ctrl+Shift+p and searching for Dev Containers: Rebuild and Reopen in Container.
  
To make Gazebo's and Rviz's GUI be visible from the docker container to your screen, download and install the following program: https://sourceforge.net/projects/vcxsrv/

When this is done, start the XLaunch program and configure it with these settings:

#### Select display settings
◉ Multiple windows
◯ Fullscreen
◯ One large window
◯ One window without titlebar

Display number: -1

---

#### Select how to start clients
◉ Start no client
◯ Start a program
◯ Open session via XDMCP

---

#### Extra settings
☑ Clipboard (optional)
☑ Primary Selection (optional)
☐ Native opengl
☑ Disable access control

---

Before continuing, make sure Docker Desktop is running.

