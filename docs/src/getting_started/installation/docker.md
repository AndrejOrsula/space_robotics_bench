# Installation (Docker)

This section provides instructions for running the simulation within a Docker container. Before proceeding, ensure that your system meets the [system requirements](../requirements.md). If you are using a different operating system, you may need to adjust the following steps accordingly or refer to the official documentation for each step.

## 1. Install [Docker Engine](https://docs.docker.com/engine)

First, install Docker Engine by following the [official installation instructions](https://docs.docker.com/engine/install). For example:

```bash
curl -fsSL https://get.docker.com | sh
sudo systemctl enable --now docker
sudo groupadd docker
sudo usermod -aG docker $USER
newgrp docker
```

## 2. Install [NVIDIA Container Toolkit](https://github.com/NVIDIA/nvidia-container-toolkit)

Next, install the NVIDIA Container Toolkit, which is required to enable GPU support for Docker containers. Follow the [official installation guide](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html) or use the following commands:

```bash
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg && curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
sudo apt-get update
sudo apt-get install -y nvidia-container-toolkit
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker
```

## 3. Clone the Repository

Next, clone the `srb` repository locally. Make sure to include the `--recurse-submodules` flag to clone also the submodule containing simulation assets.

```bash
git clone --recurse-submodules https://github.com/AndrejOrsula/space_robotics_bench.git
```

## 4. Build the Docker Image (Optional)

> This step is now optional and can be skipped by continuing directly to the instructions about the [Basic Usage](../usage.md). The Docker image is available on [Docker Hub](https://hub.docker.com/r/andrejorsula/space_robotics_bench), and it will be automatically pulled when you run the benchmark via Docker for the first time (usually faster than building it locally).

You can build the Docker image for `srb` by running the provided [`.docker/build.bash`](https://github.com/AndrejOrsula/space_robotics_bench/blob/main/.docker/build.bash) script. Note that the first build process may take up to 30 minutes (depending on your network speed and system configuration).

```bash
srb/.docker/build.bash
```

To ensure that the image was built successfully, run the following command. You should see the `srb` image listed among recently created Docker images.

```bash
docker images
```
