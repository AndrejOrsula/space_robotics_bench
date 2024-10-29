## Base <https://hub.docker.com/_/ubuntu>
ARG BASE_IMAGE_NAME="ubuntu"
ARG BASE_IMAGE_TAG="jammy"

## Isaac Sim <https://catalog.ngc.nvidia.com/orgs/nvidia/containers/isaac-sim>
## Label as isaac-sim for copying into the final image
ARG ISAAC_SIM_IMAGE_NAME="nvcr.io/nvidia/isaac-sim"
ARG ISAAC_SIM_IMAGE_TAG="4.2.0"
FROM ${ISAAC_SIM_IMAGE_NAME}:${ISAAC_SIM_IMAGE_TAG} AS isaac-sim

## Continue with the base image
FROM ${BASE_IMAGE_NAME}:${BASE_IMAGE_TAG}

## Use bash as the default shell
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

## Create a barebones entrypoint that is conditionally updated throughout the Dockerfile
RUN echo "#!/usr/bin/env bash" >> /entrypoint.bash && \
    chmod +x /entrypoint.bash

## Copy Isaac Sim into the base image
ARG ISAAC_SIM_PATH="/root/isaac-sim"
ENV ISAAC_SIM_PYTHON="${ISAAC_SIM_PATH}/python.sh"
COPY --from=isaac-sim /isaac-sim "${ISAAC_SIM_PATH}"
COPY --from=isaac-sim /root/.nvidia-omniverse/config /root/.nvidia-omniverse/config
COPY --from=isaac-sim /etc/vulkan/icd.d/nvidia_icd.json /etc/vulkan/icd.d/nvidia_icd.json
RUN ISAAC_SIM_VERSION="$(cut -d'-' -f1 < "${ISAAC_SIM_PATH}/VERSION")" && \
    echo -e "\n# Isaac Sim ${ISAAC_SIM_VERSION}" >> /entrypoint.bash && \
    echo "export ISAAC_SIM_PATH=\"${ISAAC_SIM_PATH}\"" >> /entrypoint.bash && \
    echo "export OMNI_KIT_ALLOW_ROOT=\"1\"" >> /entrypoint.bash
## Fix cosmetic issues in `isaac-sim/setup_python_env.sh` that append nonsense paths to `PYTHONPATH` and `LD_LIBRARY_PATH`
# hadolint ignore=SC2016
RUN sed -i 's|$SCRIPT_DIR/../../../$LD_LIBRARY_PATH:||' "${ISAAC_SIM_PATH}/setup_python_env.sh" && \
    sed -i 's|$SCRIPT_DIR/../../../$PYTHONPATH:||' "${ISAAC_SIM_PATH}/setup_python_env.sh"

## Optimization: Build Python to improve the runtime performance of training
ARG PYTHON_VERSION="3.10.14"
ENV PYTHONEXE="/usr/local/bin/python${PYTHON_VERSION%%.*}"
# hadolint ignore=DL3003,DL3008
RUN PYTHON_DL_PATH="/tmp/Python-${PYTHON_VERSION}.tar.xz" && \
    PYTHON_SRC_DIR="/tmp/python${PYTHON_VERSION}" && \
    apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install -yq --no-install-recommends \
    build-essential \
    ca-certificates \
    curl \
    libbz2-dev \
    libdb4o-cil-dev \
    libgdm-dev \
    libhidapi-dev \
    liblzma-dev \
    libncurses5-dev \
    libpcap-dev \
    libreadline-dev \
    libsqlite3-dev \
    libssl-dev \
    libtk8.6 \
    lzma \
    xz-utils && \
    rm -rf /var/lib/apt/lists/* && \
    curl --proto "=https" --tlsv1.2 -sSfL "https://www.python.org/ftp/python/${PYTHON_VERSION}/Python-${PYTHON_VERSION}.tar.xz" -o "${PYTHON_DL_PATH}" && \
    mkdir -p "${PYTHON_SRC_DIR}" && \
    tar xf "${PYTHON_DL_PATH}" -C "${PYTHON_SRC_DIR}" --strip-components=1 && \
    rm "${PYTHON_DL_PATH}" && \
    cd "${PYTHON_SRC_DIR}" && \
    "${PYTHON_SRC_DIR}/configure" --enable-shared --enable-optimizations --with-lto --prefix="/usr/local" && \
    make -j "$(nproc)" && \
    make install && \
    cd - && \
    rm -rf "${PYTHON_SRC_DIR}"
## Fix `PYTHONEXE` by disabling the append of "isaac-sim/kit/kernel/plugins" to `LD_LIBRARY_PATH` inside `isaac-sim/setup_python_env.sh`
# hadolint ignore=SC2016
RUN sed -i 's|$SCRIPT_DIR/kit/kernel/plugins:||' "${ISAAC_SIM_PATH}/setup_python_env.sh"

## Install system dependencies
# hadolint ignore=DL3008
RUN apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install -yq --no-install-recommends \
    # Common
    build-essential \
    ca-certificates \
    cmake \
    curl \
    git \
    mold \
    xz-utils \
    # Isaac Sim
    libgl1 \
    libglu1 \
    libxt-dev \
    # Blender
    libegl1 \
    # egui
    libxkbcommon-x11-0 \
    # r2r
    clang \
    # Video recording/processing
    ffmpeg && \
    rm -rf /var/lib/apt/lists/*

## Upgrade pip
RUN "${ISAAC_SIM_PYTHON}" -m pip install --no-input --no-cache-dir --upgrade pip

## Install Rust
ARG RUST_VERSION="1.80"
RUN echo -e "\n# Rust ${RUST_VERSION}" >> /entrypoint.bash && \
    echo "export PATH=\"${HOME}/.cargo/bin\${PATH:+:\${PATH}}\"" >> /entrypoint.bash && \
    echo "export CARGO_TARGET_DIR=\"${HOME}/.cargo/target\"" >> /entrypoint.bash && \
    echo "export CARGO_TARGET_X86_64_UNKNOWN_LINUX_GNU_RUSTFLAGS=\"-Clink-arg=-fuse-ld=mold -Ctarget-cpu=native\"" >> /entrypoint.bash && \
    echo -e "\n# PyO3" >> /entrypoint.bash && \
    echo "export PYO3_PYTHON=\"${ISAAC_SIM_PYTHON}\"" >> /entrypoint.bash && \
    curl --proto "=https" --tlsv1.2 -sSfL "https://sh.rustup.rs" | sh -s -- --no-modify-path -y --default-toolchain "${RUST_VERSION}" --profile default

## Install ROS
ARG ROS_DISTRO="humble"
# hadolint ignore=SC1091,DL3008
RUN curl --proto "=https" --tlsv1.2 -sSfL "https://raw.githubusercontent.com/ros/rosdistro/master/ros.key" -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo "${UBUNTU_CODENAME}") main" > /etc/apt/sources.list.d/ros2.list && \
    apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install -yq --no-install-recommends \
    ros-dev-tools \
    "ros-${ROS_DISTRO}-ros-base" \
    "ros-${ROS_DISTRO}-rmw-cyclonedds-cpp" && \
    rm -rf /var/lib/apt/lists/* && \
    "${ISAAC_SIM_PYTHON}" -m pip install --no-input --no-cache-dir catkin_pkg && \
    rosdep init --rosdistro "${ROS_DISTRO}" && \
    echo -e "\n# ROS ${ROS_DISTRO^}" >> /entrypoint.bash && \
    echo "source \"/opt/ros/${ROS_DISTRO}/setup.bash\" --" >> /entrypoint.bash

## Install Blender
ARG BLENDER_PATH="/root/blender"
ARG BLENDER_VERSION="4.2.2"
# hadolint ignore=SC2016
RUN echo -e "\n# Blender ${BLENDER_VERSION}" >> /entrypoint.bash && \
    echo "export PATH=\"${BLENDER_PATH}\${PATH:+:\${PATH}}\"" >> /entrypoint.bash && \
    curl --proto "=https" --tlsv1.2 -sSfL "https://download.blender.org/release/Blender${BLENDER_VERSION%.*}/blender-${BLENDER_VERSION}-linux-x64.tar.xz" -o "/tmp/blender_${BLENDER_VERSION}.tar.xz" && \
    mkdir -p "${BLENDER_PATH}" && \
    tar xf "/tmp/blender_${BLENDER_VERSION}.tar.xz" -C "${BLENDER_PATH}" --strip-components=1 && \
    rm "/tmp/blender_${BLENDER_VERSION}.tar.xz"

## Install Isaac Lab
ARG ISAACLAB_PATH="/root/isaaclab"
ARG ISAACLAB_REMOTE="https://github.com/isaac-sim/IsaacLab.git"
ARG ISAACLAB_BRANCH="main"
ARG ISAACLAB_COMMIT_SHA="0ef582badf6f257bb3c320c63d5d6d899604a138" # Oct 5, 2024
# hadolint ignore=SC2044
RUN echo -e "\n# Isaac Lab ${ISAACLAB_COMMIT_SHA}" >> /entrypoint.bash && \
    echo "export ISAACLAB_PATH=\"${ISAACLAB_PATH}\"" >> /entrypoint.bash && \
    git clone "${ISAACLAB_REMOTE}" "${ISAACLAB_PATH}" --branch "${ISAACLAB_BRANCH}" && \
    git -C "${ISAACLAB_PATH}" reset --hard "${ISAACLAB_COMMIT_SHA}" && \
    for extension in $(find -L "${ISAACLAB_PATH}/source/extensions" -mindepth 1 -maxdepth 1 -type d); do \
    if [ -f "${extension}/setup.py" ]; then \
    "${ISAAC_SIM_PYTHON}" -m pip install --no-input --no-cache-dir --editable "${extension}" ; \
    fi ; \
    done

## Define the workspace of the project
ARG SRB_PATH="/root/ws"
RUN echo -e "\n# Space Robotics Bench" >> /entrypoint.bash && \
    echo "export SRB_PATH=\"${SRB_PATH}\"" >> /entrypoint.bash
WORKDIR "${SRB_PATH}"

## If the workspace is mounted as a volume (`.git/` dir is ignored via `.dockerignore`), override
## the share object of the Rust extension module with the one generated by the Docker image
RUN echo -e "\n# Use local Rust extension module if the project is mounted as a volume" >> /entrypoint.bash && \
    echo -e "if [ -d \"${SRB_PATH}/.git\" ]; then\n  cp \"\${CARGO_TARGET_DIR}/release/libspace_robotics_bench_py.so\" \"${SRB_PATH}/space_robotics_bench/_rs.abi3.so\"\nfi" >> /entrypoint.bash

## Finalize the entrypoint
# hadolint ignore=SC2016
RUN echo -e "\n# Execute command" >> /entrypoint.bash && \
    echo -en 'exec "${@}"\n' >> /entrypoint.bash && \
    sed -i '$a source /entrypoint.bash --' ~/.bashrc
ENTRYPOINT ["/entrypoint.bash"]

## [Optional] Install Python dependencies in advance to cache the layers (speeds up rebuilds)
COPY ./pyproject.toml "${SRB_PATH}/"
# hadolint ignore=SC2046
RUN "${ISAAC_SIM_PYTHON}" -m pip install --no-input --no-cache-dir toml==0.10.2 && \
    "${ISAAC_SIM_PYTHON}" -m pip install --no-input --no-cache-dir $("${ISAAC_SIM_PYTHON}" -c "import toml, itertools; data = toml.load(open('${SRB_PATH}/pyproject.toml')); project_name = data['project']['name']; deps = [dep for dep in data['project'].get('dependencies', []) if project_name not in dep]; opt_deps = list(itertools.chain(*[opt for opt in data['project'].get('optional-dependencies', {}).values() if not any(project_name in dep for dep in opt)])); print(' '.join(deps + opt_deps))")

## Install ROS dependencies in advance to cache the layers (speeds up rebuilds)
COPY ./package.xml "${SRB_PATH}/"
RUN apt-get update && \
    rosdep update --rosdistro "${ROS_DISTRO}" && \
    DEBIAN_FRONTEND=noninteractive rosdep install --default-yes --ignore-src --rosdistro "${ROS_DISTRO}" --from-paths "${SRB_PATH}" && \
    rm -rf /var/lib/apt/lists/* /root/.ros/rosdep/sources.cache

## Copy the source code into the image
COPY . "${SRB_PATH}"

## Build Rust targets
# hadolint ignore=SC1091
RUN source /entrypoint.bash -- && \
    cargo build --release --workspace --all-targets

## Install project as editable Python module
# hadolint ignore=SC1091
RUN source /entrypoint.bash -- && \
    "${ISAAC_SIM_PYTHON}" -m pip install --no-input --no-cache-dir --editable "${SRB_PATH}[all]"

## Install project as ROS package
ARG ROS_WS="/opt/ros/${ROS_DISTRO}/ws"
# hadolint ignore=SC1091
RUN source /entrypoint.bash -- && \
    colcon build --merge-install --symlink-install --cmake-args -DPython3_EXECUTABLE=${ISAAC_SIM_PYTHON} --paths "${SRB_PATH}" --build-base "${ROS_WS}/build" --install-base "${ROS_WS}/install" && \
    rm -rf ./log && \
    sed -i "s|source \"/opt/ros/${ROS_DISTRO}/setup.bash\" --|source \"${ROS_WS}/install/setup.bash\" --|" /entrypoint.bash

## Set the default command
CMD ["bash"]

############
### Misc ###
############

## Skip writing Python bytecode to the disk to avoid polluting mounted host volume with `__pycache__` directories
ENV PYTHONDONTWRITEBYTECODE=1

###############
### Develop ###
###############

# ## Install DreamerV3 locally to enable mounting the source code into the container
# ARG DREAMERV3_PATH="/root/dreamerv3"
# ARG DREAMERV3_REMOTE="https://github.com/AndrejOrsula/dreamerv3.git"
# ARG DREAMERV3_BRANCH="dev"
# RUN git clone "${DREAMERV3_REMOTE}" "${DREAMERV3_PATH}" --branch "${DREAMERV3_BRANCH}" && \
#     "${ISAAC_SIM_PYTHON}" -m pip install --no-input --no-cache-dir -r "${DREAMERV3_PATH}/embodied/requirements.txt" && \
#     "${ISAAC_SIM_PYTHON}" -m pip install --no-input --no-cache-dir -r "${DREAMERV3_PATH}/dreamerv3/requirements.txt" -f "https://storage.googleapis.com/jax-releases/jax_cuda_releases.html" && \
#     "${ISAAC_SIM_PYTHON}" -m pip install --no-input --no-cache-dir --editable "${DREAMERV3_PATH}"
