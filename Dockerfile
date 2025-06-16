## Base <https://hub.docker.com/_/ubuntu>
ARG BASE_IMAGE_NAME="ubuntu"
ARG BASE_IMAGE_TAG="22.04"

## Isaac Sim <https://catalog.ngc.nvidia.com/orgs/nvidia/containers/isaac-sim>
## Label as isaac-sim for copying into the final image
ARG ISAAC_SIM_IMAGE_NAME="nvcr.io/nvidia/isaac-sim"
ARG ISAAC_SIM_IMAGE_TAG="4.5.0"
FROM ${ISAAC_SIM_IMAGE_NAME}:${ISAAC_SIM_IMAGE_TAG} AS isaac-sim

## Continue with the base image
FROM ${BASE_IMAGE_NAME}:${BASE_IMAGE_TAG}

## Use bash as the default shell
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

## Create a barebones entrypoint that is conditionally updated throughout the Dockerfile
RUN echo "#!/usr/bin/env bash" >> /entrypoint.bash && \
    chmod +x /entrypoint.bash

###########################
### System Dependencies ###
###########################

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

## Build Python with enabled optimizations to improve the runtime training performance
ARG PYTHON_VERSION="3.10.16"
ARG PYTHON_PREFIX="/usr/local"
ENV PYTHONEXE="${PYTHON_PREFIX}/bin/python${PYTHON_VERSION%.*}"
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
    "${PYTHON_SRC_DIR}/configure" --enable-shared --enable-optimizations --with-lto --prefix="${PYTHON_PREFIX}" && \
    make -j "$(nproc)" && \
    make install && \
    cd - && \
    rm -rf "${PYTHON_SRC_DIR}"
## Create a 'python' symlink for convenience
RUN ln -sr "${PYTHONEXE}" "${PYTHON_PREFIX}/bin/python"
## Fix `PYTHONEXE` by disabling the append of "isaac-sim/kit/kernel/plugins" to `LD_LIBRARY_PATH` inside `isaac-sim/setup_python_env.sh`
# hadolint ignore=SC2016
RUN sed -i 's|$SCRIPT_DIR/kit/kernel/plugins:||' "${ISAAC_SIM_PATH}/setup_python_env.sh"
## Make the system Python identical with Isaac Sim Python
# hadolint ignore=SC2016
RUN mv "${PYTHONEXE}" "${PYTHON_PREFIX}/bin/python${PYTHON_VERSION}" && \
    echo -e '#!/bin/bash\n${ISAAC_SIM_PYTHON} "${@}"' > "${PYTHONEXE}" && \
    chmod +x "${PYTHONEXE}"
ENV PYTHONEXE="${PYTHON_PREFIX}/bin/python${PYTHON_VERSION}"
## Fake that Python was installed via apt
# hadolint ignore=DL3008
RUN apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install -yq --no-install-recommends \
    equivs && \
    rm -rf /var/lib/apt/lists/* && \
    for pkg in "libpython${PYTHON_VERSION%.*}" "libpython${PYTHON_VERSION%.*}-dev" "python${PYTHON_VERSION%.*}-dev"; do \
    equivs-control "${pkg}" && \
    echo -e "Package: ${pkg}\nProvides: ${pkg}\nVersion: ${PYTHON_VERSION}\nArchitecture: all" > "${pkg}" && \
    equivs-build "${pkg}" && \
    dpkg -i "${pkg}_${PYTHON_VERSION}_all.deb" && \
    apt-mark hold "${pkg}" && \
    rm "${pkg}" "${pkg}_${PYTHON_VERSION}_all.deb" ; \
    done

## Install system dependencies
# hadolint ignore=DL3008
RUN apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install -yq --no-install-recommends \
    # Common
    bash-completion \
    build-essential \
    ca-certificates \
    clang \
    cmake \
    curl \
    git \
    mold \
    unzip \
    xz-utils \
    # Graphics
    libgl1 \
    libglu1 \
    libxi6 \
    libxkbcommon-x11-0 \
    libxt-dev \
    # Video recording/processing
    ffmpeg && \
    rm -rf /var/lib/apt/lists/*

## Upgrade pip
RUN "${ISAAC_SIM_PYTHON}" -m pip install --no-input --no-cache-dir --upgrade pip

## Install Rust
ARG RUST_VERSION="1.87"
RUN echo -e "\n# Rust ${RUST_VERSION}" >> /entrypoint.bash && \
    echo "export PATH=\"${HOME}/.cargo/bin\${PATH:+:\${PATH}}\"" >> /entrypoint.bash && \
    echo "export CARGO_TARGET_DIR=\"${HOME}/.cargo/target\"" >> /entrypoint.bash && \
    echo "export CARGO_TARGET_X86_64_UNKNOWN_LINUX_GNU_RUSTFLAGS=\"-Clink-arg=-fuse-ld=mold -Ctarget-cpu=native\"" >> /entrypoint.bash && \
    echo -e "\n# PyO3" >> /entrypoint.bash && \
    echo "export PYO3_PYTHON=\"${ISAAC_SIM_PYTHON}\"" >> /entrypoint.bash && \
    curl --proto "=https" --tlsv1.2 -sSfL "https://sh.rustup.rs" | sh -s -- --no-modify-path -y --default-toolchain "${RUST_VERSION}" --profile default

## Install (Space) ROS
ARG INSTALL_SPACEROS=false
ARG ROS_DISTRO="humble"
ARG SPACEROS_TAG="${ROS_DISTRO}-2024.10.0"
# hadolint ignore=SC1091,DL3008
RUN if [[ "${INSTALL_SPACEROS,,}" != true ]]; then \
    curl --proto "=https" --tlsv1.2 -sSfL "https://raw.githubusercontent.com/ros/rosdistro/master/ros.key" -o /usr/share/keyrings/ros-archive-keyring.gpg && \
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
    echo "source \"/opt/ros/${ROS_DISTRO}/setup.bash\" --" >> /entrypoint.bash ; \
    fi
# hadolint ignore=SC1091,DL3003,DL3008
RUN if [[ "${INSTALL_SPACEROS,,}" = true ]]; then \
    curl --proto "=https" --tlsv1.2 -sSfL "https://raw.githubusercontent.com/ros/rosdistro/master/ros.key" -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo "${UBUNTU_CODENAME}") main" > /etc/apt/sources.list.d/ros2.list && \
    apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install -yq --no-install-recommends \
    clang-14 \
    cmake \
    g++ \
    gcc \
    libboost-dev \
    libboost-filesystem-dev \
    libboost-test-dev \
    libboost-thread-dev \
    libedit-dev \
    libgmp-dev \
    libsqlite3-dev \
    libtbb-dev \
    libz-dev \
    llvm-14 \
    llvm-14-dev \
    llvm-14-tools \
    ros-dev-tools && \
    rm -rf /var/lib/apt/lists/* && \
    "${ISAAC_SIM_PYTHON}" -m pip install --no-input --no-cache-dir catkin_pkg rosinstall_generator && \
    rosdep init --rosdistro "${ROS_DISTRO}" && \
    SPACEROS_DIR="/opt/spaceros" && \
    SPACEROS_SRC_DIR="${SPACEROS_DIR}/src" && \
    SPACEROS_INSTALL_DIR="${SPACEROS_DIR}/install" && \
    SPACEROS_LOG_DIR="${SPACEROS_DIR}/log" && \
    mkdir -p "${SPACEROS_SRC_DIR}" && \
    vcs import --shallow --recursive --force --input "https://raw.githubusercontent.com/space-ros/space-ros/refs/tags/${SPACEROS_TAG}/ros2.repos" "${SPACEROS_SRC_DIR}" && \
    vcs import --shallow --recursive --force --input "https://raw.githubusercontent.com/space-ros/space-ros/refs/tags/${SPACEROS_TAG}/ikos.repos" "${SPACEROS_SRC_DIR}" && \
    vcs import --shallow --recursive --force --input "https://raw.githubusercontent.com/space-ros/space-ros/refs/tags/${SPACEROS_TAG}/spaceros.repos" "${SPACEROS_SRC_DIR}" && \
    apt-get update && \
    rosdep update --rosdistro "${ROS_DISTRO}" && \
    DEBIAN_FRONTEND=noninteractive rosdep install --default-yes --ignore-src -r --rosdistro "${ROS_DISTRO}" --from-paths "${SPACEROS_SRC_DIR}" --skip-keys "$(curl --proto =https --tlsv1.2 -sSfL https://raw.githubusercontent.com/space-ros/space-ros/refs/tags/${SPACEROS_TAG}/excluded-pkgs.txt | tr '\n' ' ') urdfdom_headers ikos" && \
    rm -rf /var/lib/apt/lists/* /root/.ros/rosdep/sources.cache && \
    cd "${SPACEROS_DIR}" && \
    colcon build --cmake-args -DPython3_EXECUTABLE="${ISAAC_SIM_PYTHON}" -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DLLVM_CONFIG_EXECUTABLE="/usr/lib/llvm-14/bin/llvm-config" --no-warn-unused-cli && \
    cd - && \
    rm -rf "${SPACEROS_LOG_DIR}" && \
    echo -e "\n# Space ROS ${ROS_DISTRO^}" >> /entrypoint.bash && \
    echo "source \"${SPACEROS_INSTALL_DIR}/setup.bash\" --" >> /entrypoint.bash ; \
    fi

## Install Blender
ARG BLENDER_PATH="/root/blender"
ARG BLENDER_VERSION="4.3.2"
ARG BLENDER_PYTHON="${BLENDER_PATH}/${BLENDER_VERSION%.*}/python/bin/python3.11"
# hadolint ignore=SC2016
RUN echo -e "\n# Blender ${BLENDER_VERSION}" >> /entrypoint.bash && \
    echo "export PATH=\"${BLENDER_PATH}\${PATH:+:\${PATH}}\"" >> /entrypoint.bash && \
    curl --proto "=https" --tlsv1.2 -sSfL "https://download.blender.org/release/Blender${BLENDER_VERSION%.*}/blender-${BLENDER_VERSION}-linux-x64.tar.xz" -o "/tmp/blender_${BLENDER_VERSION}.tar.xz" && \
    mkdir -p "${BLENDER_PATH}" && \
    tar xf "/tmp/blender_${BLENDER_VERSION}.tar.xz" -C "${BLENDER_PATH}" --strip-components=1 && \
    rm "/tmp/blender_${BLENDER_VERSION}.tar.xz"

## Install Omniverse Hub
RUN curl --proto "=https" --tlsv1.2 -sSfL "https://api.ngc.nvidia.com/v2/resources/nvidia/omniverse/hub_workstation_cache/versions/1.0.0/files/omni_hub.linux-x86_64@1.0.0+8e3e0971.zip" -o "/tmp/omni_hub.zip" && \
    mkdir -p /tmp/omni_hub && \
    unzip -q /tmp/omni_hub.zip -d /tmp/omni_hub && \
    /tmp/omni_hub/scripts/install.sh && \
    rm -rf /tmp/omni_hub /tmp/omni_hub.zip

###################
### Development ###
###################
ARG DEV=true

## Simulation
ARG ISAACLAB_DEV=true
ARG ISAACLAB_PATH="/root/isaaclab"
ARG ISAACLAB_REMOTE="https://github.com/isaac-sim/IsaacLab.git"
ARG ISAACLAB_BRANCH="main"
ARG ISAACLAB_COMMIT_SHA="7de6d6fef9424c95fc68dc767af67ffbe0da6832" # 2025-04-11
# hadolint ignore=SC2044
RUN if [[ "${DEV,,}" = true && "${ISAACLAB_DEV,,}" = true ]]; then \
    echo -e "\n# Isaac Lab ${ISAACLAB_COMMIT_SHA}" >> /entrypoint.bash && \
    echo "export ISAACLAB_PATH=\"${ISAACLAB_PATH}\"" >> /entrypoint.bash && \
    git clone "${ISAACLAB_REMOTE}" "${ISAACLAB_PATH}" --branch "${ISAACLAB_BRANCH}" && \
    git -C "${ISAACLAB_PATH}" reset --hard "${ISAACLAB_COMMIT_SHA}" && \
    for extension in $(find -L "${ISAACLAB_PATH}/source" -mindepth 1 -maxdepth 1 -type d); do \
    if [ -f "${extension}/pyproject.toml" ] || [ -f "${extension}/setup.py" ]; then \
    "${ISAAC_SIM_PYTHON}" -m pip install --no-input --no-cache-dir --editable "${extension}" ; \
    fi ; \
    done && \
    ln -sf "${ISAAC_SIM_PATH}" "${ISAACLAB_PATH}/_isaac_sim" ; \
    fi
ARG OXIDASIM_DEV=false
ARG OXIDASIM_PATH="/root/oxidasim"
ARG OXIDASIM_REMOTE="https://github.com/AndrejOrsula/oxidasim.git"
ARG OXIDASIM_BRANCH="main"
# hadolint ignore=SC1091
RUN if [[ "${DEV,,}" = true && "${OXIDASIM_DEV,,}" = true ]]; then \
    git clone "${OXIDASIM_REMOTE}" "${OXIDASIM_PATH}" --branch "${OXIDASIM_BRANCH}" && \
    source /entrypoint.bash -- && \
    "${ISAAC_SIM_PYTHON}" -m pip install --no-input --no-cache-dir --editable "${OXIDASIM_PATH}[all]" ; \
    fi

## Assets
ARG SIMFORGE_DEV=true
ARG SIMFORGE_PATH="/root/simforge"
ARG SIMFORGE_REMOTE="https://github.com/AndrejOrsula/simforge.git"
ARG SIMFORGE_BRANCH="main"
RUN if [[ "${DEV,,}" = true && "${SIMFORGE_DEV,,}" = true ]]; then \
    git clone "${SIMFORGE_REMOTE}" "${SIMFORGE_PATH}" --branch "${SIMFORGE_BRANCH}" && \
    "${ISAAC_SIM_PYTHON}" -m pip install --no-input --no-cache-dir --editable "${SIMFORGE_PATH}[all]" && \
    "${BLENDER_PYTHON}" -m pip install --no-input --no-cache-dir --editable "${SIMFORGE_PATH}[dev]" ; \
    fi
ARG SIMFORGE_FOUNDRY_DEV=true
ARG SIMFORGE_FOUNDRY_PATH="/root/simforge_foundry"
ARG SIMFORGE_FOUNDRY_REMOTE="https://github.com/AndrejOrsula/simforge_foundry.git"
ARG SIMFORGE_FOUNDRY_BRANCH="main"
RUN if [[ "${DEV,,}" = true && "${SIMFORGE_FOUNDRY_DEV,,}" = true ]]; then \
    git clone "${SIMFORGE_FOUNDRY_REMOTE}" "${SIMFORGE_FOUNDRY_PATH}" --branch "${SIMFORGE_FOUNDRY_BRANCH}" && \
    "${ISAAC_SIM_PYTHON}" -m pip install --no-input --no-cache-dir --editable "${SIMFORGE_FOUNDRY_PATH}" && \
    "${BLENDER_PYTHON}" -m pip install --no-input --no-cache-dir --editable "${SIMFORGE_FOUNDRY_PATH}" ; \
    fi

## Reinforcement Learning
ARG DREAMER_DEV=true
ARG DREAMER_PATH="/root/dreamerv3"
ARG DREAMER_REMOTE="https://github.com/AndrejOrsula/dreamerv3.git"
ARG DREAMER_BRANCH="main"
ARG DREAMER_COMMIT_SHA="45c3955afb6ee26310fc5bce65102320b8d98b18" # 2025-04-11
RUN if [[ "${DEV,,}" = true && "${DREAMER_DEV,,}" = true ]]; then \
    git clone "${DREAMER_REMOTE}" "${DREAMER_PATH}" --branch "${DREAMER_BRANCH}" && \
    git -C "${DREAMER_PATH}" reset --hard "${DREAMER_COMMIT_SHA}" && \
    "${ISAAC_SIM_PYTHON}" -m pip install --no-input --no-cache-dir --editable "${DREAMER_PATH}" ; \
    fi
ARG RL_ZOO3_DEV=true
ARG RL_ZOO3_PATH="/root/rl_zoo3"
ARG RL_ZOO3_REMOTE="https://github.com/AndrejOrsula/rl-baselines3-zoo.git"
ARG RL_ZOO3_BRANCH="master"
ARG RL_ZOO3_COMMIT_SHA="e04921b4ccbadbc9f6bcc46cc1787ffc1d2c8963" # 2025-01-18
RUN if [[ "${DEV,,}" = true && "${RL_ZOO3_DEV,,}" = true ]]; then \
    git clone "${RL_ZOO3_REMOTE}" "${RL_ZOO3_PATH}" --branch "${RL_ZOO3_BRANCH}" && \
    git -C "${RL_ZOO3_PATH}" reset --hard "${RL_ZOO3_COMMIT_SHA}" && \
    "${ISAAC_SIM_PYTHON}" -m pip install --no-input --no-cache-dir --editable "${RL_ZOO3_PATH}" ; \
    fi
ARG SKRL_DEV=false
ARG SKRL_PATH="/root/skrl"
ARG SKRL_REMOTE="https://github.com/Toni-SM/skrl.git"
ARG SKRL_BRANCH="main"
ARG SKRL_COMMIT_SHA="cdf570902b1eaba193cc8ef69426cd4edde1b0bc" # 2025-04-06
RUN if [[ "${DEV,,}" = true && "${SKRL_DEV,,}" = true ]]; then \
    git clone "${SKRL_REMOTE}" "${SKRL_PATH}" --branch "${SKRL_BRANCH}" && \
    git -C "${SKRL_PATH}" reset --hard "${SKRL_COMMIT_SHA}" && \
    "${ISAAC_SIM_PYTHON}" -m pip install --no-input --no-cache-dir --editable "${SKRL_PATH}" ; \
    fi

##################
### Entrypoint ###
##################

## Define the workspace of the project
ARG SRB_PATH="/root/ws"
RUN echo -e "\n# Space Robotics Bench" >> /entrypoint.bash && \
    echo "export SRB_PATH=\"${SRB_PATH}\"" >> /entrypoint.bash
WORKDIR "${SRB_PATH}"

## Finalize the entrypoint
# hadolint ignore=SC2016
RUN echo -e "\n# Execute command" >> /entrypoint.bash && \
    echo -en 'exec "${@}"\n' >> /entrypoint.bash && \
    sed -i '$a source /entrypoint.bash --' ~/.bashrc
ENTRYPOINT ["/entrypoint.bash"]

####################
### Dependencies ###
####################

## Install ROS dependencies
RUN --mount=type=bind,source="package.xml",target="${SRB_PATH}/package.xml" \
    apt-get update && \
    rosdep update --rosdistro "${ROS_DISTRO}" && \
    DEBIAN_FRONTEND=noninteractive rosdep install --default-yes --ignore-src --rosdistro "${ROS_DISTRO}" --from-paths "${SRB_PATH}" && \
    rm -rf /var/lib/apt/lists/* /root/.ros/rosdep/sources.cache

## Install Python dependencies
# hadolint ignore=DL3013,SC2046
RUN --mount=type=bind,source="pyproject.toml",target="${SRB_PATH}/pyproject.toml" \
    "${ISAAC_SIM_PYTHON}" -m pip install --no-input --no-cache-dir --ignore-installed toml~=0.10 && \
    "${ISAAC_SIM_PYTHON}" -m pip install --no-input --no-cache-dir $("${ISAAC_SIM_PYTHON}" -c "f='${SRB_PATH}/pyproject.toml'; from toml import load; print(' '.join(filter(lambda d: not d.startswith(p['name'] + '['), (*p.get('dependencies', ()), *(d for ds in p.get('optional-dependencies', {}).values() for d in ds)))) if (p := load(f).get('project', None)) else '')")

###############
### Project ###
###############

## Copy the source code into the image
COPY . "${SRB_PATH}"

## Build Rust targets of the project
ARG BUILD_RUST=true
# hadolint ignore=SC1091
RUN if [[ "${BUILD_RUST,,}" = true ]]; then \
    source /entrypoint.bash -- && \
    cargo build --release --workspace --all-targets --all-features ; \
    fi

## Install project as ROS 2 package
ARG ROS_WS="/opt/ros/${ROS_DISTRO}/ws"
# hadolint ignore=SC1091
RUN source /entrypoint.bash -- && \
    colcon build --merge-install --symlink-install --cmake-args -DPython3_EXECUTABLE="${ISAAC_SIM_PYTHON}" --paths "${SRB_PATH}" --build-base "${ROS_WS}/build" --install-base "${ROS_WS}/install" && \
    rm -rf ./log && \
    sed -i "s|source \"/opt/ros/${ROS_DISTRO}/setup.bash\" --|source \"${ROS_WS}/install/setup.bash\" --|" /entrypoint.bash

## Install project as editable Python package
# hadolint ignore=SC1091
RUN source /entrypoint.bash -- && \
    "${ISAAC_SIM_PYTHON}" -m pip install --no-input --no-cache-dir --no-deps --editable "${SRB_PATH}[all]"

ARG EXECUTABLES="simforge srb space_robotics_bench"

## Update the shebang of installed executables (ISAAC_SIM_PYTHON is bypassed by default)
RUN for exe in ${EXECUTABLES}; do \
    sed -i "1s|.*|#!${ISAAC_SIM_PYTHON}|" "$(which "${exe}")" ; \
    done

## Configure argcomplete
RUN echo "source /etc/bash_completion" >> "/etc/bash.bashrc" && \
    for exe in ${EXECUTABLES}; do \
    register-python-argcomplete3 "${exe}" > "/etc/bash_completion.d/${exe}" ; \
    done

## Set the default command
CMD ["bash"]

############
### Misc ###
############

## Skip writing Python bytecode to the disk to avoid polluting mounted host volume with `__pycache__` directories
ENV PYTHONDONTWRITEBYTECODE=1

## Enable full error backtrace with Hydra
ENV HYDRA_FULL_ERROR=1
