# syntax=docker/dockerfile:1.20
ARG BASE_IMAGE_NAME="ubuntu"
ARG BASE_IMAGE_TAG="noble-20260210.1"
FROM ${BASE_IMAGE_NAME}:${BASE_IMAGE_TAG}

## Supported target: linux/amd64+nvidia-rtx
ARG TARGETARCH

SHELL ["/bin/bash", "-exo", "pipefail", "-c"]

RUN printf '#!/usr/bin/env bash\n' >> /entrypoint.bash && \
    chmod +x /entrypoint.bash


## Configure retries globally; later stages also fetch packages.
# hadolint ignore=DL3008
RUN printf 'Acquire::Retries "5";\n' > /etc/apt/apt.conf.d/80-srb-retries && \
    apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends software-properties-common && \
    add-apt-repository -y ppa:fish-shell/release-4 && \
    add-apt-repository -y ppa:maveonair/helix-editor && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
    bash-completion \
    build-essential \
    ca-certificates \
    clang \
    cmake \
    curl \
    ffmpeg \
    fish \
    g++-11 \
    gcc-11 \
    git \
    git-lfs \
    helix \
    jq \
    libatomic1 \
    libegl-dev \
    libegl1 \
    libgl-dev \
    libgl1 \
    libgles-dev \
    libglib2.0-0 \
    libglu1 \
    libglu1-mesa \
    libglx0 \
    libgomp1 \
    libnghttp2-14 \
    libopengl-dev \
    libsm6 \
    libxi6 \
    libxkbcommon-x11-0 \
    libxrandr2 \
    libxt-dev \
    libxt6 \
    llvm \
    micro \
    mold \
    nano \
    pkg-config \
    unzip \
    wget \
    xz-utils \
    && \
    apt-get -y autoremove && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*


ARG RUST_VERSION="1.94"
## Verify the versioned rustup installer against the release digest.
ARG RUSTUP_VERSION="1.29.1"
ARG RUSTUP_SHA256_AMD64="dda7234360b7f578ca8b0ddcb80145646fa61a67c1720a5abc7051b35c9fcb71"
ARG RUSTUP_SHA256_ARM64="15f6e4ce9f583b929c996c91562bad6d4454f3281de858b02cdfdef615fac433"
RUN if [ "$TARGETARCH" = "amd64" ]; then RUSTUP_ARCH="x86_64"; RUSTUP_SHA256="${RUSTUP_SHA256_AMD64}"; else RUSTUP_ARCH="aarch64"; RUSTUP_SHA256="${RUSTUP_SHA256_ARM64}"; fi && \
    curl --proto "=https" --tlsv1.2 -sSfL --retry 5 --retry-all-errors --connect-timeout 30 "https://static.rust-lang.org/rustup/archive/${RUSTUP_VERSION}/${RUSTUP_ARCH}-unknown-linux-gnu/rustup-init" -o /tmp/rustup-init && \
    echo "${RUSTUP_SHA256}  /tmp/rustup-init" | sha256sum --check --strict - && \
    chmod +x /tmp/rustup-init && \
    /tmp/rustup-init --no-modify-path -y --default-toolchain "${RUST_VERSION}" --profile default && \
    rm /tmp/rustup-init
ENV PATH="/root/.cargo/bin:${PATH}"
ENV CARGO_TARGET_DIR="/root/.cargo/target"
## Raise Cargo's network limits for large, cold dependency downloads.
ENV CARGO_NET_RETRY=10
ENV CARGO_HTTP_TIMEOUT=120
ENV RUSTFLAGS="-Clink-arg=-fuse-ld=mold"

ARG MDBOOK_VERSION="0.5.3"
RUN --mount=type=cache,target="/root/.cargo/git" \
    --mount=type=cache,target="/root/.cargo/registry" \
    --mount=type=cache,target="/root/.cargo/target" \
    cargo install mdbook --locked --version "${MDBOOK_VERSION}" && \
    mdbook --version


ARG UV_VERSION="0.11.3"
## Keep the installer version and checksum in sync.
ADD --checksum=sha256:b953b3f2a2764cbf860eee4578a5949fa90ed010644c6be1006f29010bada946 "https://astral.sh/uv/${UV_VERSION}/install.sh" /tmp/uv_install.sh
RUN sh /tmp/uv_install.sh && rm /tmp/uv_install.sh
ENV PATH="/root/.local/bin/:${PATH}"
ENV UV_COMPILE_BYTECODE=1
ENV UV_LINK_MODE=copy
## Large concurrent CUDA wheel downloads can exceed uv's per-read timeout.
## Raise the timeout; retries are configured separately below.
ENV UV_HTTP_TIMEOUT=300
## Raise retries for the large wheel sets resolved by later uv installs.
ENV UV_HTTP_RETRIES=5

ARG PYTHON_VERSION="3.12"
ARG VENV_PATH="/root/.venv"
ENV PATH="${VENV_PATH}/bin:${PATH}"
RUN --mount=type=cache,target="/root/.cache/uv" \
    uv python install "${PYTHON_VERSION}" && \
    uv venv --python "${PYTHON_VERSION}" "${VENV_PATH}" && \
    printf '\n# Python %s via uv %s\n' "${PYTHON_VERSION}" "${UV_VERSION}" >> /entrypoint.bash && \
    printf 'source "%s/bin/activate" --\n' "${VENV_PATH}" >> /entrypoint.bash
ENV PYTHONEXE="${VENV_PATH}/bin/python"
ENV PYO3_PYTHON="${PYTHONEXE}"

## Isaac Sim aarch64 builds need X11 headers for dependencies without wheels.
# hadolint ignore=DL3008
RUN if [ "$TARGETARCH" != "amd64" ]; then \
    apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends libx11-dev xorg-dev && \
    apt-get -y autoremove && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/* ; \
    fi


ARG ISAACSIM_PATH="/root/isaac-sim"
ARG ISAACSIM_PATH_SRC="/root/.isaac-sim"
ARG ISAACSIM_REMOTE="https://github.com/isaac-sim/IsaacSim.git"
ARG ISAACSIM_BRANCH="release/6.1"
ARG ISAACSIM_COMMIT_SHA="7c206f75bdadd9e05fc457f19863ca4c3f0cb693" # v6.1.0 (source VERSION: 6.1.0-rc.26)
ARG ISAACSIM_VERSION="6.1.0-rc.26"
ARG ISAACSIM_RELEASE_VERSION="6.1.0"
## Retry only resumable downloads/builds; a failed clone leaves its destination.
RUN --mount=type=cache,target="/root/.cache/uv" \
    --mount=type=cache,target="/root/.cache/pip" \
    [[ "${ISAACSIM_REMOTE}" = "https://github.com/isaac-sim/IsaacSim.git" ]] || { echo "ISAACSIM_REMOTE is not the reviewed upstream repository" >&2; exit 64; } && \
    { [[ "${ISAACSIM_BRANCH}" = "release/6.1" ]] || { echo "ISAACSIM_BRANCH must be release/6.1 for the promoted source" >&2; exit 64; } ; } && \
    { [[ "${ISAACSIM_COMMIT_SHA}" =~ ^[0-9a-f]{40}$ ]] || { echo "ISAACSIM_COMMIT_SHA must be a full 40-character lowercase hex commit" >&2; exit 64; } ; } && \
    { [[ "${ISAACSIM_COMMIT_SHA}" = "7c206f75bdadd9e05fc457f19863ca4c3f0cb693" ]] || { echo "ISAACSIM_COMMIT_SHA must match the reviewed Isaac Sim source pin" >&2; exit 64; } ; } && \
    { [[ "${ISAACSIM_VERSION}" = "6.1.0-rc.26" ]] || { echo "ISAACSIM_VERSION must be 6.1.0-rc.26 for the pinned source" >&2; exit 64; } ; } && \
    { [[ "${ISAACSIM_RELEASE_VERSION}" = "6.1.0" ]] || { echo "ISAACSIM_RELEASE_VERSION must be 6.1.0 for the pinned source" >&2; exit 64; } ; } && \
    srb_retry() { srb_retry_attempt=1; while true; do if "$@"; then return 0; fi; if [ "${srb_retry_attempt}" -ge 3 ]; then echo "[ERROR] \`$*\` failed on all 3 attempts" >&2; return 1; fi; echo "[WARNING] \`$*\` failed (attempt ${srb_retry_attempt} of 3); retrying in $(( srb_retry_attempt * 30 ))s" >&2; sleep "$(( srb_retry_attempt * 30 ))"; srb_retry_attempt=$(( srb_retry_attempt + 1 )); done; } && \
    git clone "${ISAACSIM_REMOTE}" "${ISAACSIM_PATH_SRC}" --branch "${ISAACSIM_BRANCH}" && \
    git -C "${ISAACSIM_PATH_SRC}" reset --hard "${ISAACSIM_COMMIT_SHA}" && \
    ISAACSIM_HEAD="$(git -C "${ISAACSIM_PATH_SRC}" rev-parse HEAD)" && \
    { test "${ISAACSIM_HEAD}" = "${ISAACSIM_COMMIT_SHA}" || { echo "Isaac Sim HEAD ${ISAACSIM_HEAD} does not match pinned commit ${ISAACSIM_COMMIT_SHA}" >&2; exit 65; } ; } && \
    { test -f "${ISAACSIM_PATH_SRC}/VERSION" || { echo "Isaac Sim source has no VERSION file" >&2; exit 65; } ; } && \
    ISAACSIM_SOURCE_VERSION="$(tr -d '\r\n' < "${ISAACSIM_PATH_SRC}/VERSION")" && \
    { test "${ISAACSIM_SOURCE_VERSION}" = "${ISAACSIM_VERSION}" || { echo "Isaac Sim VERSION ${ISAACSIM_SOURCE_VERSION} does not match expected ${ISAACSIM_VERSION}" >&2; exit 65; } ; } && \
    restore_compiler_alternatives() { update-alternatives --auto gcc >/dev/null 2>&1 || true; update-alternatives --auto g++ >/dev/null 2>&1 || true; } && \
    trap restore_compiler_alternatives EXIT && \
    git -C "${ISAACSIM_PATH_SRC}" lfs install && \
    srb_retry git -C "${ISAACSIM_PATH_SRC}" lfs pull && \
    update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-11 200 && \
    update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-11 200 && \
    touch "${ISAACSIM_PATH_SRC}/.eula_accepted" && \
    srb_retry "${ISAACSIM_PATH_SRC}/build.sh" --release-only --no-licensing && \
    if [ "$TARGETARCH" = "amd64" ]; then ISAAC_ARCH="x86_64"; else ISAAC_ARCH="aarch64"; fi && \
    ISAACSIM_RELEASE_PATH="${ISAACSIM_PATH_SRC}/_build/linux-${ISAAC_ARCH}/release" && \
    { test -d "${ISAACSIM_RELEASE_PATH}" || { echo "Isaac Sim release output is missing at ${ISAACSIM_RELEASE_PATH}" >&2; exit 65; } ; } && \
    for required in python.sh isaac-sim.sh VERSION kit/libcarb.so kit/kit; do test -e "${ISAACSIM_RELEASE_PATH}/${required}" || { echo "Isaac Sim release output is missing ${required}" >&2; exit 65; }; done && \
    for executable in python.sh isaac-sim.sh kit/kit; do test -x "${ISAACSIM_RELEASE_PATH}/${executable}" || { echo "Isaac Sim release output is not executable: ${executable}" >&2; exit 65; }; done && \
    ISAACSIM_BUILT_VERSION="$(tr -d '\r\n' < "${ISAACSIM_RELEASE_PATH}/VERSION")" && \
    { test "${ISAACSIM_BUILT_VERSION%%+*}" = "${ISAACSIM_VERSION}" || { echo "Isaac Sim release VERSION ${ISAACSIM_BUILT_VERSION} does not match source VERSION ${ISAACSIM_VERSION} (build metadata after + is ignored)" >&2; exit 65; } ; } && \
    cp -Lr "${ISAACSIM_PATH_SRC}/_build/linux-${ISAAC_ARCH}/release" "${ISAACSIM_PATH}" && \
    cp -f "${ISAACSIM_PATH_SRC}/VERSION" "${ISAACSIM_PATH}/VERSION" && \
    test -x "${ISAACSIM_PATH}/python.sh" && \
    test -x "${ISAACSIM_PATH}/isaac-sim.sh" && \
    test -f "${ISAACSIM_PATH}/VERSION" && \
    test -f "${ISAACSIM_PATH}/kit/libcarb.so" && \
    test "$(tr -d '\r\n' < "${ISAACSIM_PATH}/VERSION")" = "${ISAACSIM_VERSION}" && \
    printf 'source_repository=%s\nsource_branch=%s\nsource_commit=%s\nsource_version=%s\nrelease_version=%s\nbuild_platform=linux-%s\n' \
    "${ISAACSIM_REMOTE}" "${ISAACSIM_BRANCH}" "${ISAACSIM_HEAD}" "${ISAACSIM_SOURCE_VERSION}" "${ISAACSIM_RELEASE_VERSION}" "${ISAAC_ARCH}" \
    > "${ISAACSIM_PATH}/.srb-build-metadata" && \
    test -s "${ISAACSIM_PATH}/.srb-build-metadata" && \
    rm -rf "${ISAACSIM_PATH_SRC}" /root/.local/share/ov/data/exts/ /root/.cache/packman /root/.cache/ov && \
    restore_compiler_alternatives && \
    trap - EXIT && \
    mkdir -p /usr/share/glvnd/egl_vendor.d /etc/vulkan/icd.d /etc/vulkan/implicit_layer.d && \
    printf '{\n"file_format_version": "1.0.0",\n"ICD": {\n"library_path": "libEGL_nvidia.so.0"\n}\n}\n' > /usr/share/glvnd/egl_vendor.d/10_nvidia.json && \
    printf '{\n"file_format_version": "1.0.0",\n"ICD": {\n"library_path": "libEGL_mesa.so.0"\n}\n}\n' > /usr/share/glvnd/egl_vendor.d/50_mesa.json && \
    printf '{\n"file_format_version": "1.0.0",\n"ICD": {\n"library_path": "libGLX_nvidia.so.0",\n"api_version": "1.3.194"\n}\n}\n' > /etc/vulkan/icd.d/nvidia_icd.json && \
    printf '{\n"file_format_version": "1.0.0",\n"layer": {\n"name": "VK_LAYER_NV_optimus",\n"type": "INSTANCE",\n"library_path": "libGLX_nvidia.so.0",\n"api_version": "1.3.194",\n"implementation_version": "1",\n"description": "NVIDIA Optimus layer",\n"functions": {\n"vkGetInstanceProcAddr": "vk_optimusGetInstanceProcAddr",\n"vkGetDeviceProcAddr": "vk_optimusGetDeviceProcAddr"\n},\n"enable_environment": {\n"__NV_PRIME_RENDER_OFFLOAD": "1"\n},\n"disable_environment": {\n"DISABLE_LAYER_NV_OPTIMUS_1": ""\n}\n}\n}\n' > /etc/vulkan/implicit_layer.d/nvidia_layers.json && \
    printf '#!/usr/bin/env bash\n' >> /usr/local/bin/isaacsim && \
    printf 'exec "%s/isaac-sim.sh"\n' "${ISAACSIM_PATH}" >> /usr/local/bin/isaacsim && \
    chmod +x /usr/local/bin/isaacsim
ENV ISAAC_PATH="${ISAACSIM_PATH}"
ENV CARB_APP_PATH="${ISAACSIM_PATH}/kit"
ENV EXP_PATH="${ISAACSIM_PATH}/apps"
ENV ISAACSIM_PYTHON="${ISAACSIM_PATH}/python.sh"
ENV LD_PRELOAD="${ISAACSIM_PATH}/kit/libcarb.so"
ENV VK_DRIVER_FILES="/etc/vulkan/icd.d/nvidia_icd.json"
ENV OMNI_SERVER="https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/6.1"
ENV OMNI_KIT_ALLOW_ROOT="1"

# hadolint ignore=SC2016
RUN source /entrypoint.bash -- && \
    source "${ISAACSIM_PATH}/setup_python_env.sh" && \
    printf '%s\n' "${PYTHONPATH//:/$'\n'}" > "$("${PYTHONEXE}" -c "import site; print(site.getsitepackages()[0])")/isaac_sim.pth" && \
    sed -i 's|$SCRIPT_DIR/kit/kernel/plugins:||' "${ISAACSIM_PATH}/setup_python_env.sh"

## On aarch64, use Isaac Sim's matching USD build when usd-core has no wheel.
## Add its extension and library paths before process startup; a .pth file is too late.
RUN if [ "$TARGETARCH" != "amd64" ]; then \
    USD_EXT="$(find /root/isaac-sim/extscache -maxdepth 1 -type d -name 'omni.usd.libs-*' | sort | head -n 1)" && \
    { test -n "${USD_EXT}" || { echo "Isaac Sim ships no omni.usd.libs extension; cannot supply pxr on ${TARGETARCH}" >&2; exit 65; } ; } && \
    { test -d "${USD_EXT}/pxr" || { echo "${USD_EXT} carries no pxr package" >&2; exit 65; } ; } && \
    PY_LIB="$(dirname "$(find /root/.local/share/uv/python -name 'libpython3.12.so.1.0' | head -n 1)")" && \
    { test -n "${PY_LIB}" || { echo "no libpython3.12.so.1.0 found for the venv interpreter" >&2; exit 65; } ; } && \
    printf '\n# Isaac Sim bundled USD (no usd-core wheel for this architecture)\n' >> /entrypoint.bash && \
    printf "export PYTHONPATH=\"%s\${PYTHONPATH:+:\${PYTHONPATH}}\"\n" "${USD_EXT}" >> /entrypoint.bash && \
    printf "export LD_LIBRARY_PATH=\"%s/bin:%s\${LD_LIBRARY_PATH:+:\${LD_LIBRARY_PATH}}\"\n" "${USD_EXT}" "${PY_LIB}" >> /entrypoint.bash ; \
    fi

ARG OMNI_HUB_PATH="/tmp/omni_hub"
COPY  --exclude="*.md" .docker/assets/omni_hub/ "${OMNI_HUB_PATH}/"
RUN if [ "$TARGETARCH" = "amd64" ]; then HUB_ARCH="x86_64"; else HUB_ARCH="aarch64"; fi && \
    if [ -n "$(find "${OMNI_HUB_PATH}" -maxdepth 1 -name "omni_hub*${HUB_ARCH}*.zip" -print -quit)" ]; then \
    ZIP_FILE="$(find "${OMNI_HUB_PATH}" -maxdepth 1 -name "omni_hub*${HUB_ARCH}*.zip" | head -n 1)" && \
    mkdir -p "${OMNI_HUB_PATH}" && \
    unzip -q "$ZIP_FILE" -d "${OMNI_HUB_PATH}" && \
    chmod +x "${OMNI_HUB_PATH}/scripts/install.sh" && \
    "${OMNI_HUB_PATH}/scripts/install.sh" && \
    rm -rf "${OMNI_HUB_PATH}" || exit 1 ; \
    else \
    rm -rf "${OMNI_HUB_PATH}" || exit 1 ; \
    fi


ARG ROS_DISTRO="jazzy"
ARG ROS_BUILD_FROM_SRC=false
## Pin the ROS signing key and CycloneDDS forks; update key commit and digest together.
ARG ROSDISTRO_COMMIT="888f5d3a3f8a33fa8a7cf8deb86bdf8bccc3a1c1"
ARG ROS_ARCHIVE_INTEGRITY="4a91c49af0d6f0016108b93698782b596c27ccd836937e18e0e36c3347dc602f"
ARG RMW_DDS_COMMON_COMMIT_SHA="1172c3691a5c6ca5fdbd70aac2f6e765be44b4d7"
ARG RMW_CYCLONEDDS_COMMIT_SHA="d1e2c252047f5d7390f0f2942086f9806fc89de9"
ARG ROS_BUILD_CYCLONEDDS_PATCH_JAZZY=true
ARG ROS_BUILD_PACKAGES_SKIP="python_orocos_kdl_vendor qt_gui_cpp"
ENV ROS_UNDERLAY_PATH="/root/ros/${ROS_DISTRO}"
# hadolint ignore=DL3008,SC1091,SC2046
RUN --mount=type=cache,target="/root/.cache/uv" \
    if [[ "${ROS_BUILD_FROM_SRC,,}" = true ]]; then \
    source /entrypoint.bash -- && \
    curl --proto "=https" --tlsv1.2 -sSfL --retry 5 --retry-all-errors --connect-timeout 30 "https://raw.githubusercontent.com/ros/rosdistro/${ROSDISTRO_COMMIT}/ros.key" -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "${ROS_ARCHIVE_INTEGRITY}  /usr/share/keyrings/ros-archive-keyring.gpg" | sha256sum --check --strict - && \
    printf 'deb [arch=%s signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu %s main\n' "$(dpkg --print-architecture)" "$(. /etc/os-release && printf '%s' "$UBUNTU_CODENAME")" | tee /etc/apt/sources.list.d/ros2.list > /dev/null && \
    apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends ros-dev-tools && \
    rosdep init --rosdistro "${ROS_DISTRO}" && \
    rosdep update && \
    uv pip install build setuptools-scm catkin_pkg lark pytest empy && \
    mkdir -p "${ROS_UNDERLAY_PATH}/src" && \
    curl --proto "=https" --tlsv1.2 -sSfL --retry 5 --retry-all-errors --connect-timeout 30 "https://raw.githubusercontent.com/ros2/ros2/${ROS_DISTRO}/ros2.repos" -o "${ROS_UNDERLAY_PATH}/src/ros2.repos" && \
    sed -i -e 's| \([^/]*\)/\([^:]*\):| \2:|g' "${ROS_UNDERLAY_PATH}/src/ros2.repos" && \
    vcs import "${ROS_UNDERLAY_PATH}/src" < "${ROS_UNDERLAY_PATH}/src/ros2.repos" && \
    rm -rf $(find "${ROS_UNDERLAY_PATH}/src" -type d -name .git) && \
    DEBIAN_FRONTEND=noninteractive rosdep install --default-yes --ignore-src --rosdistro "${ROS_DISTRO}" --from-paths "${ROS_UNDERLAY_PATH}/src" --skip-keys "$(rosdep db 2>/dev/null | grep -i "example\|demo\|tutorial" | awk '{print $1}' | tr '\n' ' ')" --skip-keys rti-connext-dds-6.0.1 --skip-keys python3-pybind11 && \
    if [[ "${ROS_BUILD_CYCLONEDDS_PATCH_JAZZY,,}" = true && "${ROS_DISTRO}" = "jazzy" ]]; then \
    colcon build --merge-install --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DPython3_EXECUTABLE="${PYTHONEXE}" --build-base "${ROS_UNDERLAY_PATH}/build" --install-base "${ROS_UNDERLAY_PATH}/install" --packages-skip ${ROS_BUILD_PACKAGES_SKIP} --packages-skip-by-dep ${ROS_BUILD_PACKAGES_SKIP} --packages-skip-regex ".*examples.*" ".*tutorial.*" && \
    rm -rf "${ROS_UNDERLAY_PATH}/src/rmw_dds_common" "${ROS_UNDERLAY_PATH}/src/rmw_cyclonedds" && \
    git clone --filter=blob:none --no-checkout https://github.com/AndrejOrsula/rmw_dds_common.git "${ROS_UNDERLAY_PATH}/src/rmw_dds_common" && \
    git -C "${ROS_UNDERLAY_PATH}/src/rmw_dds_common" checkout --detach "${RMW_DDS_COMMON_COMMIT_SHA}" && \
    test "$(git -C "${ROS_UNDERLAY_PATH}/src/rmw_dds_common" rev-parse HEAD)" = "${RMW_DDS_COMMON_COMMIT_SHA}" && \
    git clone --filter=blob:none --no-checkout https://github.com/AndrejOrsula/rmw_cyclonedds.git "${ROS_UNDERLAY_PATH}/src/rmw_cyclonedds" && \
    git -C "${ROS_UNDERLAY_PATH}/src/rmw_cyclonedds" checkout --detach "${RMW_CYCLONEDDS_COMMIT_SHA}" && \
    test "$(git -C "${ROS_UNDERLAY_PATH}/src/rmw_cyclonedds" rev-parse HEAD)" = "${RMW_CYCLONEDDS_COMMIT_SHA}" && \
    dpkg -i "${ROS_UNDERLAY_PATH}/src/rmw_dds_common/dummy_package/ros-jazzy-rmw-dds-common_0.0.1_all.deb" && \
    colcon build --merge-install --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DPython3_EXECUTABLE="${PYTHONEXE}" --build-base "${ROS_UNDERLAY_PATH}/build" --install-base "${ROS_UNDERLAY_PATH}/install" --packages-up-to rmw_cyclonedds_cpp && \
    apt-get -y autoremove && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/* || exit 1 ; \
    else \
    colcon build --merge-install --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DPython3_EXECUTABLE="${PYTHONEXE}" --build-base "${ROS_UNDERLAY_PATH}/build" --install-base "${ROS_UNDERLAY_PATH}/install" --packages-skip ${ROS_BUILD_PACKAGES_SKIP} --packages-skip-by-dep ${ROS_BUILD_PACKAGES_SKIP} --packages-skip-regex ".*examples.*" ".*tutorial.*" && \
    apt-get -y autoremove && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/* || exit 1 ; \
    fi && \
    printf '\n# ROS %s (built from source)\n' "${ROS_DISTRO^}" >> /entrypoint.bash && \
    printf 'source "%s/install/setup.bash" --\n' "${ROS_UNDERLAY_PATH}" >> /entrypoint.bash && \
    apt-get -y autoremove && \
    apt-get clean && \
    rm -rf "${ROS_UNDERLAY_PATH}/src/ros2.repos" /var/lib/apt/lists/* /root/.ros/rosdep/sources.cache ./log || exit 1 ; \
    \
    else \
    \
    ## Install ROS from binaries
    source /entrypoint.bash -- && \
    curl --proto "=https" --tlsv1.2 -sSfL --retry 5 --retry-all-errors --connect-timeout 30 "https://raw.githubusercontent.com/ros/rosdistro/${ROSDISTRO_COMMIT}/ros.key" -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "${ROS_ARCHIVE_INTEGRITY}  /usr/share/keyrings/ros-archive-keyring.gpg" | sha256sum --check --strict - && \
    printf 'deb [arch=%s signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu %s main\n' "$(dpkg --print-architecture)" "$(source /etc/os-release && printf '%s' "${UBUNTU_CODENAME}")" > /etc/apt/sources.list.d/ros2.list && \
    apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install -yq --no-install-recommends \
    ros-dev-tools \
    "ros-${ROS_DISTRO}-ros-base" \
    "ros-${ROS_DISTRO}-rmw-cyclonedds-cpp" \
    "ros-${ROS_DISTRO}-rmw-fastrtps-cpp" \
    && \
    apt-get -y autoremove && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/* && \
    uv pip install "catkin_pkg==1.1.1" && \
    rosdep init --rosdistro "${ROS_DISTRO}" && \
    printf '\n# ROS %s\n' "${ROS_DISTRO^}" >> /entrypoint.bash && \
    printf 'source "/opt/ros/%s/setup.bash" --\n' "${ROS_DISTRO}" >> /entrypoint.bash || exit 1 ; \
    fi


ARG BLENDER_PATH="/root/blender"
ARG BLENDER_VERSION="4.5.8"
ARG BLENDER_PYTHON="${BLENDER_PATH}/${BLENDER_VERSION%.*}/python/bin/python3.11"
## Blender 4.5.8 has no aarch64 archive, so that target builds the same tag from source.
## The checksum applies to the amd64 archive.
ARG BLENDER_SHA256_AMD64="8cc3997ca2148a43187ca625f150b41bd3ef7c2991988725a34b46cbf25ba82f"
ARG BLENDER_REMOTE="https://projects.blender.org/blender/blender.git"
ARG BLENDER_ARM64_LIB_REMOTE="https://projects.blender.org/blender/lib-linux_arm64.git"
## Pin the moving Blender library branch for reproducible aarch64 builds.
## Update deliberately when SimForge needs different Blender libraries.
ARG BLENDER_ARM64_LIB_COMMIT="f854e1198a793e2c4107612bfb16023f6259c584"
# hadolint ignore=DL3008
RUN if [ "$TARGETARCH" = "amd64" ]; then \
    test -n "${BLENDER_SHA256_AMD64}" || { echo "No Blender ${BLENDER_VERSION} digest is pinned for ${TARGETARCH}; set BLENDER_SHA256_AMD64" >&2; exit 64; } && \
    curl --proto "=https" --tlsv1.2 -sSfL --retry 5 --retry-all-errors --connect-timeout 30 "https://download.blender.org/release/Blender${BLENDER_VERSION%.*}/blender-${BLENDER_VERSION}-linux-x64.tar.xz" -o "/tmp/blender_${BLENDER_VERSION}.tar.xz" && \
    echo "${BLENDER_SHA256_AMD64}  /tmp/blender_${BLENDER_VERSION}.tar.xz" | sha256sum --check --strict - && \
    mkdir -p "${BLENDER_PATH}" && \
    tar xf "/tmp/blender_${BLENDER_VERSION}.tar.xz" -C "${BLENDER_PATH}" --strip-components=1 && \
    rm "/tmp/blender_${BLENDER_VERSION}.tar.xz" && \
    ln -sf "${BLENDER_PATH}/blender" /usr/local/bin/blender ; \
    else \
    echo "[WARNING] Blender ${BLENDER_VERSION} publishes no ${TARGETARCH} archive; building it from the v${BLENDER_VERSION} tag instead. This is the long stage of an arm64 build." >&2 && \
    apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
    git-lfs libx11-dev libxxf86vm-dev libxcursor-dev libxi-dev libxrandr-dev \
    libxinerama-dev libegl-dev libwayland-dev wayland-protocols \
    libxkbcommon-dev libdbus-1-dev libvulkan-dev libssl-dev zlib1g-dev && \
    git clone "${BLENDER_REMOTE}" /tmp/blender-src --branch "v${BLENDER_VERSION}" --depth 1 && \
    git clone "${BLENDER_ARM64_LIB_REMOTE}" /tmp/blender-src/lib/linux_arm64 \
    --branch "blender-v${BLENDER_VERSION%.*}-release" --depth 1 && \
    if [ -n "${BLENDER_ARM64_LIB_COMMIT}" ]; then \
    [ "${#BLENDER_ARM64_LIB_COMMIT}" = 40 ] || { echo "BLENDER_ARM64_LIB_COMMIT must be a 40-character commit" >&2; exit 64; } && \
    { git -C /tmp/blender-src/lib/linux_arm64 fetch --depth 1 origin "${BLENDER_ARM64_LIB_COMMIT}" || \
    { echo "[WARNING] the server refused a shallow fetch by commit; deepening the clone instead" >&2 && \
    git -C /tmp/blender-src/lib/linux_arm64 fetch --unshallow ; } ; } && \
    git -C /tmp/blender-src/lib/linux_arm64 checkout --detach "${BLENDER_ARM64_LIB_COMMIT}" && \
    test "$(git -C /tmp/blender-src/lib/linux_arm64 rev-parse HEAD)" = "${BLENDER_ARM64_LIB_COMMIT}" ; \
    fi && \
    make -C /tmp/blender-src release BUILD_DIR=/tmp/blender-build && \
    { test -x /tmp/blender-build/bin/blender || { echo "Blender source build produced no binary" >&2; exit 65; } ; } && \
    mkdir -p "${BLENDER_PATH}" && \
    cp -r /tmp/blender-build/bin/. "${BLENDER_PATH}/" && \
    rm -rf /tmp/blender-src /tmp/blender-build && \
    apt-get -y autoremove && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/* && \
    ## Clear Isaac Sim loader variables so Blender uses its own libraries.
    ## SimForge invokes Blender by name from a Python subprocess, so use a wrapper.
    printf '#!/usr/bin/env bash\nunset LD_LIBRARY_PATH LD_PRELOAD PYTHONPATH\nexec "%s/blender" "$@"\n' "${BLENDER_PATH}" > /usr/local/bin/blender && \
    chmod +x /usr/local/bin/blender && \
    "${BLENDER_PYTHON}" --version && \
    /usr/local/bin/blender --version | head -n 1 ; \
    fi


ARG ISAACLAB_PATH="/root/isaaclab"
ARG ISAACLAB_REMOTE="https://github.com/isaac-sim/IsaacLab.git"
ARG ISAACLAB_BRANCH="release/3.0.0"
ARG ISAACLAB_COMMIT_SHA="fdc650c88dfd86aa158b9a1760b6537fef360555" # release/3.0.0, VERSION 3.0.0
ARG ISAACLAB_VERSION="3.0.0"
ARG ISAACLAB_ISAACSIM_VERSION="6.1.0.0"
LABEL org.space-robotics-bench.isaacsim.branch="${ISAACSIM_BRANCH}" \
    org.space-robotics-bench.isaacsim.commit="${ISAACSIM_COMMIT_SHA}" \
    org.space-robotics-bench.isaacsim.source-version="${ISAACSIM_VERSION}" \
    org.space-robotics-bench.isaacsim.release-version="${ISAACSIM_RELEASE_VERSION}" \
    org.space-robotics-bench.isaaclab.branch="${ISAACLAB_BRANCH}" \
    org.space-robotics-bench.isaaclab.commit="${ISAACLAB_COMMIT_SHA}" \
    org.space-robotics-bench.isaaclab.source-version="${ISAACLAB_VERSION}" \
    org.space-robotics-bench.isaaclab.isaacsim-version="${ISAACLAB_ISAACSIM_VERSION}"
## Isaac Sim prebundles multiple attrs copies; a newer venv copy can conflict at runtime.
## Constrain later uv installs to the version in omni.kit.pip_archive/pip_prebundle.
ENV UV_CONSTRAINT="/root/.uv-constraints.txt"
# hadolint ignore=SC2016
RUN version="$(find "${ISAACSIM_PATH}/extscache" -maxdepth 3 -type d -path '*/omni.kit.pip_archive-*/pip_prebundle/attrs-*.dist-info' | sed -E 's|.*/attrs-([0-9.]+)\.dist-info|\1|' | sort -V | head -n 1)" && \
    test -n "${version}" && \
    printf 'attrs==%s\n' "${version}" > "${UV_CONSTRAINT}" && \
    cat "${UV_CONSTRAINT}"

# Isaac Lab extensions declare two NVIDIA-internal dependencies unavailable on PyPI.
# Mask only those dependencies; runtime uses Isaac Sim bundles where applicable.
# Install the Isaac Lab root pins separately; the editable extensions declare none.
ARG ISAACLAB_ROOT_REQUIREMENTS_SCRIPT="/tmp/isaaclab_root_requirements.py"
# hadolint ignore=SC2044
RUN --mount=type=cache,target="/root/.cache/uv" \
    --mount=type=bind,source=".docker/helpers/isaaclab_root_requirements.py",target="${ISAACLAB_ROOT_REQUIREMENTS_SCRIPT}" \
    source /entrypoint.bash -- && \
    { [[ "${ISAACLAB_REMOTE}" = "https://github.com/isaac-sim/IsaacLab.git" ]] || { echo "ISAACLAB_REMOTE is not the reviewed upstream repository" >&2; exit 64; } ; } && \
    { [[ "${ISAACLAB_BRANCH}" = "release/3.0.0" ]] || { echo "ISAACLAB_BRANCH must be release/3.0.0 for the promoted source" >&2; exit 64; } ; } && \
    { [[ "${ISAACLAB_COMMIT_SHA}" =~ ^[0-9a-f]{40}$ ]] || { echo "ISAACLAB_COMMIT_SHA must be a full 40-character lowercase hex commit" >&2; exit 64; } ; } && \
    { [[ "${ISAACLAB_COMMIT_SHA}" = "fdc650c88dfd86aa158b9a1760b6537fef360555" ]] || { echo "ISAACLAB_COMMIT_SHA must match the reviewed Isaac Lab source pin" >&2; exit 64; } ; } && \
    { [[ "${ISAACLAB_VERSION}" = "3.0.0" ]] || { echo "ISAACLAB_VERSION must be 3.0.0 for the pinned source" >&2; exit 64; } ; } && \
    { [[ "${ISAACLAB_ISAACSIM_VERSION}" = "6.1.0.0" ]] || { echo "ISAACLAB_ISAACSIM_VERSION must be 6.1.0.0 for the pinned source" >&2; exit 64; } ; } && \
    git clone "${ISAACLAB_REMOTE}" "${ISAACLAB_PATH}" --branch "${ISAACLAB_BRANCH}" && \
    git -C "${ISAACLAB_PATH}" reset --hard "${ISAACLAB_COMMIT_SHA}" && \
    ISAACLAB_HEAD="$(git -C "${ISAACLAB_PATH}" rev-parse HEAD)" && \
    { test "${ISAACLAB_HEAD}" = "${ISAACLAB_COMMIT_SHA}" || { echo "Isaac Lab HEAD ${ISAACLAB_HEAD} does not match pinned commit ${ISAACLAB_COMMIT_SHA}" >&2; exit 65; } ; } && \
    { test -f "${ISAACLAB_PATH}/VERSION" || { echo "Isaac Lab source has no VERSION file" >&2; exit 65; } ; } && \
    ISAACLAB_SOURCE_VERSION="$(tr -d '\r\n' < "${ISAACLAB_PATH}/VERSION")" && \
    { test "${ISAACLAB_SOURCE_VERSION}" = "${ISAACLAB_VERSION}" || { echo "Isaac Lab VERSION ${ISAACLAB_SOURCE_VERSION} does not match expected ${ISAACLAB_VERSION}" >&2; exit 65; } ; } && \
    test -x "${ISAACLAB_PATH}/isaaclab.sh" && \
    { grep -qF "isaacsim = \"${ISAACLAB_ISAACSIM_VERSION}\"" "${ISAACLAB_PATH}/pyproject.toml" || { echo "Isaac Lab manifest does not target Isaac Sim ${ISAACLAB_ISAACSIM_VERSION}" >&2; exit 65; } ; } && \
    printf 'source_repository=%s\nsource_branch=%s\nsource_commit=%s\nsource_version=%s\nisaacsim_manifest_version=%s\n' \
    "${ISAACLAB_REMOTE}" "${ISAACLAB_BRANCH}" "${ISAACLAB_HEAD}" "${ISAACLAB_SOURCE_VERSION}" "${ISAACLAB_ISAACSIM_VERSION}" \
    > "${ISAACLAB_PATH}/.srb-build-metadata" && \
    test -s "${ISAACLAB_PATH}/.srb-build-metadata" && \
    ln -sf "${ISAACSIM_PATH}" "${ISAACLAB_PATH}/_isaac_sim" && \
    mkdir -p /tmp/isaaclab_overrides && \
    printf 'omniverseclient ; python_version < "0"\nisaacteleop ; python_version < "0"\n' \
    > /tmp/isaaclab_overrides/overrides.txt && \
    uv pip install "lazy_loader==0.5" && \
    for extension in $(find -L "${ISAACLAB_PATH}/source" -mindepth 1 -maxdepth 1 -type d | sort); do \
    if [ -f "${extension}/pyproject.toml" ] || [ -f "${extension}/setup.py" ]; then \
    uv pip install --overrides /tmp/isaaclab_overrides/overrides.txt --editable "${extension}" || exit 1 ; \
    fi ; \
    done && \
    python "${ISAACLAB_ROOT_REQUIREMENTS_SCRIPT}" requirements "${ISAACLAB_PATH}/pyproject.toml" > /tmp/isaaclab_overrides/root-requirements.txt && \
    python "${ISAACLAB_ROOT_REQUIREMENTS_SCRIPT}" overrides "${ISAACLAB_PATH}/pyproject.toml" > /tmp/isaaclab_overrides/root-overrides.txt && \
    uv pip install --overrides /tmp/isaaclab_overrides/root-overrides.txt -r /tmp/isaaclab_overrides/root-requirements.txt && \
    python -c "import newton, prettytable, warp; print('warp', warp.__version__, 'newton', newton.__version__)" && \
    rm -rf /tmp/isaaclab_overrides && \
    ## Use the installed `isaaclab` entrypoint; `isaaclab.sh` is deprecated for 3.1.
    { test -x "${VENV_PATH}/bin/isaaclab" || { echo "Isaac Lab did not install its isaaclab console script" >&2; exit 66; } ; }


ARG OXIDASIM_VERSION="0.1.0"
ARG OXIDASIM_DEV=false
ARG OXIDASIM_PATH="/root/oxidasim"
ARG OXIDASIM_REMOTE="https://github.com/AndrejOrsula/oxidasim.git"
ARG OXIDASIM_COMMIT_SHA=""
# hadolint ignore=SC1091
RUN --mount=type=cache,target="/root/.cache/uv" \
    source /entrypoint.bash -- && \
    if [[ "${OXIDASIM_DEV,,}" = true ]]; then \
    [[ "${OXIDASIM_COMMIT_SHA}" =~ ^[0-9a-f]{40}$ ]] || \
    { echo "OXIDASIM_DEV requires OXIDASIM_COMMIT_SHA as 40 lowercase hex characters" >&2; exit 64; } ; \
    git clone --filter=blob:none --no-checkout "${OXIDASIM_REMOTE}" "${OXIDASIM_PATH}" && \
    git -C "${OXIDASIM_PATH}" checkout --detach "${OXIDASIM_COMMIT_SHA}" && \
    test "$(git -C "${OXIDASIM_PATH}" rev-parse HEAD)" = "${OXIDASIM_COMMIT_SHA}" && \
    uv pip install --editable "${OXIDASIM_PATH}[all]" || exit 1 ; \
    else \
    uv pip install "oxidasim[all]==${OXIDASIM_VERSION}" && \
    test "$(python -c 'from importlib.metadata import version; print(version("oxidasim"))')" = "${OXIDASIM_VERSION}" || exit 1 ; \
    fi

ARG SIMFORGE_VERSION="0.4.0"
ARG SIMFORGE_DEV=false
ARG SIMFORGE_PATH="/root/simforge"
ARG SIMFORGE_REMOTE="https://github.com/AndrejOrsula/simforge.git"
ARG SIMFORGE_COMMIT_SHA=""
ARG SIMFORGE_PATCH_SCRIPT="/tmp/patch_simforge_isaaclab.bash"
# Bind-mount this patch: project source is copied only in a later stage.
# hadolint ignore=SC1091
RUN --mount=type=cache,target="/root/.cache/uv" \
    --mount=type=bind,source=".docker/install/patch_simforge_isaaclab.bash",target="${SIMFORGE_PATCH_SCRIPT}" \
    source /entrypoint.bash -- && \
    if [[ "${SIMFORGE_DEV,,}" = true ]]; then \
    [[ "${SIMFORGE_COMMIT_SHA}" =~ ^[0-9a-f]{40}$ ]] || \
    { echo "SIMFORGE_DEV requires SIMFORGE_COMMIT_SHA as 40 lowercase hex characters" >&2; exit 64; } ; \
    git clone --filter=blob:none --no-checkout "${SIMFORGE_REMOTE}" "${SIMFORGE_PATH}" && \
    git -C "${SIMFORGE_PATH}" checkout --detach "${SIMFORGE_COMMIT_SHA}" && \
    test "$(git -C "${SIMFORGE_PATH}" rev-parse HEAD)" = "${SIMFORGE_COMMIT_SHA}" && \
    bash "${SIMFORGE_PATCH_SCRIPT}" "${SIMFORGE_PATH}" && \
    uv pip install --editable "${SIMFORGE_PATH}[assets,cli,dev]" && \
    uv pip install --python "${BLENDER_PYTHON}" --editable "${SIMFORGE_PATH}[dev]" || exit 1 ; \
    else \
    uv pip install "simforge[assets,cli,dev]==${SIMFORGE_VERSION}" && \
    test "$(python -c 'from importlib.metadata import version; print(version("simforge"))')" = "${SIMFORGE_VERSION}" && \
    SIMFORGE_SITE="$(python -c 'import importlib.util, pathlib; print(pathlib.Path(importlib.util.find_spec("simforge").origin).parent)')" && \
    bash "${SIMFORGE_PATCH_SCRIPT}" "${SIMFORGE_SITE}" && \
    uv pip install --python "${BLENDER_PYTHON}" "simforge[dev]==${SIMFORGE_VERSION}" && \
    test "$("${BLENDER_PYTHON}" -c 'from importlib.metadata import version; print(version("simforge"))')" = "${SIMFORGE_VERSION}" && \
    SIMFORGE_BLENDER_SITE="$("${BLENDER_PYTHON}" -c 'import importlib.util, pathlib; print(pathlib.Path(importlib.util.find_spec("simforge").origin).parent)')" && \
    bash "${SIMFORGE_PATCH_SCRIPT}" "${SIMFORGE_BLENDER_SITE}" || exit 1 ; \
    fi

ARG SIMFORGE_FOUNDRY_VERSION="0.3.1"
ARG SIMFORGE_FOUNDRY_DEV=false
ARG SIMFORGE_FOUNDRY_PATH="/root/simforge_foundry"
ARG SIMFORGE_FOUNDRY_REMOTE="https://github.com/AndrejOrsula/simforge_foundry.git"
ARG SIMFORGE_FOUNDRY_COMMIT_SHA=""
# hadolint ignore=SC1091
RUN --mount=type=cache,target="/root/.cache/uv" \
    source /entrypoint.bash -- && \
    if [[ "${SIMFORGE_FOUNDRY_DEV,,}" = true ]]; then \
    [[ "${SIMFORGE_FOUNDRY_COMMIT_SHA}" =~ ^[0-9a-f]{40}$ ]] || \
    { echo "SIMFORGE_FOUNDRY_DEV requires SIMFORGE_FOUNDRY_COMMIT_SHA as 40 lowercase hex characters" >&2; exit 64; } ; \
    git clone --filter=blob:none --no-checkout "${SIMFORGE_FOUNDRY_REMOTE}" "${SIMFORGE_FOUNDRY_PATH}" && \
    git -C "${SIMFORGE_FOUNDRY_PATH}" checkout --detach "${SIMFORGE_FOUNDRY_COMMIT_SHA}" && \
    test "$(git -C "${SIMFORGE_FOUNDRY_PATH}" rev-parse HEAD)" = "${SIMFORGE_FOUNDRY_COMMIT_SHA}" && \
    uv pip install --editable "${SIMFORGE_FOUNDRY_PATH}" && \
    uv pip install --python "${BLENDER_PYTHON}" --editable "${SIMFORGE_FOUNDRY_PATH}" || exit 1 ; \
    else \
    uv pip install "simforge-foundry==${SIMFORGE_FOUNDRY_VERSION}" && \
    test "$(python -c 'from importlib.metadata import version; print(version("simforge-foundry"))')" = "${SIMFORGE_FOUNDRY_VERSION}" && \
    uv pip install --python "${BLENDER_PYTHON}" "simforge-foundry==${SIMFORGE_FOUNDRY_VERSION}" && \
    test "$("${BLENDER_PYTHON}" -c 'from importlib.metadata import version; print(version("simforge-foundry"))')" = "${SIMFORGE_FOUNDRY_VERSION}" || exit 1 ; \
    fi


ARG DREAMER_DEV=true
ARG DREAMER_PATH="/root/dreamerv3"
ARG DREAMER_REMOTE="https://github.com/AndrejOrsula/dreamerv3.git"
ARG DREAMER_BRANCH="main"
ARG DREAMER_COMMIT_SHA="4049794d4135e41c691f18da38a9af7541b01553"
# hadolint ignore=SC1091
RUN --mount=type=cache,target="/root/.cache/uv" \
    if [[ "${DREAMER_DEV,,}" = true ]]; then \
    source /entrypoint.bash -- && \
    git clone "${DREAMER_REMOTE}" "${DREAMER_PATH}" --branch "${DREAMER_BRANCH}" && \
    if [ -n "${DREAMER_COMMIT_SHA}" ]; then \
    git -C "${DREAMER_PATH}" reset --hard "${DREAMER_COMMIT_SHA}" || exit 1 ; \
    fi && \
    if [ "$TARGETARCH" = "amd64" ]; then \
    uv pip install --editable "${DREAMER_PATH}"; \
    else \
    echo "[WARNING] On ${TARGETARCH}, DreamerV3's Atari-only pins have no wheels; installing it without them. Every non-Atari environment, and JAX on the GPU, are unaffected." >&2 && \
    mkdir -p /tmp/dreamer_overrides && \
    { echo 'ale-py ; platform_machine == "x86_64"'; \
    echo 'nvidia-cuda-nvcc-cu12>=12.3.107'; } > /tmp/dreamer_overrides/overrides.txt && \
    uv pip install --overrides /tmp/dreamer_overrides/overrides.txt --editable "${DREAMER_PATH}" && \
    python -c 'import dreamerv3, embodied; import jax; print("[INFO] dreamerv3 imports; jax", jax.__version__, "backend", jax.default_backend())'; \
    fi || exit 1 ; \
    fi

ARG SKRL_DEV=false
ARG SKRL_PATH="/root/skrl"
ARG SKRL_REMOTE="https://github.com/Toni-SM/skrl.git"
ARG SKRL_BRANCH="develop"
ARG SKRL_COMMIT_SHA="5a078cff1a611d51eb2164101cdd6fa84e3eec2d"
# hadolint ignore=SC1091
RUN --mount=type=cache,target="/root/.cache/uv" \
    if [[ "${SKRL_DEV,,}" = true ]]; then \
    source /entrypoint.bash -- && \
    git clone "${SKRL_REMOTE}" "${SKRL_PATH}" --branch "${SKRL_BRANCH}" && \
    if [ -n "${SKRL_COMMIT_SHA}" ]; then \
    git -C "${SKRL_PATH}" reset --hard "${SKRL_COMMIT_SHA}" || exit 1 ; \
    fi && \
    uv pip install --editable "${SKRL_PATH}" || exit 1 ; \
    fi

ARG TDMPC2_DEV=true
ARG TDMPC2_PATH="/root/tdmpc2"
ARG TDMPC2_REMOTE="https://github.com/nicklashansen/tdmpc2.git"
ARG TDMPC2_BRANCH="main"
ARG TDMPC2_COMMIT_SHA="8bbc14ebabdb32ea7ada5c801dc525d0dc73bafe"
COPY srb/integrations/tdmpc2/patches/0001-fix-q-ensemble-weight-init.patch /tmp/tdmpc2-q-init.patch
# hadolint ignore=SC1091
RUN --mount=type=cache,target="/root/.cache/uv" \
    if [[ "${TDMPC2_DEV,,}" = true ]]; then \
    source /entrypoint.bash -- && \
    git clone "${TDMPC2_REMOTE}" "${TDMPC2_PATH}" --branch "${TDMPC2_BRANCH}" && \
    if [ -n "${TDMPC2_COMMIT_SHA}" ]; then \
    git -C "${TDMPC2_PATH}" reset --hard "${TDMPC2_COMMIT_SHA}" || exit 1 ; \
    fi || exit 1 ; \
    GIT_COMMITTER_NAME="SRB Maintainers" \
    GIT_COMMITTER_EMAIL="srb-maintainers@localhost" \
    git -C "${TDMPC2_PATH}" -c commit.gpgsign=false \
    am --ignore-whitespace --committer-date-is-author-date /tmp/tdmpc2-q-init.patch || exit 1 ; \
    test "$(git -C "${TDMPC2_PATH}" rev-parse HEAD)" = \
    "94c4d503fe7924b3f6860dd962bbc47d09382a68" || exit 1 ; \
    fi
ENV TDMPC2_PATH="${TDMPC2_PATH}/tdmpc2"

ARG ROBOMIMIC_DEV=true
ARG ROBOMIMIC_PATH="/root/robomimic"
ARG ROBOMIMIC_REMOTE="https://github.com/ARISE-Initiative/robomimic.git"
ARG ROBOMIMIC_BRANCH="master"
ARG ROBOMIMIC_COMMIT_SHA="e10526b9a40c78b41f1e37e60041dc0ec0a5f60f"
# Whitespace hooks normalize patch context; exact revisions still bind source bytes.
COPY srb/integrations/robomimic/patches/0001-lazy-language-provider.patch /tmp/robomimic-language-provider.patch
# hadolint ignore=SC1091
RUN --mount=type=cache,target="/root/.cache/uv" \
    if [[ "${ROBOMIMIC_DEV,,}" = true ]]; then \
    source /entrypoint.bash -- && \
    git clone "${ROBOMIMIC_REMOTE}" "${ROBOMIMIC_PATH}" --branch "${ROBOMIMIC_BRANCH}" && \
    if [ -n "${ROBOMIMIC_COMMIT_SHA}" ]; then \
    git -C "${ROBOMIMIC_PATH}" reset --hard "${ROBOMIMIC_COMMIT_SHA}" || exit 1 ; \
    fi && \
    test "$(git -C "${ROBOMIMIC_PATH}" rev-parse HEAD)" = \
    "e10526b9a40c78b41f1e37e60041dc0ec0a5f60f" && \
    GIT_COMMITTER_NAME="Space Robotics Bench" \
    GIT_COMMITTER_EMAIL="srb@localhost" \
    git -C "${ROBOMIMIC_PATH}" -c commit.gpgsign=false \
    am -C0 --committer-date-is-author-date /tmp/robomimic-language-provider.patch && \
    test "$(git -C "${ROBOMIMIC_PATH}" rev-parse HEAD)" = \
    "9e88eb3288079c6014f912e8d805c76dcd40157a" && \
    uv pip install --editable "${ROBOMIMIC_PATH}" || exit 1 ; \
    fi

# Keep the lock-pinned rsl-rl-lib release unless opting in to the editable patch.
ARG RSL_RL_DEV=false
ARG RSL_RL_PATH="/root/rsl_rl"
ARG RSL_RL_REMOTE="https://github.com/leggedrobotics/rsl_rl.git"
ARG RSL_RL_BRANCH="main"
ARG RSL_RL_COMMIT_SHA="c7beb6f02f71abfbbd9cd52d37bbe8377837b406" # main @ 2026-03-24
# hadolint ignore=SC1091
RUN --mount=type=cache,target="/root/.cache/uv" \
    if [[ "${RSL_RL_DEV,,}" = true ]]; then \
    source /entrypoint.bash -- && \
    git clone "${RSL_RL_REMOTE}" "${RSL_RL_PATH}" --branch "${RSL_RL_BRANCH}" && \
    if [ -n "${RSL_RL_COMMIT_SHA}" ]; then \
    git -C "${RSL_RL_PATH}" reset --hard "${RSL_RL_COMMIT_SHA}" || exit 1 ; \
    fi && \
    uv pip install --editable "${RSL_RL_PATH}" || exit 1 ; \
    fi


## Insert required environment setup before the `# Execute command` marker.
## Executed entrypoints stop at `exec`; sourced entrypoints continue past it.
# hadolint ignore=SC2016
RUN printf '\n# Execute command\n' >> /entrypoint.bash && \
    printf 'exec "${@}"\n' >> /entrypoint.bash && \
    sed -i '$a source /entrypoint.bash --' /root/.bashrc
ENTRYPOINT ["/entrypoint.bash"]


ENV SRB_PATH="/root/ws"
WORKDIR "${SRB_PATH}"

# hadolint ignore=SC1091
RUN --mount=type=bind,source="package.xml",target="${SRB_PATH}/package.xml" \
    source /entrypoint.bash -- && \
    apt-get update && \
    rosdep update --rosdistro "${ROS_DISTRO}" && \
    DEBIAN_FRONTEND=noninteractive rosdep install --default-yes --ignore-src --rosdistro "${ROS_DISTRO}" --from-paths "${SRB_PATH}" && \
    apt-get -y autoremove && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/* /root/.ros/rosdep/sources.cache

## hidapi arrives with Isaac Sim, but its Python package needs the system library.
# hadolint ignore=DL3008
RUN apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
    libhidapi-dev && \
    apt-get -y autoremove && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*



# hadolint ignore=SC2016
COPY --exclude=".docker/" . "${SRB_PATH}"

ARG BUILD_RUST=false
# hadolint ignore=SC1091
RUN if [[ "${BUILD_RUST,,}" = true ]]; then \
    source /entrypoint.bash -- && \
    cargo build --release --workspace --all-targets --all-features || exit 1 ; \
    fi

## Source the ROS prefix produced by either the source or binary install.
## Fail if no overlay is found; otherwise `ros2 run srb main` cannot find the package.
ARG ROS_WS="${ROS_UNDERLAY_PATH}/ws"
# hadolint ignore=SC1091
RUN source /entrypoint.bash -- && \
    colcon build --merge-install --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DPython3_EXECUTABLE="${PYTHONEXE}" --paths "${SRB_PATH}" --build-base "${ROS_WS}/build" --install-base "${ROS_WS}/install" && \
    rm -rf ./log && \
    # Replace per-file links with one directory link so mounted modules stay visible.
    find "${ROS_WS}/install/lib" -mindepth 3 -maxdepth 3 -type d -path '*/site-packages/srb' -exec sh -c 'rm -rf "$1" && ln -s "$2" "$1"' _ {} "${SRB_PATH}/srb" \; && \
    test -L "$(find "${ROS_WS}/install/lib" -mindepth 3 -maxdepth 3 -path '*/site-packages/srb' | head -n 1)" && \
    sed -i "s|source \"${ROS_UNDERLAY_PATH}/setup.bash\" --|source \"${ROS_WS}/install/setup.bash\" --|" /entrypoint.bash && \
    sed -i "s|source \"${ROS_UNDERLAY_PATH}/install/setup.bash\" --|source \"${ROS_WS}/install/setup.bash\" --|" /entrypoint.bash && \
    sed -i "s|source \"/opt/ros/${ROS_DISTRO}/setup.bash\" --|source \"${ROS_WS}/install/setup.bash\" --|" /entrypoint.bash && \
    grep -qF "source \"${ROS_WS}/install/setup.bash\" --" /entrypoint.bash

## Install the committed Python lock without resolving it; pinned source checkouts
## above remain explicit exceptions. Install the project with `--no-deps` afterward.
## `SOURCE_PACKAGE_FLAGS` omits only distributions already installed by `*_DEV` stages.
## Set the build arg to use the pinned v2e base and reviewed local iterator patch.
ARG SRB_EVENT_V2E_STREAMING=false
# hadolint ignore=SC1091
RUN --mount=type=cache,target="/root/.cache/uv" \
    source /entrypoint.bash -- && \
    SOURCE_PACKAGE_FLAGS=() && \
    if [[ "${OXIDASIM_DEV,,}" = true ]]; then SOURCE_PACKAGE_FLAGS+=(--no-emit-package oxidasim); fi && \
    if [[ "${SIMFORGE_DEV,,}" = true ]]; then SOURCE_PACKAGE_FLAGS+=(--no-emit-package simforge); fi && \
    if [[ "${SIMFORGE_FOUNDRY_DEV,,}" = true ]]; then SOURCE_PACKAGE_FLAGS+=(--no-emit-package simforge-foundry); fi && \
    if [[ "${SKRL_DEV,,}" = true ]]; then SOURCE_PACKAGE_FLAGS+=(--no-emit-package skrl); fi && \
    if [[ "${ROBOMIMIC_DEV,,}" = true ]]; then SOURCE_PACKAGE_FLAGS+=(--no-emit-package robomimic); fi && \
    if [[ "${RSL_RL_DEV,,}" = true ]]; then SOURCE_PACKAGE_FLAGS+=(--no-emit-package rsl-rl-lib); fi && \
    if [ "$TARGETARCH" != "amd64" ]; then \
    echo "[WARNING] usd-core publishes no ${TARGETARCH} wheel; using Isaac Sim's bundled USD instead." >&2 && \
    SOURCE_PACKAGE_FLAGS+=(--no-emit-package usd-core) && \
    ## Install CUDA torch first so the CPU-only aarch64 PyPI wheel is never selected.
    TORCH_VERSION="$(sed -n 's/^[[:space:]]*"torch==\([0-9][^"]*\)".*/\1/p' "${SRB_PATH}/pyproject.toml" | head -n 1)" && \
    { [ -n "${TORCH_VERSION}" ] || { echo "could not read the torch pin from pyproject.toml; the arm64 pre-install cannot proceed" >&2; exit 70; } ; } && \
    echo "[INFO] pre-installing torch ${TORCH_VERSION}+cu130 for ${TARGETARCH}" >&2 && \
    uv pip install --python "${PYTHONEXE}" --index-url https://download.pytorch.org/whl/cu130 "torch==${TORCH_VERSION}+cu130"; \
    fi && \
    uv export --frozen --all-extras --group image-runtime --group vcs-sources --no-emit-project "${SOURCE_PACKAGE_FLAGS[@]}" \
    --format requirements.txt --output-file /tmp/srb-requirements.txt && \
    uv pip install --python "${PYTHONEXE}" --requirements /tmp/srb-requirements.txt && \
    uv pip install --python "${PYTHONEXE}" --no-deps --editable "${SRB_PATH}" && \
    if [[ "${SRB_EVENT_V2E_STREAMING,,}" = true ]]; then \
    "${PYTHONEXE}" -m srb.core.sensor.event_camera.backends.v2e_install --apply; \
    fi && \
    if [[ "${OXIDASIM_DEV,,}" = true ]]; then uv pip install --python "${PYTHONEXE}" --no-deps --editable "${OXIDASIM_PATH}"; fi && \
    if [[ "${SIMFORGE_DEV,,}" = true ]]; then \
    uv pip install --python "${PYTHONEXE}" --no-deps --editable "${SIMFORGE_PATH}" && \
    uv pip install --python "${BLENDER_PYTHON}" --no-deps --editable "${SIMFORGE_PATH}"; \
    fi && \
    if [[ "${SIMFORGE_FOUNDRY_DEV,,}" = true ]]; then \
    uv pip install --python "${PYTHONEXE}" --no-deps --editable "${SIMFORGE_FOUNDRY_PATH}" && \
    uv pip install --python "${BLENDER_PYTHON}" --no-deps --editable "${SIMFORGE_FOUNDRY_PATH}"; \
    fi && \
    if [[ "${DREAMER_DEV,,}" = true ]]; then uv pip install --python "${PYTHONEXE}" --no-deps --editable "${DREAMER_PATH}"; fi && \
    if [[ "${SKRL_DEV,,}" = true ]]; then uv pip install --python "${PYTHONEXE}" --no-deps --editable "${SKRL_PATH}"; fi && \
    if [[ "${ROBOMIMIC_DEV,,}" = true ]]; then uv pip install --python "${PYTHONEXE}" --no-deps --editable "${ROBOMIMIC_PATH}"; fi && \
    if [[ "${RSL_RL_DEV,,}" = true ]]; then uv pip install --python "${PYTHONEXE}" --no-deps --editable "${RSL_RL_PATH}"; fi && \
    rm -f /tmp/srb-requirements.txt && \
    python -c 'from importlib.metadata import version; import numpy, pyproj, rasterio; print("srb", version("srb"), "oxidasim", version("oxidasim"), "simforge", version("simforge"), "simforge-foundry", version("simforge-foundry")); print("numpy", version("numpy"), "pyproj", version("pyproj"), "proj", pyproj.proj_version_str, "rasterio", version("rasterio"), "gdal", rasterio.__gdal_version__)' && \
    ## Kit loads its bundled copies of these compiled packages, so the venv must match them.
    python -c 'import glob, os; from importlib.metadata import version; stale = [p for p in ("matplotlib", "scipy") if not glob.glob(os.path.join(os.environ["ISAAC_PATH"], "ext*", "*", "pip_prebundle", f"{p}-{version(p)}.dist-info"))]; assert not stale, f"{stale} must match the versions bundled with Isaac Sim"' && \
    python -c 'import jax; print("jax", jax.__version__, jax.devices())' && \
    if [ "$TARGETARCH" != "amd64" ]; then python -c 'from pxr import Usd; print("[INFO] pxr resolves to", Usd.__file__, "version", Usd.GetVersion())'; fi

## PyPI provides CPU-only torch for aarch64; SRB requires CUDA for batched envs.
## Install the CUDA wheel from PyTorch's index, using cu130 for the GB10 platform.
## The locked torch version drives both architectures; only the aarch64 build differs.
# hadolint ignore=SC1091
RUN --mount=type=cache,target="/root/.cache/uv" \
    if [ "$TARGETARCH" != "amd64" ]; then \
    source /entrypoint.bash -- && \
    ## Assert CUDA torch survived dependency resolution; fail if a CPU wheel replaced it.
    python -c 'import torch; assert torch.version.cuda, "torch has no CUDA build: the cu130 pre-install did not survive srb[all] resolution"; assert "+cu" in torch.__version__, f"torch is {torch.__version__}, expected a +cuXXX local version"; print("[INFO] torch", torch.__version__, "built against CUDA", torch.version.cuda)' && \
    ## The cu13 wheel's NVRTC builtins are outside loader search paths, breaking JIT ops.
    ## Expose only that library: adding the wheel directory would shadow Isaac Sim CUDA libs.
    NVRTC_BUILTINS="$(set -- /root/.venv/lib/python*/site-packages/nvidia/cu13/lib/libnvrtc-builtins.so.13.0; if [ "$#" -eq 1 ] && [ "$1" = '/root/.venv/lib/python*/site-packages/nvidia/cu13/lib/libnvrtc-builtins.so.13.0' ] && [ ! -e "$1" ] && [ ! -L "$1" ]; then set --; fi; printf '%s\n' "$@" | LC_ALL=C sort | sed -n '1p')" && \
    if [ -n "${NVRTC_BUILTINS}" ]; then \
    ln -sf "${NVRTC_BUILTINS}" /usr/local/lib/libnvrtc-builtins.so.13.0 && \
    ldconfig && \
    { ldconfig -p | grep -q 'libnvrtc-builtins\.so\.13\.0' || { echo "libnvrtc-builtins.so.13.0 did not reach the loader cache" >&2; exit 69; } ; } ; \
    else \
    echo "[WARNING] libnvrtc-builtins.so.13.0 not found in the cu13 wheel; the NVRTC jiterator will fail for ops compiled at runtime" >&2 ; \
    fi ; \
    fi

## Keep NumPy and Pydantic from the frozen dependency export.

## Kit requires the aarch64 libgomp preload before other libraries and checks its path.
## Use Kit's expected `/lib/<arch>-linux-gnu/` spelling, even though `/lib` resolves to `/usr/lib`.
## Prepend libgomp to the existing LD_PRELOAD through the entrypoint.
RUN if [ "$TARGETARCH" != "amd64" ]; then \
    GOMP="/lib/$(uname -m)-linux-gnu/libgomp.so.1" && \
    if [ ! -e "${GOMP}" ]; then GOMP="$(find /lib /usr/lib -name 'libgomp.so.1' 2>/dev/null | head -n 1)"; fi && \
    { test -n "${GOMP}" && test -e "${GOMP}" || { echo "libgomp.so.1 not found; Isaac Sim will not start on ${TARGETARCH}" >&2; exit 65; } ; } && \
    { grep -q "^# Execute command\$" /entrypoint.bash || { echo "entrypoint marker '# Execute command' not found; cannot install the libgomp preload" >&2; exit 66; } ; } && \
    printf '\n# Isaac Sim on aarch64 needs libgomp loaded ahead of the rest\n' > /tmp/gomp.bash && \
    printf "export LD_PRELOAD=\"%s\${LD_PRELOAD:+:\${LD_PRELOAD}}\"\n" "${GOMP}" >> /tmp/gomp.bash && \
    awk "FNR==NR{ins = ins \$0 ORS; next} /^# Execute command\$/{printf \"%s\", ins} 1" /tmp/gomp.bash /entrypoint.bash > /tmp/entrypoint.bash && \
    cat /tmp/entrypoint.bash > /entrypoint.bash && \
    rm -f /tmp/entrypoint.bash /tmp/gomp.bash && \
    { test -x /entrypoint.bash || { echo "/entrypoint.bash lost its executable bit" >&2; exit 68; } ; } && \
    awk "index(\$0, \"export LD_PRELOAD=\") == 1 && index(\$0, \"libgomp.so.1\") > 0 {n++; if (!p) p=NR} \$0 == \"exec \\\"\${@}\\\"\"{e=NR} END{if (n != 1) {print \"expected exactly 1 libgomp LD_PRELOAD line, found \" n+0 > \"/dev/stderr\"; exit 67} if (!e || p > e) {print \"libgomp preload is not ahead of the exec line\" > \"/dev/stderr\"; exit 67}}" /entrypoint.bash ; \
    fi


ENV DEFAULT_SHELL="fish"
ARG ARGCOMPLETE_EXES="simforge srb"
ENV FISH_THEME="ayu"
# Fish config
COPY .docker/config/config.fish /root/.config/fish/config.fish
# hadolint ignore=SC2016
RUN source /entrypoint.bash -- && \
    git config --global --add safe.directory "${SRB_PATH}" && \
    # Shell selector
    printf '\nif [[ -n "${DEFAULT_SHELL}" ]] && [[ "${DEFAULT_SHELL,,}" != "bash" ]] && [[ $- == *i* ]]; then\nif command -v "${DEFAULT_SHELL}" > /dev/null 2>&1; then\nexec "${DEFAULT_SHELL}"\nfi\nfi\n' >> /entrypoint.bash && \
    # Bash completion
    printf 'source /etc/bash_completion\n' >> "/etc/bash.bashrc" && \
    for exe in ${ARGCOMPLETE_EXES}; do \
    register-python-argcomplete "${exe}" > "/etc/bash_completion.d/${exe}" || exit 1 ; \
    done && \
    # Fish history, completion
    mkdir -p /.history && \
    ln -sf /root/.local/share/fish/fish_history /.history/fish_history && \
    mkdir -p /root/.config/fish/completions && \
    for exe in ${ARGCOMPLETE_EXES}; do \
    register-python-argcomplete --shell fish "${exe}" > "/root/.config/fish/completions/${exe}.fish" || exit 1 ; \
    done && \
    # Generate and install Hydra override completions (including config keys + values)
    # Unset the build-only ISAACSIM_VERSION so the cache path matches the runtime one
    env -u ISAACSIM_VERSION "${SRB_PATH}/srb/cli/setup_cli.bash" --completions-only

ENV PYTHONDONTWRITEBYTECODE=1
ENV HYDRA_FULL_ERROR=1
ENV XLA_PYTHON_CLIENT_PREALLOCATE=0

SHELL ["/bin/bash", "-o", "pipefail", "-c"]
CMD ["/bin/bash"]
