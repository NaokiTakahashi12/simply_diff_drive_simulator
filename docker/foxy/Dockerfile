FROM naokitakahashi12/ignition:citadel-devel-cudagl-11.0-focal

ENV IGNITION_VERSION=citadel

RUN apt update --quiet \
    && apt upgrade --quiet -y \
    && apt install --quiet --no-install-recommends -y \
        curl \
        gnupg2 \
        lsb-release \
        sudo \
    && curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - \
    && echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list \
    && apt update \
    && apt install --quiet --no-install-recommends -y \
        ros-foxy-ros-base \
        ros-foxy-desktop \
        ros-foxy-navigation2 \
        python3-argcomplete \
    && apt --purge remove -y \
        gnupg2 \
    && apt clean \
    && apt autoremove -y \
    && rm -rf /var/lib/apt/lists/*

COPY preinstall.repos.yaml /tmp/.

RUN apt update --quiet \
    && apt install --quiet --no-install-recommends -y \
        python3-vcstool \
        python3-rosdep \
        git \
        apt-utils \
    && rosdep init \
    && rosdep update \
    && mkdir -p /preinstall/src \
    && vcs import /preinstall/src < /tmp/preinstall.repos.yaml \
    && sh -c "\
        . /opt/ros/foxy/setup.sh \
        && . /ign_ws/install/setup.sh \
        && rosdep install \
            --rosdistro foxy \
            --from-paths /preinstall/src \
            --ignore-src \
            --skip-keys=ignition-gazebo2 \
            --skip-keys=ignition-gazebo3 \
            --skip-keys=ignition-msgs4 \
            --skip-keys=ignition-msgs5 \
            --skip-keys=ignition-rendering2 \
            --skip-keys=ignition-rendering3 \
            --skip-keys=ignition-sensors2 \
            --skip-keys=ignition-sensors3 \
            --skip-keys=ignition-transport7 \
            --skip-keys=ignition-transport8 \
            -r -y" \
    && apt --purge remove -y \
        gcc \
        g++ \
        python3-vcstool \
        python-rosdep \
        git \
        apt-utils \
    && apt clean \
    && apt autoremove -y \
    && rm -rf /var/lib/apt/lists/*

RUN apt update --quiet \
    && apt install --quiet --no-install-recommends -y \
        g++ \
        gcc \
        patch \
        cmake \
        make \
        python3-colcon-common-extensions \
        # Required ros_ign
        ros-foxy-launch-testing-ament-cmake \
        ros-foxy-image-transport \
    && cd /preinstall \
    && sh -c "\
        . /ign_ws/install/setup.sh \
        && . /opt/ros/foxy/setup.sh \
        && colcon build \
            --merge-install \
            --cmake-args -DCMAKE_BUILD_TYPE=Release" \
    && rm -rf build log \
    && apt --purge remove -y \
        python3-vcstool \
        python-rosdep \
        git \
        patch \
        g++ \
        gcc \
        python3-colcon-common-extensions \
    && apt clean \
    && apt autoremove -y \
    && rm -rf /var/lib/apt/lists/*

RUN echo "source /preinstall/install/setup.bash" >> ~/.bashrc

COPY ros2_entrypoint.override.bash /.
ENTRYPOINT ["/ros2_entrypoint.override.bash"]

CMD ["/bin/bash"]

