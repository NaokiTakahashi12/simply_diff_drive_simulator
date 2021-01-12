FROM naokitakahashi12/ignition:citadel-devel-bionic

ENV IGNITION_VERSION=citadel

RUN apt update --quiet \
    && apt upgrade --quiet -y \
    && apt install --quiet --no-install-recommends -y \
	curl \
	gnupg2 \
	lsb-release \
	sudo \
	cmake \
	make \
    && echo "deb http://packages.ros.org/ros/ubuntu bionic main" > /etc/apt/sources.list.d/ros-latest.list \
    && curl -sSL 'http://keyserver.ubuntu.com/pks/lookup?op=get&search=0xC1CF6E31E6BADE8868B172B4F42ED6FBAB17C654' | apt-key add - \
    && apt update \
    && apt install --quiet --no-install-recommends -y \
	ros-melodic-ros-base \
    && apt --purge remove -y \
	gnupg2 \
	curl \
    && apt clean \
    && apt autoremove -y \
    && rm -rf /var/lib/apt/lists/*

COPY preinstall.repos.yaml /tmp/.

RUN apt update --quiet \
    && apt install --quiet --no-install-recommends -y \
	python3-vcstool \
	python-rosdep \
	git \
	apt-utils \
    && rosdep init \
    && rosdep update \
    && mkdir -p /preinstall/src \
    && vcs import /preinstall/src < /tmp/preinstall.repos.yaml \
    && /opt/ros/melodic/env.sh \
	rosdep install \
	    --rosdistro melodic \
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
	    -r -y \
    && apt --purge remove -y \
	python3-vcstool \
	python-rosdep \
	git \
	apt-utils \
    && apt clean \
    && apt autoremove -y \
    && rm -rf /var/lib/apt/lists/*

RUN apt update --quiet \
    && apt install --quiet --no-install-recommends -y \
	g++-8 \
	python-catkin-tools \
    && cd /preinstall \
    && sh -c "\
	CXX=g++-8 \
	&& . /opt/ros/melodic/setup.sh \
	&& . /ign_ws/install/setup.sh \
	&& catkin build \
	    --save-config \
	    --no-status \
	    --no-notify \
	    --no-summarize \
	    --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo" \
    && rm -rf build logs \
    && apt --purge remove -y \
	g++-8 \
	python-catkin-tools \
    && apt clean \
    && apt autoremove -y \
    && rm -rf /var/lib/apt/lists/*

RUN echo "source /preinstall/devel/setup.bash" >> ~/.bashrc

COPY ros_entrypoint.override.bash /.
ENTRYPOINT ["/ros_entrypoint.override.bash"]

CMD ["/bin/bash"]

