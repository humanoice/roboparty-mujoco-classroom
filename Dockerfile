FROM ros:humble-ros-base-jammy
ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# System packages: build tools for the RoboParty stack, the browser desktop, and Python.
RUN apt-get update && apt-get install -y --no-install-recommends \
    git ccache cmake build-essential libfmt-dev libspdlog-dev libeigen3-dev libboost-system-dev \
    python3-dev python3-pip python3-yaml python3-numpy python3-tk ros-humble-pybind11-vendor \
    ros-humble-rviz2 ros-humble-robot-state-publisher ros-humble-joy ros-humble-tf2-ros \
    python3-colcon-common-extensions \
    xvfb x11vnc novnc websockify openbox supervisor xterm mesa-utils libgl1-mesa-dri libglfw3 scrot xdotool \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install --no-cache-dir "mujoco==3.3.7" "numpy<2"

# Pinned upstream versions. roboparty_deploy pins the inference, motors and imu commits.
ARG DEPLOY_REF=a8a0f1557cc5d085234b8bac248a8f543342f531
ARG RPO_DESCRIPTION_REF=37aac9ca665e92731444a1618320078e7ba21569

WORKDIR /opt/roboparty_deploy
RUN git init -q . && git remote add origin https://github.com/Roboparty/roboparty_deploy.git \
    && git fetch -q --depth 1 origin "$DEPLOY_REF" && git checkout -q FETCH_HEAD \
    && git submodule update -q --init --depth 1 src/inference src/motors src/imu \
    && git -C src/inference submodule update -q --init --depth 1

RUN git init -q /opt/rpo_description && cd /opt/rpo_description \
    && git remote add origin https://github.com/Roboparty/rpo_description.git \
    && git fetch -q --depth 1 origin "$RPO_DESCRIPTION_REF" && git checkout -q FETCH_HEAD

# Add the "SIM" motor and IMU types. See upstream-patches/README.md.
COPY upstream-patches /tmp/upstream-patches
RUN cd /opt/roboparty_deploy/src \
    && mkdir -p motors/src/drivers/sim imu/src/drivers/sim \
    && cp /tmp/upstream-patches/motors/drivers/sim/* /tmp/upstream-patches/common/rpo_sim_shm.hpp motors/src/drivers/sim/ \
    && cp /tmp/upstream-patches/imu/drivers/sim/* /tmp/upstream-patches/common/rpo_sim_shm.hpp imu/src/drivers/sim/ \
    && git -C motors apply /tmp/upstream-patches/motors/*.patch \
    && git -C imu apply /tmp/upstream-patches/imu/*.patch

# Build the real RoboParty stack (motors, imu, inference) as its own workspace.
RUN source /opt/ros/humble/setup.bash && cd /opt/roboparty_deploy \
    && CCACHE_DISABLE=1 colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release \
    && rm -rf build log

# Light copy of the robot model for drawing only (635k -> ~63k triangles).
# Physics keeps the original rpo.xml. See scripts/make_view_model.py.
COPY scripts/make_view_model.py /opt/tools/make_view_model.py
RUN pip3 install --no-cache-dir "fast-simplification==0.2.0" \
    && python3 /opt/tools/make_view_model.py /opt/rpo_description

# Build the classroom workspace on top of it.
WORKDIR /opt/ros2_ws
COPY ros2_ws/src ./src
RUN source /opt/roboparty_deploy/install/setup.bash && colcon build --symlink-install

COPY docker /etc/classroom
COPY scripts ./scripts
RUN chmod +x /etc/classroom/*.sh /opt/ros2_ws/scripts/*.sh \
    && ln -sf /opt/ros2_ws/scripts/party.sh /usr/local/bin/party \
    && printf '\nsource /etc/classroom/entrypoint-rc.sh\n' >> /root/.bashrc

ENV DISPLAY=:1 LIBGL_ALWAYS_SOFTWARE=1 ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=1 \
    RPO_MJCF=/opt/rpo_description/mjcf/rpo.xml RPO_VIEW_MJCF=/opt/rpo_description/mjcf/rpo_view.xml
EXPOSE 6080
ENTRYPOINT ["/etc/classroom/entrypoint.sh"]
CMD ["/usr/bin/supervisord", "-c", "/etc/classroom/supervisord.conf"]
