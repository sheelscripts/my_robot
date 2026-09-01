FROM ros:jazzy

ENV DEBIAN_FRONTEND=noninteractive \
    LANG=en_US.UTF-8 \
    DISPLAY=:1 \
    LIBGL_ALWAYS_SOFTWARE=1 \
    QT_X11_NO_MITSHM=1

# Install GUI desktop, TigerVNC, noVNC, and ROS 2 simulation/nav stack
RUN apt-get update && apt-get install -y --no-install-recommends \
    xfce4 \
    xfce4-terminal \
    dbus-x11 \
    vim \
    tigervnc-standalone-server \
    novnc \
    websockify \
    ros-jazzy-desktop \
    ros-jazzy-ros-gz \
    ros-jazzy-nav2-map-server \
    ros-jazzy-joint-state-publisher-gui \
    ros-jazzy-urdf-tutorial \
    ros-jazzy-teleop-twist-keyboard \
    ros-jazzy-xacro \
    ros-jazzy-slam-toolbox \
    ros-jazzy-navigation2 \
    ros-jazzy-nav2-bringup \
    ros-jazzy-robot-localization \
    ros-jazzy-pointcloud-to-laserscan \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# Symlink noVNC index for direct access at http://localhost:6080
RUN ln -sf /usr/share/novnc/vnc.html /usr/share/novnc/index.html

# Setup workspace directory & auto-source ROS 2
WORKDIR /ros2_ws
RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc \
    && echo "if [ -f /ros2_ws/install/setup.bash ]; then source /ros2_ws/install/setup.bash; fi" >> /root/.bashrc

EXPOSE 6080

# Start TigerVNC (passwordless) + noVNC web proxy
CMD ["sh", "-c", "\
    rm -rf /tmp/.X1-lock /tmp/.X11-unix/X1 && \
    vncserver :1 -geometry 1920x1080 -depth 24 -SecurityTypes None -xstartup /usr/bin/startxfce4 && \
    websockify --web=/usr/share/novnc 6080 localhost:5901 \
    "]
