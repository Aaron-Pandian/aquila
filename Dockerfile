FROM px4io/px4-dev-simulation-jammy:latest

RUN apt-get update && apt-get install -y \
    sudo \
    nano \
    iputils-ping \
    python3-pip \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-libav \
    gstreamer1.0-gl \
    libgstreamer1.0-dev \          
    libgstreamer-plugins-base1.0-dev \ 
    && rm -rf /var/lib/apt/lists/*

# Install dependencies for rendering Gazebo Harmonic GUI on Mac
RUN apt-get update && apt-get install -y \
    libgz-rendering8 \
    libgz-rendering8-dev \
    libgz-common5 \
    libgz-math7 \
    libgz-plugin2 \
    && rm -rf /var/lib/apt/lists/*

ENV QT_X11_NO_MITSHM=1

RUN python3 -m pip install mavsdk

WORKDIR /src/PX4-Autopilot