FROM osrf/ros2:nightly

RUN apt update && apt full-upgrade -y
RUN apt install -y xauth libfreetype6 libglu1-mesa libxi6 libxrender1 xz-utils
RUN apt install -y mesa-utils libegl1-mesa libegl1-mesa-dev libgbm-dev libgbm1 libgl1-mesa-dev libgl1-mesa-dri libglu1-mesa libglu1-mesa-dev
RUN apt install -y libopenexr-dev zlib1g-dev libfontconfig1 libgl1-mesa-glx libxmu-dev
RUN apt update && apt install -y mesa-utils apt-utils xvfb llvm-dev

#NVIDIA
ENV NVIDIA_VISIBLE_DEVICES ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics
LABEL com.nvidia.volumes.needed="nvidia_driver"
ENV PATH /usr/local/nvidia/bin:${PATH}
ENV LD_LIBRARY_PATH /usr/local/nvidia/lib:/usr/local/nvidia/lib64:${LD_LIBRARY_PATH}

RUN apt update