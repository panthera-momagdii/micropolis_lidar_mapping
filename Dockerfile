FROM ros:jazzy-ros-base
SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

# build tooling not declared in package.xml: colcon, compiler/cmake, python venv for the evaluator
RUN apt-get update && apt-get install -y --no-install-recommends \
      build-essential cmake git \
      python3-colcon-common-extensions python3-venv python3-pip \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ws
COPY . /ws/src/app

# ROS deps straight from the package.xml manifests
# (rosbag2_cpp, rosbag2_storage, rosbag2_storage_mcap, sensor_msgs, nav_msgs, eigen, yaml-cpp)
RUN apt-get update \
    && rosdep update --rosdistro jazzy \
    && rosdep install --from-paths src --ignore-src -y -r \
    && rm -rf /var/lib/apt/lists/*

# isolated evaluator venv: evo + numpy + matplotlib only -> evo.tools.file_interface imports
# cleanly (this is what fixes the rosbags conflict; no shim needed in the container)
RUN python3 -m venv /opt/eval-venv \
    && /opt/eval-venv/bin/pip install --no-cache-dir -U pip \
    && /opt/eval-venv/bin/pip install --no-cache-dir -r src/app/lidar_mapper/eval/requirements.txt

# build Release (NDEBUG + optimization; OpenMP via gcc/libgomp)
RUN source /opt/ros/jazzy/setup.bash \
    && colcon build --packages-select lidar_mapper gnss_fusion \
         --cmake-args -DCMAKE_BUILD_TYPE=Release

RUN cp /ws/src/app/docker/run_pipeline.sh /usr/local/bin/run_pipeline.sh \
    && chmod +x /usr/local/bin/run_pipeline.sh

ENV BAGS_DIR=/bags OUT_DIR=/out
ENTRYPOINT ["/usr/local/bin/run_pipeline.sh"]
