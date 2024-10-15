FROM ros:noetic-robot-focal as base

# Install ros packages
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
       ros-noetic-rosbridge-server \
    && rm -rf /var/lib/apt/lists/* \
    && apt-get clean

ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]