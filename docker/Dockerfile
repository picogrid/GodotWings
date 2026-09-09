# ArduPilot SITL (ArduPlane, ArduCopter, ArduRover) with the JSON physics backend
# (for GodotWings) and a MAVLink output for QGroundControl / Mission Planner.
#
# Builds the SITL binary from source. Rather than ArduPilot's full
# install-prereqs-ubuntu.sh (which also pulls ARM cross-compilers, SFML,
# matplotlib, ... and assumes an interactive user account), we install only the
# minimal toolchain SITL needs — faster and more reliable in a container.
FROM ubuntu:22.04

# Pin a Plane release for reproducibility; override with --build-arg ARDUPILOT_TAG=...
ARG ARDUPILOT_TAG=Plane-4.5.7
ENV DEBIAN_FRONTEND=noninteractive TZ=UTC

# Build toolchain + libs SITL/waf need, and a few runtime helpers.
RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential ccache g++ gawk git make wget \
        python3 python3-dev python3-pip python3-setuptools python3-wheel \
        python3-numpy python3-psutil \
        libtool libxml2-dev libxslt1-dev pkg-config \
        ca-certificates iputils-ping \
    && rm -rf /var/lib/apt/lists/*

# Python packages for code generation (waf) and MAVProxy. empy must be 3.3.4 —
# empy 4.x breaks ArduPilot's mavlink header generation.
RUN python3 -m pip install --no-cache-dir \
        future lxml pymavlink pyserial MAVProxy pexpect "empy==3.3.4" \
        dronecan ptyprocess

# Clone + check out the pinned tag with submodules.
WORKDIR /opt
RUN git clone https://github.com/ArduPilot/ardupilot.git \
    && cd ardupilot \
    && git checkout "${ARDUPILOT_TAG}" \
    && git submodule update --init --recursive

# Build the SITL ArduPlane + ArduCopter + ArduRover binaries (pick at runtime via VEHICLE).
WORKDIR /opt/ardupilot
RUN ./waf configure --board sitl && ./waf plane && ./waf copter && ./waf rover

# sim_vehicle.py on PATH.
ENV PATH="/opt/ardupilot/Tools/autotest:${PATH}"

COPY sitl-defaults.parm /sitl-defaults.parm
COPY fpv-quad.parm /fpv-quad.parm
COPY sitl-rover.parm /sitl-rover.parm
COPY --chmod=755 entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
