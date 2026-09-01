# Build environment for cross-compiling to a Raspberry Pi 3 running 64-bit Raspberry Pi OS.
# See ../readme.md for the walkthrough that runs inside it.
#
# Ubuntu 24.04 ships the Python 3.12 the ros-kilted recipe expects, and its glibc 2.39 is
# older than Raspberry Pi OS Trixie's 2.41, so binaries built here load on the Pi.

FROM ubuntu:24.04
ARG CONAN_VERSION=2.31.2
ENV DEBIAN_FRONTEND=noninteractive
ENV CONAN_HOME=/opt/conan2

# Multiarch. archive.ubuntu.com serves no arm64, so its stanzas are pinned to amd64 and the
# arm64 packages come from ports.ubuntu.com. This is what makes cross-building ros-kilted
# possible: rosidl_generator_py compiles an extension module for the target, and multiarch
# puts the target Python headers and library where the cross toolchain already looks.
RUN dpkg --add-architecture arm64 \
    && sed -i 's|^Components: \(.*\)$|Components: \1\nArchitectures: amd64|' \
        /etc/apt/sources.list.d/ubuntu.sources \
    && printf '%s\n' \
        'Types: deb' \
        'URIs: http://ports.ubuntu.com/ubuntu-ports/' \
        'Suites: noble noble-updates noble-backports noble-security' \
        'Components: main universe restricted multiverse' \
        'Architectures: arm64' \
        'Signed-By: /usr/share/keyrings/ubuntu-archive-keyring.gpg' \
        > /etc/apt/sources.list.d/arm64-ports.sources

# g++-13 matches the compiler.version pinned in ../profiles/rpi3-armv8. NumPy is needed for
# its headers only, so the amd64 package serves. rsync and openssh-client transfer the result
# to the robot.
RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential ca-certificates cmake curl git ninja-build patch pkg-config \
        openssh-client rsync \
        g++-13-aarch64-linux-gnu libpython3.12-dev:arm64 \
        python3 python3-venv python3-pip python3-numpy \
    && rm -rf /var/lib/apt/lists/*

# Optional local-only CA (gitignored extra-ca.crt). Used on machines behind a TLS-inspecting
# proxy; CI and other clones skip this because the file is absent. Dockerfile is copied
# alongside only so that the wildcard always has a match.
COPY Dockerfile extra-ca.cr[t] /usr/local/share/ca-certificates/
RUN rm -f /usr/local/share/ca-certificates/Dockerfile && update-ca-certificates

# pip, requests and Conan read certifi's bundle rather than the system store, so they have to
# be pointed at it for a local extra-ca.crt to count.
ENV PIP_CERT=/etc/ssl/certs/ca-certificates.crt \
    REQUESTS_CA_BUNDLE=/etc/ssl/certs/ca-certificates.crt \
    SSL_CERT_FILE=/etc/ssl/certs/ca-certificates.crt

# PEP 668 marks the system interpreter as externally managed, so Conan lives in its own venv.
RUN python3 -m venv /opt/conan-venv \
    && /opt/conan-venv/bin/pip install --no-cache-dir "conan==${CONAN_VERSION}"
ENV PATH=/opt/conan-venv/bin:$PATH

RUN conan profile detect

WORKDIR /repo
CMD ["/bin/bash"]
