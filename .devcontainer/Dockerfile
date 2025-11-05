FROM leograba/xeno3:ubuntu-22.04-xenomai-3.2.4

################# BASE DEPS #################
# (Flatbuffers / nlohmann_json / Protobuf / gRPC / libmodbus / Eigen)
RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y \
    build-essential ninja-build git cmake pkg-config curl ca-certificates \
    libeigen3-dev libmodbus-dev \
    nlohmann-json3-dev \
    protobuf-compiler libprotobuf-dev \
    libgrpc++-dev grpc-proto \
    libssl-dev \
    libtinyxml2-dev \
    libsqlite3-dev \
    && rm -rf /var/lib/apt/lists/*

################# RUST (cargo) #################
# zenoh-c 빌드에 필요
RUN curl -sSf https://sh.rustup.rs | sh -s -- -y --profile minimal --default-toolchain stable
ENV PATH="/root/.cargo/bin:${PATH}"

################# zenoh-c #################
RUN git clone --depth=1 https://github.com/eclipse-zenoh/zenoh-c /opt/zenoh-c && \
    cmake -S /opt/zenoh-c -B /opt/zenoh-c/build \
          -DCMAKE_BUILD_TYPE=Release -DBUILD_SHARED_LIBS=ON && \
    cmake --build /opt/zenoh-c/build -j && \
    cmake --install /opt/zenoh-c/build
# CMake에서 참고하도록(프로젝트가 이 변수를 사용한다면)
ENV ZENOH_C_PATH=/opt/zenoh-c

################ CMAKE REINSTALLATION (옵션) ################
ARG REINSTALL_CMAKE_VERSION_FROM_SOURCE="3.31.9"
COPY ./reinstall-cmake.sh /tmp/
RUN if [ "${REINSTALL_CMAKE_VERSION_FROM_SOURCE}" != "none" ]; then \
        chmod +x /tmp/reinstall-cmake.sh && /tmp/reinstall-cmake.sh ${REINSTALL_CMAKE_VERSION_FROM_SOURCE}; \
    fi && rm -f /tmp/reinstall-cmake.sh

################# FLATBUFFERS (from source) #################
RUN git clone --depth=1 --branch v25.2.10 https://github.com/google/flatbuffers.git /tmp/flatbuffers \
    && cmake -S /tmp/flatbuffers -B /tmp/flatbuffers/build -DFLATBUFFERS_BUILD_TESTS=OFF \
    && cmake --build /tmp/flatbuffers/build --target install \
    && rm -rf /tmp/flatbuffers

################# RBDL (from source) #################
ARG RBDL_BUILD_URDFREADER=OFF
RUN git clone --depth=1 --recursive https://github.com/rbdl/rbdl /opt/rbdl && \
    cmake -S /opt/rbdl -B /opt/rbdl/build \
          -DCMAKE_BUILD_TYPE=Release \
          -DBUILD_SHARED_LIBS=ON \
          -DRBDL_BUILD_TESTS=OFF \
          -DRBDL_BUILD_EXAMPLES=OFF \
          -DRBDL_BUILD_BENCHMARKS=OFF \
          -DRBDL_ENABLE_ASSERTS=OFF \
          -DRBDL_BUILD_ADDON_URDFREADER=${RBDL_BUILD_URDFREADER} && \
    cmake --build /opt/rbdl/build -j && \
    cmake --install /opt/rbdl/build

################# LINKER CACHE #################
RUN echo "/usr/local/lib" > /etc/ld.so.conf.d/usr-local-lib.conf && ldconfig

################# WORKDIR #################
WORKDIR /workspaces
