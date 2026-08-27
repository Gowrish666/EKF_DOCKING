FROM ubuntu:26.04

RUN apt-get update && \
    apt-get install -y \
        build-essential \
        cmake \
        libxcb-cursor0 \
        qt6-base-dev \
        libpaho-mqtt-dev \
        libpaho-mqttpp-dev \
        libssl-dev \
        && \
    rm -rf /var/lib/apt/lists/*

WORKDIR /app

COPY . .

RUN mkdir -p build && \
    cd build && \
    cmake .. && \
    cmake --build .

CMD ["./build/visualizer_ui"]