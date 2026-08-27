FROM ubuntu:26.04

RUN apt-get update && \
    apt-get install -y \
        build-essential \
        cmake \
        libpaho-mqtt-dev \
        libpaho-mqttpp-dev \
        libssl-dev \
        && \
        rm -rf /var/lib/apt/lists/*

RUN echo "=== OPENSSL CHECK ===" && \
    dpkg -l | grep libssl && \
    ls -l /usr/include/openssl/ssl.h && \
    ls -l /usr/lib/x86_64-linux-gnu/libssl* && \
    ls -l /usr/lib/x86_64-linux-gnu/libcrypto*

WORKDIR /app

COPY . .

RUN mkdir -p build && \
    cd build && \
    cmake .. && \
    cmake --build .

CMD ["./build/simulator"]