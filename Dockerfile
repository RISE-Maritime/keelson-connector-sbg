FROM ghcr.io/rise-maritime/porla:v0.4.1

# Set environment
ENV DEBIAN_FRONTEND=noninteractive

# Install build dependencies
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    wget \
    unzip \
    libusb-1.0-0-dev \
    && rm -rf /var/lib/apt/lists/*

# Install Python requirements
COPY requirements.txt requirements.txt
RUN pip3 install --no-cache-dir -r requirements.txt

# Download and build SBG SDK
WORKDIR /opt
RUN wget https://download.sbg-systems.com/dev/sbgECom-3.2.765.zip -O sbgECom.zip && \
    unzip sbgECom.zip -d sbgECom && \
    rm sbgECom.zip

WORKDIR /opt/sbgECom
RUN cmake -S . -B build -DCMAKE_INSTALL_PREFIX=/usr/local && \
    cmake --build build -- -j$(nproc) && \
    cmake --install build && \
    ln -s /usr/local/bin/tools/sbgBasicLogger/sbgBasicLogger /usr/local/bin/sbgBasicLogger

# Copy user-provided binaries
COPY --chmod=555 ./bin/* /usr/local/bin/

# Final entrypoint
ENTRYPOINT ["/tini", "-g", "--", "/bin/bash", "-c"]

