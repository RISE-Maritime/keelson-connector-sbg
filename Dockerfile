FROM ubuntu:22.04

# Set environment
ENV DEBIAN_FRONTEND=noninteractive

# Install build dependencies and runtime requirements
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    wget \
    unzip \
    libusb-1.0-0-dev \
    python3 \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Install Python requirements
COPY requirements.txt requirements.txt
RUN pip3 install --no-cache-dir -r requirements.txt

# Clone and build SBG SDK
WORKDIR /opt
RUN git clone https://github.com/SBG-Systems/sbgECom.git

WORKDIR /opt/sbgECom
RUN cmake -Bbuild -DBUILD_EXAMPLES=ON -DBUILD_TOOLS=ON && \
    cmake --build build --config Release && \
    cmake --install build && \
    cp build/sbgBasicLogger /usr/local/bin/sbgBasicLogger && \
    chmod +x /usr/local/bin/sbgBasicLogger

# Copy user-provided binaries
COPY --chmod=555 ./bin/* /usr/local/bin/

WORKDIR /app

# Create a wrapper script to pipe sbgBasicLogger output to Python script
RUN echo '#!/bin/bash\nsbgBasicLogger "$@" | python3 /usr/local/bin/main ${PYTHON_ARGS:-}' > /usr/local/bin/sbg-connector.sh && \
    chmod +x /usr/local/bin/sbg-connector.sh

# Use the wrapper script as entrypoint
ENTRYPOINT ["/usr/local/bin/sbg-connector.sh"]
