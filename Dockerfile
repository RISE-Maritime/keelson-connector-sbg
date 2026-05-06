FROM python:3.11-slim-bookworm

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    wget \
    unzip \
    libusb-1.0-0-dev \
    bash \
    && rm -rf /var/lib/apt/lists/*

COPY requirements.txt requirements.txt
RUN python -m pip install --upgrade pip && \
    python -m pip install --no-cache-dir -r requirements.txt

WORKDIR /opt
RUN git clone https://github.com/SBG-Systems/sbgECom.git

WORKDIR /opt/sbgECom
RUN cmake -Bbuild -DBUILD_EXAMPLES=ON -DBUILD_TOOLS=ON && \
    cmake --build build --config Release && \
    cmake --install build && \
    cp build/sbgBasicLogger /usr/local/bin/sbgBasicLogger && \
    chmod +x /usr/local/bin/sbgBasicLogger

COPY --chmod=555 ./bin/* /usr/local/bin/

WORKDIR /app

RUN printf '%s\n' \
    '#!/bin/bash' \
    'set -e' \
    'sbgBasicLogger "$@" | python /usr/local/bin/main ${PYTHON_ARGS:-}' \
    > /usr/local/bin/sbg-connector.sh && \
    chmod +x /usr/local/bin/sbg-connector.sh

ENTRYPOINT ["/usr/local/bin/sbg-connector.sh"]
