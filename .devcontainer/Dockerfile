FROM ubuntu:24.04

ENV DEBIAN_FRONTEND=noninteractive

# Create non-root user
RUN useradd -ms /bin/bash vscode

RUN apt update && apt install -y \
    curl \
    git \
    sudo \
    build-essential \
    && rm -rf /var/lib/apt/lists/*

# Give sudo access
RUN echo "vscode ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

USER vscode
WORKDIR /workspaces
