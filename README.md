# Quick Start

> First, make sure you have docker installed on your machine. If you don't have it, you can download it [here](https://www.docker.com/products/docker-desktop).

```bash
# Clone the repository
git clone <this-repo>

# Build and run the docker container
make

# The container will be removed once you stop it
# The image will also be removed once you remove the container
```

## Open another terminal

```bash
# Make sure you have the container opened
# Use the following command to attach the terminal
make attach
```