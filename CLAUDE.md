# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Cluster Hardware & Software

| Role | Device | OS | Notes |
|------|--------|----|-------|
| Master node | Lenovo ThinkPad X280 | — | kubectl / cluster control plane |
| Worker nodes | Raspberry Pi 4 | Raspbian GNU/Linux 11 | general compute |
| Worker nodes | TurtleBot3 Burger | Ubuntu 22.04 + ROS 2 Humble | mobile robot nodes |

- Docker images targeting TurtleBot3 must be **ARM64** (`linux/arm64`)
- ROS 2 distro: **Humble** (Ubuntu 22.04 base)

## Project Overview

Workspace for deploying ROS 2 robotics applications on a heterogeneous Kubernetes cluster. Combines ROS 2 package development (`src/`), containerization (`docker/`), and cluster orchestration (`k8s/`).

## Repository Structure

- `src/` — ROS 2 packages (ament/colcon)
- `docker/` — Dockerfiles; `docker-compose.override.yml` is gitignored (local dev overrides)
- `k8s/` — Kubernetes manifests; supports both raw YAML, Helm (`charts/`), and Kustomize
- `k8s/secrets/` — gitignored; never commit secrets here
- `thoughts/shared/` — design docs, research, plans, deploy notes, postmortems
- `.claude/commands/` — custom Claude Code slash commands

## ROS 2

```bash
# Build all packages
colcon build --symlink-install
source install/setup.bash

# Build single package
colcon build --packages-select <package_name>

# Run tests
colcon test --packages-select <package_name>
colcon test-result --verbose
```

## Docker

```bash
docker build -f docker/Dockerfile -t ros-k8s:latest .
```

## Kubernetes

```bash
# Apply all manifests
kubectl apply -f k8s/

# Helm
helm upgrade --install <release> charts/<chart> -f charts/<chart>/values.yaml

# Kustomize
kubectl apply -k k8s/<overlay>

# Check rollout
kubectl rollout status deployment/<name>
```
