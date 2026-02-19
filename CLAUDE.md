# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This workspace is for deploying ROS 2 robotics applications on Kubernetes. It combines ROS 2 package development (`src/`) with containerization (`docker/`) and cluster orchestration (`k8s/`).

## Repository Structure

- `src/` — ROS 2 packages (ament/colcon-based)
- `docker/` — Dockerfiles and docker-compose files for building ROS images
- `k8s/` — Kubernetes manifests (Deployments, Services, ConfigMaps, etc.)
- `.claude/commands/` — Custom Claude Code slash commands for this project
- `thoughts/shared/` — Shared design documents, research, plans, deployment notes, and postmortems
- `thoughts/meta/notion_sync.json` — Notion sync configuration

## Key Conventions

- Kubernetes secrets and credentials are **never committed** — use `k8s/secrets/` (gitignored) or an external secret manager
- `docker-compose.override.yml` is gitignored — use it for local dev overrides
- Empty directories are preserved with `.gitkeep` files

## ROS 2 Development

Build all packages:
```bash
colcon build --symlink-install
source install/setup.bash
```

Build a single package:
```bash
colcon build --packages-select <package_name>
```

Run tests:
```bash
colcon test --packages-select <package_name>
colcon test-result --verbose
```

## Docker

Build image:
```bash
docker build -f docker/Dockerfile -t ros-k8s:latest .
```

## Kubernetes

Apply manifests:
```bash
kubectl apply -f k8s/
```

Check rollout:
```bash
kubectl rollout status deployment/<name>
```
