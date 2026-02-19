# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with Kubernetes manifests in this directory.

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
