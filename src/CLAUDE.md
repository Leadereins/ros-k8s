# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with ROS 2 packages in this directory.

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
