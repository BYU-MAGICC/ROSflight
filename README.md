# telemetry

This repository contains the ROS stack for interfacing with an autopilot running the telemetry firmware. For more information on the telemetry autopilot firmware stack, visit http://telemetry.org.

The following sections describe each of the packages contained in this stack.

## telemetry_pkgs

This is a metapackage for grouping the other packages into a ROS stack.

## rosflight_msgs

This package contains the telemetry message and service definitions.

## telemetry

This package contains the `telemetry_io` node, which provides the core functionality for interfacing an onboard computer with the autopilot. This node streams autopilot sensor and status data to the onboard computer, streams control setpoints to the autopilot, and provides an interface for configuring the autopilot.

## telemetry_utils

This package contains additional supporting scripts and libraries that are not part of the core telemetry package functionality. This package also helps support the [ros_plane](https://github.com/byu-magicc/ros_plane) and [ros_copter](https://github.com/byu-magicc/ros_copter) projects.
