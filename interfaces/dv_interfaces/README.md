# DV interfaces

This package contains the ROS 1 messages required by the control applications.
It belongs to `FS-AGH-Racing-DV-Control`, is built in the same workspace, and
does not require an external interface repository.

The Control repository does not import message packages from a simulator.
External systems can communicate with the controllers only when their ROS
topics and message definitions are compatible with this local package.
