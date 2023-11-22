.. _Introduction:

############
Introduction
############

************
What is CRTK
************

CRTK (Collaborative Robotics Toolkit) is primarily a convention for
command names and payloads used in robotics.

It is not necessary for robots to support all the CRTK commands, but
if a command is implemented, it should adhere to this convention for
the command name and payload. The API does not require any specific
programming language nor middleware.  As such the Robot Operating
System (ROS) is not required, but the equivalent ROS service/topic
name and message type are included since it is our primary development
environment.

*********
Use cases
*********

The initial use cases were motivated by the `Raven II
<https://applieddexterity.com/>`_ and `da Vinci Research Kit (dVRK)
<https://github.com/jhu-dvrk/sawIntuitiveResearchKit>`_, which are
telerobotic systems primarily used for research in medical
applications. However, it also encompasses teleoperation or
cooperative (hands-on) control of other robot systems, including
typical industrial robots. It is not intended for mobile
robots. Typical use cases include:

* Teleoperation with or without force feedback, using any input
  device (leader) to teleoperate any follower robot

  * Teleoperation over high-speed links, for example when leader and
    follower are controlled from same computer or from computers with
    high-speed local connection

  * Teleoperation with lower-quality channels, such as over the
    Internet. These may have lower bandwidth and/or significant
    latency.

* Autonomous (CNC-style) motion on the follower robot

* Virtual fixtures on the leader robot


*****************
Supported devices
*****************

This is a non-exhaustive list of devices with CRTK interfaces.  The devices with a *cisstMultiTask* interface rely on the `ROS1 bridge <https://github.com/jhu-cisst/cisst-ros>`_ or `ROS2 bridge <https://github.com/jhu-cisst/cisst_ros2_crtk>`_ to provide the ROS interface.

* `Raven II <https://applieddexterity.com/>`_

  * ROS1 interface

* `dVRK (daVinci Research Kit) <https://github.com/jhu-dvrk/sawIntuitiveResearchKit>`_

  * `ROS1 <https://github.com/jhu-dvrk/dvrk-ros>`_ and `ROS2 <https://github.com/jhu-dvrk/ros2_dvrk_robot>`_ interfaces as well as Python and Matlab clients

  * *cisstMultiTask* interface

* `ForceDimension haptic devices (and Novint Falcon) <https://github.com/jhu-saw/sawForceDimensionSDK>`_

  * `ROS1 <https://github.com/jhu-saw/sawForceDimensionSDK>`_ and `ROS2 <https://github.com/jhu-saw/sawForceDimensionSDKROS2>`_ interfaces as well as Python client

  * *cisstMultiTask* interface

* `SensablePhantom haptic devices (aka GeoMagic or 3DS Touch) <https://github.com/jhu-saw/sawSensablePhantom>`_

  * `ROS1 <https://github.com/jhu-saw/sawSensablePhantom>`_ and `ROS2 <https://github.com/jhu-saw/sawSensablePhantomROS2>`_ interfaces as well as Python client

  * *cisstMultiTask* interface

* `Universal Robot (UR) <https://github.com/jhu-saw/sawUniversalRobot>`_

  * `ROS1 <https://github.com/jhu-saw/sawUniversalRobot>`_ and `ROS2 <https://github.com/jhu-saw/sawUniversalRobotROS2>`_ interfaces

  * *cisstMultiTask* interface

* `Northern Digital Inc tracking devices (NDi) <https://github.com/jhu-saw/sawNDITracker>`_.  Current implementation supports devices using a serial port or USB interface (Polaris, Aurora...)

  * `ROS1 <https://github.com/jhu-saw/sawNDITracker>`_ and `ROS2 <https://github.com/jhu-saw/sawNDITrackerROS2>`_ interfaces

  * *cisstMultiTask* interface

* `Atracsys tracking devices (NDi) <https://github.com/jhu-saw/sawAtracsysFusionTrack>`_

  * `ROS1 <https://github.com/jhu-saw/sawAtracsysFusionTrack>`_ 

  * *cisstMultiTask* interface

* `Optoforce force sensors <https://github.com/jhu-saw/sawOptoforceSensor>`_

  * `ROS1 <https://github.com/jhu-saw/sawOptoforceSensor>`_ and `ROS2 <https://github.com/jhu-saw/sawOptoforceSensorROS2>`_ interfaces

  * *cisstMultiTask* interface

