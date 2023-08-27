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

* Teleoperation with or without force feedback, using any master input
  device to teleoperate any slave robot

  * Teleoperation over high-speed links, for example when master and
    slave are controlled from same computer or from computers with
    high-speed local connection

  * Teleoperation with lower-quality channels, such as over the
    Internet. These may have lower bandwidth and/or significant
    latency.

* Autonomous (CNC-style) motion on the slave robot

* Virtual fixtures on the master robot
