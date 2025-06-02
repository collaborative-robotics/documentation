Overview
########

Motions
*******

We identified the following basic types of motions for collaborative
robotics. All these commands can be defined either in the cartesian or
joint space.

``servo``
=========

Direct access to the low-level controller:

* **Use cases:**

  * User has a smooth and continuous trajectory coming from a leader
    arm or recorded trajectory and can send commands at a high rate.

  * Closing the loop in velocity mode using a fast external sensor
    with a task based Jacobian.

  * Haptic feedback on leader arm.

* **Type:** These commands can be either position, velocity or effort
  based. Positions can be provided relative from the latest setpoint
  position.

* **Continuity:** Users should send continuous commands. The low-level
  controller is not required to generate intermediate setpoints to
  ensure that the command is feasible (i.e., setpoint should be close
  to current state). The low-level controller can enforce limits,
  e.g., reject a command if the difference from the previous command
  is greater than a defined threshold.

* **Time:** Users are expected to send commands periodically at a rate
  close to the low-level rate. These commands are preemptive.

``interpolate``
===============

Simple interpolation:

* **Use cases:**

  * User has a smooth trajectory coming from a leader arm or recorded
    trajectory, but **cannot** send commands at a high rate (e.g., 50Hz
    visual tracking, remote tele-operation).

  * In general, similar usage as ``servo`` but the application can not
    send commands fast enough to provide a smooth command so the
    low-level controller needs to interpolate the user commands
    (smooths but adds latency).

* **Type:** These commands can be either position, velocity or effort
  based. Positions can be provided relative from the latest setpoint
  position.

* **Continuity**: Users must send continuous commands. The low-level
  controller will compute intermediary positions to smooth the motion.

* **Time:** Users are expected to send commands periodically at a rate
  lower than the low-level controller. The velocity of the motion is
  defined by the user commands. These commands are preemptive.

``move``
========

Move with trajectory generation:

* **Use case**: User wants to move to a given position and stop there
  (e.g., home position, pick and place)

* **Type**: These commands are position based, either absolute or relative.

* **Continuity**: Users must send feasible commands. The low-level
  controller will compute a complete trajectory to move from the
  current state (position and velocity) to the desired goal (position).

* **Time**: Users are expected to send a single command and wait for
  completion before sending a new one. Time of execution is defined by
  the trajectory generation parameters (acceleration and velocity).


Naming convention
*****************

Command names are based on the space, type and control level. The
prefix identifies the control level (``servo``, ``interpolate`` or
``move``), it is followed by an underscore (``_``) and two letters
identifying the space and type. For example, ``servo_cp`` is a "servo"
command with a cartesian (``c``) position (``p``) setpoint.

Query commands
==============

* Space: ``j`` (joint), ``c`` (cartesian)

* Type:

  * Joint: ``s`` (state: position, velocity and effort)

  * Cartesian: ``p`` (pose), ``v`` (twist), ``f`` (wrench)

* Level:

  * Measured: ``measured`` (physical measure from sensors)

  * Low-level: ``setpoint`` (current servo setpoint)

  * Mid-level: ``goal`` (current interpolate or move goal)

Motion commands
===============

* Space: ``j`` (joint), ``c`` (cartesian)

* Type: ``p`` (position or pose), ``r`` (relative position or pose),
  ``v`` (velocity or twist), ``f`` (force, wrench or effort)

* Control level: ``servo`` (low-level), ``interpolate`` (basic
  interpolation), ``move`` (full trajectory planning)

Data validity
=============

For all query commands, it might be necessary to indicate that the
data is not valid. For all invalid data, the header timestamp should
be set to 0 (recall that the timestamp represents an absolute time
since January 1, 1970).

*ROS Specific*: The header timestamp is the field time
``Header.stamp``. Since it is not necessary to keep publishing
invalid data, it is recommended to not publish on the corresponding
topic until the data becomes valid again. To allow new a ROS node to
detect invalid data, the ROS topic should be latched. See `ROS
publisher options <http://wiki.ros.org/roscpp/Overview/Publishers%20and%20Subscribers#Publisher_Options>`_.

Summary
*******

Table
=====

.. list-table::
   :widths: 20 80
   :header-rows: 1

   * -
     - **Syntax**
   * - Control level
     - ``servo``: direct real-time stream (pre-emptive)

       ``interpolate``: interpolated stream (pre-emptive)

       ``move``: plan trajectory to goal (pre-emptive), monitor with ``is_moving``
   * - Feedback
     - ``measured``: sensor feedback

       ``measuredN``: redundant sensor feedback (N=2, 3...)

       ``setpoint``: current setpoint to low-level controller

       ``goal``: most recent ``interpolate`` or ``move`` goal
   * - Space
     - ``j``: joint

       ``c``: cartesian
   * - Type
     - ``p``: position

       ``r``: relative

       ``v``: velocity or twist

       ``f``: generalized force (effort and wrench)

       ``s``: state for joint feedback (includes position, velocity and effort)


Diagram
=======

.. image:: /images/CommonAPI.png
  :width: 400
  :align: center
  :alt: CRTK robot motion commands


Namespaces
**********

Since the CRTK API is fairly simple and the payloads don't necessarily
contain a string to specify which part of robot is used (for both
query and motion commands), namespaces can be used to define which
part of the robot is addressed.  For example, a robot manipulator will
often be composed of a kinematic chain for cartesian control and a
gripper at its tip.  The gripper can usually be driven only in joint
space.  Let's assume a Universal Robot (6 DOFs) with a gripper.  On
ROS, this robot can be represented using the following topics:

* ``/UR/`` namespace for the serial links with joints and cartesian commands

  * ``/UR/measured_js``, joint state for the first 6 joints

  * ``/UR/measured_cp``

  * ``/UR/setpoint_js``

  * ``/UR/setpoint_cp``

  * ``/UR/servo_jp``

  * ...

* ``/UR/gripper`` namespace for the gripper, only joint commands.

  * ``/UR/gripper/measured_js``, joint state for the single joint controlling the gripper

  * ``/UR/gripper/setpoint_js``

  * ``/UR/gripper/servo_js``, servo command to control the gripper's opening using one joint

Namespaces can also be used to organize different topics:

* Providing the forward kinematic with respect to a different
  reference frame.  For the dVRK, ``/PSM1/measured_cp`` is defined
  with respect to the camera coordinate system.  If a user needs
  access to the cartesian position with respect to the local
  coordinate system of the PSM, i.e. it's RCM (remote center of
  motion), they can use the topic ``/PSM1/local/measured_cp``.

* Providing information for redundant sensors.  The da Vinci arms used
  with the dVRK have redundant sensors on all joints.  To access the
  state of the default sensors (encoders), the ROS topic is
  ``/PSM1/measured_js``.  To access the potentiometers state, the
  topic is ``/PSM1/actuators/measured_js``.

* A namespace can also be used to define a new behavior.  By default
  ``servo_cf`` would be used to control the amount of force applied by
  the robot on its environment (for example with an haptic device).
  For a device with a handle mounted on a force sensor, the compliant
  control can be exposed using something like
  ``/galen/compliance/servo_cf``.  Sending a zero wrench would
  activate the force compliant mode where the robot is trying to
  maintain a zero force on the handle by following the forces applied
  by the user.


Namespaces can also be used besides ROS.  For example in Python or C++, one can create a struct or class to group some methods:

.. code-block:: python

   my_ur = ur('/UR')
   js_robot = my_ur.measured_js()
   js_gripper = my_ur.gripper.measured_js()


Pending issues, limitations
***************************

The following issues appeared as we implemented different robots using
CRTK, mostly on the dVRK.

* In the current implementation, ``servo_jp`` uses a ROS message type
  ``JointState`` that contains 3 vectors, position, velocity and
  effort.  The position is required but velocity and effort and
  ignored.  If the user can provide a velocity, this could potentially
  be used by the low-level controller (e.g. PID) to provide a better
  trajectory following.  The effort vector could be used as a bias
  force for the PID controller for such things as gravity compensation
  or haptic feedback.  To note, it would be hard to port this behavior
  ``servo_cp`` since the ROS payload currently used doesn't provide a
  placehold for the velocity nor effort.

* In the current specification and implementation, ``move`` commands
  use existing ROS message types.  As such there is no way to specify
  the desired velocity at the goal point.  Current implementation
  assumes the goal velocity is zero.

* Many commands, both query and motion ones, assume a reference frame.
  For example, a ``servo_cf`` command can be defined with respect to
  the base frame (aka space) or end effector (aka body).  Instead of
  using the ROS ``frame_id`` to define the reference frame, we used
  namespaces to define the reference frame, i.e. we define both
  ``spatial/servo_cf`` and ``body/servo_cf``.  The same can be applied
  for measured twist and wrench (``measured_cv`` and ``measured_cf``).

* For relative cartesian command, the specifications don't specify if
  the relative transformation is defined with respect to the end
  effector or the base frame of the robot.  These commands have not
  been implemented on the dVRK.


Other notes
***********

For temporarily unavailable data, set the time ``header.stamp``
to 0. For commands not supported at all by the robot, make sure
the topic is not available.

For all commands (aka topics) using dynamic vectors (e.g
``sensor_msgs/JointState``), all the non-empty dynamic vectors must
have the same size and the size must match the number of joints on the
robot used. If a subset of the information is not available, the
corresponding vector (name, position, velocity or effort) should be
empty, i.e. of size 0.

When using ROS, all query commands related to the robot telemetry
should be implemented as publishers on the robot side.
