General concepts
================

Motions
-------

We identified the following basic types of motions for collaborative
robotics. All these commands can be defined either in the cartesian or
joint space.

``servo``
^^^^^^^^^

Direct access to the low-level controller:

* **Use cases:**

  * User has a smooth and continuous trajectory coming from a master
    arm or recorded trajectory and can send commands at a high rate.

  * Closing the loop in velocity mode using a fast external sensor
    with a task based Jacobian.

  * Haptic feedback on master arm.

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
^^^^^^^^^^^^^^^

Simple interpolation:

* **Use cases:**

  * User has a smooth trajectory coming from a master arm or recorded
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
^^^^^^^^

Move with trajectory generation:

* **Use case**: User wants to move to a given position and stop there (e.g., home position, pick and place)

* **Type**: These commands are position based, either absolute or relative.

* **Continuity**: Users must send feasible commands. The low-level
  controller will compute a complete trajectory to move from the
  current state (position and velocity) to the desired goal.

* **Time**: Users are expected to send a single command and wait for
  completion before sending a new one. Time of execution is defined by
  the trajectory generation parameters (acceleration and velocity).


Naming convention
-----------------

Command names are based on the space, type and control level. The
prefix identifies the control level (``servo``, ``interpolate`` or
``move``), it is followed by an underscore (``_``) and two letters
identifying the space and type. For example, ``servo_cp`` is a "servo"
command with a cartesian (``c``) position (``p``) setpoint.

Query commands
^^^^^^^^^^^^^^

* Space: ``j`` (joint), ``c`` (cartesian)

* Type:

  * Joint: ``s`` (state: position, velocity and effort)

  * Cartesian: ``p`` (pose), ``v`` (twist), ``f`` (wrench)

* Level:

  * Measured: ``measured`` (physical measure from sensors)

  * Low-level: ``setpoint`` (current servo setpoint)

  * Mid-level: ``goal`` (current interpolate or move goal)

Motion commands
^^^^^^^^^^^^^^^

* Space: ``j`` (joint), ``c`` (cartesian)

* Type: ``p`` (position or pose), ``r`` (relative position or pose),
  ``v`` (velocity or twist), ``f`` (force, wrench or effort)

* Control level: ``servo`` (low-level), ``interpolate`` (basic
  interpolation), ``move`` (full trajectory planning)

Data validity
^^^^^^^^^^^^^

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

Overview
--------

Table
^^^^^

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
^^^^^^^

.. image:: ../images/CommonAPI.png
  :width: 400
  :align: center
  :alt: CRTK robot motion commands


Detailed API
============


General requirements
--------------------

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


Query commands
--------------


``measured_js``, measured joint state
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* **Payload:** ``sensor_msgs/JointState``

* **Specification:**
  * ``time header.stamp``: time of measurement [*required*]

  * ``string header.frame_id``: reference frame (this should match the
    ``header.frame_id`` string in ``measured_cp``) [*required*]

  * ``string name[]``: array of joint names [*required*]

  * ``float64 position[]``: array of measured joint positions [*optional*]

  * ``float64 velocity[]``: array of measured joint velocities [*optional*]

  * ``float64 effort[]``: array of measured joint efforts [*optional*]

* **Notes:**

  * velocity should be provided if the low-level controller has the
    ability to perform velocity estimation

  * effort should be provided if the low-level controller has torque
    sensors or current feedback per joints


``measured_cp``, measured cartesian position
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* **Payload:** ``geometry_msgs/PoseStamped`` (before 01/2022 payload
  was ``geometry_msgs/TransformStamped``, see #1)

* **Specification:**

  * ``time header.stamp``: time of measurement, if the measured
    cartesian position is based on a measured joint position, the time
    stamp should be the same as ``measured_js`` [*required*]

  * ``string header.frame_id``: reference frame [*required*]

  * ``string frame_id``: moving frame [*not available on ROS*]

  * ``Transform transform``: translation and rotation for the measured
    cartesian position (e.g. forward kinematics based on measured
    joint position from ``measured_js``) [*required*]


``measured_cv``, measured cartesian velocity (twist)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* **Payload:** ``geometry_msgs/TwistStamped``

* **Specification:**

  * ``time header.stamp``: time of measurement, if the measured twist
    is based on a measured joint velocity, the time stamp should be
    the same as ``measured_js`` [*required*]

  * ``string header.frame_id``: reference frame, see measured_cp [*required*]

  * ``string frame_id``: moving frame, see measured_cp [*not available on ROS*]

  * ``Twist twist``: linear and angular components for the measured
    cartesian velocity (e.g. Jacobian applied to measured joint
    velocities) [*required*]

* **Notes:**

  * This command will not be available if the low-level controller
    doesn't have a way to estimate joint velocity.


``measured_cf``, measured cartesian force (wrench)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* **Payload:** ``geometry_msgs/WrenchStamped``

* **Specification:**

  * ``time header.stamp``: time of measurement, if the measured wrench
    is based on a measured joint efforts, the time stamp should be the
    same as measured_js [*required*]

  * ``string header.frame_id``: reference frame, see measured_cp [*required*]

  * ``string frame_id``: moving frame, see `measured_cp` [*not available on ROS*]

  * ``Wrench wrench``: force and torque components for the measured
    cartesian wrench (e.g. Jacobian applied to measured joint efforts)
    [*required*]

* **Notes:**

  * This command will not be available if the low-level controller
    doesn't have a way to estimate joint efforts.


``setpoint_js``, joint setpoint (low-level controller)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* **Payload:** `sensor_msgs/JointState`

* **Specification:**

  * ``time Header.stamp``: time associated to last servo command. This
    can be defined by a direct servo command or an intermediary set
    point calculated by interpolate or move.

  * ``string header.frame_id``: reference frame, see `measured_js` [*required*]

  * ``string name[]``: array of joint names [*required*]

  * ``float64 position[]``: array of setpoint joint positions [see notes]

  * ``float64 velocity[]``: array of setpoint joint velocities [see notes]

  * ``float64 effort[]``: array of setpoint joint efforts [see notes]

* **Notes:**

  * At least one of the 3 vectors (position, velocity and effort) should be used.

  * Generally, this message should return *at least* the setpoint
    corresponding to the last motion command (for example, a
    ``position`` setpoint if the last motion specified a
    position). Depending on the type of low-level controller, other
    quantities could be included, as in the following examples:

    * If the controller hardware implements motor current (torque or
      effort) control, the low-level controller will convert a
      specified ``position`` or ``velocity`` setpoint to an ``effort``
      setpoint, ``F_llc``. In this case, the effort setpoint can also
      be included.

    * If the controller hardware implements position control, the
      low-level controller will integrate a ``velocity`` setpoint to
      become a ``position`` setpoint. In this case, both the
      ``position`` and ``velocity`` setpoint could be included.

  * When using the commands ``interpolate`` or ``move``, ``position``
    and ``velocity`` at time ``t`` should be computed by the
    interpolator or the trajectory generator (``p(t)`` and ``v(t)``).

  * If the command is defined in cartesian space, the corresponding
    joint space value should be provided (inverse kinematics for
    ``position``, use Jacobian for ``velocity`` and ``effort``).

  * Summary:

    .. list-table::
       :widths: 30 20 20 20
       :header-rows: 1

       * - **command level**
         - **``position``**
         - **``velocity``**
         - **``effort``**
       * - ``servo_{j,c}p``
 	 - ``setpoint``
 	 - n/a
 	 - ``F_llc`` or n/a
       * - ``servo_{j,c}v``
 	 - n/a
 	 - setpoint
 	 - ``F_llc`` or n/a
       * - ``servo_{j,c}f``
 	 - n/a
 	 - n/a
 	 - setpoint
       * - ``interpolate_{j,c}p``
 	 - ``p(t)``
 	 - ``V_llc`` or ``v(t)``
 	 - ``F_llc`` or n/a
       * - ``interpolate_{j,c}v``
 	 - n/a
 	 - ``V_llc`` or ``v(t)``
 	 - ``F_llc`` or n/a
       * - ``interpolate_{j,c}f``
 	 - n/a
 	 - n/a
 	 - ``f(t)``
       * - ``move_{j,c}p``
 	 - ``p(t)``
 	 - ``V_llc`` or ``v(t)``
 	 - ``F_llc`` or n/a


``setpoint_cp``, cartesian position setpoint (low-level controller)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* **Payload:** ``geometry_msgs/PoseStamped`` (before 01/2022 payload
  was ``geometry_msgs/TransformStamped``, see #1)

* **Specification:**

  * ``time header.stamp``: see ``setpoint_js`` [*required*]

  * ``string header.frame_id``: reference frame [*required*]

  * ``string frame_id``: moving frame [required]

  * ``Transform transform``: translation and rotation for the
    commanded cartesian position (e.g. forward kinematics based on
    joint positions from ``setpoint_js``) [*required*]

* **Notes:**

  * This query is valid only if the ``position`` field in
    ``setpoint_js`` is valid, i.e. when the motion commands are
    position based, i.e. ``{servo,interpolate,move}_{j,c}p``. For all
    other motion commands, the data should be marked as invalid by
    zeroing the ``time header.stamp``.


``setpoint_cv``, cartesian velocity setpoint (low-level controller)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

TODO, similar notes to ``setpoint_cp``


``setpoint_cf``, cartesian force setpoint (low-level controller)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

TODO, similar notes to ``setpoint_cp``


``goal_js``, joint goal (mid-level controller)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

TODO, should just report the end goal of ``{interpolate,move}_{c,j}{p,v,f}``


``goal_cp``, cartesian position goal (mid-level controller)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

TODO, should just report the goal in ``{interpolate,move}_{j,c}p``


``goal_cv``, cartesian position goal (mid-level controller)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

TODO, should just report the goal in ``interpolate_{j,c}v``


Motion commands
---------------


``servo_jp``, set position joint setpoint (low-level)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* **Payload:** ``sensor_msgs/JointState``

* **Specification:**

  * ``time Header.stamp``: time associated to the ``servo`` command [*not used but recommended*]

  * ``string header.frame_id``: reference frame, see ``measured_js`` [*not used but recommended*]

  * ``string name[]``: array of joint names [*not used but recommended*]

  * ``float64 position[]``: array of setpoint joint positions [*required*]

  * ``float64 velocity[]``: [*not used*]

  * ``float64 effort[]``: [*not used*]

* **Notes:**

  * These commands are pre-emptive, the latest command received will
    set the position setpoint used by the low-level controller.

  * `stamp`, `frame_id` and `name` are not used by the command so they
    could be left empty. It is nevertheless recommended to use them
    for data collection or further validation.


``servo_jr``, set position joint relative setpoint (low-level)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* **Payload:** ``sensor_msgs/JointState``

* **Specification:**

  * ``time Header.stamp``: time associated to the ``servo`` command [*not used but recommended*]

  * ``string header.frame_id``: reference frame, see ``measured_js`` [*not used but recommended*]

  * ``string name[]``: array of joint names [*not used but recommended*]

  * ``float64 position[]``: array of setpoint joint relative position [*required*]

  * ``float64 velocity[]``: [*not used*]

  * ``float64 effort[]``: [*not used*]

* **Notes:** See `servo_jp`.


``servo_jv``, set velocity joint setpoint (low-level)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* **Payload:** ``sensor_msgs/JointState``

* **Specification:**

  * ``time Header.stamp``: time associated to the ``servo`` command [*not used but recommended*]

  * ``string header.frame_id``: reference frame, see ``measured_js`` [*not used but recommended*]

  * ``string name[]``: array of joint names [*not used but recommended*]

  * ``float64 position[]``: [*not used*]

  * ``float64 velocity[]``: array of setpoint joint velocities [*required*]

  * ``float64 effort[]``: [*not used*]

* **Notes:** See `servo_jp`.


``servo_jf``, set effort joint setpoint (low-level)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

``servo_cp``, set position cartesian setpoint (low-level)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

``servo_cr``, set position cartesian relative setpoint (low-level)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

``servo_cv``, set velocity cartesian setpoint (low-level)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

``servo_cf``, set effort cartesian setpoint (low-level)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

``interpolate_jp``, set position joint goal (with interpolation)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

``interpolate_jr``, set position joint relative goal (with interpolation)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

``interpolate_jv``, set velocity joint goal (with interpolation)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

``interpolate_jf``, set effort joint goal (with interpolation)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

``interpolate_cp``, set position cartesian goal (with interpolation)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

``interpolate_cr``, set position cartesian relative goal (with interpolation)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

``interpolate_cv``, set velocity cartesian goal (with interpolation)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

``interpolate_cf``, set effort cartesian goal (with interpolation)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^


``move_jp``, set position joint goal (with trajectory generation)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* **Payload:** ``sensor_msgs/JointState``

* **Specification:**

  * ``time Header.stamp``: time associated to the ``servo`` command
    [*not used but recommended*]

  * ``string header.frame_id``: reference frame, see ``measured_js``
    [*not used but recommended*]

  * ``string name[]``: array of joint names [*not used but
    recommended*]

  * ``float64 position[]``: array of goal joint positions [*required*]

  * ``float64 velocity[]``: [*not used*]

  * ``float64 effort[]``: [*not used*]


``move_jr``, set position joint relative goal (with trajectory generation)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* **Payload:** ``sensor_msgs/JointState``

* **Specification:**

  * ``time Header.stamp``: time associated to the ``servo`` command
    [*not used but recommended*]

  * ``string header.frame_id``: reference frame, see ``measured_js``
    [*not used but recommended*]

  * ``string name[]``: array of joint names [*not used but
    recommended*]

  * ``float64 position[]``: array of goal joint relative positions
    [*required*]

  * ``float64 velocity[]``: [*not used*]

  * ``float64 effort[]``: [*not used*]


``move_cp``, set position cartesian goal (with trajectory generation)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

``move_cr``, set position cartesian relative goal (with trajectory generation)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
